#!/usr/bin/env python3
"""Serial bridge + encoder odometry node for the bebot skid-steer AMR.

Talks to an Arduino Uno over USB serial. The Uno drives four skid-steer
motors (front-left/front-right mirror rear-left/rear-right) and reports
back cumulative quadrature encoder ticks from the two rear wheels only.

Serial protocol (newline terminated ASCII, matches bebot_arduino.ino):
  Arduino -> Jetson :  "E,<left_ticks>,<right_ticks>\\n"   (periodic)
  Jetson  -> Arduino:  "M,<left_pwm>,<right_pwm>\\n"       (periodic, also acts as watchdog heartbeat)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SerialOdometryNode(Node):

    def __init__(self):
        super().__init__('serial_odometry_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.34)
        self.declare_parameter('ticks_per_rev', 960)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('cmd_timer_hz', 20.0)
        self.declare_parameter('serial_poll_hz', 50.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_encoder_time = None

        self.target_left_pwm = 0
        self.target_right_pwm = 0

        self.rx_buffer = ''

        if serial is None:
            self.get_logger().error(
                "pyserial is not installed. Install it with 'pip3 install pyserial' "
                "(or apt install python3-serial).")
            raise SystemExit(1)

        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0)
        self.get_logger().info(
            f'Opened serial port {self.serial_port} @ {self.baud_rate} baud')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        serial_poll_hz = self.get_parameter('serial_poll_hz').value
        self.serial_timer = self.create_timer(1.0 / serial_poll_hz, self.read_serial)

        cmd_timer_hz = self.get_parameter('cmd_timer_hz').value
        self.cmd_timer = self.create_timer(1.0 / cmd_timer_hz, self.send_motor_command)

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - (angular * self.wheel_separation / 2.0)
        right_speed = linear + (angular * self.wheel_separation / 2.0)

        self.target_left_pwm = self.speed_to_pwm(left_speed)
        self.target_right_pwm = self.speed_to_pwm(right_speed)

    def speed_to_pwm(self, speed: float) -> int:
        if self.max_linear_speed <= 0.0:
            return 0
        pwm = (speed / self.max_linear_speed) * self.max_pwm
        return int(max(-self.max_pwm, min(self.max_pwm, pwm)))

    def send_motor_command(self):
        line = f'M,{self.target_left_pwm},{self.target_right_pwm}\n'
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().warning(f'Serial write failed: {exc}')

    def read_serial(self):
        try:
            waiting = self.ser.in_waiting
        except serial.SerialException as exc:
            self.get_logger().warning(f'Serial read failed: {exc}')
            return

        if waiting <= 0:
            return

        try:
            chunk = self.ser.read(waiting).decode('ascii', errors='ignore')
        except serial.SerialException as exc:
            self.get_logger().warning(f'Serial read failed: {exc}')
            return

        self.rx_buffer += chunk
        while '\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split('\n', 1)
            self.process_line(line.strip())

    def process_line(self, line: str):
        if not line.startswith('E,'):
            return

        parts = line.split(',')
        if len(parts) != 3:
            return

        try:
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
        except ValueError:
            return

        now = self.get_clock().now()

        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_encoder_time = now
            return

        dt = (now - self.last_encoder_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        delta_left_ticks = left_ticks - self.last_left_ticks
        delta_right_ticks = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_encoder_time = now

        delta_left = delta_left_ticks * self.dist_per_tick
        delta_right = delta_right_ticks * self.dist_per_tick

        delta_s = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_separation

        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))

        self.vx = delta_s / dt
        self.vth = delta_theta / dt

        self.publish_odometry(now)

    def publish_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        try:
            self.ser.write(b'M,0,0\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
