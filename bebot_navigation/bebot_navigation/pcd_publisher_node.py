#!/usr/bin/env python3
"""Latch-publish a saved ASCII PCD as a PointCloud2 on /map_cloud so it can
be viewed in RViz alongside the 2D occupancy grid (view_maps.launch.py).

Usage: ros2 run bebot_navigation pcd_publisher_node --ros-args -p pcd_path:=~/bebot_maps/bebot_map_loop_closed.pcd
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_pcd_ascii(path: str) -> np.ndarray:
    points = []
    with open(path, 'r') as f:
        in_data = False
        for line in f:
            if not in_data:
                if line.strip().upper().startswith('DATA'):
                    in_data = True
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            points.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return np.array(points, dtype=np.float32)


def make_cloud_msg(points: np.ndarray, frame_id: str) -> PointCloud2:
    header = Header()
    header.frame_id = frame_id
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * points.shape[0]
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


class PcdPublisherNode(Node):
    def __init__(self):
        super().__init__('pcd_publisher_node')

        self.declare_parameter('pcd_path', os.path.expanduser('~/bebot_maps/bebot_map_loop_closed.pcd'))
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_sec', 1.0)

        pcd_path = os.path.expanduser(
            self.get_parameter('pcd_path').get_parameter_value().string_value)
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').value

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(PointCloud2, '/map_cloud', qos)

        self.get_logger().info(f'Loading {pcd_path} ...')
        points = read_pcd_ascii(pcd_path)
        self.get_logger().info(f'Loaded {len(points)} points, publishing on /map_cloud (frame={frame_id})')
        self.msg = make_cloud_msg(points, frame_id)
        self.timer = self.create_timer(period, self.publish_once)

    def publish_once(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = PcdPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
