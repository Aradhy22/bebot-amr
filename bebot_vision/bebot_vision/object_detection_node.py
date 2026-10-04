#!/usr/bin/env python3
"""Object detection + tracking + depth-based distance estimation on the
bebot's front camera (simulated D435i in Gazebo, real D435i on hardware).

Uses YOLOv8n (Ultralytics) with its built-in ByteTrack tracker for
persistent per-object IDs across frames - chosen specifically because it
has a well-supported TensorRT export path for Jetson Orin Nano deployment
(`model.export(format='engine')`), unlike older OpenCV-DNN-based detectors.
On this desktop it runs on CPU; on the Jetson it should be re-exported to a
TensorRT engine for real-time performance rather than run as a raw .pt file.

Color and depth frames come from the RGBD camera sensor added to
bebot_gazebo.urdf.xacro (sim) / the real D435i's ROS driver (hardware) -
both publish already pixel-aligned color/depth, so no explicit
depth-to-color registration is needed here.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO

BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (0, 0, 0)


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('color_topic', '/camera/image')
        self.declare_parameter('depth_topic', '/camera/depth_image')

        model_name = self.get_parameter('model').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        self.get_logger().info(f'Loading {model_name} (first run downloads the pretrained weights) ...')
        self.model = YOLO(model_name)
        self.get_logger().info('Model loaded.')

        self.bridge = CvBridge()

        self.annotated_pub = self.create_publisher(Image, '/camera/detections/image', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, '/camera/detections', 10)

        color_sub = message_filters.Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info(
            f'Subscribed to {color_topic} + {depth_topic}, '
            f'publishing /camera/detections/image and /camera/detections')

    def image_callback(self, color_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        results = self.model.track(frame, persist=True, verbose=False, conf=self.conf_threshold)

        annotated = frame.copy()
        det_array = Detection2DArray()
        det_array.header = color_msg.header

        boxes = results[0].boxes if results else None
        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                track_id = int(box.id[0]) if box.id is not None else -1
                label = self.model.names[cls_id]

                distance = self._median_depth(depth, x1, y1, x2, y2)
                self._draw_detection(annotated, x1, y1, x2, y2, label, conf, track_id, distance)
                det_array.detections.append(
                    self._make_detection_msg(color_msg, x1, y1, x2, y2, label, conf, track_id, distance))

        self.annotated_pub.publish(self._bgr8_to_imgmsg(annotated, color_msg.header))
        self.detections_pub.publish(det_array)

    @staticmethod
    def _bgr8_to_imgmsg(image, header: Header) -> Image:
        # cv_bridge's own cv2_to_imgmsg(encoding='bgr8') raises KeyError on
        # this system's OpenCV 5.0 build - its internal cvtype-name lookup
        # table used to sanity-check the encoding wasn't updated for it.
        # Reading images (imgmsg_to_cv2) isn't affected, only this publish
        # path, so this constructs the message directly instead.
        msg = Image()
        msg.header = header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = image.shape[1] * 3
        msg.data = image.tobytes()
        return msg

    @staticmethod
    def _median_depth(depth, x1, y1, x2, y2):
        h, w = depth.shape[:2]
        x1c, y1c, x2c, y2c = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
        if x2c <= x1c or y2c <= y1c:
            return float('nan')
        roi = depth[y1c:y2c, x1c:x2c]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        return float(np.median(valid)) if valid.size > 0 else float('nan')

    @staticmethod
    def _draw_detection(image, x1, y1, x2, y2, label, conf, track_id, distance):
        cv2.rectangle(image, (x1, y1), (x2, y2), BOX_COLOR, 2)
        dist_str = f'{distance:.2f}m' if np.isfinite(distance) else '? m'
        id_str = f'#{track_id} ' if track_id >= 0 else ''
        text = f'{id_str}{label} {conf:.2f} {dist_str}'
        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_y1 = max(0, y1 - th - baseline - 4)
        cv2.rectangle(image, (x1, label_y1), (x1 + tw + 4, y1), BOX_COLOR, -1)
        cv2.putText(image, text, (x1 + 2, y1 - baseline - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1, cv2.LINE_AA)

    @staticmethod
    def _make_detection_msg(color_msg, x1, y1, x2, y2, label, conf, track_id, distance):
        det = Detection2D()
        det.header = color_msg.header
        det.id = str(track_id) if track_id >= 0 else ''
        det.bbox.center.position.x = float((x1 + x2) / 2.0)
        det.bbox.center.position.y = float((y1 + y2) / 2.0)
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = label
        hyp.hypothesis.score = conf
        # Not a full 3D pose estimate - just carrying the depth-camera-derived
        # range (meters, along the camera's optical axis) in a standard field
        # so downstream consumers don't need a bebot-specific message type.
        hyp.pose.pose.position.z = distance if np.isfinite(distance) else 0.0
        det.results.append(hyp)
        return det


def main():
    rclpy.init()
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
