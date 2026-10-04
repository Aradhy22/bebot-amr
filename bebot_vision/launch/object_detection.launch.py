#!/usr/bin/env python3
"""Launch the camera-based object detection/tracking/distance node.

Prerequisite: the camera publishing on /camera/image and /camera/depth_image
(bebot_gazebo.urdf.xacro's simulated D435i, or the real D435i's ROS driver).

View the annotated output with:
  ros2 run rqt_image_view rqt_image_view /camera/detections/image
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model', default_value='yolov8n.pt',
        description='Ultralytics model name/path (yolov8n.pt for dev; a TensorRT .engine on Jetson)'
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.25',
        description='Minimum detection confidence to keep/draw (Ultralytics\' own default - 0.5 '
                    'was too strict for a photo-trained model on Gazebo\'s flat-shaded rendering)'
    )
    color_topic_arg = DeclareLaunchArgument(
        'color_topic', default_value='/camera/image', description='Color image topic'
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/depth_image', description='Depth image topic (32FC1, meters)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    object_detection_node = Node(
        package='bebot_vision',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        model_arg,
        confidence_arg,
        color_topic_arg,
        depth_topic_arg,
        use_sim_time_arg,
        object_detection_node,
    ])
