#!/usr/bin/env python3
"""Bring up the bebot Gazebo simulation together with RViz."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bebot_gazebo = get_package_share_directory('bebot_gazebo')
    pkg_bebot_vision = get_package_share_directory('bebot_vision')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file name (in worlds/ directory or full path)'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty_world',
        description="The <world name=\"...\"> value inside the world file (used to target spawning)"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn X position')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0', description='Spawn Y position')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.1', description='Spawn Z position')

    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Launch object detection/tracking on the front camera feed'
    )

    gazebo_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bebot_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'rviz': 'true',
        }.items()
    )

    object_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bebot_vision, 'launch', 'object_detection.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('camera')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Delayed rather than started alongside Gazebo's GUI + RViz (both
    # already starting at t=0 here): a third GUI window grabbing an OpenGL/
    # EGL context at the same instant as the other two raises the same
    # startup race documented in sim_robot_mapping.launch.py/
    # sim_robot_navigation.launch.py, where it was reproduced directly.
    detections_view = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='detections_view',
                condition=IfCondition(LaunchConfiguration('camera')),
                arguments=['/camera/detections/image'],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        world_name_arg,
        use_sim_time_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        camera_arg,
        gazebo_and_rviz,
        object_detection,
        detections_view,
    ])
