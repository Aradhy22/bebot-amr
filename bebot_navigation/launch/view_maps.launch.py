#!/usr/bin/env python3
"""View the saved 2D occupancy grid and 3D point cloud map together in
RViz - no simulation, no robot, just the two saved map artifacts.

  ros2 launch bebot_navigation view_maps.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bebot_navigation = get_package_share_directory('bebot_navigation')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~/bebot_ws/src/bebot_navigation/maps'), 'warehouse_map.yaml'),
        description='Full path to the 2D map yaml file (see pcd_to_occupancy_grid)'
    )

    pcd_arg = DeclareLaunchArgument(
        'pcd',
        default_value=os.path.expanduser('~/bebot_maps/bebot_map_loop_closed.pcd'),
        description='Full path to the 3D PCD file (see bebot_slam loop_closure_node /loop_closure/save_map)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz with both maps loaded'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False,
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    pcd_publisher_node = Node(
        package='bebot_navigation',
        executable='pcd_publisher_node',
        name='pcd_publisher_node',
        output='screen',
        parameters=[{
            'pcd_path': LaunchConfiguration('pcd'),
            'frame_id': 'map',
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='view_maps_rviz',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_bebot_navigation, 'rviz', 'view_maps.rviz')],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        map_arg,
        pcd_arg,
        rviz_arg,
        map_server_node,
        lifecycle_manager,
        pcd_publisher_node,
        rviz_node,
    ])
