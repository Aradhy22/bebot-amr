#!/usr/bin/env python3
"""Launch FAST-LIO 3D lidar-inertial mapping against the bebot lidar/imu data.

Point this at the simulated Ouster OS1-32 by default (config_file:=
fast_lio_ouster_sim.yaml); swap in a real-hardware config (with lidar_type: 3
so FAST-LIO uses its native Ouster per-point-time handler) once the real OS1
replaces the simulated one.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bebot_slam = get_package_share_directory('bebot_slam')
    pkg_fast_lio = get_package_share_directory('fast_lio')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='fast_lio_ouster_sim.yaml',
        description='FAST-LIO config file name (in bebot_slam/config/)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description="Launch FAST-LIO's own RViz view (registered map/path)"
    )

    # The config file's map_file_path is a relative path, which resolves
    # against whatever directory the node happens to be launched from - not
    # predictable/discoverable. Override it here with a stable absolute path
    # instead, and make sure that directory actually exists.
    map_save_dir = os.path.expanduser('~/bebot_maps')
    os.makedirs(map_save_dir, exist_ok=True)
    map_file_path = os.path.join(map_save_dir, 'bebot_map.pcd')

    config_path = PathJoinSubstitution([pkg_bebot_slam, 'config', LaunchConfiguration('config_file')])

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            config_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_path': map_file_path,
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='fastlio_rviz',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_fast_lio, 'rviz', 'fastlio.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        rviz_arg,
        fast_lio_node,
        rviz_node,
    ])
