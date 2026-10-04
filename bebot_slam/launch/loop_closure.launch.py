#!/usr/bin/env python3
"""Launch the loop-closure backend on top of an already-running FAST-LIO.

Consumes FAST-LIO's /Odometry and /cloud_registered_body - launch this
alongside (not instead of) bebot_slam fast_lio.launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bebot_slam = get_package_share_directory('bebot_slam')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch the loop-closure RViz view (corrected map/path)'
    )

    loop_closure_node = Node(
        package='bebot_slam',
        executable='loop_closure_node',
        name='loop_closure_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='loop_closure_rviz',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_bebot_slam, 'rviz', 'loop_closure.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        loop_closure_node,
        rviz_node,
    ])
