#!/usr/bin/env python3
"""Keyboard teleop with reduced default speeds for the bebot AMR.

teleop_twist_keyboard's own defaults (speed=0.5 m/s, turn=1.0 rad/s) are
faster than comfortable for driving through a warehouse full of shelves -
this wraps it with gentler defaults, both still overridable via launch args.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.15',
        description='Linear speed (m/s) per forward/backward keypress'
    )

    turn_arg = DeclareLaunchArgument(
        'turn',
        default_value='0.3',
        description='Angular speed (rad/s) per turn keypress'
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'turn': LaunchConfiguration('turn'),
        }]
    )

    return LaunchDescription([
        speed_arg,
        turn_arg,
        teleop_node,
    ])
