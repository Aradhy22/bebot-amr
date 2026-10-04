#!/usr/bin/env python3
"""Bring up Nav2 (AMCL + planner/controller/costmaps) against a pre-built
2D map, for the bebot AMR.

Prerequisites (start these first, this launch file does not):
  - The simulation (or real robot) publishing /lidar/points and TF
  - bebot_odom's EKF (publishes /odometry/filtered and odom->base_footprint,
    which Nav2's controller/velocity_smoother consume)
  - A map generated via bebot_navigation's pcd_to_occupancy_grid tool, e.g.:
      ros2 run bebot_navigation pcd_to_occupancy_grid \\
        --pcd ~/bebot_maps/bebot_map_loop_closed.pcd

AMCL needs a 2D LaserScan, but this robot's only lidar is the 3D OS1 - a
pointcloud_to_laserscan node slices the live 3D cloud into one here.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bebot_navigation = get_package_share_directory('bebot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(
        os.path.expanduser('~/bebot_ws/src/bebot_navigation/maps'), 'warehouse_map.yaml')
    default_params = os.path.join(pkg_bebot_navigation, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml file (see pcd_to_occupancy_grid)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 params yaml file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Auto-start the Nav2 lifecycle nodes'
    )

    # The OS1's raw scan is in the sensor's own frame; target_frame transforms
    # it into base_footprint first so min/max height are simply "meters above
    # the floor" - matching the same convention pcd_to_occupancy_grid uses
    # for the obstacle band when building the static map, rather than having
    # to reason about the sensor's own mounting height separately.
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.2,
            'min_height': 0.05,
            'max_height': 0.6,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 20.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/lidar/points'),
            ('scan', '/scan'),
        ]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'slam': 'False',
            'use_localization': 'True',
        }.items()
    )

    # nav2_bringup's bringup_launch.py does NOT start collision_monitor, but
    # velocity_smoother (which IS started) publishes its output on
    # cmd_vel_smoothed expecting collision_monitor to police it and forward
    # to cmd_vel - the topic the robot actually drives on. Without this node
    # the controller computes valid velocities that simply go nowhere and
    # every goal times out with "Failed to make progress". It needs its own
    # tiny lifecycle manager since it isn't in nav2_bringup's node list.
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    lifecycle_manager_collision_monitor = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['collision_monitor'],
        }]
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        pointcloud_to_laserscan_node,
        nav2_bringup,
        collision_monitor_node,
        lifecycle_manager_collision_monitor,
    ])
