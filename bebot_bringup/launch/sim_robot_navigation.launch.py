#!/usr/bin/env python3
"""Bring up everything needed for autonomous navigation in one launch file:
Gazebo simulation, the EKF (wheel + IMU fusion), and the full Nav2 stack
(AMCL + planner/controller/costmaps + collision monitor) against a
pre-built 2D map. Also opens RViz with Nav2's default view so goals can be
sent right away.

This replaces running gazebo.launch.py / bebot_odom ekf.launch.py /
bebot_navigation nav2.launch.py in three separate terminals.

Once RViz is up: if the robot's shown position doesn't match its real spot
on the map, click "2D Pose Estimate" and set it there first - the map's
built-in initial-pose guess assumes the robot spawns where the map was
recorded from, which isn't always exactly right. Then click "Nav2 Goal" and
click a destination.
"""

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
    pkg_bebot_odom = get_package_share_directory('bebot_odom')
    pkg_bebot_navigation = get_package_share_directory('bebot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_bebot_vision = get_package_share_directory('bebot_vision')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse.sdf',
        description='World file name (in bebot_gazebo/worlds/ directory or full path)'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='industrial-warehouse',
        description="The <world name=\"...\"> value inside the world file (used to target spawning)"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn X position')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='-7.0', description='Spawn Y position')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.1', description='Spawn Z position')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~/bebot_ws/src/bebot_navigation/maps'), 'warehouse_map.yaml'),
        description='Full path to the map yaml file (see pcd_to_occupancy_grid)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Auto-start the Nav2 lifecycle nodes'
    )

    # Named differently from the plain 'rviz' argument that gazebo.launch.py
    # declares internally (hardcoded to 'false' below regardless):
    # LaunchConfiguration names aren't scoped per-include, so a same-named
    # top-level argument here would get silently overwritten before this
    # one is ever read, and RViz would never launch regardless of the CLI
    # value passed in - see sim_robot_mapping.launch.py's 'mapping_rviz' for
    # the same issue hit there first.
    rviz_arg = DeclareLaunchArgument(
        'nav_rviz',
        default_value='true',
        description="Launch Nav2's default RViz view for sending goals"
    )

    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Launch object detection/tracking on the front camera feed'
    )

    gazebo = IncludeLaunchDescription(
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
            'rviz': 'false',
        }.items()
    )

    # Delayed rather than started alongside Gazebo: gives the server, GUI,
    # and bridge time to fully come up (and the spawned robot time to
    # settle physically) before the EKF and Nav2 stack start consuming
    # their topics - matches the multi-terminal sequence that was always
    # used manually (gazebo first, then wait, then the rest).
    ekf = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bebot_odom, 'launch', 'ekf.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )

    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bebot_navigation, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                }.items()
            )
        ]
    )

    # Grouped with EKF/Nav2 at the same 8s mark: no settling-race concern
    # (just subscribes and waits for /camera/image + depth once they're up).
    object_detection = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bebot_vision, 'launch', 'object_detection.launch.py')
                ),
                condition=IfCondition(LaunchConfiguration('camera')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )

    # Delayed further still (past the EKF/Nav2 start at 8s): Gazebo's GUI
    # and RViz both need an OpenGL/EGL context at startup, and starting
    # them at the exact same instant races for that - reproduced directly
    # during development of sim_robot_mapping.launch.py (RViz silently
    # disappeared shortly after startup with no error logged).
    rviz_node = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='nav2_rviz',
                condition=IfCondition(LaunchConfiguration('nav_rviz')),
                arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    # Staggered a further 3s past RViz for the same OpenGL/EGL context-race
    # reason - a third GUI window starting at the exact same instant as the
    # other two risks the same silent-disappearance failure.
    detections_view = TimerAction(
        period=16.0,
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
        map_arg,
        autostart_arg,
        rviz_arg,
        camera_arg,
        gazebo,
        ekf,
        nav2,
        object_detection,
        rviz_node,
        detections_view,
    ])
