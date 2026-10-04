#!/usr/bin/env python3
"""Bring up everything needed for a 3D mapping session in one launch file:
Gazebo simulation, FAST-LIO (lidar-inertial mapping), and the loop-closure
backend with its live map view in RViz.

This replaces running gazebo.launch.py / fast_lio.launch.py /
loop_closure.launch.py in three separate terminals. Still needed
separately, in their own terminal(s):
  - teleop to actually drive the robot:
      ros2 launch bebot_bringup teleop.launch.py
  - saving the map once you're done driving:
      ros2 service call /loop_closure/save_map std_srvs/srv/Trigger {}
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
    pkg_bebot_slam = get_package_share_directory('bebot_slam')
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

    fastlio_config_arg = DeclareLaunchArgument(
        'fastlio_config_file',
        default_value='fast_lio_ouster_sim.yaml',
        description='FAST-LIO config file name (in bebot_slam/config/)'
    )

    # Named differently from the plain 'rviz' argument that gazebo.launch.py
    # and fast_lio.launch.py each declare (both hardcoded to 'false' below):
    # LaunchConfiguration names aren't scoped per-include, so reusing 'rviz'
    # here would get silently overwritten by those two declarations before
    # this one is ever read - the loop-closure view would never launch
    # regardless of the CLI value passed in.
    rviz_arg = DeclareLaunchArgument(
        'mapping_rviz',
        default_value='true',
        description='Launch the loop-closure RViz view (live corrected map/path)'
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

    # Delayed rather than started alongside Gazebo: the robot spawns 0.1m
    # above the floor and drops/settles under physics for the first couple
    # of seconds. That settling is a real, brief high-acceleration event -
    # not sensor noise - and FAST-LIO's iEKF has no local map yet to weigh
    # it against. Starting FAST-LIO before the robot has actually come to
    # rest measurably diverges the state estimate (position reaching
    # hundreds/thousands of meters within seconds, loop closure logging
    # hundreds of spurious keyframes) - reproduced directly by launching
    # concurrently, and confirmed fixed by waiting for the robot to settle
    # (verified via Gazebo's own ground-truth pose) before starting
    # FAST-LIO. 10s is a generous margin over the ~2s it actually takes.
    fast_lio = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bebot_slam, 'launch', 'fast_lio.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'config_file': LaunchConfiguration('fastlio_config_file'),
                    'rviz': 'false',
                }.items()
            )
        ]
    )

    # loop_closure_node only consumes FAST-LIO's already-published output
    # topics, so it isn't part of the settling-race above - it just needs
    # to start at or after FAST-LIO, which this also satisfies.
    loop_closure = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bebot_slam, 'launch', 'loop_closure.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    # Always false here - RViz is launched separately below,
                    # further delayed, instead of bundled synchronously with
                    # loop_closure_node.
                    'rviz': 'false',
                }.items()
            )
        ]
    )

    # Grouped with fast_lio/loop_closure at the same 10s mark: doesn't have
    # their settling-race problem (no stateful integration over time to
    # diverge), just subscribes and waits for /camera/image + depth once
    # they're up.
    object_detection = TimerAction(
        period=10.0,
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

    # Delayed further still (past the fast_lio/loop_closure start at 10s):
    # Gazebo's GUI and RViz both need an OpenGL/EGL context at startup, and
    # starting them at the exact same instant races for that - in every
    # manual multi-terminal test during development this only worked
    # reliably by staggering the launches a few seconds apart, and the
    # concurrent version reproducibly had the RViz process silently
    # disappear shortly after startup with no error logged.
    rviz_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='loop_closure_rviz',
                condition=IfCondition(LaunchConfiguration('mapping_rviz')),
                arguments=['-d', os.path.join(pkg_bebot_slam, 'rviz', 'loop_closure.rviz')],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    # Staggered a further 3s past RViz for the same OpenGL/EGL context-race
    # reason - a third GUI window starting at the exact same instant as the
    # other two risks the same silent-disappearance failure.
    detections_view = TimerAction(
        period=17.0,
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
        fastlio_config_arg,
        rviz_arg,
        camera_arg,
        gazebo,
        fast_lio,
        loop_closure,
        object_detection,
        rviz_node,
        detections_view,
    ])
