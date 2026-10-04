#!/usr/bin/env python3
"""Launch Gazebo Harmonic simulation with the bebot robot."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_bebot_gazebo = get_package_share_directory('bebot_gazebo')
    pkg_bebot_description = get_package_share_directory('bebot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
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

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Spawn X position')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0', description='Spawn Y position')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.1', description='Spawn Z position')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )

    # Paths
    urdf_file = os.path.join(pkg_bebot_description, 'urdf', 'bebot_gazebo.urdf.xacro')
    # Resolves against the package's worlds/ dir; os.path.join (used internally
    # by PathJoinSubstitution) still returns an absolute path untouched, so a
    # full path passed via the 'world' arg keeps working too.
    world_file = PathJoinSubstitution([pkg_bebot_gazebo, 'worlds', LaunchConfiguration('world')])

    # Process robot description. Wrapped as an explicit string parameter so
    # launch_ros doesn't try to auto-detect the parameter type by running the
    # (very long, multi-line) URDF text through a YAML parser.
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description
        }]
    )

    # Gazebo Harmonic launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file]
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', LaunchConfiguration('world_name'),
            '-name', 'bebot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z')
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for common topics. DiffDrive's own odom->base_footprint
    # TF (gz topic "tf") is intentionally not bridged: bebot_odom's EKF is the
    # canonical TF publisher for that transform, and bridging both would fight
    # over the same TF edge.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # sdformat's URDF->SDF conversion lumps every fixed-jointed link (laser_link,
    # imu_link, camera_link, rear_box_link, base_link) into a single SDF link
    # "base_footprint", so gz-sensors publishes lidar/imu/camera data under
    # its own scoped frame names (bebot/base_footprint/lidar,
    # bebot/base_footprint/imu_sensor, bebot/base_footprint/d435i) instead of
    # laser_link/imu_link/camera_link. These identity static transforms alias
    # those scoped frames onto base_footprint so RViz/tf2 can resolve them.
    lidar_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'bebot/base_footprint/lidar'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    imu_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'bebot/base_footprint/imu_sensor'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    camera_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'bebot/base_footprint/d435i'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_bebot_description, 'rviz', 'bebot_description.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        world_arg,
        world_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        use_sim_time_arg,
        rviz_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        lidar_frame_alias,
        imu_frame_alias,
        camera_frame_alias,
        rviz
    ])
