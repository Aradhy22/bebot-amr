import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('bebot_description')
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'bebot.urdf.xacro')
    default_rviz_path = os.path.join(pkg_share, 'rviz', 'bebot_description.rviz')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_urdf_path,
        description='Absolute path to the robot xacro/URDF file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_path,
        description='Absolute path to the RViz2 config file'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui to move wheel joints with sliders'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher_gui'))
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_joint_state_publisher_gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])
