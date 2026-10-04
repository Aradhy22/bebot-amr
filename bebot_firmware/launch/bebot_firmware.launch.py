import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bebot_firmware')
    default_params = os.path.join(pkg_share, 'config', 'bebot_firmware.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the bebot_firmware parameters YAML file'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial device the Arduino Uno is connected to'
    )

    serial_odometry_node = Node(
        package='bebot_firmware',
        executable='serial_odometry_node',
        name='serial_odometry_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'serial_port': LaunchConfiguration('serial_port')}
        ]
    )

    return LaunchDescription([
        params_arg,
        serial_port_arg,
        serial_odometry_node,
    ])
