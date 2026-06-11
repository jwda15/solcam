import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('ros2_driver_bridge'),
        'config', 'driver_params.yaml')

    port = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    mock = DeclareLaunchArgument('mock', default_value='false')

    return LaunchDescription([
        port,
        mock,
        Node(
            package='ros2_driver_bridge',
            executable='driver_bridge',
            name='driver_bridge',
            output='screen',
            parameters=[
                params,
                {'port': LaunchConfiguration('port')},
                {'mock': LaunchConfiguration('mock')},
            ],
        ),
    ])
