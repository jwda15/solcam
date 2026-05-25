from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('ros2_follow_node'),
        'config',
        'follow_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_follow_node',
            executable='follow_node',
            name='follow_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='ros2_follow_node',
            executable='mecanum_driver',
            name='mecanum_driver',
            output='screen',
            parameters=[params_file],
        ),
    ])
