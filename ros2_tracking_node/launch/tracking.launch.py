from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # config 파일 경로
    params_file = os.path.join(
        get_package_share_directory('ros2_tracking_node'),
        'config',
        'tracking_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_tracking_node',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            parameters=[params_file],  # YAML 파라미터 파일 로드
        )
    ])
