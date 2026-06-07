# gesture_node (+선택: LCD UI) 실행.
#   ros2 launch ros2_gesture_node gesture.launch.py            # 인식+UI
#   ros2 launch ros2_gesture_node gesture.launch.py ui:=false  # 인식만
#   ros2 launch ros2_gesture_node gesture.launch.py recognizer:=mock  # 모델 없이
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ros2_gesture_node'),
                          'config', 'gesture_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('ui', default_value='true'),
        DeclareLaunchArgument('recognizer', default_value='hagrid'),
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('model_path', default_value='models/YOLOv10n_gestures.pt'),
        Node(package='ros2_gesture_node', executable='gesture_node',
             parameters=[params,
                         {'recognizer': LaunchConfiguration('recognizer'),
                          'image_topic': LaunchConfiguration('image_topic'),
                          'model_path': LaunchConfiguration('model_path')}],
             output='screen'),
        Node(package='ros2_gesture_node', executable='ui_node',
             parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('ui'))),
    ])
