"""
oak_tracking.launch.py
======================
OAK-D 실시간 추적 풀 파이프라인.
  oak_detector (ros2_yolo_oak) → /detections, /camera/color/camera_info, /oak/rgb/image_raw
  tracking_node (ros2_tracking_node) → /owner_pose
  oak_viz (ros2_yolo_oak) → OpenCV 창

실행:
  ros2 launch ros2_yolo_oak oak_tracking.launch.py
  # viz 끄기: viz:=false

모델: oak_params.yaml의 'model' 키(zoo 모델명, 예: yolov6-nano). blob_path 안 씀(v3).
주의: tracking_params.yaml의 image_width/height가 oak preview(512x384)와 일치해야 함.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    oak_params = os.path.join(
        get_package_share_directory('ros2_yolo_oak'),
        'config', 'oak_params.yaml')
    tracking_params = os.path.join(
        get_package_share_directory('ros2_tracking_node'),
        'config', 'tracking_params.yaml')

    viz_arg = DeclareLaunchArgument(
        'viz', default_value='true',
        description='OpenCV 시각화 창 사용 여부')

    viz = LaunchConfiguration('viz')

    oak_detector = Node(
        package='ros2_yolo_oak', executable='oak_detector', name='oak_detector',
        output='screen',
        # [0525] v3 detector는 blob_path를 쓰지 않음(zoo 모델명 사용).
        #   과거 blob_path 강제 주입은 제거. 모델은 oak_params.yaml의 'model' 키.
        parameters=[oak_params])

    tracking_node = Node(
        package='ros2_tracking_node', executable='tracking_node', name='tracking_node',
        output='screen',
        parameters=[tracking_params])

    oak_viz = Node(
        package='ros2_yolo_oak', executable='oak_viz', name='oak_viz',
        output='screen',
        condition=IfCondition(viz))

    return LaunchDescription([
        viz_arg,
        oak_detector, tracking_node, oak_viz,
    ])
