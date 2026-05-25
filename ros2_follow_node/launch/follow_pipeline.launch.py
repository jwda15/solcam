"""
follow_pipeline.launch.py
=========================
별도 PC(ROS2)에서 실행:
  1. yolo_detector       : D435i RGB+Depth → /detections
  2. tracking_node       : /detections → /owner_pose
  3. follow_node         : /owner_pose → /cmd_vel

D435i 자체는 자칼(ROS1)에서 실행되고, ros1_bridge로 토픽이 넘어옴:
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /camera/color/camera_info
  /cmd_vel  (역방향: ROS2 → ROS1)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    follow_params = os.path.join(
        get_package_share_directory('ros2_follow_node'),
        'config', 'follow_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_yolo_d435i',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'conf_threshold': 0.4,
                'imgsz': 640,
                'person_only': True,
            }],
        ),
        Node(
            package='ros2_tracking_node',
            executable='tracking_node',
            name='tracking_node',
            output='screen',
            parameters=[{
                'frame_rate': 30,
                'image_width': 640,
                'image_height': 480,
                # 0508 수정: max_lost_frames 제거.
                # OwnerTracker 동작은 grace_frames + max_reassign_dist_px로 제어.
                'grace_frames': 10,
                'max_reassign_dist_px': 300.0,
            }],
        ),
        Node(
            package='ros2_follow_node',
            executable='follow_node',
            name='follow_node',
            output='screen',
            parameters=[follow_params],
        ),
    ])
