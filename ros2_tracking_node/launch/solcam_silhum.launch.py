"""
solcam_silhum.launch.py
=======================
데이터셋(TUM 폴더) 기반 풀 파이프라인 검증용 launch.

띄우는 노드:
  1. tum_publisher    : TUM 폴더 → /camera/* 토픽
  2. yolo_detector    : RGB+Depth → /detections
  3. tracking_node    : /detections + /camera/color/camera_info → /owner_pose
                        (config/tracking_params.yaml 자동 적용)
  4. viz_overlay      : RGB 위에 detection bbox + 주인 마커 + HUD → /viz/image
  5. viz_topdown      : 주인 위치 평면도 → /viz/topdown

사용:
  ros2 launch ros2_tracking_node solcam_silhum.launch.py
  ros2 launch ros2_tracking_node solcam_silhum.launch.py rate:=15.0
  ros2 launch ros2_tracking_node solcam_silhum.launch.py loop:=false
  ros2 launch ros2_tracking_node solcam_silhum.launch.py tum_dir:=/path/to/other_tum
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction으로 LaunchConfiguration을 즉시 평가해서 cmd 인자로 사용."""
    tum_dir = LaunchConfiguration('tum_dir').perform(context)
    rate    = LaunchConfiguration('rate').perform(context)
    loop    = LaunchConfiguration('loop').perform(context).lower() == 'true'
    fx      = LaunchConfiguration('fx').perform(context)
    fy      = LaunchConfiguration('fy').perform(context)
    cx      = LaunchConfiguration('cx').perform(context)
    cy      = LaunchConfiguration('cy').perform(context)

    # 0525 수정: 절대경로 하드코딩 제거.
    # launch 파일 위치(<repo>/ros2_tracking_node/launch/)에서 repo 루트를 역산.
    # 심볼릭 링크 워크스페이스 빌드에서도 원본 위치를 정확히 가리키도록
    # os.path.realpath로 링크를 해제한 뒤 계산.
    solcam_dir = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__))))

    tracking_params = os.path.join(
        get_package_share_directory('ros2_tracking_node'),
        'config', 'tracking_params.yaml'
    )

    tum_cmd = ['python3', os.path.join(solcam_dir, 'tum_publisher.py'),
               '--tum-dir', tum_dir,
               '--rate',    rate]
    if loop:
        tum_cmd.append('--loop')
    # intrinsics는 ROS2 파라미터로 전달 (tum_publisher.py가 declare_parameter로 받음)
    tum_cmd += ['--ros-args',
                '-p', f'fx:={fx}',
                '-p', f'fy:={fy}',
                '-p', f'cx:={cx}',
                '-p', f'cy:={cy}']

    return [
        ExecuteProcess(cmd=tum_cmd, output='screen', name='tum_publisher_proc'),
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
            parameters=[tracking_params],
        ),
        ExecuteProcess(
            cmd=['python3', os.path.join(solcam_dir, 'viz_overlay.py')],
            output='screen', name='viz_overlay_proc'),
        ExecuteProcess(
            cmd=['python3', os.path.join(solcam_dir, 'viz_topdown.py')],
            output='screen', name='viz_topdown_proc'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tum_dir',
            default_value='/media/jw/로컬 디스크/datasets/rgbd_bonn_person_tracking',
            description='TUM-format 데이터셋 디렉토리'),
        DeclareLaunchArgument(
            'rate',
            default_value='30.0',
            description='TUM publisher 재생 속도 (Hz)'),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='true면 데이터셋 끝나면 처음부터 다시 재생'),
        # Kinect v1 (Bonn rgbd_bonn_*) 기본값. D435i 쓸 때는 600/600/320/240으로 override.
        DeclareLaunchArgument('fx', default_value='542.822841'),
        DeclareLaunchArgument('fy', default_value='542.576870'),
        DeclareLaunchArgument('cx', default_value='315.593520'),
        DeclareLaunchArgument('cy', default_value='237.756098'),
        OpaqueFunction(function=launch_setup),
    ])
