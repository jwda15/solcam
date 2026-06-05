# ============================================================================
#  control.launch.py  —  control_node 단독 실행 런치
#
#  사용:
#    ros2 launch ros2_control_node control.launch.py
#    ros2 launch ros2_control_node control.launch.py mode:=1   # 바로 FOLLOW
#
#  config/control_params.yaml 의 파라미터를 로드한다.
# ============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_control_node')
    params_file = os.path.join(pkg_share, 'config', 'control_params.yaml')

    # 시작 모드를 런치 인자로 덮어쓸 수 있게(테스트 편의)
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='0',
        description='시작 주행 모드 (0=IDLE, 1=FOLLOW)'
    )

    control_node = Node(
        package='ros2_control_node',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[
            params_file,
            {'mode': ParameterValue(LaunchConfiguration('mode'), value_type=int)},
        ],
    )

    return LaunchDescription([
        mode_arg,
        control_node,
    ])
