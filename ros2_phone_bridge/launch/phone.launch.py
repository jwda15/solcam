"""phone_bridge 실행. 실기: scrcpy/v4l2 준비 후 mock:=false.
  ros2 launch ros2_phone_bridge phone.launch.py                 # 실기
  ros2 launch ros2_phone_bridge phone.launch.py mock:=true      # 폰 없이 점검
  ros2 launch ros2_phone_bridge phone.launch.py manage_scrcpy:=true  # scrcpy 자동
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("video_device", default_value="/dev/video2"),
        DeclareLaunchArgument("video_url", default_value=""),   # 폰 IP Webcam URL(설정 시 scrcpy 불필요)
        DeclareLaunchArgument("manage_scrcpy", default_value="false"),
        DeclareLaunchArgument("adb_serial", default_value=""),
    ]
    node = Node(
        package="ros2_phone_bridge",
        executable="phone_bridge",
        name="phone_bridge",
        output="screen",
        parameters=[{
            "mock": LaunchConfiguration("mock"),
            "video_device": LaunchConfiguration("video_device"),
            "video_url": LaunchConfiguration("video_url"),
            "manage_scrcpy": LaunchConfiguration("manage_scrcpy"),
            "adb_serial": LaunchConfiguration("adb_serial"),
        }],
    )
    return LaunchDescription(args + [node])
