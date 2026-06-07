#!/usr/bin/env python3
"""가짜 정지 odom + 상단yaw 상태 발행 (펌웨어 없이 FOLLOW 검증용).

control_node 는 /odom 이 신선해야(odom_timeout=0.5s) FOLLOW 가 engage 된다.
실차 펌웨어가 없을 때, 로봇이 원점에 가만히 있다고 속여서
상단yaw 락온(azimuth 추종)만 따로 확인하기 위한 도구.

발행:
  /odom          (nav_msgs/Odometry)  원점·정지·단위쿼터니언, 20Hz
  /top_yaw_state (std_msgs/Float32)   0.0 (상단yaw 현재각=0)

★주의: 진짜 주행 테스트에는 쓰지 말 것. 몸체가 절대 안 움직이는 것으로
  추정되므로 FOLLOW 의 몸체 vx/vy 는 "원점 기준 목표로 가려는 값"이 나온다.
  상단yaw 검증 전용.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class FakeOdom(Node):
    def __init__(self):
        super().__init__("fake_odom")
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.yaw = self.create_publisher(Float32, "/top_yaw_state", 10)
        self.create_timer(0.05, self.tick)   # 20Hz
        self.get_logger().info("fake_odom: 원점 정지 odom + top_yaw_state=0 발행 (20Hz)")

    def tick(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.orientation.w = 1.0   # 단위 쿼터니언 (yaw=0)
        # position/twist 전부 0 (기본값)
        self.odom.publish(o)
        self.yaw.publish(Float32(data=0.0))


def main():
    rclpy.init()
    try:
        rclpy.spin(FakeOdom())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
