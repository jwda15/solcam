#!/usr/bin/env python3
"""가짜 owner_pose 발행 — 상단yaw(azimuth 추종) 검증 전용.

azimuth 를 0 → +0.30 → 0 → -0.30 → 0 (rad) 로 천천히 바꿔가며 20Hz 발행.
(+0.30rad ~ 주인이 오른쪽 약 17도, -0.30 ~ 왼쪽 17도)
distance=1.5m 고정. is_detected=True.

fake_odom.py(원점 정지 odom) + mode=FOLLOW 와 함께 쓰면,
control_node 의 trackTopYaw 가 top_yaw_target 을 azimuth 따라 움직이는지 확인 가능.
"""
import math
import rclpy
from rclpy.node import Node
from ros2_tracking_node.msg import OwnerPose

# (지속시간 s, azimuth rad) 시퀀스
PLAN = [(1.0, 0.0), (1.5, 0.30), (1.2, 0.0), (1.5, -0.30), (1.2, 0.0)]


class FakeOwner(Node):
    def __init__(self):
        super().__init__("fake_owner")
        self.pub = self.create_publisher(OwnerPose, "/owner_pose", 10)
        self.t = 0.0
        self.dt = 0.05
        self.create_timer(self.dt, self.tick)
        self.get_logger().info("fake_owner: azimuth 스윕 시작 (0→+0.3→0→-0.3→0)")

    def cur_az(self):
        acc = 0.0
        for dur, az in PLAN:
            if self.t < acc + dur:
                return az
            acc += dur
        return None  # 끝

    def tick(self):
        az = self.cur_az()
        if az is None:
            self.get_logger().info("fake_owner: 시퀀스 끝 — 종료")
            rclpy.shutdown(); return
        d = 1.5
        m = OwnerPose()
        m.header.stamp = self.get_clock().now().to_msg()
        m.is_detected = True
        m.spatial_z = float(d * math.cos(az))   # 전방 depth
        m.spatial_x = float(d * math.sin(az))   # 좌우 (+오른쪽)
        m.spatial_y = 0.0
        m.azimuth = float(az)
        m.distance = float(d)
        m.confidence = 0.9
        m.track_id = 1
        self.pub.publish(m)
        self.t += self.dt


def main():
    rclpy.init()
    try:
        rclpy.spin(FakeOwner())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
