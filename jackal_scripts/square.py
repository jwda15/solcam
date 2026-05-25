#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
square.py
=========
자칼 사각형 주행 (저텍스처 시퀀스 + 폐곡선 GT 검증)

  start ─→ A m 직진
              └ 90° 회전 (TURN_DIR, IMU 피드백)
                    └ B m 직진
                          └ 90° 회전 (TURN_DIR)
                                └ A m 직진 ...

종점이 시작점과 같으므로:
  - 위치 GT: (0, 0) — 줄자로 잰 종점-시작점 차이가 GT 오차
  - 회전 GT: 0 rad — 자칼 yaw가 시작과 같아야 정답

90도 회전을 시간 기반 대신 /odometry/filtered yaw 누적으로 닫음.
→ 매 코너 정확히 90도 → 한 바퀴 끝나면 정확히 360도 복귀.
"""

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from yaw_helpers import YawTracker

# ============== 수정 포인트 ==============
SIDE_A    = 3.0     # 가로 (m)
SIDE_B    = 3.0     # 세로 (m)
NUM_LAPS  = 1       # 몇 바퀴 (1 권장 — 종점 GT 측정용)
LIN_SPEED = 0.25    # m/s
ANG_SPEED = 0.5     # rad/s
TURN_DIR  = +1      # +1=좌회전(반시계), -1=우회전(시계)
RATE_HZ   = 20

# 회전 정확도 임계값
TURN_TOL   = math.radians(1.5)  # 목표각 도달 판정 (1.5도)
TURN_SLOW  = math.radians(15.0) # 이 각도 남으면 속도 절반 (오버슈트 방지)
# =========================================


class Square(object):
    def __init__(self):
        rospy.init_node('jackal_square')
        self.pub      = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.stop_sub = rospy.Subscriber('/jackaljandi/stop', Bool,
                                         self.stop_cb, queue_size=1)
        self.rate     = rospy.Rate(RATE_HZ)
        self.stopped  = False
        self.yt       = YawTracker()
        rospy.on_shutdown(self.shutdown_hook)

    def stop_cb(self, msg):
        if msg.data:
            rospy.logwarn('Stop 토픽 수신. 즉시 정지 명령 publish.')
            self.stopped = True
            # 즉시 정지 명령 5번 보내서 자칼 cmd_vel timeout 전에 멈추게
            for _ in range(5):
                self.pub.publish(Twist())
                rospy.sleep(0.02)

    def go_straight(self, distance):
        """distance(m) 시간 기반 직진."""
        if self.stopped: return
        duration = abs(distance) / LIN_SPEED
        sign = 1.0 if distance >= 0 else -1.0
        rospy.loginfo('  직진 %.2f m (%.2fs)', sign*abs(distance), duration)
        twist = Twist()
        twist.linear.x = sign * LIN_SPEED
        end = rospy.Time.now() + rospy.Duration.from_sec(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if self.stopped: break
            self.pub.publish(twist)
            self.rate.sleep()
        self.publish_stop_brief()

    def rotate_to(self, target_rad):
        """yaw 피드백으로 정확히 target_rad만큼 회전. 양수=좌회전."""
        if self.stopped: return
        self.yt.reset()
        rospy.sleep(0.1)  # reset 후 콜백 한두 번 들어오게
        sign = 1.0 if target_rad >= 0 else -1.0
        target = abs(target_rad)
        rospy.loginfo('  회전 %.1f deg (yaw 피드백)', math.degrees(sign*target))

        twist = Twist()
        timeout = rospy.Time.now() + rospy.Duration.from_sec(target / ANG_SPEED * 3.0 + 2.0)

        while not rospy.is_shutdown():
            if self.stopped: break
            if rospy.Time.now() > timeout:
                rospy.logwarn('  회전 타임아웃!')
                break

            done = abs(self.yt.delta())   # 누적 회전량 절대값
            remain = target - done
            if remain <= TURN_TOL:
                break

            # 목표 가까우면 속도 줄여서 오버슈트 방지
            speed = ANG_SPEED if remain > TURN_SLOW else ANG_SPEED * 0.4
            twist.angular.z = sign * speed
            self.pub.publish(twist)
            self.rate.sleep()

        rospy.loginfo('    → 실제 회전: %.2f deg', math.degrees(self.yt.delta()))
        self.publish_stop_brief()

    def publish_stop_brief(self):
        twist = Twist()
        for _ in range(int(RATE_HZ * 0.3)):
            self.pub.publish(twist)
            self.rate.sleep()

    def shutdown_hook(self):
        rospy.loginfo('shutdown: 정지 명령 publish')
        for _ in range(5):
            self.pub.publish(Twist())
            rospy.sleep(0.05)

    def run(self):
        rospy.loginfo('=== 사각형 주행 (yaw 피드백) ===')
        rospy.loginfo('SIDE_A=%.1f, SIDE_B=%.1f, NUM_LAPS=%d',
                      SIDE_A, SIDE_B, NUM_LAPS)

        if not self.yt.wait_until_ready(timeout=5.0):
            rospy.logerr('odom 토픽 안 들어옴. /odometry/filtered 확인.')
            return
        rospy.sleep(2.0)   # rosbag 시작 여유

        for lap in range(NUM_LAPS):
            if self.stopped or rospy.is_shutdown(): break
            rospy.loginfo('--- lap %d / %d ---', lap+1, NUM_LAPS)
            for side_len in (SIDE_A, SIDE_B, SIDE_A, SIDE_B):
                if self.stopped: break
                self.go_straight(side_len)
                self.rotate_to(TURN_DIR * math.pi / 2.0)

        rospy.loginfo('=== 종료 — 종점에서 줄자/각도기로 GT 측정 ===')
        self.shutdown_hook()


if __name__ == '__main__':
    try:
        Square().run()
    except rospy.ROSInterruptException:
        pass
