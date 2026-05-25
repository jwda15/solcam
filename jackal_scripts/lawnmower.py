#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
lawnmower.py
============
자칼 잔디깎이 패턴 자율주행 (저텍스처 시퀀스 수집용)

패턴:
  ┌─→ STRIPE_LEN m 직진
  │       └ 90° 회전 (ROW_DIR, IMU 피드백)
  │             └ ROW_GAP m 직진
  │                   └ 90° 회전 (반대)
  └─────  STRIPE_LEN m 직진 (반대 방향) ... 반복

90도 회전을 시간 대신 /odometry/filtered yaw 누적으로 닫음.
→ 매 코너 정확히 90도 → 줄 간격이 어긋나지 않음.

NUM_STRIPES 짝수면 평행이동 결과물이라 종점은 시작점에서
  (가로) ROW_GAP*(NUM_STRIPES-1) 만큼 옆에 있음.
NUM_STRIPES 홀수면 (세로) STRIPE_LEN 만큼 떨어진 곳.
GT 종점은 사람이 줄자로 잴 수 있음.
"""

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from yaw_helpers import YawTracker

# ============== 수정 포인트 ==============
STRIPE_LEN  = 4.0       # 한 줄 길이 (m)
ROW_GAP     = 0.8       # 줄 간격 (m)
NUM_STRIPES = 4         # 총 몇 줄 (왕복 1쌍 = 2줄)
LIN_SPEED   = 0.25      # 직진 속도 (m/s)
ANG_SPEED   = 0.5       # 회전 속도 (rad/s)
ROW_DIR     = +1        # 첫 회전 방향: +1=좌(반시계), -1=우(시계)
RATE_HZ     = 20

TURN_TOL   = math.radians(1.5)
TURN_SLOW  = math.radians(15.0)
# =========================================


class Lawnmower(object):
    def __init__(self):
        rospy.init_node('jackal_lawnmower')
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
        if self.stopped: return
        sign = 1.0 if distance >= 0 else -1.0
        dist = abs(distance)
        duration = dist / LIN_SPEED
        rospy.loginfo('  직진 %.2f m (%.2fs)', sign*dist, duration)
        twist = Twist()
        twist.linear.x = sign * LIN_SPEED
        end = rospy.Time.now() + rospy.Duration.from_sec(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            if self.stopped: break
            self.pub.publish(twist)
            self.rate.sleep()
        self.publish_stop_brief()

    def rotate_to(self, target_rad):
        """yaw 피드백 회전. 양수=좌회전."""
        if self.stopped: return
        self.yt.reset()
        rospy.sleep(0.1)
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

            done   = abs(self.yt.delta())
            remain = target - done
            if remain <= TURN_TOL:
                break

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
        rospy.loginfo('=== 잔디깎이 (yaw 피드백) ===')
        rospy.loginfo('STRIPE_LEN=%.1fm, ROW_GAP=%.1fm, NUM_STRIPES=%d',
                      STRIPE_LEN, ROW_GAP, NUM_STRIPES)

        if not self.yt.wait_until_ready(timeout=5.0):
            rospy.logerr('odom 토픽 안 들어옴. /odometry/filtered 확인.')
            return
        rospy.sleep(2.0)

        turn_dir = ROW_DIR
        for i in range(NUM_STRIPES):
            if self.stopped or rospy.is_shutdown(): break
            rospy.loginfo('--- stripe %d / %d ---', i+1, NUM_STRIPES)

            self.go_straight(STRIPE_LEN)

            if i == NUM_STRIPES - 1:
                break

            self.rotate_to(turn_dir * math.pi / 2.0)
            self.go_straight(ROW_GAP)
            self.rotate_to(turn_dir * math.pi / 2.0)

            turn_dir *= -1

        rospy.loginfo('=== 잔디깎이 종료 ===')
        self.shutdown_hook()


if __name__ == '__main__':
    try:
        Lawnmower().run()
    except rospy.ROSInterruptException:
        pass
