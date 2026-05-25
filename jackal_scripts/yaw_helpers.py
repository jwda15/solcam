#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yaw_helpers.py
==============
자칼 /odometry/filtered 의 yaw를 누적 회전량으로 추적하는 헬퍼.

핵심:
  자칼 odom의 yaw는 -π ~ +π로 wrap됨. 360도 회전 시 누적값이 그대로 안 늘어남.
  → 이전 yaw와의 차이를 wrap 처리해서 누적 적분.

ROS Melodic / Python2 호환.

사용:
  from yaw_helpers import YawTracker
  yt = YawTracker()
  yt.reset()                    # 현재를 0으로
  ... 회전 명령 publish ...
  while abs(yt.delta()) < math.pi/2:   # 90도까지
      pub.publish(twist)
      rate.sleep()
"""

import math
import rospy
from nav_msgs.msg import Odometry


def quat_to_yaw(qx, qy, qz, qw):
    """쿼터니언 → yaw (rad). roll/pitch는 무시."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a):
    """각도를 -π ~ +π로."""
    while a > math.pi:  a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


class YawTracker(object):
    """
    /odometry/filtered 구독해서 누적 yaw (절대 적분값) 제공.

    delta(): reset() 이후 누적 회전량 (rad). 양수=좌회전(반시계).
    """
    def __init__(self, topic='/odometry/filtered'):
        self._sub = rospy.Subscriber(topic, Odometry, self._cb, queue_size=10)
        self._last_yaw = None        # 직전 콜백 yaw
        self._cumulative = 0.0       # 누적 회전량
        self._reset_offset = 0.0     # reset 시점의 누적값
        self._got_data = False

    def _cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        if self._last_yaw is not None:
            d = wrap_to_pi(yaw - self._last_yaw)
            self._cumulative += d
        self._last_yaw = yaw
        self._got_data = True

    def wait_until_ready(self, timeout=5.0):
        """odom 첫 메시지 도착까지 대기."""
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self._got_data:
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn('YawTracker: %s 타임아웃' % self._sub.resolved_name)
                return False
            rate.sleep()
        return True

    def reset(self):
        """현재 값을 0으로."""
        self._reset_offset = self._cumulative

    def delta(self):
        """reset() 이후 누적 회전량 (rad)."""
        return self._cumulative - self._reset_offset
