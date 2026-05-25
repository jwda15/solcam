#!/usr/bin/env python3
"""
viz_overlay.py
==============
실험용 시각화. RGB 영상 위에:
- 모든 YOLO detection bbox (회색)
- 주인 위치 마커 (빨간 원, OwnerPose.spatial을 픽셀로 역투영)
- 좌상단: 주인 track_id, distance(m), azimuth(deg), is_detected

publish: /viz/image  → rqt_image_view로 보면 됨

빌드 필요 없음. 그냥:
  python3 viz_overlay.py
"""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ros2_tracking_node.msg import DetectionArray, OwnerPose


class VizOverlay(Node):
    def __init__(self):
        super().__init__('viz_overlay')
        self.bridge = CvBridge()

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        # 인트린식: 한 번 받아서 캐시
        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(CameraInfo,
                                 '/camera/color/camera_info',
                                 self.cb_info, qos)

        # 주인 정보: 가장 최근 것 캐시
        self.last_owner = None
        self.create_subscription(OwnerPose, '/owner_pose',
                                 self.cb_owner, 10)

        # RGB + detections 동기화
        # detections는 yolo가 변동 rate(17~28Hz)로 publish해서 sync 깨지기 쉬움.
        # queue_size 크게, slop 넉넉히. detection의 QoS도 BEST_EFFORT로 맞춤.
        self.sub_rgb = Subscriber(self, Image,
                                  '/camera/color/image_raw',
                                  qos_profile=qos)
        self.sub_det = Subscriber(self, DetectionArray, '/detections',
                                  qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_det], queue_size=30, slop=0.3)
        self.sync.registerCallback(self.cb_sync)

        self.pub = self.create_publisher(Image, '/viz/image', 5)

        # 통계
        self.frame_count = 0
        self.owner_track_ids = []   # 마지막 N개 track_id (재매핑 횟수 추적)
        self.lost_count = 0

        self.get_logger().info('VizOverlay 시작. /viz/image publish.')

    def cb_info(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]; self.fy = msg.k[4]
            self.cx = msg.k[2]; self.cy = msg.k[5]
            self.get_logger().info(
                f'intrinsic 받음: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def cb_owner(self, msg: OwnerPose):
        self.last_owner = msg
        if msg.is_detected:
            tid = int(msg.track_id)
            if not self.owner_track_ids or self.owner_track_ids[-1] != tid:
                self.owner_track_ids.append(tid)
                if len(self.owner_track_ids) > 50:
                    self.owner_track_ids.pop(0)
        else:
            self.lost_count += 1

    def cb_sync(self, rgb_msg: Image, det_msg: DetectionArray):
        self.frame_count += 1
        try:
            img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 실패: {e}')
            return
        H, W = img.shape[:2]

        # 모든 detection bbox (회색)
        for d in det_msg.detections:
            x1, y1 = int(d.x), int(d.y)
            x2, y2 = int(d.x + d.w), int(d.y + d.h)
            cv2.rectangle(img, (x1, y1), (x2, y2), (180, 180, 180), 1)
            cv2.putText(img, f'{d.score:.2f} d={d.depth/1000:.2f}m',
                        (x1, max(15, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        # 주인 마커 (spatial_x/y/z → 픽셀 역투영)
        owner = self.last_owner
        if (owner is not None and owner.is_detected and
                self.fx is not None and owner.spatial_z > 0):
            u = int(self.fx * owner.spatial_x / owner.spatial_z + self.cx)
            v = int(self.fy * owner.spatial_y / owner.spatial_z + self.cy)
            if 0 <= u < W and 0 <= v < H:
                cv2.circle(img, (u, v), 14, (0, 0, 255), 2)
                cv2.circle(img, (u, v), 3, (0, 0, 255), -1)
                cv2.putText(img, f'OWNER#{int(owner.track_id)}',
                            (u + 18, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 2)

        # 좌상단 HUD
        if owner is not None:
            status = 'OK' if owner.is_detected else 'LOST'
            color = (0, 255, 0) if owner.is_detected else (0, 165, 255)
            azi_deg = math.degrees(owner.azimuth)
            n_unique = len(set(self.owner_track_ids))
            n_remaps = max(0, len(self.owner_track_ids) - 1)
            lines = [
                f'state : {status}',
                f'track : id={int(owner.track_id)}  unique={n_unique}  remaps={n_remaps}',
                f'pose  : d={owner.distance:.2f}m  azi={azi_deg:+6.1f}deg',
                f'spat  : x={owner.spatial_x:+.2f} y={owner.spatial_y:+.2f} z={owner.spatial_z:+.2f}',
                f'frame : {self.frame_count}  lost_evts={self.lost_count}',
            ]
            for i, t in enumerate(lines):
                cv2.putText(img, t, (10, 22 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, t, (10, 22 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            color, 1, cv2.LINE_AA)

        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = rgb_msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VizOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
