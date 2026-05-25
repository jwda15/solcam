#!/usr/bin/env python3
"""
viz_topdown.py
==============
주인 위치를 위에서 내려다본 (top-down) 2D 그림으로 시각화.

카메라 좌표계 → 평면도:
  x (m): 좌우 (양수=오른쪽)
  z (m): 전방 거리 (양수=앞)
  y는 무시 (높이는 평면도에 안 그림)

화면 구성:
  - 카메라 위치: 화면 하단 중앙 (원점)
  - 카메라 시야각 가이드 라인 (60° 가정, FOV)
  - 거리 링 (1m, 2m, 3m, 4m, 5m)
  - 주인 위치: 색깔 점 (is_detected=true면 빨강, KF 외삽이면 주황)
  - 최근 N개 위치 trail (옅은 색)
  - 좌상단 HUD: distance, azimuth, confidence

publish: /viz/topdown  → rqt_image_view로 보면 됨

빌드 필요 없음. 그냥:
  python3 viz_topdown.py
"""
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ros2_tracking_node.msg import OwnerPose


# --------------------------------------------------------
# 그림 파라미터
# --------------------------------------------------------
IMG_W = 600          # 가로 픽셀
IMG_H = 600          # 세로 픽셀
MAX_RANGE_M = 5.0    # 화면 위쪽 끝까지의 최대 거리 (m)
MARGIN_PX = 40       # 위/좌우 여백
CAM_Y_PX = IMG_H - 50  # 카메라 아이콘 y 픽셀

# 색깔 (BGR)
BG_COLOR        = (30, 30, 30)
GRID_COLOR      = (60, 60, 60)
RING_COLOR      = (80, 80, 80)
RING_TEXT       = (120, 120, 120)
FOV_COLOR       = (50, 50, 80)
CAM_COLOR       = (200, 200, 200)
OWNER_COLOR     = (60, 60, 240)    # 빨강 (정상 detect)
OWNER_KF_COLOR  = (40, 165, 255)   # 주황 (KF 외삽, confidence 낮음)
TRAIL_COLOR     = (180, 180, 240)
HUD_COLOR       = (220, 220, 220)
HUD_BG          = (20, 20, 20)

# trail 길이
TRAIL_LEN = 30


def world_to_px(x_m: float, z_m: float):
    """
    카메라 좌표(m) → 화면 픽셀.
    카메라는 화면 하단 중앙 (cx_px, CAM_Y_PX).
    z_m=0 → CAM_Y_PX,  z_m=MAX_RANGE_M → MARGIN_PX (위쪽)
    x_m=0 → cx_px,     |x_m|=MAX_RANGE_M → 좌/우 끝
    """
    cx_px = IMG_W // 2
    usable_h = CAM_Y_PX - MARGIN_PX
    usable_w = (IMG_W - 2 * MARGIN_PX) // 2

    px = int(cx_px + (x_m / MAX_RANGE_M) * usable_w)
    py = int(CAM_Y_PX - (z_m / MAX_RANGE_M) * usable_h)
    return px, py


def draw_static(img):
    """배경, 거리 링, FOV 가이드, 카메라 아이콘 등 매 프레임 같은 것."""
    img[:] = BG_COLOR

    # 거리 링 (반원, 카메라에서 1, 2, 3, 4, 5m)
    cx_px = IMG_W // 2
    usable_h = CAM_Y_PX - MARGIN_PX
    for r_m in range(1, int(MAX_RANGE_M) + 1):
        r_px = int((r_m / MAX_RANGE_M) * usable_h)
        cv2.ellipse(img, (cx_px, CAM_Y_PX), (r_px, r_px),
                    0, 180, 360, RING_COLOR, 1)
        cv2.putText(img, f'{r_m}m',
                    (cx_px + r_px - 25, CAM_Y_PX - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, RING_TEXT, 1)

    # FOV 가이드 (D435i 색카메라 H-FOV ≈ 69°, ±34.5°)
    # 평면도에선 좌우 각도만 표시
    fov_half = math.radians(69.0 / 2)
    far_x = math.tan(fov_half) * MAX_RANGE_M
    p_left  = world_to_px(-far_x, MAX_RANGE_M)
    p_right = world_to_px( far_x, MAX_RANGE_M)
    p_cam   = (cx_px, CAM_Y_PX)
    cv2.line(img, p_cam, p_left,  FOV_COLOR, 1)
    cv2.line(img, p_cam, p_right, FOV_COLOR, 1)

    # 중앙 세로선 (정면 가이드)
    cv2.line(img, p_cam, world_to_px(0, MAX_RANGE_M), GRID_COLOR, 1)

    # 카메라 아이콘 (작은 삼각형, 위 방향)
    pts = np.array([
        [cx_px,      CAM_Y_PX - 12],
        [cx_px - 10, CAM_Y_PX + 8],
        [cx_px + 10, CAM_Y_PX + 8],
    ], np.int32)
    cv2.fillPoly(img, [pts], CAM_COLOR)
    cv2.putText(img, 'CAM', (cx_px - 18, CAM_Y_PX + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, CAM_COLOR, 1)


def draw_hud(img, owner: OwnerPose, is_kf):
    """좌상단 HUD."""
    lines = []
    if owner is None:
        lines.append('owner: NO MSG')
    elif not owner.is_detected:
        lines.append('owner: LOST')
        lines.append(f'last id: {owner.track_id}')
    else:
        tag = 'KF' if is_kf else 'DET'
        lines.append(f'owner: {tag}  id={owner.track_id}')
        lines.append(f'dist:  {owner.distance:.2f} m')
        lines.append(f'azim:  {math.degrees(owner.azimuth):+.1f} deg')
        lines.append(f'x,z:   ({owner.spatial_x:+.2f}, {owner.spatial_z:.2f})')
        lines.append(f'conf:  {owner.confidence:.2f}')

    # 배경 박스
    box_w, box_h = 230, 18 + 18 * len(lines)
    cv2.rectangle(img, (8, 8), (8 + box_w, 8 + box_h), HUD_BG, -1)
    cv2.rectangle(img, (8, 8), (8 + box_w, 8 + box_h), HUD_COLOR, 1)

    for i, t in enumerate(lines):
        cv2.putText(img, t, (16, 28 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, HUD_COLOR, 1)


class VizTopdown(Node):
    def __init__(self):
        super().__init__('viz_topdown')
        self.bridge = CvBridge()

        self.last_owner = None
        self.trail = deque(maxlen=TRAIL_LEN)   # [(x_m, z_m, is_kf), ...]

        self.create_subscription(OwnerPose, '/owner_pose',
                                 self.cb_owner, 10)

        self.pub = self.create_publisher(Image, '/viz/topdown', 5)

        # 30Hz 주기로 그림 publish (owner_pose 안 와도 화면 유지)
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

        self.get_logger().info('VizTopdown 시작. /viz/topdown publish.')

    def cb_owner(self, msg: OwnerPose):
        self.last_owner = msg
        if msg.is_detected:
            # confidence < 0.5 * 원본은 KF 외삽으로 간주 (tracking_node에서 0.5×)
            # 정확한 구분이 필요하면 OwnerPose에 from_kf 필드 추가하는 게 깔끔.
            # 지금은 confidence < 0.4를 KF로 추정 (yolo conf_threshold=0.4 이상이 detect)
            is_kf = msg.confidence < 0.4
            self.trail.append((msg.spatial_x, msg.spatial_z, is_kf))

    def tick(self):
        img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        draw_static(img)

        # trail 그리기 (오래된 것일수록 옅게)
        n = len(self.trail)
        for i, (x_m, z_m, is_kf) in enumerate(self.trail):
            alpha = (i + 1) / max(n, 1)   # 0 ~ 1
            base = OWNER_KF_COLOR if is_kf else TRAIL_COLOR
            color = tuple(int(c * alpha + BG_COLOR[k] * (1 - alpha))
                          for k, c in enumerate(base))
            px, py = world_to_px(x_m, z_m)
            cv2.circle(img, (px, py), 3, color, -1)

        # 현재 위치
        is_kf_now = False
        if self.last_owner is not None and self.last_owner.is_detected:
            x_m, z_m = self.last_owner.spatial_x, self.last_owner.spatial_z
            is_kf_now = self.last_owner.confidence < 0.4
            color = OWNER_KF_COLOR if is_kf_now else OWNER_COLOR
            px, py = world_to_px(x_m, z_m)

            # 큰 원 + 외곽선
            cv2.circle(img, (px, py), 10, color, -1)
            cv2.circle(img, (px, py), 12, (255, 255, 255), 1)

            # 카메라에서 주인까지 라인
            cv2.line(img, (IMG_W // 2, CAM_Y_PX), (px, py), color, 1)

        draw_hud(img, self.last_owner, is_kf_now)

        # [0525] 창으로 직접 표시 (rqt 없이 바로 보기). q/ESC 종료.
        cv2.imshow('Owner Top-Down (2D map)', img)
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            rclpy.shutdown()

        # /viz/topdown 으로도 계속 publish (rqt나 녹화용)
        try:
            out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'topdown'
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'publish 실패: {e}')


def main():
    rclpy.init()
    node = VizTopdown()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
