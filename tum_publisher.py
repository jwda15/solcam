#!/usr/bin/env python3
"""
tum_publisher.py
================
TUM-format dataset (associations.txt + rgb/ + depth/) → ROS2 토픽 publisher.
ttest2_ros2 bag의 metadata 파싱 에러 우회용. 깊이 파지 말자고 했으니.

publish 토픽:
  /camera/color/image_raw                       sensor_msgs/Image (BGR8)
  /camera/aligned_depth_to_color/image_raw      sensor_msgs/Image (16UC1, mm)
  /camera/color/camera_info                     sensor_msgs/CameraInfo

QoS: BEST_EFFORT (RealSense 호환 — yolo_detector / tracking_node H1 모두 BEST_EFFORT 받음)

사용:
  python3 tum_publisher.py \
      --tum-dir "/media/jw/로컬 디스크/datasets/ttest2_tum" \
      --rate 30.0 --loop

ROS2 파라미터로도 받음:
  fx fy cx cy width height frame_id
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def load_associations(path):
    """associations.txt → [(rgb_ts, rgb_path, depth_ts, depth_path), ...]"""
    pairs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            rgb_ts, rgb_p, depth_ts, depth_p = parts[0], parts[1], parts[2], parts[3]
            pairs.append((float(rgb_ts), rgb_p, float(depth_ts), depth_p))
    return pairs


def make_camera_info(fx, fy, cx, cy, width, height, frame_id, stamp):
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


class TumPublisher(Node):
    def __init__(self, args):
        super().__init__('tum_publisher')

        # 인트린식 (D435i 색 스트림 640x480 기본값)
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.bridge = CvBridge()
        self.tum_dir = args.tum_dir
        self.rate = args.rate
        self.loop_dataset = args.loop
        self.depth_to_mm = args.depth_to_mm

        self.pairs = load_associations(os.path.join(self.tum_dir, 'associations.txt'))
        if not self.pairs:
            self.get_logger().error(f'associations.txt 비어있음: {self.tum_dir}')
            raise SystemExit(1)
        self.get_logger().info(f'TUM 페어 {len(self.pairs)}개 로드')

        # 외장 HDD I/O가 보틀넥이라 시작할 때 모든 프레임을 메모리에 프리로드.
        # 893 × (640×480×3 + 640×480×2) ≈ 1.3GB 정도라 충분히 RAM에 들어감.
        self.get_logger().info('이미지 프리로드 중... (외장 디스크 I/O 우회)')
        self.cache = []   # [(rgb_ndarray, depth_ndarray), ...]
        for i, (_, rgb_rel, _, depth_rel) in enumerate(self.pairs):
            rgb_path = os.path.join(self.tum_dir, rgb_rel)
            depth_path = os.path.join(self.tum_dir, depth_rel)
            rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
            depth_raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if rgb is None or depth_raw is None:
                self.get_logger().warn(f'읽기 실패: {rgb_path} / {depth_path}')
                continue
            # depth scale 변환은 미리 끝내둠
            if self.depth_to_mm == 1.0:
                depth = depth_raw
            else:
                d = depth_raw.astype(np.float32) * self.depth_to_mm
                d = np.clip(d, 0.0, 65535.0)
                depth = d.astype(np.uint16)
            self.cache.append((rgb, depth))
            if (i + 1) % 100 == 0:
                self.get_logger().info(f'  프리로드 [{i+1}/{len(self.pairs)}]')
        self.get_logger().info(f'프리로드 완료: {len(self.cache)} 프레임')

        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub_rgb   = self.create_publisher(Image, '/camera/color/image_raw', qos)
        self.pub_depth = self.create_publisher(Image,
                                               '/camera/aligned_depth_to_color/image_raw',
                                               qos)
        self.pub_info  = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)

        self.idx = 0
        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'publish 시작: {self.rate:.1f} Hz, intrinsic fx={self.fx} fy={self.fy} '
            f'cx={self.cx} cy={self.cy} {self.width}x{self.height}'
        )

    def tick(self):
        if self.idx >= len(self.cache):
            if self.loop_dataset:
                self.idx = 0
                self.get_logger().info('데이터셋 끝 → 루프')
            else:
                self.get_logger().info('데이터셋 끝. 종료.')
                rclpy.shutdown()
                return

        rgb, depth = self.cache[self.idx]
        self.idx += 1

        # depth는 캐시 단계에서 이미 mm 스케일로 변환됨.

        stamp = self.get_clock().now().to_msg()

        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.frame_id

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.frame_id

        info_msg = make_camera_info(self.fx, self.fy, self.cx, self.cy,
                                    self.width, self.height,
                                    self.frame_id, stamp)

        self.pub_rgb.publish(rgb_msg)
        self.pub_depth.publish(depth_msg)
        self.pub_info.publish(info_msg)

        if self.idx % 30 == 0:
            self.get_logger().info(f'[{self.idx}/{len(self.pairs)}]')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tum-dir',
                        default='/media/jw/로컬 디스크/datasets/ttest2_tum')
    parser.add_argument('--rate', type=float, default=30.0)
    parser.add_argument('--loop', action='store_true',
                        help='데이터셋 끝나면 처음부터 다시 재생')
    parser.add_argument('--depth-to-mm', type=float, default=0.2,
                        help='저장된 depth 값 * 이 값 = mm. '
                             'TUM convention (5000 = 1m) → 0.2. '
                             '이미 mm로 저장돼있으면 1.0.')
    # ROS args 분리
    parser.add_argument('--ros-args', action='store_true', default=False)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = TumPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
