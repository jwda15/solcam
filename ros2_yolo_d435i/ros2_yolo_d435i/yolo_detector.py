"""
yolo_detector.py
================
D435i RGB → YOLOv8 추론 → bbox 영역 depth 중앙값 → /detections publish

입력 토픽:
  /camera/color/image_raw       (sensor_msgs/Image, BGR8)
  /camera/aligned_depth_to_color/image_raw  (sensor_msgs/Image, 16UC1, mm)
  ※ realsense2_camera launch에 align_depth=true 필수

출력 토픽:
  /detections   (ros2_tracking_node/DetectionArray)

YOLO:
  ultralytics YOLOv8n (사람 클래스만, COCO id 0)
  CPU에서도 30fps 정도는 나오고, GPU 있으면 더 빨라
  처음 실행 시 yolov8n.pt 자동 다운로드 (~6MB)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import cv2

from ros2_tracking_node.msg import Detection, DetectionArray


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # 파라미터
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('person_only', True)
        self.declare_parameter('depth_patch_ratio', 0.15)  # bbox 중앙 15% (회전/옆모습에서 배경 침투 더 차단)
        self.declare_parameter('depth_min_mm', 200.0)      # D435i min range
        self.declare_parameter('depth_max_mm', 6000.0)     # 6m
        self.declare_parameter('depth_near_percentile', 15.0)
        # ↑ patch에서 valid depth 분포 중 "가까운 N%"만 골라서 median.
        # 사람이 회전할 때 patch에 배경이 끼어드는 현상 더 강하게 거름.
        # 15%는 가장 가까운 15%의 median = 사람 몸통 거리에 잘 수렴.
        # 너무 작으면 patch에 valid 픽셀 부족 시 0 반환 → tracking_node에서 KF z 사용.

        model_path = self.get_parameter('model_path').value
        self.conf  = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.person_only = bool(self.get_parameter('person_only').value)
        self.depth_patch_ratio = float(self.get_parameter('depth_patch_ratio').value)
        self.depth_min = float(self.get_parameter('depth_min_mm').value)
        self.depth_max = float(self.get_parameter('depth_max_mm').value)
        self.depth_near_percentile = float(
            self.get_parameter('depth_near_percentile').value)

        # YOLO 로드
        from ultralytics import YOLO
        import torch
        self.get_logger().info(f'YOLO 모델 로드: {model_path}')
        self.model = YOLO(model_path)
        # GPU 강제 (가능하면)
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.device = 'cuda'
            self.get_logger().info('YOLO device: cuda')
        else:
            self.device = 'cpu'
            self.get_logger().info('YOLO device: cpu (CUDA 없음)')

        # 토픽 구독: RGB + Depth 동기화
        self.bridge = CvBridge()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_rgb   = Subscriber(self, Image, '/camera/color/image_raw',
                                    qos_profile=qos)
        self.sub_depth = Subscriber(self, Image,
                                    '/camera/aligned_depth_to_color/image_raw',
                                    qos_profile=qos)
        # slop 0.05s = 50ms. realsense는 보통 잘 맞음
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)

        self.pub = self.create_publisher(DetectionArray, '/detections', 10)

        self.frame_count = 0
        self.get_logger().info('YoloDetector 준비 완료. RGB+Depth 대기 중...')

    def callback(self, rgb_msg: Image, depth_msg: Image):
        try:
            rgb   = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        # YOLO 추론 (GPU 강제 — 매 호출마다 명시 안 하면 가끔 cpu로 떨어짐)
        results = self.model.predict(rgb,
                                     imgsz=self.imgsz,
                                     conf=self.conf,
                                     device=self.device,
                                     verbose=False)

        msg = DetectionArray()
        msg.header = rgb_msg.header
        if not msg.header.frame_id:
            msg.header.frame_id = 'camera_color_optical_frame'

        H, W = depth.shape[:2]

        for r in results:
            if r.boxes is None:
                continue
            xyxy = r.boxes.xyxy.cpu().numpy()
            conf = r.boxes.conf.cpu().numpy()
            cls  = r.boxes.cls.cpu().numpy().astype(int)

            for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
                if self.person_only and k != 0:
                    continue
                x1 = max(0.0, float(x1)); y1 = max(0.0, float(y1))
                x2 = min(float(W - 1), float(x2)); y2 = min(float(H - 1), float(y2))
                w = x2 - x1; h = y2 - y1
                if w <= 1 or h <= 1:
                    continue

                # bbox 중앙 patch에서 depth 중앙값
                cx = (x1 + x2) * 0.5; cy = (y1 + y2) * 0.5
                pw = w * self.depth_patch_ratio * 0.5
                ph = h * self.depth_patch_ratio * 0.5
                px1 = int(max(0, cx - pw)); py1 = int(max(0, cy - ph))
                px2 = int(min(W, cx + pw)); py2 = int(min(H, cy + ph))
                patch = depth[py1:py2, px1:px2]
                if patch.size == 0:
                    continue

                # invalid (0) 제거 + 범위 필터
                valid = patch[(patch > self.depth_min) & (patch < self.depth_max)]
                if valid.size == 0:
                    depth_mm = 0.0
                else:
                    # 가까운 N% (depth 작은 쪽 = 사람 몸)만 골라서 median.
                    # 배경/벽이 patch에 끼어도 "사람이 더 가깝다"는 가정으로 reject.
                    p = self.depth_near_percentile
                    if p >= 100.0 or valid.size < 5:
                        depth_mm = float(np.median(valid))
                    else:
                        threshold = np.percentile(valid, p)
                        near = valid[valid <= threshold]
                        depth_mm = float(np.median(near)) if near.size > 0 \
                                   else float(np.median(valid))

                d = Detection()
                d.x = float(x1); d.y = float(y1)
                d.w = float(w);  d.h = float(h)
                d.depth = depth_mm
                d.score = float(c)
                d.label = int(k)
                msg.detections.append(d)

        self.pub.publish(msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'프레임 {self.frame_count}, det={len(msg.detections)}')


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
