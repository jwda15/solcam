"""
oak_viz.py
==========
실시간 시각화 노드. OpenCV 창에 다음을 오버레이한다.
  - 모든 detection bbox (회색) + 거리(m)
  - 주인 추정 bbox (초록, 굵게) + ID
  - 우상단 정보 패널 (거리/방위각/z/conf/id)
  - is_detected=false면 빨간 테두리 + "OWNER LOST"

구독:
  /oak/rgb/image_raw   (sensor_msgs/Image, BGR8)   ← oak_detector publish
  /detections          (ros2_tracking_node/DetectionArray)
  /owner_pose          (ros2_tracking_node/OwnerPose)

viz는 추적 파이프라인과 독립. 헤드리스면 save_video:=true.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ros2_tracking_node.msg import DetectionArray, OwnerPose


class OakViz(Node):
    def __init__(self):
        super().__init__('oak_viz')

        self.declare_parameter('show_window', True)
        self.declare_parameter('save_video', False)
        self.declare_parameter('video_path', '/tmp/oak_tracking.avi')
        self.declare_parameter('video_fps', 30.0)

        self.show_window = bool(self.get_parameter('show_window').value)
        self.save_video  = bool(self.get_parameter('save_video').value)
        self.video_path  = self.get_parameter('video_path').value
        self.video_fps   = float(self.get_parameter('video_fps').value)

        self.bridge = CvBridge()
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_rgb = self.create_subscription(
            Image, '/oak/rgb/image_raw', self._rgb_cb, sensor_qos)
        self.sub_det = self.create_subscription(
            DetectionArray, '/detections', self._det_cb, 10)
        self.sub_owner = self.create_subscription(
            OwnerPose, '/owner_pose', self._owner_cb, 10)

        self.latest_dets = []
        self.latest_owner = None
        self.writer = None

        self.get_logger().info(
            f'OakViz 준비. show_window={self.show_window} save_video={self.save_video}')

    # ── 콜백 ───────────────────────────────────────────────
    def _det_cb(self, msg: DetectionArray):
        self.latest_dets = list(msg.detections)

    def _owner_cb(self, msg: OwnerPose):
        self.latest_owner = msg

    def _rgb_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        self._draw(frame)

        if self.save_video:
            if self.writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.writer = cv2.VideoWriter(
                    self.video_path, fourcc, self.video_fps, (w, h))
                self.get_logger().info(f'녹화 시작: {self.video_path}')
            self.writer.write(frame)

        if self.show_window:
            cv2.imshow('OAK-D solcam tracking', frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                self.get_logger().info('q 입력 → 종료')
                rclpy.shutdown()

    # ── 그리기 ─────────────────────────────────────────────
    def _draw(self, frame):
        # 모든 detection: 회색 박스 + 거리
        for d in self.latest_dets:
            x1 = int(d.x); y1 = int(d.y)
            x2 = int(d.x + d.w); y2 = int(d.y + d.h)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (160, 160, 160), 1)
            cv2.putText(frame, f'{d.depth * 0.001:.2f}m', (x1, max(0, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)

        owner = self.latest_owner
        H, W = frame.shape[:2]
        if owner is not None and owner.is_detected:
            # owner의 화면 마커: spatial_z에 가장 가까운 detection을 강조 (viz 휴리스틱)
            owner_det = self._nearest_det_by_depth(owner.spatial_z * 1000.0)
            if owner_det is not None:
                x1 = int(owner_det.x); y1 = int(owner_det.y)
                x2 = int(owner_det.x + owner_det.w)
                y2 = int(owner_det.y + owner_det.h)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 3)
                cv2.putText(frame, f'OWNER id={owner.track_id}',
                            (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
            self._draw_panel(frame, owner, W)
        else:
            cv2.rectangle(frame, (0, 0), (W - 1, H - 1), (0, 0, 200), 4)
            cv2.putText(frame, 'OWNER LOST', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 220), 2)

    def _nearest_det_by_depth(self, target_depth_mm):
        """owner의 spatial_z(mm)에 depth가 가장 가까운 detection (표시용 휴리스틱)."""
        best = None
        best_diff = 1e9
        for d in self.latest_dets:
            if d.depth <= 1.0:
                continue
            diff = abs(d.depth - target_depth_mm)
            if diff < best_diff:
                best_diff = diff
                best = d
        return best

    def _draw_panel(self, frame, owner, W):
        az_deg = math.degrees(owner.azimuth)
        lines = [
            f'dist : {owner.distance:.2f} m',
            f'azim : {az_deg:+.1f} deg',
            f'z    : {owner.spatial_z:.2f} m',
            f'conf : {owner.confidence:.2f}',
            f'id   : {owner.track_id}',
        ]
        x0 = W - 200
        y0 = 20
        cv2.rectangle(frame, (x0 - 10, 5), (W - 5, y0 + 18 * len(lines) + 5),
                      (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv2.putText(frame, t, (x0, y0 + 18 * i + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = OakViz()
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
