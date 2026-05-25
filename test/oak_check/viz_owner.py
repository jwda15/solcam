#!/usr/bin/env python3
"""OAK 추적 시각화 뷰어
   /oak/rgb/image_raw + /detections + /owner_pose 를 한 창에 합쳐 표시.
   - 모든 사람 detection: 회색 박스
   - 주인(track_id 일치): 초록 굵은 박스 + OWNER
   - 좌상단 HUD: is_detected, distance, azimuth, z, track_id, conf
   q 또는 ESC로 종료."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ros2_tracking_node.msg import DetectionArray, OwnerPose
from cv_bridge import CvBridge
import cv2

class VizOwner(Node):
    def __init__(self):
        super().__init__('viz_owner')
        self.bridge = CvBridge()
        self.last_dets = []
        self.owner = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/oak/rgb/image_raw', self.cb_rgb, qos)
        self.create_subscription(DetectionArray, '/detections', self.cb_det, 10)
        self.create_subscription(OwnerPose, '/owner_pose', self.cb_owner, 10)
        self.get_logger().info('VizOwner 시작. 창에서 q/ESC 종료.')

    def cb_det(self, msg):
        self.last_dets = list(msg.detections)

    def cb_owner(self, msg):
        self.owner = msg

    def cb_rgb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        o = self.owner
        otid = o.track_id if (o and o.is_detected) else -999
        for d in self.last_dets:
            x1, y1 = int(d.x), int(d.y)
            x2, y2 = int(d.x + d.w), int(d.y + d.h)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (160,160,160), 1)
            cv2.putText(frame, f"{d.depth/1000:.2f}m", (x1, y2+14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
        # HUD
        if o is not None and o.is_detected:
            az_deg = o.azimuth * 180.0 / math.pi
            hud = [
                "OWNER DETECTED",
                f"id      : {o.track_id}",
                f"distance: {o.distance:.2f} m",
                f"azimuth : {az_deg:+.1f} deg",
                f"z       : {o.spatial_z:.2f} m",
                f"x       : {o.spatial_x:+.2f} m",
                f"conf    : {o.confidence:.2f}",
            ]
            col = (0,255,0)
        else:
            hud = ["OWNER LOST"]
            col = (0,0,255)
        y = 22
        for line in hud:
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
            y += 24
        # azimuth 방향 화살표 (중앙 하단)
        if o is not None and o.is_detected:
            h, w = frame.shape[:2]
            cxp, cyp = w//2, h-30
            ex = int(cxp + 80*math.sin(o.azimuth))
            cv2.arrowedLine(frame, (cxp, cyp), (ex, cyp-30), (0,255,255), 2, tipLength=0.3)
        cv2.imshow("OAK Owner Tracking", frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            rclpy.shutdown()

def main():
    rclpy.init()
    node = VizOwner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,):
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
