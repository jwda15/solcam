#!/usr/bin/env python3
"""데이터셋(tum_publisher) 추적 시각화.
   /camera/color/image_raw + /detections + /owner_pose 를 한 창에.
   q/ESC 종료."""
import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from ros2_tracking_node.msg import DetectionArray, OwnerPose
from cv_bridge import CvBridge
import cv2

class VizDS(Node):
    def __init__(self):
        super().__init__('viz_dataset')
        self.bridge = CvBridge()
        self.dets = []
        self.owner = None
        # [0525] 재매핑 허용 범위 시각화용 (tracking_node 기본값과 맞춤)
        self.RE_BASE = 50.0       # reassign_base_dist_px
        self.RE_GROWTH = 5.0      # reassign_growth_px_per_frame
        self.RE_GRACE = 10        # grace_frames
        self.last_owner_cx = None # owner 박스를 마지막으로 본 픽셀 위치
        self.last_owner_cy = None
        self.lost_frames = 0      # owner 박스를 못 본 연속 프레임 수(viz 자체 추정)
        # [0525] /owner_pose는 픽셀이 아닌 spatial(m)만 실림 → intrinsic으로 역투영해 점으로 찍는다.
        self.fx = None; self.fy = None; self.cx = None; self.cy = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_rgb, qos)
        self.create_subscription(DetectionArray, '/detections', self.cb_det, 10)
        self.create_subscription(OwnerPose, '/owner_pose', self.cb_owner, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cb_info, qos)
        self.get_logger().info('VizDS 시작. 창에서 q/ESC 종료.')

    def cb_det(self, m): self.dets = list(m.detections)
    def cb_owner(self, m): self.owner = m
    def cb_info(self, m):
        # k = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fx = m.k[0]; self.fy = m.k[4]; self.cx = m.k[2]; self.cy = m.k[5]

    def cb_rgb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        o = self.owner
        # 모든 사람 detection: 회색 + 거리
        for d in self.dets:
            x1,y1 = int(d.x), int(d.y); x2,y2 = int(d.x+d.w), int(d.y+d.h)
            cv2.rectangle(frame,(x1,y1),(x2,y2),(160,160,160),1)
            cv2.putText(frame,f"{d.depth/1000:.1f}m",(x1,y2+13),
                        cv2.FONT_HERSHEY_SIMPLEX,0.4,(180,180,180),1)
        # 주인: spatial_z 최근접 det 강조 (OwnerPose엔 bbox 없으니)
        H,W = frame.shape[:2]
        owner_box_found = False
        if o is not None and o.is_detected:
            tgt = o.spatial_z*1000.0
            best=None; bd=1e9
            for d in self.dets:
                if d.depth<=1: continue
                if abs(d.depth-tgt)<bd: bd=abs(d.depth-tgt); best=d
            if best is not None:
                x1,y1=int(best.x),int(best.y); x2,y2=int(best.x+best.w),int(best.y+best.h)
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,220,0),3)
                cv2.putText(frame,f"OWNER id={o.track_id}",(x1,max(0,y1-8)),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,220,0),2)
                # owner 박스 위치 기억 (재매핑 원의 중심)
                self.last_owner_cx = (x1+x2)*0.5
                self.last_owner_cy = (y1+y2)*0.5
                owner_box_found = True
            lines=[f"dist {o.distance:.2f}m", f"az {math.degrees(o.azimuth):+.0f}deg",
                   f"z {o.spatial_z:.2f}m", f"conf {o.confidence:.2f}", f"id {o.track_id}"]
            for i,t in enumerate(lines):
                cv2.putText(frame,t,(10,22+i*22),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,0),2)
        else:
            cv2.rectangle(frame,(0,0),(W-1,H-1),(0,0,200),4)
            cv2.putText(frame,"OWNER LOST",(20,40),cv2.FONT_HERSHEY_SIMPLEX,1.0,(0,0,220),2)

        # [0525] 재매핑 허용 범위 원 (얕은 빨간선).
        #   owner 박스를 못 본 동안만, lost_frames가 grace를 넘으면 표시.
        #   반지름 = base + growth*(lost_frames - grace)  (tracking_node 공식과 동일)
        if owner_box_found:
            self.lost_frames = 0
        else:
            self.lost_frames += 1
            if (self.last_owner_cx is not None
                    and self.lost_frames > self.RE_GRACE):
                radius = self.RE_BASE + self.RE_GROWTH * (self.lost_frames - self.RE_GRACE)
                cx = int(self.last_owner_cx); cy = int(self.last_owner_cy)
                cv2.circle(frame, (cx, cy), int(radius), (0,0,255), 1)  # 얕은 빨간선
                cv2.putText(frame, f"reassign r={radius:.0f}px (lost {self.lost_frames}f)",
                            (max(0,cx-90), max(14,cy-int(radius)-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)

        # [0525] 실제 발행되는 owner 위치를 점으로 표시 (spatial → 픽셀 역투영).
        #   px = cx + spatial_x*fx/spatial_z,  py = cy + spatial_y*fy/spatial_z
        #   색: HIT(실제 검출, conf가 큼)=노랑 / KF 외삽(conf가 작음)=자홍(magenta).
        #   tracking_node가 KF 외삽 시 confidence를 0.5배로 깍으므로 그걸로 구분.
        if (o is not None and o.is_detected and self.fx is not None
                and o.spatial_z > 0.05):
            px = int(self.cx + o.spatial_x * self.fx / o.spatial_z)
            py = int(self.cy + o.spatial_y * self.fy / o.spatial_z)
            # KF 외삽 추정: conf가 낮으면(≤0.5 근처) 외삽일 가능성 높음.
            is_kf = (o.confidence <= 0.35)
            color = (255,0,255) if is_kf else (0,255,255)  # 자홍 / 노랑
            label = "KF" if is_kf else "HIT"
            cv2.circle(frame, (px, py), 7, color, -1)
            cv2.circle(frame, (px, py), 9, (0,0,0), 1)
            cv2.putText(frame, f"owner({label})", (px+10, py),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        cv2.imshow("Dataset Tracking (2 people)", frame)
        if (cv2.waitKey(1)&0xFF) in (ord('q'),27): rclpy.shutdown()

def main():
    rclpy.init(); n=VizDS()
    try: rclpy.spin(n)
    except: pass
    finally: cv2.destroyAllWindows()

if __name__=='__main__': main()
