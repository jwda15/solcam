"""
oak_detector.py (DepthAI v3.6.1)
================================
OAK-D S2 온보드 YOLO(SpatialDetectionNetwork) → /detections + /camera_info publish.
tracking_node는 detection 소스에 독립적이므로 그대로 사용 (무수정).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import depthai as dai
from ros2_tracking_node.msg import Detection, DetectionArray


class OakDetector(Node):
    def __init__(self):
        super().__init__('oak_detector')
        self.declare_parameter('model', 'yolov6-nano')
        # [0525] preview를 NN 입력(512x384)과 동일 종횡비로 맞춤.
        #   불일치 시(예: 640x400) NN이 crop/scale하면서 정규화 좌표가 어긋나
        #   박스가 가로로 밀려 보임. zoo yolov6-nano = 512x384.
        self.declare_parameter('preview_width', 512)
        self.declare_parameter('preview_height', 384)
        self.declare_parameter('person_only', True)
        self.declare_parameter('depth_lower_mm', 100)
        self.declare_parameter('depth_upper_mm', 8000)
        self.declare_parameter('bbox_scale', 0.3)
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('publish_rgb', True)

        self.model       = self.get_parameter('model').value
        self.prev_w      = int(self.get_parameter('preview_width').value)
        self.prev_h      = int(self.get_parameter('preview_height').value)
        self.person_only = bool(self.get_parameter('person_only').value)
        self.depth_lower = int(self.get_parameter('depth_lower_mm').value)
        self.depth_upper = int(self.get_parameter('depth_upper_mm').value)
        self.bbox_scale  = float(self.get_parameter('bbox_scale').value)
        self.conf_thresh = float(self.get_parameter('conf_thresh').value)
        self.publish_rgb = bool(self.get_parameter('publish_rgb').value)

        self.bridge = CvBridge()
        self.frame_id = 'camera_color_optical_frame'
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_det  = self.create_publisher(DetectionArray, '/detections', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)
        if self.publish_rgb:
            self.pub_rgb = self.create_publisher(Image, '/oak/rgb/image_raw', qos)

        self._build_and_start()
        self.cam_info_msg = self._build_camera_info()
        self.frame_count = 0
        self.timer = self.create_timer(0.02, self._poll)
        self.get_logger().info('OakDetector 준비 완료.')

    def _build_and_start(self):
        size = (self.prev_w, self.prev_h)
        self.pipeline = dai.Pipeline()
        camRgb    = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        monoLeft  = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        stereo = self.pipeline.create(dai.node.StereoDepth)
        # [0525 임시/확인용] extended disparity OFF.
        #   zoo 모델이 8-shave 컴파일이라, extended/subpixel을 켜면 StereoDepth가
        #   VPU 코어를 많이 써서 NN과 shave 충돌 -> 디바이스 크래시.
        #   품질 최적화(extended ON)는 모델을 6-shave로 재변환한 뒤 별도로.
        #
        # [0525 중요] setDepthAlign / setOutputSize 는 호출하지 않는다.
        #   SpatialDetectionNetwork가 stereo depth의 RGB 정렬을 내부적으로 관리하므로,
        #   여기서 수동으로 setDepthAlign(CAM_A)+setOutputSize를 걸면 내부 동기화와
        #   충돌해 NN out 큐가 멈춘다(검출이 한 건도 안 나옴). 실측으로 확인됨.
        #   (순수 depth 뷰어에서는 align이 필요하지만, Spatial NN에서는 빼야 한다.)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(False)
        stereo.setLeftRightCheck(True)
        monoLeft.requestOutput(size).link(stereo.left)
        monoRight.requestOutput(size).link(stereo.right)

        self.nn = self.pipeline.create(dai.node.SpatialDetectionNetwork).build(
            camRgb, stereo, dai.NNModelDescription(self.model))
        self.nn.input.setBlocking(False)
        self.nn.setConfidenceThreshold(self.conf_thresh)  # [0525] NN단 신뢰도 필터 (누락 보완)
        self.nn.setBoundingBoxScaleFactor(self.bbox_scale)
        # [0525] 깊이 집계를 MEDIAN으로 명시.
        #   기본 집계는 bbox 중앙 영역(bbox_scale)의 depth를 모아 대표값 1개를 내는데,
        #   사람이 좌우로 움직이면(azimuth 변화) 영역에 들어오는 신체부위/배경이 바뀌며
        #   depth 분포가 출렁여 z가 요동친다.
        #   MEDIAN은 배경/이상치 몇 점이 끼어도 정렬 후 가운데값이라 출렁임에 강하다.
        #   (MEAN은 먼 배경 한 점에도 끌려 올라가 더 튄다.)
        self.nn.setSpatialCalculationAlgorithm(dai.SpatialLocationCalculatorAlgorithm.MEDIAN)
        self.nn.setDepthLowerThreshold(self.depth_lower)
        self.nn.setDepthUpperThreshold(self.depth_upper)
        self.labelMap = self.nn.getClasses()

        self.qRgb = self.nn.passthrough.createOutputQueue(maxSize=4, blocking=False)
        self.qDet = self.nn.out.createOutputQueue(maxSize=4, blocking=False)
        self.get_logger().info('OAK 파이프라인 시작 (모델 첫 다운로드 시 지연 가능)...')
        self.pipeline.start()

    def _build_camera_info(self):
        import math
        info = CameraInfo()
        info.width  = self.prev_w
        info.height = self.prev_h
        # passthrough(=preview) 해상도 기준 근사 intrinsic. tracking_node backprojection용.
        fx = fy = self.prev_w / (2.0 * math.tan(math.radians(69.0) / 2.0))
        cx = self.prev_w / 2.0
        cy = self.prev_h / 2.0
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _poll(self):
        inDet = self.qDet.tryGet()
        if inDet is None:
            return
        stamp = self.get_clock().now().to_msg()

        self.cam_info_msg.header.stamp = stamp
        self.cam_info_msg.header.frame_id = self.frame_id
        self.pub_info.publish(self.cam_info_msg)

        msg = DetectionArray()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        W = float(self.prev_w); H = float(self.prev_h)
        for d in inDet.detections:
            if self.person_only and d.label != 0:
                continue
            x1 = max(0.0, d.xmin * W); y1 = max(0.0, d.ymin * H)
            x2 = min(W - 1.0, d.xmax * W); y2 = min(H - 1.0, d.ymax * H)
            w = x2 - x1; h = y2 - y1
            if w <= 1.0 or h <= 1.0:
                continue
            depth_mm = float(d.spatialCoordinates.z)
            # 유효 depth 없으면(스테레오 미검출 sentinel z=0) 추적기에 넣지 않는다.
            #  z=0 을 칼만 측정값으로 먹으면 주인 위치/매칭(Mahalanobis)이 오염되므로,
            #  차라리 이 프레임은 빼고 트래커의 Lost 외삽에 맡긴다.
            if depth_mm <= 0.0:
                continue
            det = Detection()
            det.x = float(x1); det.y = float(y1)
            det.w = float(w);  det.h = float(h)
            det.depth = depth_mm
            det.score = float(d.confidence)
            det.label = int(d.label)
            msg.detections.append(det)
        self.pub_det.publish(msg)

        if self.publish_rgb:
            inRgb = self.qRgb.tryGet()
            if inRgb is not None:
                img = self.bridge.cv2_to_imgmsg(inRgb.getCvFrame(), encoding='bgr8')
                img.header.stamp = stamp
                img.header.frame_id = self.frame_id
                self.pub_rgb.publish(img)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'프레임 {self.frame_count}, det={len(msg.detections)}')

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = OakDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
