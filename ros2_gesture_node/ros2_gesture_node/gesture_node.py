"""손동작 supervisor 노드.

구독:
  /oak/rgb/image_raw (sensor_msgs/Image)  OAK RGB (recognizer=mediapipe|hagrid)
  /gesture_mock      (std_msgs/String)    개발용 제스처 주입 (recognizer=mock)
발행:
  /gesture_active (std_msgs/Bool)         메뉴 열림=true → 컨트롤이 몸체 감속정지
  /control_mode   (std_msgs/Int32)        주행 모드 변경
  /adjust_cmd     (ros2_control_node/AdjustCmd)  거리·헤딩·리프트 조정
  /phone_cmd      (std_msgs/String)       폰 카메라 제어
  /system_cmd     (std_msgs/String)       전원 등 시스템 (자리만)
  /gesture_ui     (std_msgs/String, JSON) LCD UI·디버그용 상태 스냅샷

구조: 인식(recognizer) → [문맥 변환] → 상태기계(menu) → 사건을 토픽으로 번역.
  ★문맥 변환: 인식기는 단일 검지를 방향(point_*)으로만 준다. 손가락-개수 맥락
   (Mode/Other/메인)에서는 검지=손가락1개로 봐야 하므로 'one'으로 바꾼다.
   방향 맥락(Wheel/Lift)에서는 p_up/p_down/p_left/p_right 로 매핑한다.
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Image

from ros2_control_node.msg import AdjustCmd

from .menu import MenuStateMachine, build_menu
from .recognizer import (MediaPipeHandsRecognizer, HagridYoloRecognizer,
                         MockRecognizer)

ADJUST_PARAMS = {
    "SEG_DISTANCE": AdjustCmd.PARAM_SEG_DISTANCE,
    "SEG_ANGLE": AdjustCmd.PARAM_SEG_ANGLE,
    "HEADING_OFFSET": AdjustCmd.PARAM_HEADING_OFFSET,
    "LIFT_HEIGHT": AdjustCmd.PARAM_LIFT_HEIGHT,
}
DEG = math.pi / 180.0


class GestureNode(Node):
    def __init__(self):
        super().__init__("gesture_node")
        # ----- 파라미터 (기본값은 config/gesture_params.yaml 과 일치 유지) -----
        self.declare_parameter("recognizer", "mediapipe")   # mediapipe | hagrid | mock
        self.declare_parameter("model_path", "models/YOLOv10n_gestures.pt")  # hagrid 전용
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("idle_rate", 5.0)
        self.declare_parameter("menu_rate", 15.0)
        self.declare_parameter("trigger_hold", 1.5)
        self.declare_parameter("select_hold", 1.5)
        self.declare_parameter("menu_timeout", 10.0)
        self.declare_parameter("dropout_tol", 0.4)
        self.declare_parameter("dist_step", 0.3)            # m, +멀리
        self.declare_parameter("bearing_step_deg", 8.0)     # deg, 공전 1스텝
        self.declare_parameter("heading_step_deg", 15.0)    # deg, 자전/헤딩 1스텝
        self.declare_parameter("lift_step", 0.1)            # m, +올림
        # 부호(실차에서 방향 반대면 여기만 뒤집기). +1 기준:
        #  orbit: 검지 좌=CCW(+φ)  /  spin: 쓰리건 우=CCW(+off), 좌=CW(−off)
        self.declare_parameter("orbit_sign", 1.0)
        self.declare_parameter("spin_sign", 1.0)
        gp = lambda n: self.get_parameter(n).value

        # ----- 부품 조립 (부호까지 반영한 스텝을 메뉴에 전달) -----
        steps = {
            "dist_step": float(gp("dist_step")),
            "seg_angle_step": float(gp("bearing_step_deg")) * DEG * float(gp("orbit_sign")),
            "heading_step": float(gp("heading_step_deg")) * DEG * float(gp("spin_sign")),
            "lift_step": float(gp("lift_step")),
        }
        self.sm = MenuStateMachine(
            build_menu(steps),
            trigger_hold=float(gp("trigger_hold")),
            select_hold=float(gp("select_hold")),
            menu_timeout=float(gp("menu_timeout")),
            dropout_tol=float(gp("dropout_tol")))
        self.idle_period = 1.0 / float(gp("idle_rate"))
        self.menu_period = 1.0 / float(gp("menu_rate"))
        self._last_infer_t = 0.0
        self.ui_flags = {"oak_view": False}

        rtype = str(gp("recognizer"))
        self.mock_mode = rtype == "mock"
        if self.mock_mode:
            self.recognizer = MockRecognizer()
            self.create_subscription(String, "/gesture_mock", self._mock_cb, 10)
            self.create_timer(self.menu_period, self._mock_step)
            self.get_logger().info("recognizer=mock (/gesture_mock 으로 canonical 라벨 주입)")
        else:
            if rtype == "hagrid":
                self.recognizer = HagridYoloRecognizer(
                    str(gp("model_path")), float(gp("conf_threshold")))
                self.get_logger().warn("recognizer=hagrid: 방향(point_*) 불가 — 방향 손동작 미지원")
            else:
                self.recognizer = MediaPipeHandsRecognizer()
                self.get_logger().info("recognizer=mediapipe (손 21랜드마크, 양손/손바닥·손등 무관)")
            self.create_subscription(Image, str(gp("image_topic")),
                                     self._image_cb, qos_profile_sensor_data)

        # ----- 발행자 -----
        self.pub_active = self.create_publisher(Bool, "/gesture_active", 10)
        self.pub_mode = self.create_publisher(Int32, "/control_mode", 10)
        self.pub_adjust = self.create_publisher(AdjustCmd, "/adjust_cmd", 10)
        self.pub_phone = self.create_publisher(String, "/phone_cmd", 10)
        self.pub_system = self.create_publisher(String, "/system_cmd", 10)
        self.pub_ui = self.create_publisher(String, "/gesture_ui", 10)

    # ----- 입력 경로 ------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _image_cb(self, msg: Image):
        t = self._now()
        period = self.menu_period if self.sm.state == "MENU" else self.idle_period
        if t - self._last_infer_t < period:
            return
        self._last_infer_t = t
        frame = self._to_bgr(msg)
        if frame is None:
            return
        label, _conf = self.recognizer.infer(frame)
        self._step(label, t)

    def _mock_cb(self, msg: String):
        self.recognizer.set_gesture(msg.data.strip(), self._now())

    def _mock_step(self):
        t = self._now()
        label, _ = self.recognizer.infer_at(t)
        self._step(label, t)

    @staticmethod
    def _to_bgr(msg: Image):
        if msg.encoding not in ("bgr8", "rgb8"):
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            frame = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        except ValueError:
            return None
        if msg.encoding == "rgb8":
            frame = frame[:, :, ::-1]
        return frame

    # ----- 문맥 변환: 인식 canonical 라벨 → 현재 메뉴 노드가 받는 키 --------
    def _to_menu_label(self, raw):
        if raw is None:
            return None
        if raw.startswith("point_"):          # 단일 검지(방향)
            d = raw[len("point_"):]
            cur = self.sm.path[-1] if self.sm.path else None
            ch = cur.children if cur else None
            if ch:
                if ("p_" + d) in ch:          # 방향 맥락(Wheel/Lift)
                    return "p_" + d
                if "one" in ch:               # 손가락-개수 맥락 → 검지=1개
                    return "one"
            return None
        return raw                            # like/dislike/gun_*/two/three/four 그대로

    # ----- 사건 → 토픽 ----------------------------------------------------
    def _step(self, raw_label, t):
        label = self._to_menu_label(raw_label)
        for ev in self.sm.update(label, t):
            self._handle(ev)
        snap = self.sm.snapshot()
        snap["ui_flags"] = self.ui_flags
        self.pub_ui.publish(String(data=json.dumps(snap, ensure_ascii=False)))

    def _handle(self, ev):
        if ev.kind == "open":
            self.pub_active.publish(Bool(data=True))
            self.get_logger().info("메뉴 열림 → 몸체 일시정지 요청")
        elif ev.kind == "close":
            self.pub_active.publish(Bool(data=False))
            self.get_logger().info(f"메뉴 닫힘({ev.reason}) → 주행 재개")
        elif ev.kind == "action":
            self._execute(ev.action)

    def _execute(self, action):
        self.get_logger().info(f"실행: {action.name}")
        if action.kind == "mode":
            self.pub_mode.publish(Int32(data=int(action.payload["mode"])))
        elif action.kind == "adjust":
            m = AdjustCmd()
            m.header.stamp = self.get_clock().now().to_msg()
            m.param = ADJUST_PARAMS[action.payload["param"]]
            m.value = float(action.payload["value"])
            m.delta = bool(action.payload.get("delta", True))
            self.pub_adjust.publish(m)
        elif action.kind == "phone":
            self.pub_phone.publish(String(data=action.payload["cmd"]))
        elif action.kind == "system":
            self.pub_system.publish(String(data=action.payload["cmd"]))
        elif action.kind == "ui":
            key = action.payload["toggle"]
            self.ui_flags[key] = not self.ui_flags.get(key, False)


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
