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
import os
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String, Empty
from sensor_msgs.msg import Image

from ros2_control_node.msg import AdjustCmd

from .menu import MenuStateMachine, build_menu, BACK, TRIGGER
from .recognizer import (MediaPipeHandsRecognizer, HagridYoloRecognizer,
                         MockRecognizer)

ADJUST_PARAMS = {
    "SEG_DISTANCE": AdjustCmd.PARAM_SEG_DISTANCE,
    "SEG_ANGLE": AdjustCmd.PARAM_SEG_ANGLE,
    "HEADING_OFFSET": AdjustCmd.PARAM_HEADING_OFFSET,
    "LIFT_HEIGHT": AdjustCmd.PARAM_LIFT_HEIGHT,
    "BODY_VX": AdjustCmd.PARAM_BODY_VX,
    "BODY_VY": AdjustCmd.PARAM_BODY_VY,
    "BODY_WZ": AdjustCmd.PARAM_BODY_WZ,
    "RADIAL_JOG": AdjustCmd.PARAM_RADIAL_JOG,
    "ORBIT_JOG": AdjustCmd.PARAM_ORBIT_JOG,
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
        self.declare_parameter("jog_lin", 0.12)             # m/s, 휠 jog 전후/좌우
        self.declare_parameter("jog_ang", 0.6)              # rad/s, 휠 jog 자전
        self.declare_parameter("radial_jog", 0.12)          # m/s, 주인기준 거리 jog(접근/멀어짐)
        self.declare_parameter("orbit_jog", 0.12)           # m/s, 주인기준 공전(접선) jog
        # 부호(실차에서 방향 반대면 여기만 뒤집기). +1 기준:
        #  orbit: 검지 좌=CCW(+φ)  /  spin: 쓰리건 우=CCW(+off), 좌=CW(−off)
        self.declare_parameter("orbit_sign", 1.0)
        self.declare_parameter("spin_sign", 1.0)
        # rock_on(검지+새끼) = OAK 케이블 0도 지정. 메뉴와 독립.
        self.declare_parameter("rockon_hold", 0.5)       # s, 유지해야 발동
        self.declare_parameter("rockon_cooldown", 3.0)   # s, 발동 후 재명령 차단(꼬임 정리 시간)
        self.declare_parameter("rockon_dropout", 0.4)    # s, 짧은 인식 끊김 허용
        self.declare_parameter("repo_dir", os.environ.get("SOLCAM_REPO", os.path.expanduser("~/solcam")))
        gp = lambda n: self.get_parameter(n).value

        # ----- 부품 조립 (부호까지 반영한 스텝을 메뉴에 전달) -----
        steps = {
            "dist_step": float(gp("dist_step")),
            "seg_angle_step": float(gp("bearing_step_deg")) * DEG * float(gp("orbit_sign")),
            "heading_step": float(gp("heading_step_deg")) * DEG * float(gp("spin_sign")),
            "lift_step": float(gp("lift_step")),
            "jog_lin": float(gp("jog_lin")),
            "jog_ang": float(gp("jog_ang")),
            "radial_jog": float(gp("radial_jog")),
            "orbit_jog": float(gp("orbit_jog")),
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

        # rock_on 홀드 상태 (메뉴 상태기계와 완전 분리)
        self._rockon_hold = float(gp("rockon_hold"))
        self._rockon_cooldown = float(gp("rockon_cooldown"))
        self._rockon_dropout = float(gp("rockon_dropout"))
        self._rockon_start = None       # rock_on 연속 유지 시작시각
        self._rockon_last_seen = 0.0    # 마지막으로 rock_on 본 시각(드롭아웃 허용용)
        self._rockon_block_until = 0.0  # 이 시각까지는 재발동 차단

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
        self._gesture_active_state = False   # 마지막으로 발행한 hold 상태(변화 시만 발행)
        self.pub_mode = self.create_publisher(Int32, "/control_mode", 10)
        self.pub_adjust = self.create_publisher(AdjustCmd, "/adjust_cmd", 10)
        self.pub_phone = self.create_publisher(String, "/phone_cmd", 10)
        self.pub_system = self.create_publisher(String, "/system_cmd", 10)
        self.pub_ui = self.create_publisher(String, "/gesture_ui", 10)
        # OAK 케이블 0도 지정 트리거(메뉴 밖). control_node=데드레코닝 0 재설정,
        #  ui_node=하단 흰 선 0.3s 플래시. 둘 다 같은 토픽 구독.
        self.pub_yaw_zero = self.create_publisher(Empty, "/yaw_set_zero", 10)

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
    def _handle_rockon(self, raw_label, t):
        # 메뉴 상태기계와 독립적으로 rock_on(검지+새끼) 0.5s 홀드를 감지해
        #  /yaw_set_zero 발행. 발동 후 cooldown 동안은 무시(꼬임 정리 + 중복 방지).
        #  짧은 인식 끊김(dropout)은 허용해 홀드가 쉽게 풀리지 않게 한다.
        if t < self._rockon_block_until:
            self._rockon_start = None
            return
        if raw_label == "rock_on":
            self._rockon_last_seen = t
            if self._rockon_start is None:
                self._rockon_start = t
            elif (t - self._rockon_start) >= self._rockon_hold:
                self.pub_yaw_zero.publish(Empty())
                self.get_logger().info("rock_on 0.5s → OAK 케이블 0도 지정(/yaw_set_zero)")
                self._rockon_start = None
                self._rockon_block_until = t + self._rockon_cooldown
        elif self._rockon_start is not None and \
                (t - self._rockon_last_seen) > self._rockon_dropout:
            self._rockon_start = None   # 다른 동작/장시간 끊김 → 홀드 취소

    def _step(self, raw_label, t):
        self._handle_rockon(raw_label, t)   # 메뉴와 무관하게 항상 먼저 처리
        label = self._to_menu_label(raw_label)
        # 도움말 오버레이 표시 중: 메뉴 입력 차단, 역따봉(뒤로)/따봉으로 닫기.
        if self.ui_flags.get("help"):
            if label in (BACK, TRIGGER):
                self.ui_flags["help"] = False
            self._publish_ui()
            return
        for ev in self.sm.update(label, t):
            self._handle(ev)
        self._update_gesture_active()
        self._publish_ui()

    def _update_gesture_active(self):
        # 메뉴 열림 동안엔 몸체 정지(hold) 요청. Wheel 명령(거리/공전/팬)으로 몸체를
        #  움직이는 건 control_node 가 "명령이 들어오는 동안만" hold 를 풀어 처리한다
        #  → 명령을 줄 때만 모터가 돌고, 안 주면 메뉴 안이어도 정지.
        active = (self.sm.state == "MENU")
        if active != self._gesture_active_state:
            self._gesture_active_state = active
            self.pub_active.publish(Bool(data=active))

    def _publish_ui(self):
        snap = self.sm.snapshot()
        snap["ui_flags"] = self.ui_flags
        self.pub_ui.publish(String(data=json.dumps(snap, ensure_ascii=False)))

    def _handle(self, ev):
        # gesture_active(몸체 hold) 발행은 _update_gesture_active 가 중앙에서 처리
        #  (Wheel 메뉴에선 hold 해제). 여기선 로그/액션만.
        if ev.kind == "open":
            self.get_logger().info("메뉴 열림")
        elif ev.kind == "close":
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
            self._run_system(action.payload["cmd"])
        elif action.kind == "cancel":
            pass   # No/취소 — 메뉴만 닫힘(아무 동작 없음)
        elif action.kind == "ui":
            if "press" in action.payload:   # 누를 때마다 +1 (ui_node 가 edge로 사이클)
                key = action.payload["press"]
                self.ui_flags[key] = int(self.ui_flags.get(key, 0)) + 1
            else:
                key = action.payload["toggle"]
                self.ui_flags[key] = not self.ui_flags.get(key, False)

    def _run_system(self, cmd):
        """전원/종료 실제 실행. (메뉴 Other>More 안쪽이라 오발동 위험 낮음)"""
        # ★끄기 직전 모드를 IDLE(0)로 → control_node 가 즉시 정지(publishStop).
        #  안 그러면 메뉴 닫히며 직전 모드(FOLLOW 등)가 재개돼 노드 죽기 전 ~1초 모터가 돈다.
        if cmd in ("shutdown", "quit"):
            self.pub_mode.publish(Int32(data=0))
        try:
            if cmd == "shutdown":
                self.get_logger().warn("로봇 전원 OFF → poweroff")
                subprocess.Popen(["bash", "-lc", "systemctl poweroff || sudo -n poweroff"])
            elif cmd == "quit":
                repo = str(self.get_parameter("repo_dir").value)
                sh = os.path.join(repo, "scripts", "solcam.sh")
                self.get_logger().warn(f"SolCam 종료 → {sh} stop")
                subprocess.Popen(["bash", sh, "stop"],
                                 start_new_session=True)   # 자기 자신도 정리되므로 분리
        except Exception as e:
            self.get_logger().error(f"system 명령 실패({cmd}): {e}")


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
