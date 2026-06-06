"""LCD UI 노드 — 7" HDMI(1024x600, Jetson 연결).

ROS 토픽을 받아 Hud(hud.py)로 렌더한다. 디자인/그리기는 전부 hud.py에 있고
(ROS 무관), 이 노드는 "토픽 → 데이터" 어댑터다. 같은 Hud를 Windows 프리뷰
(tools/ui_preview.py)도 써서 디자인이 어긋나지 않는다.

토픽:
  /gesture_ui (String JSON)  메뉴 상태 (gesture_node)
  /control_mode (Int32)      현재 모드
  /phone/image (Image)       촬영 영상 (없으면 /oak/rgb/image_raw 폴백)
  /phone/battery (BatteryState)  배터리 %
  /phone/recording (Bool)    녹화 중 여부

pygame/디스플레이 없으면 콘솔 폴백. /phone/* 는 ros2_phone_bridge(scrcpy/adb)가 발행.
"""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Image, BatteryState


class UiNode(Node):
    def __init__(self):
        super().__init__("gesture_ui_node")
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("width", 1024)
        self.declare_parameter("height", 600)
        self.declare_parameter("video_topic", "/phone/image")
        self.declare_parameter("video_fallback", "/oak/rgb/image_raw")

        self.snap = {"state": "IDLE", "items": [], "hold_gesture": "",
                     "hold_progress": 0.0}
        self.mode = 0
        self.battery = None
        self.recording = False
        self.rec_start = 0.0
        self.phone_frame = None    # /phone/image (촬영 카메라)
        self.oak_frame = None      # /oak/rgb/image_raw (손동작 인식 카메라)

        self.create_subscription(String, "/gesture_ui", self._ui_cb, 10)
        self.create_subscription(Int32, "/control_mode", self._mode_cb, 10)
        self.create_subscription(BatteryState, "/phone/battery", self._batt_cb, 10)
        self.create_subscription(Bool, "/phone/recording", self._rec_cb, 10)
        self.create_subscription(
            Image, str(self.get_parameter("video_topic").value), self._phone_img_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter("video_fallback").value), self._oak_img_cb,
            qos_profile_sensor_data)

        try:
            import pygame
            from .hud import Hud
            self.pygame = pygame
            pygame.init()
            size = (int(self.get_parameter("width").value),
                    int(self.get_parameter("height").value))
            flags = pygame.FULLSCREEN if self.get_parameter("fullscreen").value else 0
            self.screen = pygame.display.set_mode(size, flags)
            pygame.mouse.set_visible(False)
            self.hud = Hud(pygame)
            self._closing = False
            self.render_timer = self.create_timer(1.0 / 30.0, self._render)
            self.get_logger().info("LCD UI 시작 (pygame)")
        except Exception as e:
            self.pygame = None
            self.create_timer(0.5, self._render_console)
            self.get_logger().warn(f"pygame 불가({e}) -> 콘솔 폴백")

    # ----- 콜백 -----
    def _ui_cb(self, msg):
        try:
            self.snap = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _mode_cb(self, msg):
        self.mode = int(msg.data)

    def _batt_cb(self, msg):
        if msg.percentage is not None and msg.percentage >= 0.0:
            self.battery = int(round(msg.percentage * 100.0))

    def _rec_cb(self, msg):
        if msg.data and not self.recording:
            self.rec_start = time.time()
        self.recording = bool(msg.data)

    @staticmethod
    def _decode(msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        except ValueError:
            return None
        return img[:, :, ::-1] if msg.encoding == "bgr8" else img

    def _phone_img_cb(self, msg):
        f = self._decode(msg)
        if f is not None:
            self.phone_frame = f

    def _oak_img_cb(self, msg):
        f = self._decode(msg)
        if f is not None:
            self.oak_frame = f

    # ----- 렌더 -----
    def _render(self):
        pg = self.pygame
        if self._closing:
            return
        for e in pg.event.get():
            if e.type == pg.QUIT or (e.type == pg.KEYDOWN and e.key == pg.K_ESCAPE):
                self._closing = True
                try:
                    self.render_timer.cancel()
                except Exception:
                    pass
                pg.quit()               # 창 즉시 닫기(WM '응답 없음' 방지)
                rclpy.shutdown()
                return
        state = self.snap.get("state")
        oak_view = bool(self.snap.get("ui_flags", {}).get("oak_view", False))
        split = (state == "MENU") or oak_view
        bg = self.phone_frame if self.phone_frame is not None else self.oak_frame
        self.hud.draw(self.screen, self.snap, mode=self.mode, battery=self.battery,
                      recording=self.recording, rec_start=self.rec_start,
                      frame=(self.phone_frame if split else bg),
                      oak_frame=self.oak_frame, split=split)
        pg.display.flip()

    def _render_console(self):
        s = self.snap
        if s.get("state") == "MENU":
            items = " | ".join(f"{i['gesture']}:{i['label']}" for i in s.get("items", []))
            print(f"[UI] {' > '.join(s.get('path', []))}  {items}  "
                  f"hold={s.get('hold_gesture')}({s.get('hold_progress', 0):.0%})")


def main(args=None):
    rclpy.init(args=args)
    node = UiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.pygame is not None:
                node.pygame.quit()   # 창/리소스 정리(이중 호출 안전)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
