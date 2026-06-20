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
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String, Float32
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist

# 키보드 수동주행 모드 이름(오버레이 표시용). ASCII 영문만 — pygame 기본 폰트가
# 한글/화살표 글리프가 없어 □로 깨지기 때문.
MODE_NAMES = {0: "IDLE (manual)", 1: "FOLLOW", 2: "ROTATE",
              3: "FOLLOW2", 4: "ORBIT", 5: "(n/a)"}


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
        self.phone_zoom = 1.0      # /phone/zoom (현재/목표 줌 배율)

        self.create_subscription(String, "/gesture_ui", self._ui_cb, 10)
        self.create_subscription(Int32, "/control_mode", self._mode_cb, 10)
        self.create_subscription(BatteryState, "/phone/battery", self._batt_cb, 10)
        self.create_subscription(Bool, "/phone/recording", self._rec_cb, 10)
        self.create_subscription(Float32, "/phone/zoom", self._zoom_cb, 10)
        self.create_subscription(
            Image, str(self.get_parameter("video_topic").value), self._phone_img_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter("video_fallback").value), self._oak_img_cb,
            qos_profile_sensor_data)

        # ----- 키보드 수동주행(teleop) : UI 창이 포커스를 가지면 바로 조작 -----
        #  teleop_keyboard.py 의 로직을 UI 에 이식. 별도 teleop 창 없이 LCD UI 에서
        #  화살표/WASD 로 바로 주행. control_node 모드0(IDLE)에서만 실제 주행 반영된다.
        #    ↑/↓ 전후진(vx)  ←/→ 좌우 평행이동(vy, 메카넘)  a/d 좌/우회전(wz)
        #    space 정지   m→숫자(0~5) 모드변경   esc 종료
        self.declare_parameter("teleop", True)     # 키보드 주행 on/off (개발 PC면 false 가능)
        self.declare_parameter("speed", 0.3)       # m/s, 평면 목표속도
        self.declare_parameter("yaw_rate", 0.8)    # rad/s, 회전 목표속도
        self.teleop_on = bool(self.get_parameter("teleop").value)
        self.speed = float(self.get_parameter("speed").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.pub_teleop = self.create_publisher(Twist, "/teleop_cmd", 10)
        self.pub_mode_out = self.create_publisher(Int32, "/control_mode", 10)
        self.mode_select = False     # m 오버레이 상태
        self.last_key_mode = None    # 마지막으로 키로 바꾼 모드(표시용)
        self.cur_vx = self.cur_vy = self.cur_wz = 0.0

        try:
            import pygame
            from .hud import Hud
            self.pygame = pygame
            pygame.init()
            size = (int(self.get_parameter("width").value),
                    int(self.get_parameter("height").value))
            flags = pygame.FULLSCREEN if self.get_parameter("fullscreen").value else 0
            self.screen = pygame.display.set_mode(size, flags)
            pygame.display.set_caption("SolCam")
            pygame.mouse.set_visible(False)
            self.hud = Hud(pygame)
            self.kfont = pygame.font.Font(None, 26)   # 모드선택 오버레이용(ASCII)
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

    def _zoom_cb(self, msg):
        self.phone_zoom = float(msg.data)

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
            if e.type == pg.QUIT:
                self._quit_ui(); return
            if e.type == pg.KEYDOWN:
                if self.mode_select:          # 모드 오버레이: 숫자=선택, m/esc=취소
                    self._teleop_mode_key(e.key); continue
                if e.key == pg.K_ESCAPE:
                    self._quit_ui(); return
                if self.teleop_on and e.key == pg.K_m:
                    self.mode_select = True
        # 키보드 수동주행: 매 프레임 키 상태 폴링 → /teleop_cmd 발행(대각선 동시키 지원)
        if self.teleop_on and not self.mode_select:
            self._teleop_poll()
        state = self.snap.get("state")
        oak_view = bool(self.snap.get("ui_flags", {}).get("oak_view", False))
        # 폰 영상이 없으면(미연결/nophone) 메뉴에서도 분할하지 않고 OAK 전체 배경.
        #  (검은 PHONE 반쪽 방지) — OAK view 토글은 명시적이라 그대로 분할.
        split = oak_view or (state == "MENU" and self.phone_frame is not None)
        bg = self.phone_frame if self.phone_frame is not None else self.oak_frame
        self.hud.draw(self.screen, self.snap, mode=self.mode, battery=self.battery,
                      recording=self.recording, rec_start=self.rec_start,
                      frame=(self.phone_frame if split else bg),
                      oak_frame=self.oak_frame, split=split, zoom=self.phone_zoom)
        if self.mode_select:
            self._draw_mode_overlay()
        elif self.teleop_on:
            self._draw_teleop_status()   # 포커스 상태 + 키 입력값(진단/안내)
        pg.display.flip()

    def _draw_teleop_status(self):
        # 키보드 주행 진단/안내(ASCII, 기본폰트). 포커스 있으면 실시간 vx/vy/wz,
        #  없으면 빨간 클릭 안내. 키 눌렀는데 값이 0이면 포커스 문제, 값이 변하면
        #  ui_node 는 정상(그땐 control_node/IDLE 쪽 확인).
        pg = self.pygame
        if hasattr(pg.key, "get_focused") and pg.key.get_focused():
            txt = "key drive  vx=%+.2f  vy=%+.2f  wz=%+.2f" % (
                self.cur_vx, self.cur_vy, self.cur_wz)
            col = (120, 200, 240)
        else:
            txt = "CLICK THIS WINDOW for keyboard drive (no focus)"
            col = (240, 120, 120)
        self.screen.blit(self.kfont.render(txt, True, col), (16, 6))

    def _quit_ui(self):
        # ESC/창닫기 = UI만이 아니라 전체 솔캠 스택을 정지(노드 중첩 방지).
        self._stop_full_stack()
        pg = self.pygame
        self._closing = True       # main 루프가 _closing 보고 빠져나가 정상 종료
        try:
            self.render_timer.cancel()
        except Exception:
            pass
        pg.quit()                  # 창 즉시 닫기(WM '응답 없음' 방지)

    def _stop_full_stack(self):
        # 아이콘 래퍼가 export 한 SOLCAM_REPO 가 있을 때만 solcam.sh stop 을 띄워
        #  전체 노드를 graceful 종료. (단독 실행 등 REPO 없으면 UI 만 닫힘)
        #  detach(start_new_session) 해서 ui_node 가 죽어도 stop 은 끝까지 돈다.
        import os
        import subprocess
        repo = os.environ.get("SOLCAM_REPO")
        if not repo:
            return
        sh = os.path.join(repo, "scripts", "solcam.sh")
        if not os.path.exists(sh):
            return
        try:
            subprocess.Popen(["bash", "-lic", f"'{sh}' stop"],
                             start_new_session=True)
            self.get_logger().info("ESC → 전체 스택 정지(solcam.sh stop)")
        except Exception as e:
            self.get_logger().warn(f"전체 정지 실패({e}) — UI만 닫힘")

    # ----- 키보드 수동주행(teleop) -----
    def _teleop_poll(self):
        pg = self.pygame
        keys = pg.key.get_pressed()
        # 화살표 또는 WASD(이동) — 화살표가 OS/포커스에 막히는 경우 대비 이중 매핑.
        up    = keys[pg.K_UP]    or keys[pg.K_w]
        down  = keys[pg.K_DOWN]  or keys[pg.K_s]
        left  = keys[pg.K_LEFT]
        right = keys[pg.K_RIGHT]
        vx = (up - down) * self.speed                       # +전방
        vy = (left - right) * self.speed                    # +좌측(메카넘)
        wz = (keys[pg.K_a] - keys[pg.K_d]) * self.yaw_rate  # +CCW(좌회전)
        if keys[pg.K_SPACE]:
            vx = vy = wz = 0.0
        mag = math.hypot(vx, vy)                            # 대각선은 √2배 → speed로 정규화
        if mag > self.speed:
            vx *= self.speed / mag
            vy *= self.speed / mag
        self.cur_vx, self.cur_vy, self.cur_wz = vx, vy, wz
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub_teleop.publish(msg)

    def _teleop_mode_key(self, key):
        pg = self.pygame
        num = {pg.K_0: 0, pg.K_1: 1, pg.K_2: 2, pg.K_3: 3, pg.K_4: 4, pg.K_5: 5}
        if key in num:
            self.pub_mode_out.publish(Int32(data=num[key]))
            self.last_key_mode = num[key]
            self.get_logger().info(f"mode -> {num[key]} {MODE_NAMES.get(num[key], '')}")
            self.mode_select = False
        elif key in (pg.K_m, pg.K_ESCAPE):
            self.mode_select = False     # 취소

    def _draw_mode_overlay(self):
        pg = self.pygame
        scr = self.screen
        w, h = scr.get_size()
        box = pg.Surface((360, 232))
        box.set_alpha(235)
        box.fill((18, 18, 22))
        scr.blit(box, (w // 2 - 180, h // 2 - 116))
        x = w // 2 - 160
        y = h // 2 - 100
        scr.blit(self.kfont.render("Mode select - press number", True, (90, 200, 140)), (x, y))
        y += 34
        for k, name in MODE_NAMES.items():
            scr.blit(self.kfont.render(f"  {k}.  {name}", True, (210, 210, 210)), (x, y))
            y += 26
        scr.blit(self.kfont.render("M / Esc = cancel", True, (130, 130, 130)), (x, y + 6))

    def _render_console(self):
        s = self.snap
        if s.get("state") == "MENU":
            items = " | ".join(f"{i['gesture']}:{i['label']}" for i in s.get("items", []))
            print(f"[UI] {' > '.join(s.get('path', []))}  {items}  "
                  f"hold={s.get('hold_gesture')}({s.get('hold_progress', 0):.0%})")


def main(args=None):
    import signal
    rclpy.init(args=args)
    node = UiNode()
    # pygame/SDL 이 SIGINT/SIGTERM 을 가로채 종료가 안 먹는 걸 덮어쓴다
    # (node 생성=pygame.init 이후에 등록해야 SDL 핸들러를 이긴다). stop 에 항상 정상 종료.
    _stop = {"v": False}
    def _sig(*_):
        _stop["v"] = True
    try:
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)
    except Exception:
        pass
    try:
        while rclpy.ok() and not _stop["v"] and not getattr(node, "_closing", False):
            rclpy.spin_once(node, timeout_sec=0.1)
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
