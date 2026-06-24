"""LCD UI 노드 — 7" HDMI(1024x600, Jetson 연결).

ROS 토픽을 받아 Hud(hud.py)로 렌더한다. 디자인/그리기는 전부 hud.py에 있고
(ROS 무관), 이 노드는 "토픽 → 데이터" 어댑터다. 같은 Hud를 Windows 프리뷰
(tools/ui_preview.py)도 써서 디자인이 어긋나지 않는다.

토픽:
  /gesture_ui (String JSON)  메뉴 상태 (gesture_node)
  /control_mode (Int32)      현재 모드
  /phone/image (Image)       촬영 영상 (없으면 /oak/rgb/image_raw 폴백)
  /phone/battery (BatteryState)  배터리 %
  /phone/recording (Bool)    녹화 중 여부 (phone_bridge 측 상태; 보조)
  /phone_cmd (String)        "record_toggle" 면 녹화 on/off (phone 유무와 무관)

녹화(REC): REC ON 동안 "지금 화면에 나오는 메인 영상"(폰 영상, 없으면 OAK-D)을
  output_dir(기본 <SOLCAM_REPO 또는 ~/solcam>/Output)에 mp4 로 저장한다. 폰 미연결
  이어도 OAK-D 영상으로 녹화되도록 ui_node 가 직접 기록한다(cv2.VideoWriter).
  phone_bridge 가 따로 폰 v4l2 를 녹화/전송하는 건 별개 — 여긴 로컬 백업본.

pygame/디스플레이 없으면 콘솔 폴백. /phone/* 는 ros2_phone_bridge(scrcpy/adb)가 발행.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String, Float32, Empty
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist

from ros2_control_node.msg import ControlDebug

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
        # 녹화 저장 폴더. 빈값이면 (SOLCAM_REPO 또는 ~/solcam)/Output 로 자동 해석.
        self.declare_parameter("output_dir", "")
        self.declare_parameter("rec_fps", 20.0)   # 저장 영상 fps (렌더 30Hz 중 이 간격으로 기록)

        self.snap = {"state": "IDLE", "items": [], "hold_gesture": "",
                     "hold_progress": 0.0}
        self.mode = 0
        self.battery = None
        self.recording = False
        self.rec_start = 0.0
        self.phone_frame = None    # /phone/image (촬영 카메라)
        self.oak_frame = None      # /oak/rgb/image_raw (손동작 인식 카메라)
        self.phone_zoom = 1.0      # /phone/zoom (현재/목표 줌 배율)

        # ----- 녹화(REC) → 파일 저장 상태 -----
        self.output_dir = self._resolve_output_dir(
            str(self.get_parameter("output_dir").value))
        self.rec_fps = max(1.0, float(self.get_parameter("rec_fps").value))
        self._writer = None        # cv2.VideoWriter (lazy: 첫 프레임에서 해상도 확정 후 open)
        self._writer_size = None   # (w, h) — 녹화 중 해상도 고정(소스 바뀌면 resize)
        self._rec_path = None
        self._rec_last_write = 0.0 # 마지막 프레임 기록 시각(fps 게이트)

        # ----- 튜닝 표시(좌하단) / OAK 0점 플래시 상태 -----
        self._have_debug = False
        self.cur_dist = self.cur_az = 0.0   # 현재 주인 거리[m]/방위각[rad]
        self.tgt_dist = self.tgt_az = 0.0   # 타겟 거리[m]/방위각[rad]
        self._yaw_flash_until = 0.0         # 하단 흰 선 플래시 종료시각(0점 지정)
        self._yaw_limit_until = 0.0         # 하단 빨간 선 종료시각(케이블 한계 근접)

        self.create_subscription(String, "/gesture_ui", self._ui_cb, 10)
        self.create_subscription(Int32, "/control_mode", self._mode_cb, 10)
        self.create_subscription(BatteryState, "/phone/battery", self._batt_cb, 10)
        self.create_subscription(Bool, "/phone/recording", self._rec_cb, 10)
        self.create_subscription(String, "/phone_cmd", self._phone_cmd_cb, 10)
        self.create_subscription(Float32, "/phone/zoom", self._zoom_cb, 10)
        self.create_subscription(ControlDebug, "/control_debug", self._debug_cb, 10)
        self.create_subscription(Empty, "/yaw_set_zero", self._yaw_zero_cb, 10)
        self.create_subscription(Empty, "/yaw_limit_warn", self._yaw_limit_cb, 10)
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
        self.declare_parameter("yaw_rate", 0.4)    # rad/s, 회전 목표속도
                                                   #  ★0.8→0.4: 회전이 직진보다 빨라 진동 심해서 낮춤.
                                                   #   더 느리게=↓. (실차 한계는 control w_body_max 가 별도 클램프)
        self.teleop_on = bool(self.get_parameter("teleop").value)
        self.speed = float(self.get_parameter("speed").value)
        self.yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.pub_teleop = self.create_publisher(Twist, "/teleop_cmd", 10)
        self.pub_mode_out = self.create_publisher(Int32, "/control_mode", 10)
        # 긴급정지: 스페이스바 누르는 동안 전 모드에서 휠·상단yaw·리프트 정지(/estop).
        self.pub_estop = self.create_publisher(Bool, "/estop", 10)
        self._estop_state = False
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
        # phone_bridge 가 알려주는 녹화 상태(보조). _set_recording 은 멱등이라
        #  /phone_cmd 토글과 겹쳐도 안전(같은 상태면 no-op).
        self._set_recording(bool(msg.data))

    def _phone_cmd_cb(self, msg):
        # 제스처 "Rec" → /phone_cmd "record_toggle". 폰 유무와 무관하게 여기서
        #  직접 녹화를 토글한다(폰 없으면 OAK-D 가 저장 소스).
        if str(msg.data).strip() == "record_toggle":
            self._set_recording(not self.recording)

    def _zoom_cb(self, msg):
        self.phone_zoom = float(msg.data)

    def _debug_cb(self, msg):
        # control_node /control_debug → 좌하단 튜닝 표시값.
        self.cur_dist = float(msg.owner_distance)
        self.cur_az = float(msg.owner_azimuth)
        self.tgt_dist = float(msg.target_distance)
        self.tgt_az = float(msg.target_azimuth)
        self._have_debug = True

    def _yaw_zero_cb(self, _msg):
        # OAK 케이블 0점 지정 완료 → 하단 흰 선 0.3s 깜빡.
        self._yaw_flash_until = time.time() + 0.3

    def _yaw_limit_cb(self, _msg):
        # 상단yaw 케이블 한계 근접 → 하단 빨간 선 2s.
        self._yaw_limit_until = time.time() + 2.0

    # ----- 녹화(REC) → 파일 -----
    def _resolve_output_dir(self, param_val):
        if param_val:
            return os.path.expanduser(param_val)
        repo = os.environ.get("SOLCAM_REPO")
        base = repo if repo else os.path.expanduser("~/solcam")
        return os.path.join(base, "Output")

    def _set_recording(self, on):
        # 멱등: 현재와 같은 상태면 아무것도 안 함.
        if on == self.recording:
            return
        self.recording = on
        if on:
            self.rec_start = time.time()
            self._rec_last_write = 0.0
            self._writer = None        # 첫 프레임에서 lazy open(해상도 확정)
            self._writer_size = None
            self._rec_path = None
            self.get_logger().info("REC ON → 첫 프레임에서 파일 생성")
        else:
            self._close_writer()

    def _open_writer(self, w, h):
        try:
            import cv2
        except Exception as e:
            self.get_logger().warn(f"cv2 없음({e}) — 파일 녹화 비활성(HUD 표시만)")
            self._writer = False       # False=시도 실패 표시(매 프레임 재시도 방지)
            return
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._rec_path = os.path.join(self.output_dir, f"solcam_{ts}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            wr = cv2.VideoWriter(self._rec_path, fourcc, self.rec_fps, (w, h))
            if not wr.isOpened():
                raise RuntimeError("VideoWriter open 실패")
            self._writer = wr
            self._writer_size = (w, h)
            self.get_logger().info(f"녹화 시작 → {self._rec_path} ({w}x{h}@{self.rec_fps:.0f})")
        except Exception as e:
            self.get_logger().warn(f"녹화 파일 생성 실패({e}) — HUD 표시만")
            self._writer = False

    def _rec_tick(self, frame):
        # 렌더 루프에서 매 프레임 호출. recording 중이고 메인 프레임이 있으면
        #  rec_fps 간격으로 파일에 기록(소스 해상도 바뀌면 첫 해상도로 resize).
        if not self.recording or frame is None or self._writer is False:
            return
        now = time.time()
        if (now - self._rec_last_write) < (1.0 / self.rec_fps):
            return
        h, w = frame.shape[:2]
        if self._writer is None:
            self._open_writer(w, h)
            if not self._writer:       # None→False (실패) 또는 아직 미오픈
                return
        try:
            import cv2
            if (w, h) != self._writer_size:   # 소스 전환(폰↔OAK) 시 크기 맞춤
                frame = cv2.resize(frame, self._writer_size)
            # 표시 프레임은 RGB → VideoWriter 는 BGR 기대 → 채널 반전
            self._writer.write(np.ascontiguousarray(frame[:, :, ::-1]))
            self._rec_last_write = now
        except Exception as e:
            self.get_logger().warn(f"프레임 기록 실패({e}) — 녹화 중단")
            self._close_writer()

    def _close_writer(self):
        wr = self._writer
        self._writer = None
        self._writer_size = None
        if wr and wr is not False:
            try:
                wr.release()
                self.get_logger().info(f"녹화 저장 완료 → {self._rec_path}")
            except Exception as e:
                self.get_logger().warn(f"녹화 종료 실패({e})")

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
            # 거울모드: OAK-D 영상을 좌우반전해 표시(보는 사람 기준 자연스러움).
            #  hud._blit_cover 가 ascontiguousarray 로 다시 만드므로 view 로 충분.
            self.oak_frame = f[:, ::-1]

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
        # 긴급정지: 스페이스바 누르는 동안 전 모드 정지(/estop). 변할 때만 발행.
        self._poll_estop()
        # 키보드 수동주행: 매 프레임 키 상태 폴링 → /teleop_cmd 발행(대각선 동시키 지원)
        if self.teleop_on and not self.mode_select:
            self._teleop_poll()
        state = self.snap.get("state")
        oak_view = bool(self.snap.get("ui_flags", {}).get("oak_view", False))
        # 폰 영상이 없으면(미연결/nophone) 메뉴에서도 분할하지 않고 OAK 전체 배경.
        #  (검은 PHONE 반쪽 방지) — OAK view 토글은 명시적이라 그대로 분할.
        split = oak_view or (state == "MENU" and self.phone_frame is not None)
        bg = self.phone_frame if self.phone_frame is not None else self.oak_frame
        self._rec_tick(bg)   # REC ON 이면 메인 영상(폰 없으면 OAK)을 파일에 기록
        self.hud.draw(self.screen, self.snap, mode=self.mode, battery=self.battery,
                      recording=self.recording, rec_start=self.rec_start,
                      frame=(self.phone_frame if split else bg),
                      oak_frame=self.oak_frame, split=split, zoom=self.phone_zoom)
        if self.mode_select:
            self._draw_mode_overlay()
        # (키보드 안내/상태 오버레이는 거추장스러워 제거 — 키 입력은 그대로 동작)
        self._draw_tuning_text()         # 좌하단 현재/타겟 거리·방위각
        self._draw_yaw_flash()           # 하단 OAK 0점 지정 흰 선 플래시
        pg.display.flip()

    def _draw_tuning_text(self):
        # 좌하단: 현재 주인 거리/방위각(흰), 타겟 거리/방위각(파랑). 튜닝용.
        if not self._have_debug:
            return
        pg = self.pygame
        h = self.screen.get_size()[1]
        cur = "cur  d=%.2fm  az=%+.1f" % (self.cur_dist, math.degrees(self.cur_az))
        tgt = "tgt  d=%.2fm  az=%+.1f" % (self.tgt_dist, math.degrees(self.tgt_az))
        self.screen.blit(self.kfont.render(cur, True, (235, 235, 235)), (12, h - 46))
        self.screen.blit(self.kfont.render(tgt, True, (110, 170, 245)), (12, h - 24))

    def _draw_yaw_flash(self):
        # 하단 얇은 선: 빨강=케이블 한계 근접(2s, 우선) / 흰색=0점 지정(0.3s).
        pg = self.pygame
        w, h = self.screen.get_size()
        now = time.time()
        if now < self._yaw_limit_until:
            pg.draw.rect(self.screen, (255, 0, 0), (0, h - 4, w, 4))   # 빨강(RGB): 케이블 한계
        elif now < self._yaw_flash_until:
            pg.draw.rect(self.screen, (255, 255, 255), (0, h - 3, w, 3))

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
        self._close_writer()       # 녹화 중이면 파일 안전 마무리
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

    def _poll_estop(self):
        # 스페이스바 누르는 동안 True 발행 → control_node 가 전 모드에서 즉시 정지.
        #  (창 포커스 있을 때만 키 읽힘. 포커스 없으면 False=해제) 변할 때만 발행.
        pg = self.pygame
        focused = (not hasattr(pg.key, "get_focused")) or pg.key.get_focused()
        estop = bool(focused and pg.key.get_pressed()[pg.K_SPACE])
        if estop != self._estop_state:
            self._estop_state = estop
            self.pub_estop.publish(Bool(data=estop))

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
            node._close_writer()     # 녹화 파일 안전 마무리(이중 호출 안전)
        except Exception:
            pass
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
