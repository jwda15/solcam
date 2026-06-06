"""LCD UI 노드 — 7" HDMI(1024x600, Jetson 연결).

디자인 (v3, 흰+파랑 테마):
  - 배경: 촬영 영상 풀스크린 (폰 /phone/image, 없으면 OAK /oak/rgb/image_raw).
  - 좌상단: 현재 모드 인디케이터 (파란 점 + 이름).
  - 우상단: REC 경과시간 + 배터리.
  - 메뉴 열림(따봉): 하단 독에 카드 4장. 선택 중인 카드는 보라색이
    왼→오로 차오르며(hold 진행) 표시. 확정 순간 0.1초 흰색 플래시 후 전환.
    영상은 계속 보임.

토픽:
  구독 /gesture_ui      (std_msgs/String, JSON)    메뉴 상태 (gesture_node 발행)
       /control_mode    (std_msgs/Int32)           현재 모드 표시
       /phone/image     (sensor_msgs/Image)        촬영 영상 (없으면 폴백)
       /oak/rgb/image_raw(sensor_msgs/Image)       폴백 영상
       /phone/battery   (sensor_msgs/BatteryState) 배터리 % (없으면 '--')
       /phone/recording (std_msgs/Bool)            녹화 중 여부 (REC 표시)

pygame이 없거나 디스플레이가 없으면 콘솔 출력으로 폴백한다(개발 PC).
폰 토픽(/phone/*)은 아직 미연동 — 자리만. 없으면 OAK 영상 + 배터리 '--'.
"""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Image, BatteryState

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE", 3: "MODE 3",
              4: "MODE 4", 5: "MODE 5"}
GESTURE_NUM = {"one": "1", "two": "2", "three": "3", "four": "4"}

# 테마 색 (흰 70 / 파랑 20 / 검정 10)
ACCENT = (74, 144, 226)      # 파랑 (평상 카드 테두리)
FILL = (127, 119, 221)       # 보라 (hold 차오름)
WHITE = (244, 246, 248)
DIM = (174, 180, 189)
INK = (26, 29, 34)           # 흰 플래시 위 글씨(검정)
REC_RED = (255, 90, 90)
FLASH_SEC = 0.1              # 확정 시 흰 플래시 지속


class UiNode(Node):
    def __init__(self):
        super().__init__("gesture_ui_node")
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("width", 1024)
        self.declare_parameter("height", 600)
        self.declare_parameter("video_topic", "/phone/image")
        self.declare_parameter("video_fallback", "/oak/rgb/image_raw")

        self.snap = {"state": "IDLE", "path": [], "items": [],
                     "hold_gesture": "", "hold_progress": 0.0, "last_action": ""}
        self.mode = 0
        self.battery = None          # 0~100 (%) 또는 None
        self.recording = False
        self.rec_start = 0.0
        self.frame = None            # 최신 영상 (H,W,3 RGB ndarray)
        # 확정 플래시 (진행도 하강 엣지로 감지; menu.py 불변)
        self._prev_prog = 0.0
        self._prev_hold = ""
        self._last_rects = {}        # gesture → 직전 프레임 카드 Rect
        self._flash_until = 0.0
        self._flash_rect = None

        self.create_subscription(String, "/gesture_ui", self._ui_cb, 10)
        self.create_subscription(Int32, "/control_mode", self._mode_cb, 10)
        self.create_subscription(BatteryState, "/phone/battery", self._batt_cb, 10)
        self.create_subscription(Bool, "/phone/recording", self._rec_cb, 10)
        self.create_subscription(
            Image, str(self.get_parameter("video_topic").value), self._img_cb, 1)
        self.create_subscription(
            Image, str(self.get_parameter("video_fallback").value), self._img_cb, 1)

        try:
            import pygame
            self.pygame = pygame
            pygame.init()
            size = (int(self.get_parameter("width").value),
                    int(self.get_parameter("height").value))
            flags = pygame.FULLSCREEN if self.get_parameter("fullscreen").value else 0
            self.screen = pygame.display.set_mode(size, flags)
            pygame.mouse.set_visible(False)
            self.f_big = pygame.font.Font(None, 40)
            self.f_mid = pygame.font.Font(None, 30)
            self.f_small = pygame.font.Font(None, 24)
            self.create_timer(1.0 / 30.0, self._render)
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
            self.rec_start = time.time()      # 녹화 시작 시각 기록
        self.recording = bool(msg.data)

    def _img_cb(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width, :]
        except ValueError:
            return
        self.frame = img[:, :, ::-1] if msg.encoding == "bgr8" else img

    # ----- 렌더 -----
    def _render(self):
        pg = self.pygame
        for e in pg.event.get():
            if e.type == pg.QUIT or (e.type == pg.KEYDOWN and e.key == pg.K_ESCAPE):
                rclpy.shutdown()
                return
        scr = self.screen
        w, h = scr.get_size()
        self._detect_confirm()                 # 진행도 하강 엣지 -> 플래시 예약
        self._draw_video(scr, w, h)
        self._draw_topbar(scr, w)
        if self.snap.get("state") == "MENU":
            self._draw_dock(scr, w, h)
        else:
            self._hint(scr, w, h, "thumbs-up to open")
        self._draw_flash(scr)                   # 확정 흰 플래시 (독 위에)
        pg.display.flip()

    def _detect_confirm(self):
        """hold_progress가 1 근처까지 찼다가 떨어지면(=확정) 그 카드에 플래시."""
        prog = float(self.snap.get("hold_progress", 0.0))
        hold = self.snap.get("hold_gesture", "")
        fired = self._prev_prog >= 0.9 and (prog < 0.5 or hold != self._prev_hold)
        if fired and self._prev_hold in self._last_rects:
            self._flash_rect = self._last_rects[self._prev_hold]
            self._flash_until = time.time() + FLASH_SEC
        self._prev_prog = prog
        self._prev_hold = hold

    def _draw_flash(self, scr):
        if self._flash_rect is None or time.time() >= self._flash_until:
            return
        self._panel(scr, self._flash_rect, (255, 255, 255), 240)

    def _draw_video(self, scr, w, h):
        pg = self.pygame
        if self.frame is None:
            scr.fill((15, 17, 21))                 # 영상 없을 때 어두운 배경
            self._center(scr, "CAMERA", self.f_small, (58, 63, 72), h // 2)
            return
        fh, fw, _ = self.frame.shape
        surf = pg.image.frombuffer(np.ascontiguousarray(self.frame).tobytes(),
                                   (fw, fh), "RGB")
        scale = max(w / fw, h / fh)               # 비율 유지 cover
        surf = pg.transform.smoothscale(surf, (int(fw * scale), int(fh * scale)))
        sw, sh = surf.get_size()
        scr.blit(surf, ((w - sw) // 2, (h - sh) // 2))

    def _draw_topbar(self, scr, w):
        pg = self.pygame
        pg.draw.circle(scr, ACCENT, (26, 26), 5)
        scr.blit(self.f_mid.render(MODE_NAMES.get(self.mode, "?"), True, WHITE), (40, 13))

        x = w - 18
        batt = f"{self.battery}%" if self.battery is not None else "--"
        bs = self.f_small.render(batt, True, DIM)
        scr.blit(bs, (x - bs.get_width(), 16))
        x -= bs.get_width() + 8
        bx, by, bw, bh = x - 26, 16, 22, 12
        pg.draw.rect(scr, DIM, (bx, by, bw, bh), 1, border_radius=2)
        pg.draw.rect(scr, DIM, (bx + bw, by + 3, 2, bh - 6))
        if self.battery is not None:
            fillw = max(1, int((bw - 2) * self.battery / 100.0))
            pg.draw.rect(scr, DIM, (bx + 1, by + 1, fillw, bh - 2))
        x = bx - 14
        if self.recording:
            elapsed = int(time.time() - self.rec_start)
            txt = f"R {elapsed // 60:02d}:{elapsed % 60:02d}"
            rs = self.f_small.render(txt, True, REC_RED)
            scr.blit(rs, (x - rs.get_width(), 16))

    def _draw_dock(self, scr, w, h):
        pg = self.pygame
        items = self.snap.get("items", [])
        if not items:
            return
        n = len(items)
        cw, gap, ch = 150, 12, 64
        total = n * cw + (n - 1) * gap
        x0 = (w - total) // 2
        y0 = h - ch - 34
        hold_g = self.snap.get("hold_gesture", "")
        prog = float(self.snap.get("hold_progress", 0.0))
        self._last_rects = {}
        for i, it in enumerate(items):
            x = x0 + i * (cw + gap)
            rect = pg.Rect(x, y0, cw, ch)
            self._last_rects[it["gesture"]] = rect
            active = (it["gesture"] == hold_g and prog > 0)
            self._panel(scr, rect, (255, 255, 255), 26)        # 반투명 베이스
            if active:
                self._fill_lr(scr, rect, prog, FILL, 210)      # 보라 왼->오 차오름
            self._border(scr, rect, ACCENT, 200 if active else 110, 1)
            self._card_text(scr, rect, it, WHITE if active else DIM, WHITE)
        self._hint(scr, w, h, "palm to close")

    def _panel(self, scr, rect, color, alpha):
        pg = self.pygame
        s = pg.Surface((rect.w, rect.h), pg.SRCALPHA)
        pg.draw.rect(s, (*color, alpha), s.get_rect(), border_radius=12)
        scr.blit(s, rect.topleft)

    def _fill_lr(self, scr, rect, frac, color, alpha):
        """둥근 카드 안을 왼->오로 frac(0~1)만큼 보라색으로 채운다."""
        pg = self.pygame
        frac = max(0.0, min(1.0, frac))
        s = pg.Surface((rect.w, rect.h), pg.SRCALPHA)
        pg.draw.rect(s, (*color, alpha), s.get_rect(), border_radius=12)
        scr.blit(s, rect.topleft, area=pg.Rect(0, 0, int(rect.w * frac), rect.h))

    def _border(self, scr, rect, color, alpha, width):
        pg = self.pygame
        s = pg.Surface((rect.w, rect.h), pg.SRCALPHA)
        pg.draw.rect(s, (*color, alpha), s.get_rect(), width, border_radius=12)
        scr.blit(s, rect.topleft)

    def _card_text(self, scr, rect, it, numcol, txtcol):
        num = self.f_mid.render(GESTURE_NUM.get(it["gesture"], "?"), True, numcol)
        label = self.f_mid.render(it["label"], True, txtcol)
        total = num.get_width() + 7 + label.get_width()
        cx = rect.x + (rect.w - total) // 2
        cy = rect.y + (rect.h - num.get_height()) // 2
        scr.blit(num, (cx, cy))
        scr.blit(label, (cx + num.get_width() + 7, cy))

    def _hint(self, scr, w, h, text):
        s = self.f_small.render(text, True, (123, 129, 139))
        scr.blit(s, ((w - s.get_width()) // 2, h - 24))

    def _center(self, scr, text, font, color, y):
        s = font.render(text, True, color)
        scr.blit(s, ((scr.get_width() - s.get_width()) // 2, y))

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
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
