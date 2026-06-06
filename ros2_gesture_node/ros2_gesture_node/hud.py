"""HUD 렌더링 — ROS 없는 순수 pygame 그리기 (ui_node.py / tools/ui_preview.py 공용).

화면 구성/디자인은 여기 한곳. ui_node는 ROS 토픽→데이터, preview는 키보드→데이터로
같은 Hud.draw()를 부른다. 그래서 Windows(ROS 없음)에서도 디자인을 그대로 확인 가능.

카드 상태 표현:
  - 단발(모드/카테고리/단발동작): 보라 테두리가 좌→우로 차오름 → 다 차면
    0.1초 흰색 반짝(_flash) 후 다음 화면으로.
  - 연속(jog: 거리/리프트/줌 등 stay 항목): 보라 테두리가 차오른 뒤 명령이
    적용되는 순간(repeating)부터는 손을 뗄 때까지 카드를 계속 흰색으로.

draw() 데이터:
  snapshot : 메뉴 상태기계 스냅샷(state/items/hold_gesture/hold_progress/repeating)
  mode/battery/recording/rec_start/frame
"""
import time

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE", 3: "MODE 3",
              4: "MODE 4", 5: "MODE 5"}
GESTURE_NUM = {"one": "1", "two": "2", "three": "3", "four": "4"}

# 테마 (흰 70 / 파랑 20 / 검정 10)
ACCENT = (74, 144, 226)      # 파랑 (따봉 게이지)
FILL = (127, 119, 221)       # 보라 (hold 차오름 테두리)
WHITE = (244, 246, 248)
INK = (18, 20, 24)           # 흰 카드 위 글자색
DIM = (174, 180, 189)
REC_RED = (255, 90, 90)
HINT = (123, 129, 139)
BASE_BORDER = (60, 70, 92)   # 비활성/기저 테두리
FLASH_SEC = 0.1              # 확정 흰 반짝 지속


class Hud:
    def __init__(self, pygame):
        self.pg = pygame
        self.f_mid = pygame.font.Font(None, 30)
        self.f_small = pygame.font.Font(None, 24)
        self._prev_prog = 0.0
        self._prev_hold = ""
        self._prev_repeating = False
        self._last_rects = {}
        self._flash_until = 0.0
        self._flash_rect = None

    def draw(self, scr, snapshot, *, mode=0, battery=None,
             recording=False, rec_start=0.0, frame=None):
        w, h = scr.get_size()
        self._detect_confirm(snapshot)
        self._draw_video(scr, w, h, frame)
        self._draw_topbar(scr, w, mode, battery, recording, rec_start)
        if snapshot.get("state") == "MENU":
            self._draw_dock(scr, w, h, snapshot)
        else:
            self._hint(scr, w, h, "thumbs-up to open")
            self._draw_trigger(scr, w, h, snapshot)
        self._draw_flash(scr)

    # ----- 확정(단발) 흰 반짝 감지: progress 하강 엣지, 단 연속(jog)은 제외 -----
    def _detect_confirm(self, snap):
        prog = float(snap.get("hold_progress", 0.0))
        hold = snap.get("hold_gesture", "")
        fired = (snap.get("state") in ("MENU", "IDLE")
                 and self._prev_prog >= 0.9 and not self._prev_repeating
                 and (prog < 0.5 or hold != self._prev_hold))
        if fired and self._prev_hold in self._last_rects:
            self._flash_rect = self._last_rects[self._prev_hold]
            self._flash_until = time.time() + FLASH_SEC
        self._prev_prog = prog
        self._prev_hold = hold
        self._prev_repeating = bool(snap.get("repeating", False))

    def _draw_flash(self, scr):
        if self._flash_rect is None or time.time() >= self._flash_until:
            return
        self._panel(scr, self._flash_rect, (255, 255, 255), 245)

    # ----- 배경 영상 -----
    def _draw_video(self, scr, w, h, frame):
        pg = self.pg
        if frame is None:
            scr.fill((15, 17, 21))
            self._center(scr, "CAMERA", self.f_small, (58, 63, 72), h // 2)
            return
        import numpy as np
        fh, fw, _ = frame.shape
        surf = pg.image.frombuffer(np.ascontiguousarray(frame).tobytes(), (fw, fh), "RGB")
        scale = max(w / fw, h / fh)
        surf = pg.transform.smoothscale(surf, (int(fw * scale), int(fh * scale)))
        sw, sh = surf.get_size()
        scr.blit(surf, ((w - sw) // 2, (h - sh) // 2))

    # ----- 상단바 (우측은 오른쪽→왼쪽으로 배치, 겹침 방지) -----
    def _draw_topbar(self, scr, w, mode, battery, recording, rec_start):
        pg = self.pg
        pg.draw.circle(scr, ACCENT, (26, 26), 5)
        scr.blit(self.f_mid.render(MODE_NAMES.get(mode, "?"), True, WHITE), (40, 13))
        x = w - 18
        # 배터리 % 텍스트
        batt = f"{battery}%" if battery is not None else "--"
        bs = self.f_small.render(batt, True, DIM)
        scr.blit(bs, (x - bs.get_width(), 16))
        x -= bs.get_width() + 8
        # 배터리 아이콘
        bw, bh = 22, 12
        bx, by = x - bw, 16
        pg.draw.rect(scr, DIM, (bx, by, bw, bh), 1, border_radius=2)
        pg.draw.rect(scr, DIM, (bx + bw, by + 3, 2, bh - 6))
        if battery is not None:
            col = REC_RED if battery <= 20 else WHITE
            pg.draw.rect(scr, col, (bx + 1, by + 1, max(1, int((bw - 2) * battery / 100.0)), bh - 2))
        x = bx - 24
        # 녹화: ● REC mm:ss
        if recording:
            el = int(time.time() - rec_start)
            rs = self.f_small.render(f"REC {el // 60:02d}:{el % 60:02d}", True, REC_RED)
            scr.blit(rs, (x - rs.get_width(), 16))
            pg.draw.circle(scr, REC_RED, (x - rs.get_width() - 11, 23), 4)

    # ----- 따봉 트리거 게이지 (작게, 파란색) -----
    def _draw_trigger(self, scr, w, h, snap):
        if snap.get("hold_gesture") != "like":
            return
        prog = float(snap.get("hold_progress", 0.0))
        if prog <= 0:
            return
        pg = self.pg
        bw, bh = 160, 6
        x, y = (w - bw) // 2, h - 44
        pg.draw.rect(scr, BASE_BORDER, (x, y, bw, bh), border_radius=3)
        pg.draw.rect(scr, ACCENT, (x, y, int(bw * min(1.0, prog)), bh), border_radius=3)

    # ----- 하단 메뉴 독 -----
    def _draw_dock(self, scr, w, h, snap):
        pg = self.pg
        items = snap.get("items", [])
        if not items:
            return
        n = len(items)
        cw, gap, ch = 150, 12, 64
        x0 = (w - (n * cw + (n - 1) * gap)) // 2
        y0 = h - ch - 34
        hold_g = snap.get("hold_gesture", "")
        prog = float(snap.get("hold_progress", 0.0))
        repeating = bool(snap.get("repeating", False))
        self._last_rects = {}
        for i, it in enumerate(items):
            rect = pg.Rect(x0 + i * (cw + gap), y0, cw, ch)
            self._last_rects[it["gesture"]] = rect
            active = (it["gesture"] == hold_g and prog > 0)
            if active and repeating:
                # 연속(jog) 적용 중 → 카드를 계속 흰색으로
                self._panel(scr, rect, (255, 255, 255), 245)
                self._card_text(scr, rect, it, INK, INK)
            else:
                self._panel(scr, rect, (255, 255, 255), 22)        # 옅은 배경
                self._border(scr, rect, BASE_BORDER, 255, 2)        # 기저 테두리
                if active:
                    self._border_fill_lr(scr, rect, prog, FILL, 240, 3)  # 보라 좌→우
                self._card_text(scr, rect, it, WHITE if active else DIM, WHITE)
        self._hint(scr, w, h, "reverse thumbs-up to go back")

    # ----- 작은 그리기 헬퍼 -----
    def _panel(self, scr, rect, color, alpha):
        pg = self.pg
        s = pg.Surface((rect.w, rect.h), pg.SRCALPHA)
        pg.draw.rect(s, (*color, alpha), s.get_rect(), border_radius=12)
        scr.blit(s, rect.topleft)

    def _border(self, scr, rect, color, alpha, width):
        pg = self.pg
        s = pg.Surface((rect.w, rect.h), pg.SRCALPHA)
        pg.draw.rect(s, (*color, alpha), s.get_rect(), width, border_radius=12)
        scr.blit(s, rect.topleft)

    def _border_fill_lr(self, scr, rect, frac, color, alpha, width):
        """테두리를 좌→우로 채운다: 전체 테두리 surface를 좌측 frac 비율만큼
        잘라 덮어, 윗/아랫변이 왼쪽부터 차오르는 연출."""
        pg = self.pg
        frac = max(0.0, min(1.0, frac))
        s = pg.Surface((rect.w, rect.h), pg.SRCALPHA)
        pg.draw.rect(s, (*color, alpha), s.get_rect(), width, border_radius=12)
        scr.blit(s, rect.topleft, area=pg.Rect(0, 0, int(rect.w * frac), rect.h))

    def _card_text(self, scr, rect, it, numcol, txtcol):
        num = self.f_mid.render(GESTURE_NUM.get(it["gesture"], "?"), True, numcol)
        label = self.f_mid.render(it["label"], True, txtcol)
        total = num.get_width() + 7 + label.get_width()
        cx = rect.x + (rect.w - total) // 2
        cy = rect.y + (rect.h - num.get_height()) // 2
        scr.blit(num, (cx, cy))
        scr.blit(label, (cx + num.get_width() + 7, cy))

    def _hint(self, scr, w, h, text):
        s = self.f_small.render(text, True, HINT)
        scr.blit(s, ((w - s.get_width()) // 2, h - 24))

    def _center(self, scr, text, font, color, y):
        s = font.render(text, True, color)
        scr.blit(s, ((scr.get_width() - s.get_width()) // 2, y))
