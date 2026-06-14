"""HUD 렌더링 — ROS 없는 순수 pygame 그리기 (ui_node.py / tools/ui_preview.py 공용).

화면 구성/디자인은 여기 한곳. ui_node는 ROS 토픽→데이터, preview는 키보드→데이터로
같은 Hud.draw()를 부른다. 그래서 Windows(ROS 없음)에서도 디자인을 그대로 확인 가능.

카드 상태 표현:
  - 단발(모드/카테고리/단발동작): 파란 테두리가 좌→우로 차오름 → 다 차면
    0.1초 흰색 반짝(_flash) 후 다음 화면으로.
  - 연속(jog: 거리/리프트/줌 등 stay 항목): 테두리가 차오른 뒤 명령이
    적용되는 순간(repeating)부터는 손을 뗄 때까지 카드를 계속 흰색으로.

게이지(작은 파란 바):
  - 따봉(IDLE) trigger_hold 진행
  - 거꾸로 따봉(MENU) back hold 진행

draw() 데이터:
  snapshot : 메뉴 상태기계 스냅샷(state/items/hold_gesture/hold_progress/repeating)
  mode/battery/recording/rec_start/frame
"""
import time

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE", 3: "FOLLOW2",
              4: "ORBIT", 5: "MODE 5"}
GESTURE_NUM = {"one": "1", "two": "2", "three": "3", "four": "4"}
# 방향 카드 글리프(권총 방향+자전). two(=V)는 휠에서 리셋 → 'V'.
GSYM = {"p_up": "\u2191", "p_down": "\u2193", "p_left": "\u2190", "p_right": "\u2192",
        "gun_left": "\u21bb", "gun_right": "\u21ba", "two": "V"}

# 테마 (흰 70 / 파랑 20 / 검정 10) — 테마 블루 #1e90ff
ACCENT = (30, 144, 255)      # 파랑 (테두리 차오름 + 게이지)
FILL = (30, 144, 255)        # = ACCENT (hold 차오름)
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
        try:   # 화살표/회전 글리프가 있는 시스템 폰트(잿슨 우분투 기본)
            self.f_sym = pygame.font.SysFont("dejavusans", 34, bold=True)
        except Exception:
            self.f_sym = pygame.font.Font(None, 36)
        self._prev_prog = 0.0
        self._prev_hold = ""
        self._prev_repeating = False
        self._last_rects = {}
        self._flash_until = 0.0
        self._flash_rect = None

    def draw(self, scr, snapshot, *, mode=0, battery=None,
             recording=False, rec_start=0.0, frame=None,
             oak_frame=None, split=False, zoom=1.0):
        w, h = scr.get_size()
        self._detect_confirm(snapshot)
        self._draw_video(scr, w, h, frame, oak_frame, split)
        self._draw_topbar(scr, w, mode, battery, recording, rec_start)
        if snapshot.get("state") == "MENU":
            self._draw_dock(scr, w, h, snapshot, recording, zoom)
        else:
            self._hint(scr, w, h, "thumbs-up to open")
            # 따봉 게이지
            if snapshot.get("hold_gesture") == "like":
                self._gauge(scr, w // 2, h - 44, float(snapshot.get("hold_progress", 0.0)))
        self._draw_flash(scr)

    # ----- 단발 확정 흰 반짝 (progress 하강 엣지, 연속 jog 제외) -----
    def _detect_confirm(self, snap):
        prog = float(snap.get("hold_progress", 0.0))
        hold = snap.get("hold_gesture", "")
        fired = (snap.get("state") in ("MENU", "IDLE")
                 and self._prev_prog >= 0.9 and not self._prev_repeating
                 and (prog < 0.5 or hold != self._prev_hold))
        # back(dislike)은 카드가 아니므로 흰반짝 대상에서 제외
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

    # ----- 배경 영상 (단일 또는 폰|OAK 이분할) -----
    def _draw_video(self, scr, w, h, frame, oak_frame=None, split=False):
        if not split:
            self._blit_cover(scr, 0, 0, w, h, frame, "CAMERA")
            return
        half = w // 2
        self._blit_cover(scr, 0, 0, half, h, frame, "PHONE")
        self._blit_cover(scr, half, 0, w - half, h, oak_frame, "OAK")
        self.pg.draw.line(scr, (10, 12, 16), (half, 0), (half, h), 2)
        self._tag(scr, 10, 44, "PHONE")
        self._tag(scr, half + 10, 44, "OAK")

    def _blit_cover(self, scr, x, y, bw, bh, frame, fallback):
        """frame(RGB ndarray)을 (x,y,bw,bh) 영역에 cover-스케일로 채운다.
        영역 밖으로 넘치지 않게 clip. frame=None이면 어두운 플레이스홀더."""
        pg = self.pg
        clip = pg.Rect(x, y, bw, bh)
        prev = scr.get_clip()
        scr.set_clip(clip)
        if frame is None:
            scr.fill((15, 17, 21), clip)
            s = self.f_small.render(fallback, True, (58, 63, 72))
            scr.blit(s, (x + (bw - s.get_width()) // 2, y + bh // 2 - 8))
            scr.set_clip(prev)
            return
        import numpy as np
        fh, fw, _ = frame.shape
        surf = pg.image.frombuffer(np.ascontiguousarray(frame).tobytes(), (fw, fh), "RGB")
        scale = max(bw / fw, bh / fh)
        surf = pg.transform.smoothscale(surf, (int(fw * scale), int(fh * scale)))
        sw, sh = surf.get_size()
        scr.blit(surf, (x + (bw - sw) // 2, y + (bh - sh) // 2))
        scr.set_clip(prev)

    def _tag(self, scr, x, y, text):
        pg = self.pg
        s = self.f_small.render(text, True, WHITE)
        bg = pg.Surface((s.get_width() + 12, s.get_height() + 4), pg.SRCALPHA)
        pg.draw.rect(bg, (*INK, 150), bg.get_rect(), border_radius=6)
        scr.blit(bg, (x - 6, y - 2))
        scr.blit(s, (x, y))

    # ----- 상단바 (우→좌 배치, 겹침 방지) -----
    def _draw_topbar(self, scr, w, mode, battery, recording, rec_start):
        pg = self.pg
        pg.draw.circle(scr, ACCENT, (26, 26), 5)
        scr.blit(self.f_mid.render(MODE_NAMES.get(mode, "?"), True, WHITE), (40, 13))
        x = w - 18
        batt = f"{battery}%" if battery is not None else "--"
        bs = self.f_small.render(batt, True, DIM)
        scr.blit(bs, (x - bs.get_width(), 16))
        x -= bs.get_width() + 8
        bw, bh = 22, 12
        bx, by = x - bw, 16
        pg.draw.rect(scr, DIM, (bx, by, bw, bh), 1, border_radius=2)
        pg.draw.rect(scr, DIM, (bx + bw, by + 3, 2, bh - 6))
        if battery is not None:
            col = REC_RED if battery <= 20 else WHITE
            pg.draw.rect(scr, col, (bx + 1, by + 1, max(1, int((bw - 2) * battery / 100.0)), bh - 2))
        x = bx - 24
        if recording:
            el = int(time.time() - rec_start)
            rs = self.f_small.render(f"REC {el // 60:02d}:{el % 60:02d}", True, REC_RED)
            scr.blit(rs, (x - rs.get_width(), 16))
            pg.draw.circle(scr, REC_RED, (x - rs.get_width() - 11, 23), 4)

    # ----- 작은 파란 게이지 (따봉/거꾸로따봉 공용) -----
    def _gauge(self, scr, cx, cy, frac):
        if frac <= 0:
            return
        pg = self.pg
        bw, bh = 160, 6
        x, y = cx - bw // 2, cy
        pg.draw.rect(scr, BASE_BORDER, (x, y, bw, bh), border_radius=3)
        pg.draw.rect(scr, ACCENT, (x, y, int(bw * min(1.0, frac)), bh), border_radius=3)

    # ----- 하단 메뉴 독 -----
    def _draw_dock(self, scr, w, h, snap, recording=False, zoom=1.0):
        pg = self.pg
        items = snap.get("items", [])
        if not items:
            return
        hold_g = snap.get("hold_gesture", "")
        prog = float(snap.get("hold_progress", 0.0))
        repeating = bool(snap.get("repeating", False))
        oak_on = bool(snap.get("ui_flags", {}).get("oak_view", False))
        # 방향 메뉴(Wheel/Lift)면 글리프 카드, 아니면 숫자+라벨 카드
        directional = any(it["gesture"].startswith("p_") for it in items)
        n = len(items)
        cw, gap, ch = (92, 10, 78) if directional else (150, 12, 64)
        x0 = (w - (n * cw + (n - 1) * gap)) // 2
        y0 = h - ch - 34
        self._last_rects = {}
        for i, it in enumerate(items):
            rect = pg.Rect(x0 + i * (cw + gap), y0, cw, ch)
            self._last_rects[it["gesture"]] = rect
            active = (it["gesture"] == hold_g and prog > 0)
            label = it["label"]
            if label == "Rec":
                label = "Rec OFF" if recording else "Rec ON"
            elif label == "OAK view":
                label = "OAK OFF" if oak_on else "OAK ON"
            if active and repeating:
                self._panel(scr, rect, (255, 255, 255), 245); col = INK
            else:
                self._panel(scr, rect, (255, 255, 255), 22)
                self._border(scr, rect, BASE_BORDER, 255, 2)
                if active:
                    self._border_fill_lr(scr, rect, prog, FILL, 255, 3)
                col = WHITE if active else DIM
            if directional:
                self._dir_card(scr, rect, it["gesture"], label, col)
            else:
                self._card_text(scr, rect, {"gesture": it["gesture"], "label": label}, col, col)
            if active and it["label"].startswith("Zoom"):
                self._zoom_badge(scr, rect, zoom)
        if hold_g == "dislike":
            self._gauge(scr, w // 2, y0 - 16, prog)
        self._hint(scr, w, h, "reverse thumbs-up to go back")

    def _dir_card(self, scr, rect, gesture, label, col):
        g = self.f_sym.render(GSYM.get(gesture, "?"), True, col)
        scr.blit(g, (rect.x + (rect.w - g.get_width()) // 2, rect.y + 8))
        s = self.f_small.render(label, True, col)
        scr.blit(s, (rect.x + (rect.w - s.get_width()) // 2, rect.y + rect.h - 24))

    # ----- 헬퍼 -----
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
        """테두리를 좌→우로 채운다: 전체 테두리 surface를 좌측 frac 만큼만
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

    def _zoom_badge(self, scr, rect, zoom):
        """활성 Zoom 카드 위에 '손 떼면 적용될' 목표 배율(xN)을 작은 파란 알약으로 표시."""
        pg = self.pg
        s = self.f_small.render(f"x{zoom:.1f}", True, WHITE)
        pad = 8
        bw, bh = s.get_width() + pad * 2, s.get_height() + 4
        bx = rect.x + (rect.w - bw) // 2
        by = rect.y - bh - 4
        bgs = pg.Surface((bw, bh), pg.SRCALPHA)
        pg.draw.rect(bgs, (*ACCENT, 220), bgs.get_rect(), border_radius=8)
        scr.blit(bgs, (bx, by))
        scr.blit(s, (bx + pad, by + 2))

    def _hint(self, scr, w, h, text):
        s = self.f_small.render(text, True, HINT)
        scr.blit(s, ((w - s.get_width()) // 2, h - 24))

    def _center(self, scr, text, font, color, y):
        s = font.render(text, True, color)
        scr.blit(s, ((scr.get_width() - s.get_width()) // 2, y))
