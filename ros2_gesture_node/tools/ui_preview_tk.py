"""LCD UI 프리뷰 (tkinter) — 설치/ROS 불필요. Windows에서 바로 실행.

pygame이 안 깔리는 환경(예: Python 3.14처럼 최신이라 pygame wheel 미배포)에서도
UI를 확인하려고 만든 버전. tkinter는 Windows Python에 기본 포함이라
'pip install' 자체가 필요 없다.

실제 메뉴 상태기계(menu.py)를 그대로 import해서 동작/타이밍은 로봇과 동일하고,
그리기만 tkinter Canvas로 다시 그린다(테마/레이아웃은 hud.py와 동일하게 맞춤).

조작(키보드로 손동작 흉내):
  L=따봉(메뉴 열기, 1.5s 게이지)   1~4=항목 선택(꾹)   K=거꾸로 따봉(뒤로/닫기)
  R=녹화 토글   B=배터리--   ESC=종료
  ※ 모터 항목(Wheel/Lift 등)은 1.5s 후 키를 누른 채로 있으면 연속(카드 흰색).
  ※ 단발 항목은 테두리가 다 차면 0.1초 흰색 반짝 후 넘어간다.
"""
import sys, time, tkinter as tk
import tkinter.font as tkfont
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_gesture_node.menu import MenuStateMachine, build_menu

# ----- 테마 (hud.py와 동일) -----
BG      = "#0d0f12"
CARD    = "#1a1e24"
BASE    = "#3c465c"   # 기저 테두리
ACCENT  = "#1e90ff"   # 테마 블루(게이지+테두리)
FILL    = "#1e90ff"   # = ACCENT (테두리 차오름)
WHITE   = "#f4f6f8"
INK     = "#12141a"   # 흰 카드 위 글자색
DIM     = "#aeb4bd"
REC_RED = "#ff5a5a"
FLASH_SEC = 0.1

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE"}
KEY2GEST = {"l": "like", "1": "one", "2": "two", "3": "three", "4": "four",
            "k": "dislike"}
GNUM = {"one": "1", "two": "2", "three": "3", "four": "4"}

W, H = 1024, 600


class Preview:
    def __init__(self):
        adj = {"dist_step": 0.3, "heading_step_deg": 15, "lift_step": 0.05}
        self.sm = MenuStateMachine(build_menu(adj))
        self.mode = 1
        self.battery = 87
        self.recording = True
        self.rec_start = time.time() - 134
        self.t0 = time.time()

        self.held = None
        self._rel_job = None
        self._cards = {}                 # gesture -> (x,y,w,h)  (직전 프레임)
        self._prev_prog = 0.0
        self._prev_hold = ""
        self._prev_repeating = False
        self._flash_until = 0.0
        self._flash_rect = None

        self.win = tk.Tk()
        self.win.title("solcam LCD preview (tkinter)")
        self.win.configure(bg=BG)
        self.cv = tk.Canvas(self.win, width=W, height=H, bg=BG, highlightthickness=0)
        self.cv.pack()
        self.f_rec = tkfont.Font(family="Segoe UI", size=13, weight="bold")
        self.win.bind("<KeyPress>", self._press)
        self.win.bind("<KeyRelease>", self._release)
        self._loop()

    # ---- 입력 (자동반복 디바운스) ----
    def _press(self, e):
        k = e.keysym.lower()
        if k == "escape": self.win.destroy(); return
        if k == "r": self._toggle_rec(); return
        if k == "b": self.battery = max(0, self.battery - 5); return
        g = KEY2GEST.get(k)
        if g is None: return
        if self._rel_job: self.win.after_cancel(self._rel_job); self._rel_job = None
        self.held = g

    def _release(self, e):
        if KEY2GEST.get(e.keysym.lower()) is None: return
        if self._rel_job: self.win.after_cancel(self._rel_job)
        self._rel_job = self.win.after(80, self._clear_held)

    def _clear_held(self): self.held = None; self._rel_job = None

    def _toggle_rec(self):
        self.recording = not self.recording
        if self.recording: self.rec_start = time.time()

    # ---- 메인 루프 (~30fps) ----
    def _loop(self):
        t = time.time() - self.t0
        for ev in self.sm.update(self.held, t):
            if ev.kind == "action" and ev.action and ev.action.kind == "mode":
                self.mode = ev.action.payload.get("mode", self.mode)
            elif (ev.kind == "action" and ev.action and ev.action.kind == "phone"
                  and ev.action.payload.get("cmd") == "record_toggle"):
                self._toggle_rec()   # 프리뷰: 폰 대신 직접 토글(라벨 ON/OFF 확인용)
        self._draw(self.sm.snapshot())
        self.win.after(33, self._loop)

    # ---- 그리기 ----
    def _rrect(self, x, y, w, h, r, **kw):
        self.cv.create_polygon(
            x+r,y, x+w-r,y, x+w,y, x+w,y+r, x+w,y+h-r, x+w,y+h,
            x+w-r,y+h, x+r,y+h, x,y+h, x,y+h-r, x,y+r, x,y,
            smooth=True, **kw)

    def _draw(self, snap):
        c = self.cv; c.delete("all")
        c.create_rectangle(0, 0, W, H, fill=BG, outline="")
        c.create_text(W//2, H//2, text="CAMERA", fill="#2a2f37",
                      font=("Segoe UI", 30, "bold"))
        self._detect_confirm(snap)        # 직전 프레임 _cards 사용
        self._topbar()
        if snap.get("state") == "MENU":
            self._dock(snap)              # _cards 갱신
        else:
            c.create_text(W//2, H-58, text="show LIKE to open menu  (press L)",
                          fill=DIM, font=("Segoe UI", 12))
            if snap.get("hold_gesture") == "like":
                self._gauge(W//2, H-40, snap.get("hold_progress", 0.0))
        self._draw_flash()

    # 단발 확정 흰 반짝 감지 (연속 jog는 제외)
    def _detect_confirm(self, snap):
        prog = snap.get("hold_progress", 0.0)
        hold = snap.get("hold_gesture", "")
        fired = (self._prev_prog >= 0.999 and not self._prev_repeating
                 and (prog < 0.5 or hold != self._prev_hold))
        if fired and self._prev_hold in self._cards:
            self._flash_rect = self._cards[self._prev_hold]
            self._flash_until = time.time() + FLASH_SEC
        self._prev_prog = prog
        self._prev_hold = hold
        self._prev_repeating = bool(snap.get("repeating", False))

    def _draw_flash(self):
        if self._flash_rect is None or time.time() >= self._flash_until:
            return
        x, y, w, h = self._flash_rect
        self._rrect(x, y, w, h, 12, fill=WHITE, outline="")

    def _topbar(self):
        c = self.cv
        c.create_oval(20, 18, 32, 30, fill=ACCENT, outline="")
        c.create_text(40, 24, text=MODE_NAMES.get(self.mode, "?"), anchor="w",
                      fill=WHITE, font=("Segoe UI", 16, "bold"))
        x = W - 20
        c.create_text(x, 24, text=f"{self.battery}%", anchor="e",
                      fill=WHITE, font=("Segoe UI", 12))
        x -= 40
        bw, bh = 28, 14
        bx0, by = x - bw, 17
        c.create_rectangle(bx0, by, x, by+bh, outline=DIM)
        c.create_rectangle(x, by+4, x+3, by+bh-4, outline=DIM, fill=DIM)
        fillw = int((bw-2) * self.battery / 100)
        col = REC_RED if self.battery <= 20 else WHITE
        c.create_rectangle(bx0+1, by+1, bx0+1+fillw, by+bh-1, fill=col, outline="")
        x = bx0 - 26
        if self.recording:
            secs = int(time.time() - self.rec_start)
            txt = f"REC {secs//60:02d}:{secs%60:02d}"
            c.create_text(x, 24, text=txt, anchor="e",
                          fill=REC_RED, font=self.f_rec)
            dot_x = x - self.f_rec.measure(txt) - 11
            c.create_oval(dot_x-4, 20, dot_x+4, 28, fill=REC_RED, outline="")

    def _gauge(self, cx, cy, frac):
        """작은 파란 게이지 (따봉/거꾸로따봉 공용)."""
        if frac <= 0: return
        c = self.cv
        bw, bh = 160, 6
        x, y = cx - bw//2, cy
        self._rrect(x, y, bw, bh, 3, fill=BASE, outline="")
        c.create_rectangle(x, y, x+max(1, int(bw*min(1.0, frac))), y+bh,
                           fill=ACCENT, outline="")

    def _dock(self, snap):
        c = self.cv
        items = snap.get("items", [])
        if not items: return
        n = len(items)
        cw, ch, gap = 150, 64, 14
        x0 = (W - (n*cw + (n-1)*gap))//2
        y = H - ch - 28
        prog = snap.get("hold_progress", 0.0)
        hold_g = snap.get("hold_gesture", "")
        repeating = bool(snap.get("repeating", False))

        self._cards = {}
        for i, it in enumerate(items):
            x = x0 + i*(cw+gap)
            self._cards[it["gesture"]] = (x, y, cw, ch)
            active = (it["gesture"] == hold_g and prog > 0)
            if active and repeating:
                # 연속(jog) 적용 중 → 카드 계속 흰색
                self._rrect(x, y, cw, ch, 12, fill=WHITE, outline="")
                txtcol = INK
            else:
                self._rrect(x, y, cw, ch, 12, fill=CARD, outline="")
                self._rrect(x, y, cw, ch, 12, fill="", outline=BASE, width=2)
                if active:
                    self._border_lr(x, y, cw, ch, min(1.0, prog))
                txtcol = WHITE
            label = it["label"]
            if label == "Rec":   # 녹화 토글: 상태에 따라 ON/OFF
                label = "Rec OFF" if self.recording else "Rec ON"
            c.create_text(x+cw//2, y+ch//2,
                          text=f"{GNUM.get(it['gesture'],'')}  {label}",
                          fill=txtcol, font=("Segoe UI", 15, "bold"))
        # 거꾸로 따봉(back) 게이지 — 카드 위 중앙
        if hold_g == "dislike":
            self._gauge(W//2, y-16, prog)
        path = " > ".join(snap.get("path", []))
        c.create_text(x0, y-18, text=path, anchor="w", fill=DIM,
                      font=("Segoe UI", 12))
        c.create_text(W//2, H-8, text="reverse thumbs-up (K) = back   (hold 1~4 to select)",
                      fill=DIM, font=("Segoe UI", 10))

    # 테두리 좌→우 보라색 채움
    def _border_lr(self, x, y, w, h, frac):
        c = self.cv
        fw = w * frac
        c.create_line(x, y, x+fw, y, fill=FILL, width=3, capstyle="round")
        c.create_line(x, y+h, x+fw, y+h, fill=FILL, width=3, capstyle="round")
        c.create_line(x, y, x, y+h, fill=FILL, width=3, capstyle="round")
        if frac >= 0.999:
            c.create_line(x+w, y, x+w, y+h, fill=FILL, width=3, capstyle="round")


if __name__ == "__main__":
    Preview().win.mainloop()
