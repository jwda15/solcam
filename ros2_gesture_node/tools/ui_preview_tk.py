"""LCD UI 프리뷰 (tkinter) — 설치/ROS 불필요. Windows에서 바로 실행.

pygame이 안 깔리는 환경(예: Python 3.14처럼 최신이라 pygame wheel 미배포)에서도
UI를 확인하려고 만든 버전. tkinter는 Windows Python에 기본 포함이라
'pip install' 자체가 필요 없다.

실제 메뉴 상태기계(menu.py)를 그대로 import해서 동작/타이밍은 로봇과 동일하고,
그리기만 tkinter Canvas로 다시 그린다(테마/레이아웃은 hud.py와 동일하게 맞춤).
정식 렌더(hud.py, pygame)를 PC에서 보고 싶으면 Python 3.12 등에서
tools/ui_preview.py 를 쓰면 된다.

조작(키보드로 손동작 흉내):
  L=따봉(메뉴 열기)  1~4=항목 선택(꾹)  P=손바닥(뒤로/닫기)
  R=녹화 토글  B=배터리--  ESC=종료
"""
import sys, time, tkinter as tk
from pathlib import Path

# menu.py 가 패키지(ros2_gesture_node) 안에 있으므로 경로 추가 후 import
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_gesture_node.menu import MenuStateMachine, build_menu

# ----- 테마 (hud.py와 동일) -----
BG      = "#0d0f12"   # 영상 배경(여기선 플레이스홀더)
DOCK    = "#121418"
CARD    = "#1e2228"   # 비활성 카드 배경
ACCENT  = "#4a90e2"   # 파랑
FILL    = "#7f77dd"   # 보라(차오름)
WHITE   = "#f4f6f8"
DIM     = "#aeb4bd"
REC_RED = "#ff5a5a"
FLASH_SEC = 0.1

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE"}
KEY2GEST = {"l": "like", "1": "one", "2": "two", "3": "three", "4": "four", "p": "palm"}

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

        self.held = None           # 현재 눌린 제스처 키
        self._rel_job = None       # 키 릴리즈 디바운스(자동반복 흡수)
        self._prev_progress = 0.0
        self._flash_until = 0.0

        self.win = tk.Tk()
        self.win.title("solcam LCD preview (tkinter)")
        self.win.configure(bg=BG)
        self.cv = tk.Canvas(self.win, width=W, height=H, bg=BG, highlightthickness=0)
        self.cv.pack()
        self.win.bind("<KeyPress>", self._press)
        self.win.bind("<KeyRelease>", self._release)
        self._loop()

    # ---- 입력: tkinter 자동반복(Release+Press 연타) 디바운스 ----
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
        # 배경 + 영상 플레이스홀더
        c.create_rectangle(0, 0, W, H, fill=BG, outline="")
        c.create_text(W//2, H//2, text="CAMERA", fill="#2a2f37",
                      font=("Segoe UI", 30, "bold"))
        self._topbar()
        if snap.get("state") == "MENU":
            self._dock(snap)
        else:
            c.create_text(W//2, H-30, text="show LIKE to open menu  (press L)",
                          fill=DIM, font=("Segoe UI", 12))

    def _topbar(self):
        c = self.cv
        c.create_oval(20, 18, 32, 30, fill=ACCENT, outline="")
        c.create_text(40, 24, text=MODE_NAMES.get(self.mode, "?"), anchor="w",
                      fill=WHITE, font=("Segoe UI", 16, "bold"))
        x = W - 20
        # 배터리
        bw, bh = 34, 16
        bx, by = x - bw, 16
        c.create_rectangle(bx, by, bx+bw, by+bh, outline=DIM)
        c.create_rectangle(bx+bw, by+4, bx+bw+3, by+bh-4, outline=DIM, fill=DIM)
        fillw = int((bw-4) * self.battery / 100)
        col = REC_RED if self.battery <= 20 else WHITE
        c.create_rectangle(bx+2, by+2, bx+2+fillw, by+bh-2, fill=col, outline="")
        c.create_text(bx-8, 24, text=f"{self.battery}%", anchor="e",
                      fill=WHITE, font=("Segoe UI", 12))
        x = bx - 50
        if self.recording:
            secs = int(time.time() - self.rec_start)
            c.create_oval(x-10, 19, x-2, 27, fill=REC_RED, outline="")
            c.create_text(x+4, 24, text=f"{secs//60:02d}:{secs%60:02d}", anchor="w",
                          fill=REC_RED, font=("Segoe UI", 13, "bold"))

    def _dock(self, snap):
        c = self.cv
        items = snap.get("items", [])
        if not items: return
        n = len(items)
        cw, ch, gap = 150, 64, 14
        total = n*cw + (n-1)*gap
        x0 = (W - total)//2
        y = H - ch - 28
        prog = snap.get("hold_progress", 0.0)
        hold_g = snap.get("hold_gesture", "")

        # 확정(progress 하강 에지) → 흰 플래시
        if self._prev_progress >= 0.999 and prog < self._prev_progress:
            self._flash_until = time.time() + FLASH_SEC
        self._prev_progress = prog
        flashing = time.time() < self._flash_until

        for i, it in enumerate(items):
            x = x0 + i*(cw+gap)
            active = (it["gesture"] == hold_g and prog > 0)
            self._rrect(x, y, cw, ch, 12, fill=CARD, outline="")
            if active and not flashing:
                fw = max(1, int(cw * min(1.0, prog)))
                # 보라색이 좌→우로 차오름 (카드 폭만큼 클립)
                c.create_rectangle(x, y, x+fw, y+ch, fill=FILL, outline="")
                self._rrect(x, y, cw, ch, 12, outline=ACCENT, width=2)
            else:
                self._rrect(x, y, cw, ch, 12,
                            outline=(WHITE if flashing and active else "#2a3038"), width=2)
            if flashing and active:
                self._rrect(x, y, cw, ch, 12, fill=WHITE, outline="")
            num = {"one":"1","two":"2","three":"3","four":"4"}.get(it["gesture"], "")
            txtcol = BG if (flashing and active) else WHITE
            c.create_text(x+cw//2, y+ch//2,
                          text=f"{num}  {it['label']}", fill=txtcol,
                          font=("Segoe UI", 15, "bold"))
        # 경로 + 안내
        path = " > ".join(snap.get("path", []))
        c.create_text(x0, y-18, text=path, anchor="w", fill=DIM,
                      font=("Segoe UI", 12))
        c.create_text(W//2, H-8, text="palm = back   (hold 1~4 to select)",
                      fill=DIM, font=("Segoe UI", 10))


if __name__ == "__main__":
    Preview().win.mainloop()
