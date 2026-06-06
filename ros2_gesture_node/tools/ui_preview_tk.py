"""LCD UI 프리뷰 (tkinter) — 설치/ROS 불필요. Windows에서 바로 실행.

pygame이 안 깔리는 환경(예: Python 3.14처럼 최신이라 pygame wheel 미배포)에서도
UI를 확인하려고 만든 버전. tkinter는 Windows Python에 기본 포함이라
'pip install' 자체가 필요 없다.

실제 메뉴 상태기계(menu.py)를 그대로 import해서 동작/타이밍은 로봇과 동일하고,
그리기만 tkinter Canvas로 다시 그린다(테마/레이아웃은 hud.py와 동일하게 맞춤).
정식 렌더(hud.py, pygame)를 PC에서 보고 싶으면 Python 3.12 등에서
tools/ui_preview.py 를 쓰면 된다.

조작(키보드로 손동작 흉내):
  L=따봉(메뉴 열기, 1.5s 게이지)  1~4=항목 선택(꾹)  P=손바닥(뒤로/닫기)
  R=녹화 토글  B=배터리--  ESC=종료
  ※ 모터 항목(Wheel/Lift 등)은 1.5s 후 키를 누른 채로 있으면 연속 명령.
"""
import sys, time, tkinter as tk
from pathlib import Path

# menu.py 가 패키지(ros2_gesture_node) 안에 있으므로 경로 추가 후 import
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_gesture_node.menu import MenuStateMachine, build_menu

# ----- 테마 (hud.py와 동일) -----
BG      = "#0d0f12"   # 영상 배경(여기선 플레이스홀더)
CARD    = "#1a1e24"   # 카드 배경
BASE    = "#3c465c"   # 기저 테두리(옅은 파랑)
ACCENT  = "#4a90e2"   # 파랑(따봉 게이지)
FILL    = "#7f77dd"   # 보라(테두리 차오름)
WHITE   = "#f4f6f8"
DIM     = "#aeb4bd"
REC_RED = "#ff5a5a"
FLASH_SEC = 0.1

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE"}
KEY2GEST = {"l": "like", "1": "one", "2": "two", "3": "three", "4": "four", "p": "palm"}
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

        self.held = None           # 현재 눌린 제스처 키
        self._rel_job = None       # 키 릴리즈 디바운스(자동반복 흡수)
        self._prev_progress = 0.0
        self._prev_hold = ""
        self._flash_until = 0.0
        self._flash_gesture = None

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
        c.create_rectangle(0, 0, W, H, fill=BG, outline="")
        c.create_text(W//2, H//2, text="CAMERA", fill="#2a2f37",
                      font=("Segoe UI", 30, "bold"))
        self._topbar()
        if snap.get("state") == "MENU":
            self._dock(snap)
        else:
            c.create_text(W//2, H-58, text="show LIKE to open menu  (press L)",
                          fill=DIM, font=("Segoe UI", 12))
            self._trigger(snap)

    def _topbar(self):
        c = self.cv
        c.create_oval(20, 18, 32, 30, fill=ACCENT, outline="")
        c.create_text(40, 24, text=MODE_NAMES.get(self.mode, "?"), anchor="w",
                      fill=WHITE, font=("Segoe UI", 16, "bold"))
        # 오른쪽 → 왼쪽 순서로 배치(겹침 방지)
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
        x = bx0 - 28      # 배터리와 녹화시간 사이 간격
        if self.recording:
            secs = int(time.time() - self.rec_start)
            t = f"{secs//60:02d}:{secs%60:02d}"
            c.create_text(x, 24, text=t, anchor="e",
                          fill=REC_RED, font=("Segoe UI", 13, "bold"))
            # 시간 텍스트 왼쪽에 빨간 점 (대략 폭 40 가정)
            c.create_oval(x-46, 20, x-38, 28, fill=REC_RED, outline="")

    # 따봉 1.5초 게이지 (작게, 파란색)
    def _trigger(self, snap):
        if snap.get("hold_gesture") != "like": return
        prog = snap.get("hold_progress", 0.0)
        if prog <= 0: return
        c = self.cv
        bw, bh = 160, 6
        x, y = (W-bw)//2, H-40
        self._rrect(x, y, bw, bh, 3, fill=BASE, outline="")
        fw = max(1, int(bw*min(1.0, prog)))
        c.create_rectangle(x, y, x+fw, y+bh, fill=ACCENT, outline="")

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

        # 확정(progress 하강 에지, 메뉴에서만) → 흰 플래시
        if (snap.get("state") == "MENU" and self._prev_progress >= 0.999
                and (prog < 0.5 or hold_g != self._prev_hold)):
            self._flash_until = time.time() + FLASH_SEC
            self._flash_gesture = self._prev_hold
        self._prev_progress = prog
        self._prev_hold = hold_g
        flashing = time.time() < self._flash_until

        for i, it in enumerate(items):
            x = x0 + i*(cw+gap)
            active = (it["gesture"] == hold_g and prog > 0)
            flash_this = flashing and it["gesture"] == self._flash_gesture
            self._rrect(x, y, cw, ch, 12, fill=(WHITE if flash_this else CARD), outline="")
            self._rrect(x, y, cw, ch, 12, fill="", outline=BASE, width=2)
            if active and not flash_this:
                self._border_lr(x, y, cw, ch, min(1.0, prog))
            txtcol = BG if flash_this else WHITE
            c.create_text(x+cw//2, y+ch//2,
                          text=f"{GNUM.get(it['gesture'],'')}  {it['label']}",
                          fill=txtcol, font=("Segoe UI", 15, "bold"))
        path = " > ".join(snap.get("path", []))
        c.create_text(x0, y-18, text=path, anchor="w", fill=DIM,
                      font=("Segoe UI", 12))
        c.create_text(W//2, H-8, text="palm = back   (hold 1~4 to select)",
                      fill=DIM, font=("Segoe UI", 10))

    # 테두리를 좌→우로 보라색으로 채움 (윗/아랫변이 왼쪽부터 차오름)
    def _border_lr(self, x, y, w, h, frac):
        c = self.cv
        fw = w * frac
        c.create_line(x, y, x+fw, y, fill=FILL, width=3, capstyle="round")          # 윗변
        c.create_line(x, y+h, x+fw, y+h, fill=FILL, width=3, capstyle="round")      # 아랫변
        c.create_line(x, y, x, y+h, fill=FILL, width=3, capstyle="round")           # 좌변(즉시)
        if frac >= 0.999:
            c.create_line(x+w, y, x+w, y+h, fill=FILL, width=3, capstyle="round")   # 우변(완료)


if __name__ == "__main__":
    Preview().win.mainloop()
