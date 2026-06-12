#!/usr/bin/env python3
"""control_sim.py — 컨트롤 노드 제어 로직 2D 시뮬레이터 (ROS/하드웨어 불필요).

목적: 폰·OAK 없이 "가짜 주인"을 한 점에 두고, 화면 버튼으로 손동작 메뉴와
      똑같은 명령(모드 전환 / 공전 SEG_ANGLE / 거리 SEG_DISTANCE / 팬
      HEADING_OFFSET)을 주입해, 컨트롤 노드가 내는 움직임을 2D로 확인한다.

★주의: 이건 C++ 컨트롤 노드(ros2_control_node)의 FOLLOW/ROTATE/IDLE 로직을
   파이썬으로 충실히 옮긴 오프라인 시각화다. 게인/한계는 params.hpp 기본값과
   동일. 실기에선 C++ 노드가 돈다. 메카넘이라 평면 어느 방향이든 즉시 이동
   가능하다고 보고(휠 분배는 드라이버 담당) 글로벌 속도를 그대로 적분한다.

조작:
  - 오른쪽 버튼으로 명령 주입 (꾹 누르면 연속 = 실기 jog 와 동일).
  - 캔버스를 클릭하면 그 자리로 '주인'을 옮긴다 (추적 반응 확인용).
  - Follow 누르는 순간의 로봇-주인 선분(거리 D, 글로벌각 φ)을 캡처한다.

실행:  py ros2_control_node/tools/control_sim.py     (Windows, 설치 불필요)
"""
import math
import time
import tkinter as tk

# ===== 제어 파라미터 (ros2_control_node/include/control_node/params.hpp 기본값) =====
V_MAX = 0.4
W_BODY_MAX = 0.7
BODY_ACCEL_MAX = 0.5
YAW_ACCEL_MAX = 1.5
W_TOP_MAX = 1.5
KP_YAW = 0.7
AZ_DEAD = 0.03
KP_POS = 0.7
KD_POS = 0.15
POS_DEAD = 0.08
KP_BYAW = 1.0
KD_BYAW = 0.1
BYAW_DEAD = 0.05
SEG_D_MIN = 0.5
SEG_D_MAX = 3.0
FACE_OWNER = True

# 손동작 조절 폭 (menu.py / gesture_params.yaml 과 동일)
DIST_STEP = 0.3
BEARING_STEP = math.radians(8.0)
HEADING_STEP = math.radians(15.0)

# ===== 화면 =====
CANVAS = 660
PANEL = 300
SCALE = 100.0          # px/m
DT = 0.02              # s, 제어 주기 50Hz (와 렌더)

BG = "#0d0f12"
GRID = "#1c2128"
AX = "#39414d"
OWNER_C = "#ff9f43"
ROBOT_C = "#1e90ff"
CAM_C = "#2ee6c0"
TARGET_C = "#9be15d"
RING_C = "#3c465c"
TRAIL_C = "#26425f"
WHITE = "#f4f6f8"
DIM = "#aeb4bd"

MODE_NAMES = {0: "IDLE", 1: "FOLLOW", 2: "ROTATE"}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def slew(cur, tgt, rate, dt):
    step = rate * dt
    return cur + clamp(tgt - cur, -step, step)


class Sim:
    def __init__(self):
        # ----- 월드 상태 -----
        self.ox, self.oy = 0.0, 2.0      # 주인(고정, 클릭으로 이동)
        self.reset_robot(initial=True)

        # ----- UI -----
        self.win = tk.Tk()
        self.win.title("solcam control sim (2D)")
        self.win.configure(bg=BG)
        self.cv = tk.Canvas(self.win, width=CANVAS, height=CANVAS, bg=BG,
                            highlightthickness=0)
        self.cv.grid(row=0, column=0)
        self.cv.bind("<Button-1>", self._move_owner)
        self._build_panel()
        self.trail = []
        self._rep = None
        self.last = time.time()
        self._loop()

    def reset_robot(self, initial=False):
        self.rx, self.ry, self.yaw = 0.0, 0.0, math.radians(90)  # 주인 향해 시작
        self.top_yaw = 0.0           # 상단 yaw 스테이지 각(몸체 기준)
        self.mode = 1                # 시작 FOLLOW
        self.D = math.hypot(self.ox - self.rx, self.oy - self.ry)
        self.phi = math.atan2(self.oy - self.ry, self.ox - self.rx)
        self.offset = 0.0            # 헤딩(Pan) 오프셋
        self.vgx = self.vgy = self.wz = 0.0
        self.pex = self.pey = 0.0
        self.pyerr = 0.0
        if not initial:
            self.trail = []

    # ================= 패널(버튼) =================
    def _build_panel(self):
        p = tk.Frame(self.win, bg=BG, width=PANEL)
        p.grid(row=0, column=1, sticky="n", padx=12, pady=10)

        def section(title):
            tk.Label(p, text=title, bg=BG, fg=DIM,
                     font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(10, 2))

        def jog(parent, text, fn, color="#26303d"):
            b = tk.Button(parent, text=text, width=10, bg=color, fg=WHITE,
                          activebackground="#37475a", relief="flat",
                          font=("Segoe UI", 11, "bold"))
            b.bind("<ButtonPress-1>", lambda e: self._start_rep(fn))
            b.bind("<ButtonRelease-1>", lambda e: self._stop_rep())
            return b

        def once(parent, text, fn, color="#26303d"):
            return tk.Button(parent, text=text, width=10, bg=color, fg=WHITE,
                             activebackground="#37475a", relief="flat",
                             command=fn, font=("Segoe UI", 11, "bold"))

        section("모드")
        r = tk.Frame(p, bg=BG); r.pack(anchor="w")
        once(r, "Idle", lambda: self.set_mode(0)).pack(side="left", padx=2)
        once(r, "Follow", lambda: self.set_mode(1), "#1e5fa8").pack(side="left", padx=2)
        once(r, "Rotate", lambda: self.set_mode(2)).pack(side="left", padx=2)

        section("Bearing — 공전 (SEG_ANGLE)")
        r = tk.Frame(p, bg=BG); r.pack(anchor="w")
        jog(r, "↺ CCW", lambda: self.adjust_phi(+BEARING_STEP)).pack(side="left", padx=2)
        jog(r, "↻ CW", lambda: self.adjust_phi(-BEARING_STEP)).pack(side="left", padx=2)

        section("Distance — 거리 (SEG_DISTANCE)")
        r = tk.Frame(p, bg=BG); r.pack(anchor="w")
        jog(r, "Farther", lambda: self.adjust_D(+DIST_STEP)).pack(side="left", padx=2)
        jog(r, "Closer", lambda: self.adjust_D(-DIST_STEP)).pack(side="left", padx=2)

        section("Pan — 카메라 헤딩 (HEADING_OFFSET)")
        r = tk.Frame(p, bg=BG); r.pack(anchor="w")
        jog(r, "Pan L", lambda: self.adjust_off(+HEADING_STEP)).pack(side="left", padx=2)
        jog(r, "Pan R", lambda: self.adjust_off(-HEADING_STEP)).pack(side="left", padx=2)

        section("")
        once(p, "Reset", lambda: self.reset_robot(), "#5a2a2a").pack(anchor="w", pady=2)

        self.hud = tk.Label(p, text="", bg=BG, fg=WHITE, justify="left",
                            font=("Consolas", 10))
        self.hud.pack(anchor="w", pady=(14, 0))
        tk.Label(p, text="캔버스 클릭 = 주인 이동\n버튼 꾹 = 연속(jog)",
                 bg=BG, fg=DIM, justify="left",
                 font=("Segoe UI", 9)).pack(anchor="w", pady=(10, 0))

    # 버튼 꾹 누르면 연속 실행 (실기 jog 재현)
    def _start_rep(self, fn):
        fn()
        self._rep = self.win.after(140, lambda: self._start_rep(fn))

    def _stop_rep(self):
        if self._rep:
            self.win.after_cancel(self._rep); self._rep = None

    # ================= 명령 (손동작 메뉴와 동일 의미) =================
    def set_mode(self, m):
        self.mode = m
        if m == 1:   # FOLLOW 진입 = 현재 선분(D, φ) 캡처
            self.D = max(1e-3, math.hypot(self.ox - self.rx, self.oy - self.ry))
            self.phi = math.atan2(self.oy - self.ry, self.ox - self.rx)
            self.pex = self.pey = 0.0

    def adjust_phi(self, dv):   # 공전: 글로벌각 φ
        self.phi = wrap(self.phi + dv)

    def adjust_D(self, dv):     # 거리 D (손동작 경로는 클램프)
        self.D = clamp(self.D + dv, SEG_D_MIN, SEG_D_MAX)

    def adjust_off(self, dv):   # 헤딩 오프셋
        self.offset = wrap(self.offset + dv)

    def _move_owner(self, e):
        self.ox = (e.x - CANVAS / 2) / SCALE
        self.oy = (CANVAS / 2 - e.y) / SCALE

    # ================= 제어 1스텝 (C++ 노드 로직 이식) =================
    def step(self, dt):
        bearing = math.atan2(self.oy - self.ry, self.ox - self.rx)  # 로봇→주인 글로벌각

        # --- 상단 yaw: 주인 락온 (IDLE은 정지) ---
        if self.mode in (1, 2):
            az = wrap(bearing - (self.yaw + self.top_yaw))   # 카메라가 주인까지 더 돌 각
            if abs(az) > AZ_DEAD:
                self.top_yaw += clamp(KP_YAW * az, -W_TOP_MAX * dt, W_TOP_MAX * dt)

        # --- 몸체 목표 속도/각속도 ---
        vgx = vgy = 0.0
        wz_des = 0.0
        if self.mode == 1:   # FOLLOW: 선분 끝점 추종
            tx = self.ox - self.D * math.cos(self.phi)
            ty = self.oy - self.D * math.sin(self.phi)
            ex, ey = tx - self.rx, ty - self.ry
            if math.hypot(ex, ey) > POS_DEAD:
                dex, dey = (ex - self.pex) / dt, (ey - self.pey) / dt
                vgx = clamp(KP_POS * ex + KD_POS * dex, -V_MAX, V_MAX)
                vgy = clamp(KP_POS * ey + KD_POS * dey, -V_MAX, V_MAX)
                vn = math.hypot(vgx, vgy)
                if vn > V_MAX:
                    vgx *= V_MAX / vn; vgy *= V_MAX / vn
            self.pex, self.pey = ex, ey

        if self.mode in (1, 2):   # 몸체 yaw: 주인 향함 + 오프셋
            desired = wrap((bearing if FACE_OWNER else self.phi) + self.offset)
            yerr = wrap(desired - self.yaw)
            if abs(yerr) > BYAW_DEAD:
                wz_des = clamp(KP_BYAW * yerr + KD_BYAW * (yerr - self.pyerr) / dt,
                               -W_BODY_MAX, W_BODY_MAX)
            self.pyerr = yerr

        # --- 슬루(가속 제한) + 적분 ---
        self.vgx = slew(self.vgx, vgx, BODY_ACCEL_MAX, dt)
        self.vgy = slew(self.vgy, vgy, BODY_ACCEL_MAX, dt)
        self.wz = slew(self.wz, wz_des, YAW_ACCEL_MAX, dt)
        self.rx += self.vgx * dt
        self.ry += self.vgy * dt
        self.yaw = wrap(self.yaw + self.wz * dt)

        self.trail.append((self.rx, self.ry))
        if len(self.trail) > 400:
            self.trail.pop(0)

    # ================= 좌표 변환/그리기 =================
    def w2s(self, x, y):
        return CANVAS / 2 + x * SCALE, CANVAS / 2 - y * SCALE

    def _loop(self):
        now = time.time()
        # 실제 경과시간 기반으로 50Hz 스텝(렌더 끊겨도 물리 일정)
        n = 0
        while now - self.last >= DT and n < 5:
            self.step(DT)
            self.last += DT
            n += 1
        if n == 0:
            pass
        self._draw()
        self.win.after(16, self._loop)

    def _draw(self):
        c = self.cv
        c.delete("all")
        # 그리드
        for i in range(-3, 4):
            x, _ = self.w2s(i, 0); _, y = self.w2s(0, i)
            c.create_line(x, 0, x, CANVAS, fill=GRID)
            c.create_line(0, y, CANVAS, y, fill=GRID)
        ox0, oy0 = self.w2s(0, 0)
        c.create_line(0, oy0, CANVAS, oy0, fill=AX)
        c.create_line(ox0, 0, ox0, CANVAS, fill=AX)

        # 트레일
        if len(self.trail) > 1:
            pts = []
            for (x, y) in self.trail:
                sx, sy = self.w2s(x, y); pts += [sx, sy]
            c.create_line(*pts, fill=TRAIL_C, width=2)

        # FOLLOW 목표 링 + 목표점
        if self.mode == 1:
            cxo, cyo = self.w2s(self.ox, self.oy)
            rpx = self.D * SCALE
            c.create_oval(cxo - rpx, cyo - rpx, cxo + rpx, cyo + rpx,
                          outline=RING_C, dash=(4, 4))
            tx = self.ox - self.D * math.cos(self.phi)
            ty = self.oy - self.D * math.sin(self.phi)
            sx, sy = self.w2s(tx, ty)
            c.create_line(sx - 7, sy, sx + 7, sy, fill=TARGET_C, width=2)
            c.create_line(sx, sy - 7, sx, sy + 7, fill=TARGET_C, width=2)

        # 주인
        cxo, cyo = self.w2s(self.ox, self.oy)
        c.create_oval(cxo - 9, cyo - 9, cxo + 9, cyo + 9, fill=OWNER_C, outline="")
        c.create_text(cxo, cyo - 18, text="OWNER", fill=OWNER_C,
                      font=("Segoe UI", 10, "bold"))

        # 로봇 (삼각형=헤딩) + 카메라 광선
        self._draw_robot()

        # 카메라가 주인을 보고 있는지 보조선(로봇→주인, 흐리게)
        rx, ry = self.w2s(self.rx, self.ry)
        c.create_line(rx, ry, cxo, cyo, fill="#223", width=1)

        self._update_hud()

    def _draw_robot(self):
        c = self.cv
        rx, ry = self.w2s(self.rx, self.ry)
        # 카메라 광선 (몸체 yaw + 상단 yaw 방향) — 주인을 가리켜야 정상
        cam = self.yaw + self.top_yaw
        cl = 1.1 * SCALE
        c.create_line(rx, ry, rx + cl * math.cos(cam), ry - cl * math.sin(cam),
                      fill=CAM_C, width=3, arrow="last")
        # 몸체 삼각형
        L = 16
        pts = []
        for ang in (0, 2.5, -2.5):
            a = self.yaw + ang
            pts += [rx + L * math.cos(a), ry - L * math.sin(a)]
        c.create_polygon(*pts, fill=ROBOT_C, outline=WHITE)

    def _update_hud(self):
        dist = math.hypot(self.ox - self.rx, self.oy - self.ry)
        spd = math.hypot(self.vgx, self.vgy)
        cam_err = math.degrees(wrap(
            math.atan2(self.oy - self.ry, self.ox - self.rx) - (self.yaw + self.top_yaw)))
        self.hud.config(text=(
            f"mode      : {MODE_NAMES.get(self.mode,'?')}\n"
            f"dist→owner: {dist:5.2f} m\n"
            f"D (seg)   : {self.D:5.2f} m\n"
            f"phi (φ)   : {math.degrees(self.phi):6.1f}°\n"
            f"pan offset: {math.degrees(self.offset):6.1f}°\n"
            f"top_yaw   : {math.degrees(self.top_yaw):6.1f}°\n"
            f"cam→owner : {cam_err:6.1f}°  (0=주인 정조준)\n"
            f"|v|       : {spd:4.2f} m/s\n"
            f"wz        : {math.degrees(self.wz):6.1f}°/s"))


if __name__ == "__main__":
    Sim().win.mainloop()
