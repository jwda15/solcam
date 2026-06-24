#!/usr/bin/env python3
"""viz_tracking_yaw.py — OAK-D 트래킹 + 상단 yaw 제어 논리 시각화 (ROS 불필요, 윈도우 OK)

목적: 잿슨/ROS 없이 윈도우에서 OAK-D 만 꽂고 트래킹 성능과 상단 yaw 제어 논리를
      눈으로 검증한다. oak_detector.py 와 "동일한" depthai 파이프라인
      (SpatialDetectionNetwork, yolov6-nano, person-only)을 쓴다.

화면 2개:
  [OAK Tracking]  RGB + 사람 박스(회색) + 주인 박스(초록) + 주인중심 빨간점(=raw 검출)
                  가림/이탈 시 KF 외삽 예측점(주황) + 상단 yaw 발행값 텍스트
  [2D Map]        위에서 본 평면도 — 로봇 기준 주인의 거리/방위각
                  빨강=smoothed(검출중) / 주황=KF 외삽(가림중) / 회색=raw

★control_node 는 tracking_node 가 칼만+평활+Lost외삽해 발행한 /owner_pose 를 쓴다.
  이 도구엔 ByteTrack가 없어서 그 거동을 근사 재현한다:
   - 검출 중: 속도클립(±2m/s)+동적 EMA  (tracking_node.cpp 와 동일 파라미터)
   - 가림/이탈: 등속도 외삽으로 ~LOST_HOLD_S 초간 주인 유지(= track_buffer 60f@30fps)
  ByteTrack 칼만/ID 재매핑까지 똑같진 않음(잿슨 전용) → 거동 근사임.

상단 yaw 논리(velocity 모드, controller_base.cpp trackTopYaw 와 동일):
  azimuth=atan2(x,z) (+=오른쪽); |az|<=AZ_DEAD→0; az>0→-SIGN(우회전); az<0→+SIGN(좌회전)
  발행값은 control 과 동일하게 smoothed azimuth 로 계산.

실행:  py viz_tracking_yaw.py   |  필요: pip install depthai opencv-python numpy  |  종료: q/ESC
"""
import math
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("opencv 필요: pip install opencv-python")
try:
    import depthai as dai
except ImportError:
    sys.exit("depthai 필요: pip install depthai  (OAK-D 드라이버 포함)")

# ---- control_node 와 일치 (params.hpp / control_params.yaml) ----
AZ_DEAD = 0.10          # rad, 중앙 정지 불감대 (≈±5.7°)
TOP_YAW_SIGN = 1.0      # 회전 방향이 실제와 반대면 -1.0

# ---- tracking_node.cpp 와 동일한 평활/외삽 파라미터 (/owner_pose 재현용) ----
MAX_SPEED_MMPS = 2000.0   # ±2.0 m/s 속도 클립
EMA_AXY        = 0.35     # x EMA
EMA_AZ_MIN     = 0.08     # z EMA(빠른 좌우이동, 강한 평활)
EMA_AZ_MAX     = 0.40     # z EMA(정지, 빠른 반응)
DYN_Z_PX_FULL  = 15.0     # 프레임간 픽셀x 이동 이 값↑이면 z 평활 최대
VEL_EMA        = 0.5      # 외삽용 속도 추정 EMA
LOST_HOLD_S    = 2.0      # 가림/이탈 시 외삽 유지 시간 (track_buffer 60f@30fps ≈ 2s)

# ---- oak_detector.py 와 동일한 파이프라인 ----
MODEL = "yolov6-nano"
PREV_W, PREV_H = 512, 384
CONF = 0.5
BBOX_SCALE = 0.3
DEPTH_LO, DEPTH_HI = 100, 8000   # mm
DISP_SCALE = 2

# back-projection (외삽 예측점을 화면 픽셀로 환산; oak_detector 의 근사 intrinsic)
HFOV_DEG = 69.0
FX = PREV_W / (2.0 * math.tan(math.radians(HFOV_DEG) / 2.0))
CXP = PREV_W / 2.0
CYP = PREV_H / 2.0


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class OwnerTracker:
    """검출 중엔 속도클립+동적EMA 평활, 가림 시엔 등속도 외삽으로 주인을 유지.
    tracking_node 의 /owner_pose(평활) + KF Lost 외삽 거동을 근사 재현."""
    def __init__(self):
        self.alive = False      # 주인 유지 중(검출 또는 외삽)
        self.extrap = False     # 지금 외삽으로 메우는 중인지
        self.sx = self.sz = 0.0 # smoothed 위치(mm)
        self.vx = self.vz = 0.0 # 속도(mm/s) — 외삽용
        self.px = self.py = CXP # 화면 픽셀(예측점 표시용)
        self.t = None
        self.lost_since = None
        self._sm_px = None      # 동적 alpha_z 용 직전 픽셀x

    def _dt(self, now):
        dt = (now - self.t) if self.t else (1.0 / 30.0)
        return dt if (0.0 < dt <= 1.0) else (1.0 / 30.0)

    def hit(self, x, z, pxpix, pypix, now):
        if not self.alive:
            self.sx, self.sz = x, z
            self.vx = self.vz = 0.0
        else:
            dt = self._dt(now)
            maxd = MAX_SPEED_MMPS * dt
            cx = _clip(x, self.sx - maxd, self.sx + maxd)   # 속도 클립
            cz = _clip(z, self.sz - maxd, self.sz + maxd)
            az = EMA_AZ_MAX
            if self._sm_px is not None:
                ratio = _clip(abs(pxpix - self._sm_px) / DYN_Z_PX_FULL, 0.0, 1.0)
                az = EMA_AZ_MAX - ratio * (EMA_AZ_MAX - EMA_AZ_MIN)
            nsx = EMA_AXY * cx + (1.0 - EMA_AXY) * self.sx
            nsz = az * cz + (1.0 - az) * self.sz
            self.vx = _clip(VEL_EMA * (nsx - self.sx) / dt + (1 - VEL_EMA) * self.vx,
                            -MAX_SPEED_MMPS, MAX_SPEED_MMPS)
            self.vz = _clip(VEL_EMA * (nsz - self.sz) / dt + (1 - VEL_EMA) * self.vz,
                            -MAX_SPEED_MMPS, MAX_SPEED_MMPS)
            self.sx, self.sz = nsx, nsz
        self._sm_px, self.px, self.py = pxpix, pxpix, pypix
        self.t, self.alive, self.extrap, self.lost_since = now, True, False, None

    def miss(self, now):
        """이번 프레임 주인 검출 없음 → 외삽으로 메우거나(시간 내) 포기."""
        if not self.alive:
            return
        if self.lost_since is None:
            self.lost_since = now
        if now - self.lost_since > LOST_HOLD_S:
            self.alive = self.extrap = False
            return
        dt = self._dt(now)
        self.sx += self.vx * dt          # 등속도 외삽
        self.sz = max(100.0, self.sz + self.vz * dt)
        self.px = CXP + (self.sx / self.sz) * FX   # 예측점 픽셀(가로)
        self.t, self.extrap = now, True

    def lost_secs(self, now):
        return 0.0 if self.lost_since is None else (now - self.lost_since)


def build_pipeline():
    pipe = dai.Pipeline()
    cam_rgb = pipe.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    mono_l  = pipe.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_r  = pipe.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipe.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(False)
    stereo.setLeftRightCheck(True)
    mono_l.requestOutput((PREV_W, PREV_H)).link(stereo.left)
    mono_r.requestOutput((PREV_W, PREV_H)).link(stereo.right)

    nn = pipe.create(dai.node.SpatialDetectionNetwork).build(
        cam_rgb, stereo, dai.NNModelDescription(MODEL))
    nn.input.setBlocking(False)
    nn.setConfidenceThreshold(CONF)
    nn.setBoundingBoxScaleFactor(BBOX_SCALE)
    nn.setSpatialCalculationAlgorithm(dai.SpatialLocationCalculatorAlgorithm.MEDIAN)
    nn.setDepthLowerThreshold(DEPTH_LO)
    nn.setDepthUpperThreshold(DEPTH_HI)

    q_rgb = nn.passthrough.createOutputQueue(maxSize=4, blocking=False)
    q_det = nn.out.createOutputQueue(maxSize=4, blocking=False)
    return pipe, q_rgb, q_det


def yaw_cmd_for(azimuth):
    if abs(azimuth) <= AZ_DEAD:
        return 0.0
    return TOP_YAW_SIGN * (-1.0 if azimuth >= 0.0 else 1.0)


def pick_owner(persons):
    best, best_d = None, 1e9
    for p in persons:
        cx = (p["xmin"] + p["xmax"]) * 0.5
        cy = (p["ymin"] + p["ymax"]) * 0.5
        d = (cx - 0.5) ** 2 + (cy - 0.5) ** 2
        if d < best_d:
            best, best_d = p, d
    return best


def draw_tracking(frame, persons, owner_det, trk, now):
    f = cv2.resize(frame, (PREV_W * DISP_SCALE, PREV_H * DISP_SCALE))
    H, W = f.shape[:2]
    cv2.line(f, (W // 2, 0), (W // 2, H), (90, 90, 90), 1)

    for p in persons:
        x1, y1 = int(p["xmin"] * W), int(p["ymin"] * H)
        x2, y2 = int(p["xmax"] * W), int(p["ymax"] * H)
        is_owner = owner_det is not None and p is owner_det
        col = (0, 220, 0) if is_owner else (160, 160, 160)
        cv2.rectangle(f, (x1, y1), (x2, y2), col, 3 if is_owner else 1)
        cv2.putText(f, f"{p['z']/1000:.2f}m", (x1, y2 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        if is_owner:
            cv2.circle(f, (int((p["xmin"]+p["xmax"])*0.5*W),
                           int((p["ymin"]+p["ymax"])*0.5*H)), 7, (0, 0, 255), -1)
            cv2.putText(f, "OWNER", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)

    # KF 외삽 예측점(주황) — 검출이 끊긴 동안 표시
    if trk.alive and trk.extrap:
        ex = int(_clip(trk.px, 0, PREV_W - 1) * DISP_SCALE)
        ey = int(_clip(trk.py, 0, PREV_H - 1) * DISP_SCALE)
        cv2.circle(f, (ex, ey), 8, (0, 165, 255), 2)
        cv2.putText(f, f"KF extrap {trk.lost_secs(now):.1f}s", (ex + 10, ey),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

    # 상단 yaw 텍스트 (control 과 동일하게 smoothed azimuth 사용)
    if trk.alive:
        az = math.atan2(trk.sx, trk.sz)
        cmd = yaw_cmd_for(az)
        side = "CENTER" if cmd == 0 else ("RIGHT" if cmd < 0 else "LEFT")
        state = "EXTRAP(KF)" if trk.extrap else "TRACK"
        lines = [
            f"TOP-YAW (velocity) [{state}]",
            f"azimuth(smoothed) = {math.degrees(az):+.1f} deg  (deadzone +-{math.degrees(AZ_DEAD):.1f})",
            f"owner = {side}",
            f"published top_yaw_target = {cmd:+.1f}  active=True",
        ]
        col = (0, 165, 255) if trk.extrap else ((0, 255, 255) if cmd != 0 else (0, 220, 0))
    else:
        lines = ["TOP-YAW (velocity)", "no owner -> top_yaw_target = 0.0 (stop)"]
        col = (0, 0, 255)
    y = 26
    for ln in lines:
        cv2.putText(f, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(f, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1)
        y += 26
    return f


def draw_map(owner_det, trk, now, size=460, max_range=4.0):
    m = np.full((size, size + 60, 3), 22, dtype=np.uint8)
    cx, cy = (size + 60) // 2, size - 40
    ppm = (size - 70) / max_range

    for r in range(1, int(max_range) + 1):
        cv2.circle(m, (cx, cy), int(r * ppm), (55, 55, 55), 1)
        cv2.putText(m, f"{r}m", (cx + 4, cy - int(r * ppm) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1)
    cv2.line(m, (cx, cy), (cx, cy - int(max_range * ppm)), (70, 70, 70), 1)
    for s in (+1, -1):
        cv2.line(m, (cx, cy),
                 (cx + int(math.sin(s*AZ_DEAD)*max_range*ppm),
                  cy - int(math.cos(s*AZ_DEAD)*max_range*ppm)), (0, 90, 90), 1)
    cv2.drawMarker(m, (cx, cy), (0, 220, 0), cv2.MARKER_TRIANGLE_UP, 16, 2)

    def plot(x_mm, z_mm, color, rad):
        az = math.atan2(x_mm, z_mm)
        dist = math.hypot(x_mm, z_mm) / 1000.0
        px = cx + int(math.sin(az) * min(dist, max_range) * ppm)
        py = cy - int(math.cos(az) * min(dist, max_range) * ppm)
        cv2.circle(m, (px, py), rad, color, -1)
        return az, dist, px, py

    if trk.alive:
        if owner_det is not None and not trk.extrap:
            plot(owner_det["x"], owner_det["z"], (110, 110, 110), 4)   # raw(회색)
        col = (0, 165, 255) if trk.extrap else (0, 0, 255)
        az, dist, px, py = plot(trk.sx, trk.sz, col, 7)
        cv2.line(m, (cx, cy), (px, py), col, 1)
        cv2.putText(m, f"d={dist:.2f}m", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(m, f"az={math.degrees(az):+.1f}deg", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (110, 170, 245), 1)
        if trk.extrap:
            cv2.putText(m, f"KF extrap {trk.lost_secs(now):.1f}s / {LOST_HOLD_S:.0f}s",
                        (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
    else:
        cv2.putText(m, "no owner", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    cv2.putText(m, "red=smoothed orange=KF-extrap gray=raw", (10, size - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    return m


def main():
    pipe, q_rgb, q_det = build_pipeline()
    print("OAK 파이프라인 시작 (모델 첫 다운로드 시 잠시 지연). q/ESC 종료.")
    pipe.start()
    frame = np.zeros((PREV_H, PREV_W, 3), dtype=np.uint8)
    persons, owner_det = [], None
    trk = OwnerTracker()
    try:
        while pipe.isRunning():
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
            in_det = q_det.tryGet()
            if in_det is not None:
                persons = []
                for d in in_det.detections:
                    if int(d.label) != 0:
                        continue
                    z = float(d.spatialCoordinates.z)
                    if z <= 0.0:
                        continue
                    persons.append({
                        "xmin": d.xmin, "ymin": d.ymin, "xmax": d.xmax, "ymax": d.ymax,
                        "x": float(d.spatialCoordinates.x),
                        "y": float(d.spatialCoordinates.y), "z": z,
                        "conf": float(d.confidence),
                    })
                owner_det = pick_owner(persons)
                now = time.time()
                if owner_det is not None:
                    pxpix = (owner_det["xmin"] + owner_det["xmax"]) * 0.5 * PREV_W
                    pypix = (owner_det["ymin"] + owner_det["ymax"]) * 0.5 * PREV_H
                    trk.hit(owner_det["x"], owner_det["z"], pxpix, pypix, now)
                else:
                    trk.miss(now)

            now = time.time()
            cv2.imshow("OAK Tracking", draw_tracking(frame, persons, owner_det, trk, now))
            cv2.imshow("2D Map", draw_map(owner_det, trk, now))
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                break
    finally:
        try:
            pipe.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
