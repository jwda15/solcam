#!/usr/bin/env python3
"""viz_tracking_yaw.py — OAK-D 트래킹 + 상단 yaw 제어 논리 시각화 (ROS 불필요, 윈도우 OK)

목적: 잿슨/ROS 없이 윈도우에서 OAK-D 만 꽂고 트래킹 성능과 상단 yaw 제어 논리를
      눈으로 검증한다. oak_detector.py 와 "동일한" depthai 파이프라인
      (SpatialDetectionNetwork, yolov6-nano, person-only)을 쓴다.

화면 2개:
  [OAK Tracking]  RGB + 사람 박스(회색) + 주인 박스(초록) + 주인중심 빨간점(=raw 검출)
                  + 상단 yaw 발행값 텍스트(현 control_node 논리 그대로 재현)
  [2D Map]        위에서 본 평면도 — 로봇 기준 주인의 거리/방위각
                  ★빨간점 = tracking_node 와 동등한 smoothed 위치(속도클립+동적EMA)
                    회색점 = raw 검출(평활 전). 둘을 비교해 평활 효과를 본다.

★중요: 실제 로봇 control_node 는 tracking_node 가 칼만+평활해 발행한 /owner_pose 를
       쓴다(raw 아님). 이 도구엔 트래커가 없어서, /owner_pose 를 만드는 평활 단계
       (속도클립 ±2m/s + 동적 EMA, tracking_node.cpp 와 동일 파라미터)를 재현한다.
       단 ByteTrack 칼만/ID·Lost외삽까지는 재현 못 함(잿슨 전용) → 근사임.

상단 yaw 논리(velocity 모드, controller_base.cpp trackTopYaw 와 동일):
  azimuth = atan2(x, z)   (+ = 주인이 오른쪽)
  |azimuth| <= AZ_DEAD → yaw_cmd = 0.0 (정지)
  azimuth > 0 (오른쪽) → yaw_cmd = -TOP_YAW_SIGN (우회전)
  azimuth < 0 (왼쪽)  → yaw_cmd = +TOP_YAW_SIGN (좌회전)
  ※ 발행값(yaw_cmd) 은 control 과 동일하게 smoothed azimuth 로 계산(로봇이 그러므로).

실행:  py viz_tracking_yaw.py        (OAK-D USB 연결 필요)
필요:  pip install depthai opencv-python numpy
종료:  창에서 q 또는 ESC
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

# ---- control_node 와 일치시킬 파라미터 (params.hpp / control_params.yaml) ----
AZ_DEAD = 0.10          # rad, 중앙 정지 불감대 (≈±5.7°)
TOP_YAW_SIGN = 1.0      # 회전 방향이 실제와 반대면 -1.0

# ---- tracking_node.cpp 와 동일한 평활 파라미터 (/owner_pose 재현용) ----
MAX_SPEED_MMPS = 2000.0   # ±2.0 m/s 속도 클립
EMA_AXY        = 0.35     # x EMA 계수
EMA_AZ_MIN     = 0.08     # z EMA(빠른 좌우이동 시, 강한 평활)
EMA_AZ_MAX     = 0.40     # z EMA(정지 시, 빠른 반응)
DYN_Z_PX_FULL  = 15.0     # 프레임간 픽셀x 이동이 이 값 이상이면 z 평활 최대
GAP_RESET_S    = 1.0      # 주인 끊김 이 시간 넘으면 평활 시드 리셋

# ---- oak_detector.py 와 동일한 파이프라인 설정 ----
MODEL = "yolov6-nano"
PREV_W, PREV_H = 512, 384
CONF = 0.5
BBOX_SCALE = 0.3
DEPTH_LO, DEPTH_HI = 100, 8000   # mm
DISP_SCALE = 2                   # 화면 표시 확대배율


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class OwnerSmoother:
    """tracking_node.cpp 의 속도클립 + 동적 EMA 단계를 그대로 재현(x,z mm)."""
    def __init__(self):
        self.sx = self.sz = 0.0
        self.px = None          # 직전 owner 픽셀 x (동적 alpha_z 용)
        self.t = None
        self.init = False

    def reset(self):
        self.init = False
        self.px = None
        self.t = None

    def update(self, x_mm, z_mm, px, now):
        if not self.init:
            self.sx, self.sz = x_mm, z_mm
            self.px, self.t, self.init = px, now, True
            return self.sx, self.sz
        dt = (now - self.t) if self.t else (1.0 / 30.0)
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / 30.0
        self.t = now
        maxd = MAX_SPEED_MMPS * dt
        cx = _clip(x_mm, self.sx - maxd, self.sx + maxd)   # 속도 클립
        cz = _clip(z_mm, self.sz - maxd, self.sz + maxd)
        alpha_z = EMA_AZ_MAX                                # 동적 z 평활
        if self.px is not None:
            ratio = _clip(abs(px - self.px) / DYN_Z_PX_FULL, 0.0, 1.0)
            alpha_z = EMA_AZ_MAX - ratio * (EMA_AZ_MAX - EMA_AZ_MIN)
        self.sx = EMA_AXY * cx + (1.0 - EMA_AXY) * self.sx
        self.sz = alpha_z * cz + (1.0 - alpha_z) * self.sz
        self.px = px
        return self.sx, self.sz


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
    """controller_base.cpp trackTopYaw (velocity 모드) 와 동일."""
    if abs(azimuth) <= AZ_DEAD:
        return 0.0
    direction = -1.0 if azimuth >= 0.0 else 1.0   # 우측(+)→우회전(-)
    return TOP_YAW_SIGN * direction


def pick_owner(persons):
    """박스중심이 화면 중앙(0.5,0.5)에 가장 가까운 사람을 주인으로."""
    best, best_d = None, 1e9
    for p in persons:
        cx = (p["xmin"] + p["xmax"]) * 0.5
        cy = (p["ymin"] + p["ymax"]) * 0.5
        d = (cx - 0.5) ** 2 + (cy - 0.5) ** 2
        if d < best_d:
            best, best_d = p, d
    return best


def draw_tracking(frame, persons, owner):
    f = cv2.resize(frame, (PREV_W * DISP_SCALE, PREV_H * DISP_SCALE))
    H, W = f.shape[:2]
    cv2.line(f, (W // 2, 0), (W // 2, H), (90, 90, 90), 1)   # 화면 중앙 기준선

    for p in persons:
        x1, y1 = int(p["xmin"] * W), int(p["ymin"] * H)
        x2, y2 = int(p["xmax"] * W), int(p["ymax"] * H)
        is_owner = owner is not None and p is owner
        col = (0, 220, 0) if is_owner else (160, 160, 160)
        cv2.rectangle(f, (x1, y1), (x2, y2), col, 3 if is_owner else 1)
        cv2.putText(f, f"{p['z']/1000:.2f}m", (x1, y2 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        if is_owner:
            cxp = int((p["xmin"] + p["xmax"]) * 0.5 * W)
            cyp = int((p["ymin"] + p["ymax"]) * 0.5 * H)
            cv2.circle(f, (cxp, cyp), 7, (0, 0, 255), -1)      # 주인 중심 빨간점(raw)
            cv2.putText(f, "OWNER", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)

    # ---- 상단 yaw 제어값 텍스트 (control 과 동일하게 smoothed azimuth 사용) ----
    if owner is not None:
        az_sm = math.atan2(owner["sx"], owner["sz"])     # smoothed (로봇이 쓰는 값)
        az_raw = math.atan2(owner["x"], owner["z"])
        cmd = yaw_cmd_for(az_sm)
        if cmd == 0.0:
            side, act = "CENTER", "STOP (deadzone)"
        elif cmd < 0:
            side, act = "RIGHT", "turn RIGHT (cmd -1.0)"
        else:
            side, act = "LEFT", "turn LEFT (cmd +1.0)"
        lines = [
            "TOP-YAW (velocity mode)",
            f"azimuth(smoothed) = {math.degrees(az_sm):+.1f} deg  (raw {math.degrees(az_raw):+.1f})",
            f"deadzone = +-{math.degrees(AZ_DEAD):.1f} deg   owner = {side}",
            f"published top_yaw_target = {cmd:+.1f}   active=True",
            f"-> {act}",
        ]
        col = (0, 255, 255) if cmd != 0.0 else (0, 220, 0)
    else:
        lines = ["TOP-YAW (velocity mode)", "no owner -> top_yaw_target = 0.0 (stop)"]
        col = (0, 0, 255)
    y = 26
    for ln in lines:
        cv2.putText(f, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(f, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1)
        y += 26
    return f


def draw_map(owner, size=460, max_range=4.0):
    m = np.full((size, size + 60, 3), 22, dtype=np.uint8)
    cx, cy = (size + 60) // 2, size - 40         # 로봇 위치(하단 중앙)
    ppm = (size - 70) / max_range                # pixels per meter

    for r in range(1, int(max_range) + 1):       # 거리 링
        cv2.circle(m, (cx, cy), int(r * ppm), (55, 55, 55), 1)
        cv2.putText(m, f"{r}m", (cx + 4, cy - int(r * ppm) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1)
    cv2.line(m, (cx, cy), (cx, cy - int(max_range * ppm)), (70, 70, 70), 1)
    for s in (+1, -1):                            # 불감대 쐐기
        ex = cx + int(math.sin(s * AZ_DEAD) * max_range * ppm)
        ey = cy - int(math.cos(s * AZ_DEAD) * max_range * ppm)
        cv2.line(m, (cx, cy), (ex, ey), (0, 90, 90), 1)
    cv2.drawMarker(m, (cx, cy), (0, 220, 0), cv2.MARKER_TRIANGLE_UP, 16, 2)

    def plot(x_mm, z_mm, color, rad, label=None):
        az = math.atan2(x_mm, z_mm)
        dist = math.hypot(x_mm, z_mm) / 1000.0
        px = cx + int(math.sin(az) * min(dist, max_range) * ppm)
        py = cy - int(math.cos(az) * min(dist, max_range) * ppm)
        cv2.circle(m, (px, py), rad, color, -1)
        return az, dist

    if owner is not None:
        plot(owner["x"], owner["z"], (110, 110, 110), 4)         # raw(회색)
        az, dist = plot(owner["sx"], owner["sz"], (0, 0, 255), 7) # smoothed(빨강)
        cv2.line(m, (cx, cy),
                 (cx + int(math.sin(az) * min(dist, max_range) * ppm),
                  cy - int(math.cos(az) * min(dist, max_range) * ppm)),
                 (0, 180, 255), 1)
        cv2.putText(m, f"d={dist:.2f}m", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(m, f"az={math.degrees(az):+.1f}deg", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (110, 170, 245), 1)
        cv2.putText(m, "red=smoothed(/owner_pose 재현)  gray=raw", (10, size - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    else:
        cv2.putText(m, "no owner", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
    cv2.putText(m, "2D MAP (top-down)", (10, size - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (130, 130, 130), 1)
    return m


def main():
    pipe, q_rgb, q_det = build_pipeline()
    print("OAK 파이프라인 시작 (모델 첫 다운로드 시 잠시 지연). q/ESC 종료.")
    pipe.start()
    frame = np.zeros((PREV_H, PREV_W, 3), dtype=np.uint8)
    persons, owner = [], None
    smoother = OwnerSmoother()
    last_owner_t = 0.0
    try:
        while pipe.isRunning():
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
            in_det = q_det.tryGet()
            if in_det is not None:
                persons = []
                for d in in_det.detections:
                    if int(d.label) != 0:          # person only
                        continue
                    z = float(d.spatialCoordinates.z)
                    if z <= 0.0:                   # 유효 depth 없으면 제외
                        continue
                    persons.append({
                        "xmin": d.xmin, "ymin": d.ymin,
                        "xmax": d.xmax, "ymax": d.ymax,
                        "x": float(d.spatialCoordinates.x),
                        "y": float(d.spatialCoordinates.y),
                        "z": z, "conf": float(d.confidence),
                    })
                owner = pick_owner(persons)
                now = time.time()
                if owner is not None:
                    if now - last_owner_t > GAP_RESET_S:
                        smoother.reset()           # 오래 끊겼으면 시드 리셋
                    px = (owner["xmin"] + owner["xmax"]) * 0.5 * PREV_W
                    sx, sz = smoother.update(owner["x"], owner["z"], px, now)
                    owner["sx"], owner["sz"] = sx, sz
                    last_owner_t = now

            cv2.imshow("OAK Tracking", draw_tracking(frame, persons, owner))
            cv2.imshow("2D Map", draw_map(owner))
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
