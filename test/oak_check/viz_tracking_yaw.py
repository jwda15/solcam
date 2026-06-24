#!/usr/bin/env python3
"""viz_tracking_yaw.py — OAK-D 트래킹 + 상단 yaw 제어 논리 시각화 (ROS 불필요, 윈도우 OK)

목적: 잿슨/ROS 없이 윈도우에서 OAK-D 만 꽂고 트래킹 성능과 상단 yaw 제어 논리를
      눈으로 검증한다. oak_detector.py 와 "동일한" depthai 파이프라인
      (SpatialDetectionNetwork, yolov6-nano, person-only)을 쓴다.

화면 2개:
  [OAK Tracking]  RGB + 사람 박스(회색) + 주인 박스(초록) + 주인중심 빨간점
                  + 상단 yaw 발행값 텍스트(현 control_node 논리 그대로 재현)
  [2D Map]        위에서 본 평면도 — 로봇 기준 주인의 거리/방위각

상단 yaw 논리(현 펌웨어=velocity 모드, controller_base.cpp trackTopYaw 와 동일):
  azimuth = atan2(spatial_x, spatial_z)   (+ = 주인이 화면 오른쪽)
  |azimuth| <= AZ_DEAD            → yaw_cmd = 0.0           (정지: 거의 중앙)
  azimuth > 0 (오른쪽)            → yaw_cmd = -TOP_YAW_SIGN (우회전, 주인 쪽)
  azimuth < 0 (왼쪽)             → yaw_cmd = +TOP_YAW_SIGN (좌회전)
  실차에서 회전이 반대면 TOP_YAW_SIGN 을 -1 로.

주인 선택: 사람 detection 중 박스중심이 화면 중앙에 가장 가까운 것
          (control_node 의 '화면 중앙에 가장 가까운 트랙을 주인으로' 초기화와 동일 취지).

실행:  py viz_tracking_yaw.py        (OAK-D USB 연결 필요)
필요:  pip install depthai opencv-python numpy
종료:  창에서 q 또는 ESC
"""
import math
import sys

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

# ---- oak_detector.py 와 동일한 파이프라인 설정 ----
MODEL = "yolov6-nano"
PREV_W, PREV_H = 512, 384
CONF = 0.5
BBOX_SCALE = 0.3
DEPTH_LO, DEPTH_HI = 100, 8000   # mm
DISP_SCALE = 2                   # 화면 표시 확대배율


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
            cv2.circle(f, (cxp, cyp), 7, (0, 0, 255), -1)      # 주인 중심 빨간점
            cv2.putText(f, "OWNER", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)

    # ---- 상단 yaw 제어값 텍스트 패널 ----
    if owner is not None:
        az = math.atan2(owner["x"], owner["z"])
        az_deg = math.degrees(az)
        cmd = yaw_cmd_for(az)
        if cmd == 0.0:
            side, act = "CENTER", "STOP (deadzone)"
        elif cmd < 0:
            side, act = "RIGHT", "turn RIGHT (cmd -1.0)"
        else:
            side, act = "LEFT", "turn LEFT (cmd +1.0)"
        lines = [
            "TOP-YAW (velocity mode)",
            f"azimuth = {az_deg:+.1f} deg   (deadzone +-{math.degrees(AZ_DEAD):.1f})",
            f"owner side = {side}",
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

    # 거리 링
    for r in range(1, int(max_range) + 1):
        cv2.circle(m, (cx, cy), int(r * ppm), (55, 55, 55), 1)
        cv2.putText(m, f"{r}m", (cx + 4, cy - int(r * ppm) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1)
    # 정면(전방) 기준선 + 불감대 쐐기
    cv2.line(m, (cx, cy), (cx, cy - int(max_range * ppm)), (70, 70, 70), 1)
    for s in (+1, -1):
        ex = cx + int(math.sin(s * AZ_DEAD) * max_range * ppm)
        ey = cy - int(math.cos(s * AZ_DEAD) * max_range * ppm)
        cv2.line(m, (cx, cy), (ex, ey), (0, 90, 90), 1)
    # 로봇(삼각형, 전방=위)
    cv2.drawMarker(m, (cx, cy), (0, 220, 0), cv2.MARKER_TRIANGLE_UP, 16, 2)

    if owner is not None:
        az = math.atan2(owner["x"], owner["z"])
        dist = math.hypot(owner["x"], owner["z"]) / 1000.0
        px = cx + int(math.sin(az) * min(dist, max_range) * ppm)
        py = cy - int(math.cos(az) * min(dist, max_range) * ppm)
        cv2.line(m, (cx, cy), (px, py), (0, 180, 255), 1)
        cv2.circle(m, (px, py), 7, (0, 0, 255), -1)
        cv2.putText(m, f"d={dist:.2f}m", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(m, f"az={math.degrees(az):+.1f}deg", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (110, 170, 245), 1)
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
