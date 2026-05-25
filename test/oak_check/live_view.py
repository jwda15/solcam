#!/usr/bin/env python3
"""OAK-D S2 실시간 RGB + Depth 뷰어 (DepthAI v3)
   q 또는 ESC 키로 종료."""
import numpy as np, cv2, depthai as dai

def main():
    p = dai.Pipeline()
    cam   = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    rgb_out = cam.requestOutput((640, 400), dai.ImgFrame.Type.BGR888i)

    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.setExtendedDisparity(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(640, 400)
    monoL.requestOutput((640, 400)).link(stereo.left)
    monoR.requestOutput((640, 400)).link(stereo.right)

    q_rgb   = rgb_out.createOutputQueue()
    q_depth = stereo.depth.createOutputQueue()

    p.start()
    print("실시간 뷰어 시작. 창에서 q 또는 ESC 로 종료.")
    rgb = None
    while p.isRunning():
        ir  = q_rgb.tryGet()
        idp = q_depth.tryGet()
        if ir is not None:
            rgb = ir.getCvFrame()
            cv2.imshow("OAK RGB", rgb)
        if idp is not None:
            d = idp.getFrame().astype(np.float32)
            valid = d[d > 0]
            dmax = np.percentile(valid, 95) if valid.size else 4000.0
            vis = np.clip(d / max(dmax, 1.0) * 255, 0, 255).astype(np.uint8)
            vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
            vis[d == 0] = 0
            # 중앙 픽셀 거리 표시
            cy, cx = d.shape[0]//2, d.shape[1]//2
            cd = d[cy, cx]
            cv2.circle(vis, (cx, cy), 4, (255,255,255), 1)
            cv2.putText(vis, f"center: {cd/1000:.2f} m", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.imshow("OAK Depth", vis)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
    p.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
