#!/usr/bin/env python3
"""OAK-D S2 온보드 YOLO + Spatial(거리) 실시간 뷰어 (DepthAI v3)
   - YOLOv6-nano(COCO) 온보드 추론
   - 박스마다 3D 거리(spatialCoordinates.z) 표시
   - 사람(class 0)은 초록, 그 외는 회색
   q 또는 ESC로 종료."""
import numpy as np, cv2, depthai as dai

def main():
    p = dai.Pipeline()
    cam   = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(False)
    stereo.setExtendedDisparity(False)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    monoL.requestOutput((640, 400)).link(stereo.left)
    monoR.requestOutput((640, 400)).link(stereo.right)

    # 모델 (zoo 자동 다운로드, RVC2)
    model_desc = dai.NNModelDescription("yolov6-nano")
    model_desc.platform = p.getDefaultDevice().getPlatformAsString()
    nn_archive = dai.NNArchive(dai.getModelFromZoo(model_desc))

    # SpatialDetectionNetwork: 카메라 + 스테레오 묶어서 build
    sdn = p.create(dai.node.SpatialDetectionNetwork).build(cam, stereo, nn_archive)
    sdn.setConfidenceThreshold(0.5)
    sdn.setBoundingBoxScaleFactor(0.5)
    sdn.setDepthLowerThreshold(200)    # mm
    sdn.setDepthUpperThreshold(8000)   # mm

    labelMap = sdn.getClasses()

    q_rgb = sdn.passthrough.createOutputQueue()
    q_det = sdn.out.createOutputQueue()

    p.start()
    print("YOLO 실시간 시작. 창에서 q 또는 ESC 종료.")
    while p.isRunning():
        inr = q_rgb.tryGet()
        ind = q_det.tryGet()
        if inr is None:
            cv2.waitKey(1); continue
        frame = inr.getCvFrame()
        h, w = frame.shape[:2]
        if ind is not None:
            for d in ind.detections:
                x1, y1 = int(d.xmin*w), int(d.ymin*h)
                x2, y2 = int(d.xmax*w), int(d.ymax*h)
                is_person = (d.label == 0)
                color = (0,255,0) if is_person else (160,160,160)
                cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                name = labelMap[d.label] if labelMap and d.label < len(labelMap) else str(d.label)
                zx = d.spatialCoordinates.x/1000.0
                zz = d.spatialCoordinates.z/1000.0
                txt = f"{name} {d.confidence:.2f} | {zz:.2f}m"
                cv2.putText(frame, txt, (x1, max(15,y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        cv2.imshow("OAK YOLO + Spatial", frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
    p.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
