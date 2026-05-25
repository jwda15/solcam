import time, depthai as dai
p = dai.Pipeline()
cam   = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = p.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
# 품질 옵션 ON (depth 노이즈 개선) - shave 충돌 나는지 확인
stereo.setExtendedDisparity(False)
stereo.setSubpixel(False)
stereo.setLeftRightCheck(True)
monoL.requestOutput((512,384)).link(stereo.left)
monoR.requestOutput((512,384)).link(stereo.right)
# v6-nano 416 정사각 (zoo)
nn = p.create(dai.node.SpatialDetectionNetwork).build(
    cam, stereo, dai.NNModelDescription("yolov6-nano"))
nn.input.setBlocking(False)
nn.setConfidenceThreshold(0.5)
nn.setBoundingBoxScaleFactor(0.5)
nn.setDepthLowerThreshold(100); nn.setDepthUpperThreshold(8000)
qDet = nn.out.createOutputQueue(maxSize=4, blocking=False)
qRgb = nn.passthrough.createOutputQueue(maxSize=4, blocking=False)
p.start()
print("v6-nano 416 + extended/subpixel ON, 12s 테스트...")
t0=time.time(); got=0; persons=0
while time.time()-t0 < 12.0:
    inDet = qDet.tryGet()
    if inDet is None:
        time.sleep(0.02); continue
    got += 1
    pc = sum(1 for d in inDet.detections if d.label==0)
    persons += pc
    if got <= 3:
        for d in inDet.detections:
            if d.label==0:
                print(f"  person conf={d.confidence:.2f} z={d.spatialCoordinates.z/1000:.2f}m"); break
    qRgb.tryGet()
p.stop()
print(f"RESULT: det_msgs={got} total_persons={persons}")
