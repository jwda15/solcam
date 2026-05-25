import time, depthai as dai
p = dai.Pipeline()
cam   = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = p.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setExtendedDisparity(False); stereo.setSubpixel(False); stereo.setLeftRightCheck(True)
#stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
#stereo.setOutputSize(640,400)
monoL.requestOutput((640,400)).link(stereo.left)
monoR.requestOutput((640,400)).link(stereo.right)
nn = p.create(dai.node.SpatialDetectionNetwork).build(cam, stereo, dai.NNModelDescription("yolov6-nano"))
nn.input.setBlocking(False)
nn.setConfidenceThreshold(0.5)
nn.setBoundingBoxScaleFactor(0.5)
nn.setDepthLowerThreshold(100); nn.setDepthUpperThreshold(8000)
# detector와 동일: passthrough + out 둘 다 maxSize=4 blocking=False
qRgb = nn.passthrough.createOutputQueue(maxSize=4, blocking=False)
qDet = nn.out.createOutputQueue(maxSize=4, blocking=False)
p.start()
print("started. polling out queue like detector...")
t0=time.time(); got_det=0; got_none=0; total_persons=0
while time.time()-t0 < 12.0:
    inDet = qDet.tryGet()   # detector와 동일하게 out 먼저
    if inDet is None:
        got_none += 1
        time.sleep(0.02)
        continue
    got_det += 1
    n = len(inDet.detections)
    persons = sum(1 for d in inDet.detections if d.label==0)
    total_persons += persons
    if got_det <= 5:
        print(f"  det msg: {n} dets, {persons} persons")
    # detector처럼 passthrough도 읽기
    qRgb.tryGet()
p.stop()
print(f"RESULT: det_msgs={got_det} none_polls={got_none} total_persons={total_persons}")
