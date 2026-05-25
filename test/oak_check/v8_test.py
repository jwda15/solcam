import time, depthai as dai
ARCH = "/media/jw/로컬 디스크/32/capstone/0524solcam/ros2_yolo_oak/models/yolov8n.tar.xz"
p = dai.Pipeline()
cam   = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = p.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setExtendedDisparity(False); stereo.setSubpixel(False); stereo.setLeftRightCheck(True)
monoL.requestOutput((640,640)).link(stereo.left)
monoR.requestOutput((640,640)).link(stereo.right)
arch = dai.NNArchive(ARCH)
nn = p.create(dai.node.SpatialDetectionNetwork).build(cam, stereo, arch)
nn.input.setBlocking(False)
nn.setConfidenceThreshold(0.5)
nn.setBoundingBoxScaleFactor(0.5)
nn.setDepthLowerThreshold(100); nn.setDepthUpperThreshold(8000)
qDet = nn.out.createOutputQueue(maxSize=4, blocking=False)
qRgb = nn.passthrough.createOutputQueue(maxSize=4, blocking=False)
p.start()
print("v8 archive loaded, polling 12s...")
t0=time.time(); got=0; persons=0
while time.time()-t0 < 12.0:
    inDet = qDet.tryGet()
    if inDet is None:
        time.sleep(0.02); continue
    got += 1
    pc = sum(1 for d in inDet.detections if d.label==0)
    persons += pc
    if got <= 5:
        for d in inDet.detections:
            if d.label==0:
                print(f"  person conf={d.confidence:.2f} z={d.spatialCoordinates.z/1000:.2f}m")
                break
    qRgb.tryGet()
p.stop()
print(f"RESULT: det_msgs={got} total_persons={persons}")
