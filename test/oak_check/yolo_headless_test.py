import numpy as np, cv2, depthai as dai, time
p = dai.Pipeline()
cam   = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = p.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setLeftRightCheck(True); stereo.setSubpixel(False); stereo.setExtendedDisparity(False)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
monoL.requestOutput((640,400)).link(stereo.left)
monoR.requestOutput((640,400)).link(stereo.right)
md = dai.NNModelDescription("yolov6-nano"); md.platform = p.getDefaultDevice().getPlatformAsString()
arch = dai.NNArchive(dai.getModelFromZoo(md))
sdn = p.create(dai.node.SpatialDetectionNetwork).build(cam, stereo, arch)
sdn.setConfidenceThreshold(0.5); sdn.setBoundingBoxScaleFactor(0.5)
sdn.setDepthLowerThreshold(200); sdn.setDepthUpperThreshold(8000)
print("classes:", sdn.getClasses()[:5], "... total", len(sdn.getClasses()))
qd = sdn.out.createOutputQueue()
p.start()
print("running 6s, counting detections...")
t0=time.time(); frames=0; total_det=0; persons=0
while time.time()-t0 < 6.0:
    ind = qd.tryGet()
    if ind is not None:
        frames += 1
        for d in ind.detections:
            total_det += 1
            if d.label==0:
                persons += 1
                if persons<=3:
                    print("  person conf=%.2f z=%.2fm xy=(%.2f,%.2f)"%(
                        d.confidence, d.spatialCoordinates.z/1000,
                        d.spatialCoordinates.x/1000, d.spatialCoordinates.y/1000))
    time.sleep(0.01)
p.stop()
print("frames=%d total_det=%d person_det=%d"%(frames,total_det,persons))
