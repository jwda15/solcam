#!/usr/bin/env python3
import time, numpy as np, cv2, depthai as dai
OUT_DIR = "/media/jw/로컬 디스크/32/capstone/0524solcam/test/oak_check"

def main():
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    mono_l  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_r  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    rgb_out = cam_rgb.requestOutput((640, 400), dai.ImgFrame.Type.BGR888i)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(True); stereo.setSubpixel(True); stereo.setExtendedDisparity(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(640, 400)
    mono_l.requestOutput((640, 400)).link(stereo.left)
    mono_r.requestOutput((640, 400)).link(stereo.right)
    q_rgb = rgb_out.createOutputQueue(); q_depth = stereo.depth.createOutputQueue()
    pipeline.start(); print("[capture] started, warming up...")
    rgb_frame=None; depth_frame=None; t0=time.time()
    while time.time()-t0 < 8.0:
        ir=q_rgb.tryGet(); idp=q_depth.tryGet()
        if ir is not None: rgb_frame=ir.getCvFrame()
        if idp is not None: depth_frame=idp.getFrame()
        if rgb_frame is not None and depth_frame is not None:
            time.sleep(0.3)
            ir=q_rgb.tryGet(); idp=q_depth.tryGet()
            if ir is not None: rgb_frame=ir.getCvFrame()
            if idp is not None: depth_frame=idp.getFrame()
            break
        time.sleep(0.02)
    pipeline.stop()
    if rgb_frame is None or depth_frame is None:
        print("[capture] FAIL rgb=%s depth=%s"%(rgb_frame is not None, depth_frame is not None)); return
    cv2.imwrite(OUT_DIR+"/rgb.png", rgb_frame)
    d=depth_frame.astype(np.float32); valid=d[d>0]
    dmax=np.percentile(valid,95) if valid.size else 1.0
    vis=np.clip(d/max(dmax,1.0)*255,0,255).astype(np.uint8)
    vis=cv2.applyColorMap(vis,cv2.COLORMAP_JET); vis[d==0]=0
    cv2.imwrite(OUT_DIR+"/depth_vis.png", vis)
    total=d.size; nvalid=valid.size
    print("=== RGB ==="); print("  shape:", rgb_frame.shape)
    print("=== DEPTH (mm) ==="); print("  shape:", depth_frame.shape, "dtype:", depth_frame.dtype)
    print("  valid: %d/%d (%.1f%%)"%(nvalid,total,100.0*nvalid/total))
    if nvalid:
        print("  range: min=%d max=%d median=%.0f mean=%.0f"%(valid.min(),valid.max(),np.median(valid),valid.mean()))
        for lo,hi in [(0,500),(500,1000),(1000,2000),(2000,4000),(4000,99999)]:
            c=np.sum((valid>=lo)&(valid<hi)); print("    %5d-%-5dmm: %5.1f%%"%(lo,hi,100.0*c/nvalid))
    print("\n[capture] saved rgb.png, depth_vis.png")

if __name__=="__main__": main()
