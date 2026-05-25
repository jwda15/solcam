import cv2
import numpy as np
import ctypes
import os
from ultralytics import YOLO

class CObject(ctypes.Structure):
    """C++ Object 구조체와 메모리 레이아웃 일치"""
    _fields_ = [
        ("x",     ctypes.c_float),
        ("y",     ctypes.c_float),
        ("w",     ctypes.c_float),
        ("h",     ctypes.c_float),
        ("depth", ctypes.c_float),
        ("label", ctypes.c_int),
        ("prob",  ctypes.c_float),
    ]

class CTrackResult(ctypes.Structure):
    _fields_ = [
        ("x1",       ctypes.c_float),
        ("y1",       ctypes.c_float),
        ("x2",       ctypes.c_float),
        ("y2",       ctypes.c_float),
        ("track_id", ctypes.c_int),
        ("label",    ctypes.c_int),
        ("score",    ctypes.c_float),
    ]

def load_tum_pairs(dataset_path):
    def parse_tum_txt(filepath):
        entries = []
        with open(filepath) as f:
            for line in f:
                line = line.strip()
                if line.startswith("#") or not line:
                    continue
                ts, fname = line.split()
                entries.append((float(ts), fname))
        return entries
    rgb_list   = parse_tum_txt(os.path.join(dataset_path, "rgb.txt"))
    depth_list = parse_tum_txt(os.path.join(dataset_path, "depth.txt"))
    pairs = []
    depth_ts = np.array([d[0] for d in depth_list])
    for ts, rgb_fname in rgb_list:
        idx = np.argmin(np.abs(depth_ts - ts))
        if abs(depth_ts[idx] - ts) < 0.05:
            pairs.append((rgb_fname, depth_list[idx][1]))
    print(f"[데이터셋] RGB-Depth 매칭 쌍: {len(pairs)}개")
    return pairs

def get_depth_at_bbox(depth_img, x1, y1, x2, y2):
    """bbox 중심 ROI의 중앙값 depth 반환 (mm)"""
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)
    roi_size = 20
    roi = depth_img[
        max(0, cy-roi_size):min(depth_img.shape[0], cy+roi_size),
        max(0, cx-roi_size):min(depth_img.shape[1], cx+roi_size)
    ]
    valid = roi[roi > 0]
    return float(np.median(valid)) if len(valid) > 0 else 0.0

def main():
    dataset_path = "/mnt/d/mot_test/rgbd_dataset_freiburg3_walking_xyz"
    lib_path     = "/mnt/d/ByteTrack-cpp/build/libbytetrack.so"
    yolo_path    = os.path.expanduser("~/tracking_ws/yolov8n.pt")
    output_path  = "/mnt/d/mot_test/output_tracking.avi"

    print("[초기화] libbytetrack.so 로드 중...")
    lib = ctypes.CDLL(lib_path)
    lib.create_tracker.restype  = ctypes.c_void_p
    lib.create_tracker.argtypes = [ctypes.c_int, ctypes.c_int,
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.update_tracker.restype  = ctypes.c_int
    lib.update_tracker.argtypes = [ctypes.c_void_p,
                                   ctypes.POINTER(CObject), ctypes.c_int,
                                   ctypes.POINTER(CTrackResult), ctypes.c_int]
    lib.destroy_tracker.restype  = None
    lib.destroy_tracker.argtypes = [ctypes.c_void_p]
    print("[초기화] 라이브러리 로드 성공")

    print("[초기화] YOLOv8n 로드 중...")
    model = YOLO(yolo_path)
    print("[초기화] YOLOv8n 로드 성공")

    tracker = lib.create_tracker(30, 30, 0.5, 0.6, 0.8)
    print("[초기화] BYTETracker 생성 성공")

    pairs = load_tum_pairs(dataset_path)

    first_rgb = cv2.imread(os.path.join(dataset_path, pairs[0][0]))
    h, w = first_rgb.shape[:2]
    writer = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*"XVID"), 15, (w, h))

    color_map = {}
    def get_color(tid):
        if tid not in color_map:
            np.random.seed(tid * 37)
            color_map[tid] = tuple(int(c) for c in np.random.randint(50, 220, 3))
        return color_map[tid]

    max_results = 50
    results_buf = (CTrackResult * max_results)()

    print(f"[처리] 총 {len(pairs)}프레임 시작...")
    for frame_idx, (rgb_fname, depth_fname) in enumerate(pairs):
        rgb_img   = cv2.imread(os.path.join(dataset_path, rgb_fname))
        depth_img = cv2.imread(os.path.join(dataset_path, depth_fname),
                               cv2.IMREAD_ANYDEPTH).astype(np.float32) * 0.2
        if rgb_img is None or depth_img is None:
            continue

        det_results = model(rgb_img, classes=[0], verbose=False)[0]
        boxes  = det_results.boxes
        n_dets = len(boxes)
        dets_buf = (CObject * max(n_dets, 1))()
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            dets_buf[i].x     = x1
            dets_buf[i].y     = y1
            dets_buf[i].w     = x2 - x1
            dets_buf[i].h     = y2 - y1
            dets_buf[i].depth = get_depth_at_bbox(depth_img, x1, y1, x2, y2)
            dets_buf[i].label = 0
            dets_buf[i].prob  = float(box.conf[0])

        n_tracks = lib.update_tracker(tracker, dets_buf, n_dets, results_buf, max_results)

        vis = rgb_img.copy()
        for i in range(n_tracks):
            t = results_buf[i]
            c = get_color(t.track_id)
            cv2.rectangle(vis, (int(t.x1), int(t.y1)), (int(t.x2), int(t.y2)), c, 2)
            cv2.putText(vis, f"ID:{t.track_id}",
                        (int(t.x1), int(t.y1)-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
        cv2.putText(vis, f"Frame:{frame_idx+1}  Tracks:{n_tracks}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        writer.write(vis)

        if frame_idx % 50 == 0:
            print(f"  [{frame_idx+1}/{len(pairs)}] tracks={n_tracks}")

    writer.release()
    lib.destroy_tracker(tracker)
    print(f"[완료] 결과 저장: {output_path}")

if __name__ == "__main__":
    main()
