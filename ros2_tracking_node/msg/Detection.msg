# YOLO + D435i depth로 만든 사람 detection 1개
# bbox: 픽셀 좌표 (좌상단 + 폭/높이)
# depth: bbox 영역 depth 중앙값 (mm) — 0이면 invalid
float32 x        # bbox 좌상단 x (px)
float32 y        # bbox 좌상단 y (px)
float32 w        # bbox 폭 (px)
float32 h        # bbox 높이 (px)
float32 depth    # bbox 영역 depth (mm)
float32 score    # YOLO confidence (0~1)
int32   label    # COCO class id (사람=0)
