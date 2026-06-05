# models/

HaGRID 사전학습 가중치 보관 위치. 라이선스(CC BY-SA 4.0 변형) 때문에
깃에 커밋하지 않는다 — `./download.sh`로 받는다.

- `YOLOv10n_gestures.pt` — 제스처 검출기 (gesture_node 기본값)
- Jetson에서 빠르게 돌리려면 TensorRT 변환:
  `yolo export model=YOLOv10n_gestures.pt format=engine half=True`
  후 `model_path`를 `.engine`으로 변경 (ultralytics가 그대로 로드)
