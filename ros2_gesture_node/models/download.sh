#!/usr/bin/env bash
# HaGRIDv2 사전학습 제스처 검출기 다운로드 (hukenovs/hagrid 공식 배포)
#   YOLOv10n: 경량(22MB), mAP 88.2 — Jetson 권장
#   YOLOv10x: 대형, mAP 89.4 — 정확도 비교용
# 라이선스: CC BY-SA 4.0 변형 (저장소 license/en_us.pdf 확인) — 레포에 커밋하지 말 것.
set -e
cd "$(dirname "$0")"
BASE=https://rndml-team-cv.obs.ru-moscow-1.hc.sbercloud.ru/datasets/hagrid_v2/models
curl -L -o YOLOv10n_gestures.pt "$BASE/YOLOv10n_gestures.pt"
# curl -L -o YOLOv10x_gestures.pt "$BASE/YOLOv10x_gestures.pt"   # 필요시
ls -lh *.pt
