#!/usr/bin/env bash
# 폰 카메라를 받을 가상 비디오 장치(/dev/videoN) 생성. 부팅마다 1회.
# 사용: sudo ./setup_v4l2loopback.sh [device_nr]
set -e
DEV_NR="${1:-2}"
if ! lsmod | grep -q v4l2loopback; then
  echo "[setup] v4l2loopback 로드 (/dev/video${DEV_NR})"
  # ★exclusive_caps=0 필수. =1 이면 scrcpy 가 sink 헤더를 못 써 즉사("Failed to write header").
  sudo modprobe v4l2loopback video_nr="${DEV_NR}" card_label="solcam_phone" exclusive_caps=0
else
  echo "[setup] v4l2loopback 이미 로드됨"
fi
ls -l "/dev/video${DEV_NR}" || { echo "장치 생성 실패"; exit 1; }
echo "[setup] 준비됨: /dev/video${DEV_NR}"
echo "  미설치라면: sudo apt install v4l2loopback-dkms v4l-utils"
