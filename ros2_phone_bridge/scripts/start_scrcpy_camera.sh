#!/usr/bin/env bash
# 폰 후면 카메라를 USB로 받아 v4l2 장치에 흘려보낸다(scrcpy>=2.0).
# 사용: ./start_scrcpy_camera.sh [/dev/videoN]
set -e
DEV="${1:-/dev/video2}"
echo "[scrcpy] 폰 후면 카메라 → ${DEV}  (Ctrl+C 로 종료)"
echo "  사전: 폰 USB 디버깅 ON, 'adb devices' 에 기기 보일 것"
exec scrcpy \
  --video-source=camera \
  --camera-facing=back \
  --v4l2-sink="${DEV}" \
  --no-audio --no-window --no-playback
