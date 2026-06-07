#!/usr/bin/env bash
# ============================================================================
#  solcam 운영 헬퍼 — DDS/v4l2loopback 꼬임을 매번 깨끗이 풀고 올바른 순서로 기동.
#
#  증상별 원인(0607 진단):
#   - "폰 카메라 안 뜸"      → scrcpy 를 kill -9/급단절로 죽여 /dev/videoN 포맷 잠김
#                              (VIDIOC_G_FMT: Invalid argument). v4l2loopback 리로드 필요.
#   - "내가 띄우면 검은화면"  → kill -9 누적으로 /dev/shm/fastrtps_* 잔여 → DDS 합류 실패.
#
#  사용:
#    scripts/solcam.sh clean     # 전부 종료 + DDS shm 정리 + v4l2loopback 리로드
#    scripts/solcam.sh run       # 올바른 순서로 기동 (OAK→gesture→phone→ui)
#    scripts/solcam.sh stop      # graceful 종료(scrcpy 포함)
#    scripts/solcam.sh status    # 노드/장치 상태
#
#  ★자기 환경에 맞게 아래 변수만 수정(또는 env로 덮어쓰기).
# ============================================================================
set -u

REPO="${SOLCAM_REPO:-$HOME/solcam}"                 # 이 repo 경로
WS="${SOLCAM_WS:-$HOME/solcam_ws}"                  # colcon 워크스페이스
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
VIDEO_NR="${SOLCAM_VIDEO_NR:-2}"                    # /dev/video2
VIDEO_DEV="/dev/video${VIDEO_NR}"
CAM_SIZE="${SOLCAM_CAM_SIZE:-1280x720}"
SCRCPY_DIR="${SCRCPY_DIR:-$HOME/scrcpy_bin/scrcpy-linux-x86_64-v4.0}"
SCRCPY_BIN="${SCRCPY_BIN:-$SCRCPY_DIR/scrcpy}"
ADB_BIN="${ADB_BIN:-$SCRCPY_DIR/adb}"
LOG="${SOLCAM_LOG:-/tmp/solcam}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"

NODES=(phone_bridge gesture_node ui_node control_node oak_detector)

src() { source "$ROS_SETUP"; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; }

stop() {
  echo "[stop] 노드 graceful 종료(SIGINT)..."
  for n in "${NODES[@]}"; do pkill -INT -f "$n" 2>/dev/null; done
  sleep 3
  for n in "${NODES[@]}"; do pkill -INT -f "$n" 2>/dev/null; done
  # scrcpy 는 graceful(SIGTERM) — 절대 -9 금지(장치 꼬임)
  pkill -TERM -f "v4l2-sink=${VIDEO_DEV}" 2>/dev/null
  "$ADB_BIN" shell pkill -f com.genymobile.scrcpy 2>/dev/null || true
  sleep 1
  echo "[stop] 완료."
}

clean() {
  echo "[clean] 1) 노드 종료"
  for n in "${NODES[@]}"; do pkill -INT -f "$n" 2>/dev/null; done; sleep 2
  for n in "${NODES[@]}"; do pkill -KILL -f "$n" 2>/dev/null; done
  pkill -TERM -f "v4l2-sink=${VIDEO_DEV}" 2>/dev/null
  "$ADB_BIN" shell pkill -f com.genymobile.scrcpy 2>/dev/null || true
  sleep 1

  echo "[clean] 2) ROS2 daemon + DDS shm 잔여 제거(검은화면 원인)"
  ros2 daemon stop 2>/dev/null || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

  echo "[clean] 3) v4l2loopback 리로드(/dev/videoN 포맷 잠김 해제) — sudo 필요"
  sudo modprobe -r v4l2loopback 2>/dev/null || true
  sudo modprobe v4l2loopback video_nr="${VIDEO_NR}" card_label=solcam_phone exclusive_caps=1
  ls -l "$VIDEO_DEV" 2>/dev/null && echo "[clean] 완료: $VIDEO_DEV 재생성됨" \
    || echo "[clean] ! $VIDEO_DEV 없음 — v4l2loopback-dkms 설치 확인"
}

status() {
  echo "== 노드 =="; for n in "${NODES[@]}"; do
    pgrep -f "$n" >/dev/null && echo "  $n: 실행중($(pgrep -f "$n" | tr '\n' ' '))" || echo "  $n: 없음"; done
  echo "== v4l2 =="; v4l2-ctl -d "$VIDEO_DEV" --get-fmt-video 2>/dev/null | sed -n '2p' || echo "  $VIDEO_DEV 없음/미준비"
  echo "== adb =="; "$ADB_BIN" devices 2>/dev/null | sed '1d'
  echo "== shm 잔여 =="; ls /dev/shm/fastrtps_* 2>/dev/null | wc -l | sed 's/^/  fastrtps shm 파일: /'
}

run() {
  mkdir -p "$LOG"; src
  echo "[run] DISPLAY=$DISPLAY  WS=$WS  VIDEO=$VIDEO_DEV"
  echo "[run] 1) OAK 검출+추적 (로그 $LOG/oak.log)"
  ( cd "$REPO" && ros2 launch ros2_yolo_oak oak_tracking.launch.py viz:=false ) >"$LOG/oak.log" 2>&1 &
  sleep 6
  echo "[run] 2) gesture_node (입력=/phone/image, 로그 $LOG/gesture.log)"
  ros2 launch ros2_gesture_node gesture.launch.py ui:=false \
       image_topic:=/phone/image >"$LOG/gesture.log" 2>&1 &
  sleep 3
  echo "[run] 3) phone_bridge (managed scrcpy, 로그 $LOG/phone.log)"
  ros2 run ros2_phone_bridge phone_bridge --ros-args \
       -p video_device:="$VIDEO_DEV" -p manage_scrcpy:=true \
       -p scrcpy_bin:="$SCRCPY_BIN" -p adb_bin:="$ADB_BIN" \
       -p camera_size:="$CAM_SIZE" >"$LOG/phone.log" 2>&1 &
  echo "[run] 4) /phone/image 첫 프레임 대기..."
  for i in $(seq 1 20); do
    sleep 1
    if ros2 topic hz /phone/image --window 5 2>/dev/null | grep -q average; then
      echo "[run]   /phone/image 수신 OK"; break; fi
    [ "$i" = 20 ] && echo "[run]   ! 아직 프레임 없음 — $LOG/phone.log 확인(포맷게이트/ scrcpy)"
  done
  echo "[run] 5) ui_node (LCD, 로그 $LOG/ui.log) — 디스커버리+첫프레임 ~10초, 검정이어도 기다릴 것"
  ros2 run ros2_gesture_node ui_node --ros-args \
       -p fullscreen:=false >"$LOG/ui.log" 2>&1 &
  echo "[run] 기동 완료. 종료는: scripts/solcam.sh stop"
}

case "${1:-}" in
  clean) clean ;;
  run)   run ;;
  stop)  stop ;;
  status) src; status ;;
  *) echo "사용: $0 {clean|run|stop|status}"; exit 1 ;;
esac
