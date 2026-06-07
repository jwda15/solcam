#!/usr/bin/env bash
# ============================================================================
#  solcam 운영 헬퍼 — 안 꼬이게 깨끗이 기동/정리. (절대 kill -9 안 씀)
#
#  배운 교훈(0607): kill -9 남발이 두 증상의 원인이었다 —
#    · scrcpy -9  → /dev/videoN 포맷 잠김(VIDIOC_G_FMT) → 폰 영상 죽음
#    · 노드 -9    → /dev/shm/fastrtps_* 누적 → DDS 합류 실패(검은 화면)
#  이 스크립트는 graceful 종료 + shm 정리 + v4l2 리로드 + 올바른 순서 기동으로
#  그 둘을 원천 차단한다. phone_bridge 에는 자동복구(v4l2_reset_cmd)도 넘긴다.
#
#  사용: scripts/solcam.sh {clean|run|stop|status}
#  ★환경 변수만 본인 것으로 맞추거나 export 로 덮어쓰기.
# ============================================================================
set -u

REPO="${SOLCAM_REPO:-$HOME/solcam}"
WS="${SOLCAM_WS:-$HOME/solcam_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
VIDEO_NR="${SOLCAM_VIDEO_NR:-2}"
VIDEO_DEV="/dev/video${VIDEO_NR}"
CAM_SIZE="${SOLCAM_CAM_SIZE:-1280x720}"
SCRCPY_DIR="${SCRCPY_DIR:-$HOME/scrcpy_bin/scrcpy-linux-x86_64-v4.0}"
SCRCPY_BIN="${SCRCPY_BIN:-$SCRCPY_DIR/scrcpy}"
ADB_BIN="${ADB_BIN:-$SCRCPY_DIR/adb}"
LOG="${SOLCAM_LOG:-/tmp/solcam}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"

# v4l2 리로드 스크립트: 보안 설치본(root 소유) 우선, 없으면 repo 스크립트
RESET_SECURE=/usr/local/sbin/solcam_reset_v4l2.sh
if [ -x "$RESET_SECURE" ]; then RESET="$RESET_SECURE"; else RESET="$REPO/scripts/reset_v4l2.sh"; fi
RESET_CMD="sudo $RESET $VIDEO_NR"     # phone_bridge 자동복구 명령(sudoers NOPASSWD 권장)

NODES=(phone_bridge gesture_node ui_node control_node oak_detector tracking_node oak_viz oak_tracking.launch gesture.launch)

src() { set +u; source "$ROS_SETUP"; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u; }  # ROS setup.bash 는 set -u 비안전 → 소싱 동안만 해제

stop() {
  echo "[stop] 노드 graceful 종료(SIGINT→SIGTERM)..."
  for n in "${NODES[@]}"; do pkill -INT -f "$n" 2>/dev/null; done
  sleep 3
  for n in "${NODES[@]}"; do pkill -TERM -f "$n" 2>/dev/null; done   # SIGINT 무시한 잔존(GUI 등)
  sleep 2
  # graceful 끝까지 무시하는 잔존(pygame ui_node 등) → 최후수단 SIGKILL, 직후 shm 청소로 꼬임 차단
  local forced=0
  for n in "${NODES[@]}"; do
    if pgrep -f "$n" >/dev/null 2>&1; then pkill -KILL -f "$n" 2>/dev/null; forced=1; fi
  done
  pkill -TERM -f "v4l2-sink=${VIDEO_DEV}" 2>/dev/null    # scrcpy 는 SIGTERM(절대 -9 금지: /dev/video 잠김)
  "$ADB_BIN" shell pkill -f com.genymobile.scrcpy 2>/dev/null || true
  if [ "$forced" = 1 ]; then
    sleep 1; ros2 daemon stop 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    echo "[stop] (graceful 무시한 잔존 강제종료 + shm 청소 완료)"
  fi
  sleep 1; echo "[stop] 완료."
}

clean() {
  echo "[clean] 1) 노드 graceful 종료(SIGINT→SIGTERM)"
  for n in "${NODES[@]}"; do pkill -INT -f "$n" 2>/dev/null; done; sleep 3
  for n in "${NODES[@]}"; do pkill -TERM -f "$n" 2>/dev/null; done; sleep 2
  for n in "${NODES[@]}"; do pkill -KILL -f "$n" 2>/dev/null; done   # 잔존 강제(2단계서 shm 청소함)
  pkill -TERM -f "v4l2-sink=${VIDEO_DEV}" 2>/dev/null
  "$ADB_BIN" shell pkill -f com.genymobile.scrcpy 2>/dev/null || true; sleep 1
  echo "[clean] 2) ROS2 daemon + DDS shm 잔여 제거(검은화면 원인)"
  ros2 daemon stop 2>/dev/null || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  echo "[clean] 3) v4l2loopback 리로드(포맷 잠김 해제) — sudo"
  $RESET_CMD && echo "[clean] 완료: $VIDEO_DEV 재생성" \
    || echo "[clean] ! 리로드 실패 — $RESET 경로/권한 확인"
}

status() {
  echo "== 노드 =="; for n in "${NODES[@]}"; do
    pgrep -f "$n" >/dev/null && echo "  $n: 실행중($(pgrep -f "$n" | tr '\n' ' '))" || echo "  $n: 없음"; done
  echo "== v4l2 =="; v4l2-ctl -d "$VIDEO_DEV" --get-fmt-video 2>/dev/null | sed -n '2p' || echo "  $VIDEO_DEV 없음/미준비"
  echo "== adb =="; "$ADB_BIN" devices 2>/dev/null | sed '1d'
  echo "== shm =="; echo "  fastrtps shm: $(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l) 개"
}

run() {
  mkdir -p "$LOG"; src
  echo "[run] DISPLAY=$DISPLAY  WS=$WS  VIDEO=$VIDEO_DEV"
  echo "[run] 0) v4l2loopback 깨끗이 리로드(매 실행 클린 시작)"
  $RESET_CMD || echo "[run] ! 리로드 실패(무시하고 진행) — $RESET 확인"
  echo "[run] 1) OAK 검출+추적 ($LOG/oak.log)"
  ( cd "$REPO" && ros2 launch ros2_yolo_oak oak_tracking.launch.py viz:=false ) >"$LOG/oak.log" 2>&1 &
  sleep 6
  echo "[run] 2) gesture_node (입력=OAK /oak/rgb/image_raw, $LOG/gesture.log)"
  ros2 launch ros2_gesture_node gesture.launch.py ui:=false \
       image_topic:=/oak/rgb/image_raw \
       model_path:="$REPO/ros2_gesture_node/models/YOLOv10n_gestures.pt" \
       >"$LOG/gesture.log" 2>&1 &
  sleep 3
  echo "[run] 3) phone_bridge (managed scrcpy + 자동복구, $LOG/phone.log)"
  ros2 run ros2_phone_bridge phone_bridge --ros-args \
       -p video_device:="$VIDEO_DEV" -p manage_scrcpy:=true \
       -p scrcpy_bin:="$SCRCPY_BIN" -p adb_bin:="$ADB_BIN" \
       -p camera_size:="$CAM_SIZE" \
       -p v4l2_reset_cmd:="$RESET_CMD" -p wedge_timeout:=10.0 \
       >"$LOG/phone.log" 2>&1 &
  echo "[run] 4) /phone/image 첫 프레임 대기..."
  for i in $(seq 1 20); do
    sleep 1
    if ros2 topic hz /phone/image --window 5 2>/dev/null | grep -q average; then
      echo "[run]   /phone/image 수신 OK"; break; fi
    [ "$i" = 20 ] && echo "[run]   ! 아직 프레임 없음 — $LOG/phone.log 확인"
  done
  echo "[run] 5) ui_node (LCD, $LOG/ui.log) — 디스커버리+첫프레임 ~10초, 검정이어도 대기"
  ros2 run ros2_gesture_node ui_node --ros-args -p fullscreen:=false >"$LOG/ui.log" 2>&1 &
  echo "[run] 기동 완료. 종료: scripts/solcam.sh stop"
}

case "${1:-}" in
  clean) src; clean ;;
  run)   run ;;
  stop)  src; stop ;;
  status) src; status ;;
  *) echo "사용: $0 {clean|run|stop|status}"; exit 1 ;;
esac
