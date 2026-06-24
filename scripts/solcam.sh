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
SERIAL_DEV="${SOLCAM_SERIAL:-/dev/ttyTHS1}"   # STM 드라이버 UART (잿슨=/dev/ttyTHS1)
SCRCPY_DIR="${SCRCPY_DIR:-$HOME/scrcpy_bin/scrcpy-linux-x86_64-v4.0}"
SCRCPY_BIN="${SCRCPY_BIN:-$SCRCPY_DIR/scrcpy}"
ADB_BIN="${ADB_BIN:-$SCRCPY_DIR/adb}"
LOG="${SOLCAM_LOG:-/tmp/solcam}"
FULLSCREEN="${SOLCAM_FULLSCREEN:-true}"   # LCD 전체화면. 개발 PC면 SOLCAM_FULLSCREEN=false
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"

# v4l2 리로드 스크립트: 보안 설치본(root 소유) 우선, 없으면 repo 스크립트
RESET_SECURE=/usr/local/sbin/solcam_reset_v4l2.sh
if [ -x "$RESET_SECURE" ]; then RESET="$RESET_SECURE"; else RESET="$REPO/scripts/reset_v4l2.sh"; fi
RESET_CMD="sudo $RESET $VIDEO_NR"     # phone_bridge 자동복구 명령(sudoers NOPASSWD 권장)

NODES=(phone_bridge gesture_node ui_node control_node driver_bridge driver.launch oak_detector tracking_node oak_viz oak_tracking.launch gesture.launch)

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
  # 종료 시 Output 녹화영상을 폰으로 전송(adb 연결 시). 노드 종료 후라 mp4 마무리됨.
  #  영상은 WiFi(IP Webcam)라도 adb 는 USB 또는 'adb connect' 필요. 미연결이면 건너뜀.
  push_videos
  sleep 1; echo "[stop] 완료."
}

# Output/*.mp4 전부 폰으로 전송(타임스탬프 파일명이라 재전송해도 덮어쓰기=중복없음).
push_videos() {
  local out="$REPO/Output" dcim="${SOLCAM_PHONE_DCIM:-/sdcard/DCIM/solcam}"
  ls "$out"/*.mp4 >/dev/null 2>&1 || { echo "[stop] 전송할 영상 없음($out)"; return; }
  "$ADB_BIN" get-state >/dev/null 2>&1 || { echo "[stop] adb 미연결 → 영상 전송 건너뜀(영상은 $out 에 보존)"; return; }
  echo "[stop] Output 영상 폰 전송(adb → $dcim)..."
  "$ADB_BIN" shell mkdir -p "$dcim" >/dev/null 2>&1 || true
  for f in "$out"/*.mp4; do
    if "$ADB_BIN" push "$f" "$dcim/" >/dev/null 2>&1; then echo "   ✓ $(basename "$f")"
    else echo "   ✗ $(basename "$f") (전송 실패)"; fi
  done
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
  # ★재실행 시 노드 중첩 방지: 이미 떠 있는 solcam 노드가 있으면 먼저 정리.
  #  (ESC/창닫기로 UI만 죽고 나머지가 살아있던 경우에도 깨끗이 새로 시작)
  if pgrep -f "ui_node|control_node|driver_bridge|oak_detector|gesture_node" >/dev/null 2>&1; then
    echo "[run] 기존 solcam 노드 감지 → 먼저 정리(stop)"
    stop
  fi
  # 플래그(순서 무관, 조합 가능):
  #   nophone : 폰(scrcpy/v4l2/연동) 전부 생략 — 폰 거치만 하고 영상연동 안 할 때
  #   norobot : STM 드라이버(driver_bridge/control_node) 생략 — 개발 PC 등 STM 없을 때
  #  환경변수 SOLCAM_PHONE=0 / SOLCAM_ROBOT=0 로도 끌 수 있다.
  local phone=1 robot=1
  for a in "$@"; do
    case "$a" in
      nophone) phone=0 ;;
      norobot) robot=0 ;;
    esac
  done
  [ "${SOLCAM_PHONE:-1}" = "0" ] && phone=0
  [ "${SOLCAM_ROBOT:-1}" = "0" ] && robot=0
  echo "[run] DISPLAY=$DISPLAY  WS=$WS  VIDEO=$VIDEO_DEV  phone=$phone  robot=$robot"

  if [ "$phone" = 1 ]; then
    echo "[run] 0) v4l2loopback 깨끗이 리로드(매 실행 클린 시작)"
    $RESET_CMD || echo "[run] ! 리로드 실패(무시하고 진행) — $RESET 확인"
  else
    echo "[run] 0) 폰 비활성(nophone) — scrcpy/v4l2/폰연동 생략"
  fi

  echo "[run] 1) OAK 검출+추적 ($LOG/oak.log)"
  ( cd "$REPO" && ros2 launch ros2_yolo_oak oak_tracking.launch.py viz:=false ) >"$LOG/oak.log" 2>&1 &
  sleep 6
  echo "[run] 2) gesture_node (입력=OAK /oak/rgb/image_raw, $LOG/gesture.log)"
  #  ★recognizer=mediapipe 명시(런치 기본이 hagrid면 ultralytics 없어 죽음).
  #   이 인자가 런치 기본값을 덮으므로 패키지 재빌드 없이도 손동작 노드가 뜬다.
  ros2 launch ros2_gesture_node gesture.launch.py ui:=false \
       recognizer:=mediapipe \
       image_topic:=/oak/rgb/image_raw \
       model_path:="$REPO/ros2_gesture_node/models/YOLOv10n_gestures.pt" \
       >"$LOG/gesture.log" 2>&1 &
  sleep 3
  # ★UI 먼저 — 폰 영상이 없어도(또는 nophone) 즉시 뜬다. ui_node 는 /phone/image 가
  #  없으면 OAK 영상/플레이스홀더로 렌더하므로 블로킹/검은멈춤 없음. 폰은 나중에
  #  붙어도 late-join 으로 자동 표시된다.
  echo "[run] 3) ui_node (LCD 전체화면=$FULLSCREEN, $LOG/ui.log) — 폰 없어도 바로 뜸"
  #  ui_node 가 키보드 수동주행(teleop)도 겸한다 — LCD 창 포커스에서 화살표/WASD 로 주행.
  #  별도 teleop 창 불필요. (teleop:=false 로 끌 수 있음)
  ros2 run ros2_gesture_node ui_node --ros-args -p fullscreen:=$FULLSCREEN >"$LOG/ui.log" 2>&1 &

  if [ "$robot" = 1 ]; then
    echo "[run] 3b) driver_bridge (STM 드라이버, $SERIAL_DEV, $LOG/driver.log)"
    ( cd "$REPO" && ros2 launch ros2_driver_bridge driver.launch.py port:="$SERIAL_DEV" ) >"$LOG/driver.log" 2>&1 &
    sleep 2
    echo "[run] 3c) control_node (모드/주행 제어, $LOG/control.log)"
    #  ★params-file 로 control_params.yaml 로드 — 안 주면 컴파일 기본값으로만 돈다
    #   (튜닝 값이 안 먹던 원인). 이제 yaml 만 바꾸고 재시작하면 바로 반영된다.
    ros2 run ros2_control_node control_node --ros-args \
         --params-file "$REPO/ros2_control_node/config/control_params.yaml" \
         >"$LOG/control.log" 2>&1 &
  else
    echo "[run] 3b) robot 비활성(norobot) — driver_bridge/control_node 생략(STM 없는 PC)"
  fi

  if [ "$phone" = 1 ]; then
    echo "[run] 4) phone_bridge (managed scrcpy + 자동복구, $LOG/phone.log)"
    ros2 run ros2_phone_bridge phone_bridge --ros-args \
         -p video_device:="$VIDEO_DEV" -p manage_scrcpy:=true \
         -p scrcpy_bin:="$SCRCPY_BIN" -p adb_bin:="$ADB_BIN" \
         -p camera_size:="$CAM_SIZE" \
         -p v4l2_reset_cmd:="$RESET_CMD" -p wedge_timeout:=10.0 \
         >"$LOG/phone.log" 2>&1 &
    # 폰 첫 프레임 확인은 백그라운드(논블로킹) — 안 와도 UI/파이프라인은 계속 돈다.
    ( for i in $(seq 1 20); do sleep 1
        if ros2 topic hz /phone/image --window 5 2>/dev/null | grep -q average; then
          echo "[run]   /phone/image 수신 OK"; exit 0; fi
      done
      echo "[run]   ! 폰 영상 미수신(20s) — UI는 OAK/검정 배경으로 계속. $LOG/phone.log 확인" ) &
  else
    echo "[run] 4) phone_bridge 생략(nophone). 줌/녹화전송 등 잿슨→폰 기능 비활성."
  fi
  echo "[run] 기동 완료. 종료: scripts/solcam.sh stop"
}

case "${1:-}" in
  clean) src; clean ;;
  run)   shift; run "$@" ;;
  stop)  src; stop ;;
  status) src; status ;;
  *) echo "사용: $0 {clean|run [nophone] [norobot]|stop|status}"; exit 1 ;;
esac
