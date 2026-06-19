#!/usr/bin/env bash
# ============================================================================
#  잿슨 바탕화면 'SolCam' 풀 실행 아이콘 설치
#  (풀 실행 = 트래킹 + 컨트롤 + 손동작 + UI, solcam.sh run 그대로).
#
#  ★왜 이렇게 하나 (이전 아이콘 문제 해결):
#    아이콘을 그냥 setsid 로 돌리면 .bashrc 를 안 읽어서, 손동작 노드가 쓰는
#    파이썬 의존성(mediapipe/ultralytics 등, conda/pip 경로)을 못 찾아 죽었음.
#    → 여기선 '터미널 + 대화형 로그인 셸(bash -lic)' 로 solcam.sh 를 띄운다.
#      터미널에서 수동으로 켤 때와 환경이 100% 동일해져 손동작 노드가 정상 동작.
#    → .desktop 따옴표 문제를 피하려고 실행 로직은 별도 래퍼(plain bash)에 둔다.
#
#  사용:        bash scripts/desktop/install_desktop.sh
#  repo 다르면: SOLCAM_REPO=/실제/경로 bash scripts/desktop/install_desktop.sh
# ============================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="${SOLCAM_REPO:-$(cd "$HERE/../.." && pwd)}"
ICON="$HERE/solcam.svg"
BIN="$HOME/.local/bin"
APPS="$HOME/.local/share/applications"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$BIN" "$APPS" "$DESK"
chmod +x "$REPO/scripts/solcam.sh" 2>/dev/null || true

# ---- 실행 래퍼: 터미널 + 대화형 로그인 셸로 풀 실행 ----
cat > "$BIN/solcam-run" <<EOF
#!/usr/bin/env bash
# SolCam 풀 실행 (트래킹+컨트롤+손동작+UI). 대화형 로그인 셸 = 수동 실행과 동일 환경.
export SOLCAM_REPO="$REPO"
SH="\$SOLCAM_REPO/scripts/solcam.sh"
if command -v gnome-terminal >/dev/null 2>&1; then
  exec gnome-terminal -- bash -lic "'\$SH' run; exec bash"
elif command -v x-terminal-emulator >/dev/null 2>&1; then
  exec x-terminal-emulator -e bash -lic "'\$SH' run; exec bash"
else
  exec bash -lic "'\$SH' run"
fi
EOF

cat > "$BIN/solcam-stop" <<EOF
#!/usr/bin/env bash
export SOLCAM_REPO="$REPO"
bash -lic "'\$SOLCAM_REPO/scripts/solcam.sh' stop"
command -v notify-send >/dev/null 2>&1 && notify-send "SolCam" "정지" || true
EOF
chmod +x "$BIN/solcam-run" "$BIN/solcam-stop"

# ---- .desktop 파일 (Exec 은 래퍼 경로만 → 따옴표 문제 없음) ----
gen() {  # $1=표시이름  $2=Exec  $3=출력파일
  cat > "$3" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$1
Comment=SolCam 풀 실행 (트래킹+컨트롤+손동작+UI)
Exec=$2
Icon=$ICON
Terminal=false
Categories=Utility;
StartupNotify=false
EOF
  chmod +x "$3"
  gio set "$3" metadata::trusted true 2>/dev/null || true   # GNOME 더블클릭 허용
}

gen "SolCam"      "$BIN/solcam-run"  "$APPS/solcam.desktop"
gen "SolCam Stop" "$BIN/solcam-stop" "$APPS/solcam-stop.desktop"
cp -f "$APPS/solcam.desktop"      "$DESK/SolCam.desktop"
cp -f "$APPS/solcam-stop.desktop" "$DESK/SolCam-Stop.desktop"
for f in "$DESK/SolCam.desktop" "$DESK/SolCam-Stop.desktop"; do
  chmod +x "$f"; gio set "$f" metadata::trusted true 2>/dev/null || true
done

echo "설치 완료."
echo "  repo      : $REPO"
echo "  실행 래퍼 : $BIN/solcam-run , $BIN/solcam-stop"
echo "  바탕화면  : SolCam (풀 실행) / SolCam Stop"
echo
echo "더블클릭 → 터미널 열리며 OAK트래킹 → 컨트롤 → 손동작 → UI 순으로 전부 기동."
echo "종료는 'SolCam Stop' 아이콘 또는 그 터미널 닫기."
echo "처음 한 번: 아이콘 우클릭 → 'Allow Launching'(실행 허용) 필요할 수 있음."
echo "부팅 자동시작 원하면:  cp \"$APPS/solcam.desktop\" ~/.config/autostart/"
