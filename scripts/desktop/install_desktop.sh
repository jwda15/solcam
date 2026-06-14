#!/usr/bin/env bash
# ============================================================================
#  잿슨 바탕화면에 SolCam '시작'/'정지' 아이콘 설치 (GNOME/Ubuntu).
#  더블클릭 → solcam.sh run/stop. 노드는 setsid 로 세션 분리해 런처가 끝나도
#  계속 돈다(SIGHUP 안 받음). 로그는 solcam.sh 가 /tmp/solcam 에 남긴다.
#
#  사용:  bash scripts/desktop/install_desktop.sh
#  ★repo 가 ~/solcam 이 아니면 SOLCAM_REPO 를 export 한 뒤 실행하거나,
#    생성된 .desktop 의 Exec 경로를 수정.
# ============================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="${SOLCAM_REPO:-$(cd "$HERE/../.." && pwd)}"
SH_RUN="$REPO/scripts/solcam.sh"
ICON="$HERE/solcam.svg"
APPS="$HOME/.local/share/applications"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$APPS" "$DESK"

[ -x "$SH_RUN" ] || chmod +x "$SH_RUN" 2>/dev/null || true

gen() {  # $1=표시이름  $2=Exec  $3=출력파일
  cat > "$3" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$1
Comment=SolCam 팔로잉 카메라
Exec=$2
Icon=$ICON
Terminal=false
Categories=Utility;
StartupNotify=false
EOF
  chmod +x "$3"
  gio set "$3" metadata::trusted true 2>/dev/null || true   # GNOME: 신뢰(더블클릭 허용)
}

# setsid -f: 새 세션으로 분리 → 런처가 끝나도 노드 생존. SOLCAM_REPO 도 전달.
RUN_EXEC="sh -c 'SOLCAM_REPO=\"$REPO\" setsid -f \"$SH_RUN\" run; notify-send \"SolCam\" \"시작 중...\" 2>/dev/null || true'"
STOP_EXEC="sh -c 'SOLCAM_REPO=\"$REPO\" \"$SH_RUN\" stop; notify-send \"SolCam\" \"정지\" 2>/dev/null || true'"

gen "SolCam 시작" "$RUN_EXEC"  "$APPS/solcam-start.desktop"
gen "SolCam 정지" "$STOP_EXEC" "$APPS/solcam-stop.desktop"
cp -f "$APPS/solcam-start.desktop" "$DESK/solcam-start.desktop"
cp -f "$APPS/solcam-stop.desktop"  "$DESK/solcam-stop.desktop"
gio set "$DESK/solcam-start.desktop" metadata::trusted true 2>/dev/null || true
gio set "$DESK/solcam-stop.desktop"  metadata::trusted true 2>/dev/null || true
chmod +x "$DESK/solcam-start.desktop" "$DESK/solcam-stop.desktop"

echo "설치 완료."
echo "  repo   : $REPO"
echo "  바탕화면: $DESK  (SolCam 시작 / SolCam 정지)"
echo "  앱서랍 : $APPS  (Activities 검색에도 뜸)"
echo
echo "처음 한 번: 바탕화면 아이콘 우클릭 → 'Allow Launching'(실행 허용) 필요할 수 있음."
echo "부팅 시 자동시작을 원하면:"
echo "  cp \"$APPS/solcam-start.desktop\" ~/.config/autostart/"
