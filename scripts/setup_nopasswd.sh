#!/usr/bin/env bash
# ============================================================================
#  setup_nopasswd.sh — 아이콘 풀실행 시 비밀번호 안 물어보게 설정 (1회만, sudo 필요).
#
#  solcam.sh 가 v4l2loopback 리로드에 sudo 를 쓰는데, 그게 아이콘 실행 때마다
#  비번을 물어본다. 이 스크립트가 해당 명령(+손동작 Power OFF)만 NOPASSWD 로
#  허용하는 sudoers 규칙을 설치한다. 다른 sudo 권한은 일절 주지 않는다.
#
#  사용:  bash scripts/setup_nopasswd.sh      (이때 한 번 비번 입력)
#  이후:  SolCam 아이콘 더블클릭 → 비번 안 물어봄.
# ============================================================================
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
U="$(id -un)"
SECURE=/usr/local/sbin/solcam_reset_v4l2.sh

echo "[setup] 사용자=$U, repo=$REPO"

# 1) v4l2 리로드 스크립트를 root 소유 위치로 복사 (보안: 사용자가 못 바꾸게)
sudo install -o root -g root -m 755 "$REPO/scripts/reset_v4l2.sh" "$SECURE"
echo "[setup] 설치: $SECURE"

# 2) sudoers.d 규칙 설치 (현재 사용자명으로 자동 작성)
TMP="$(mktemp)"
cat > "$TMP" <<EOF
# solcam: 아이콘 풀실행 시 v4l2 리로드 + 손동작 전원종료를 비번 없이 허용.
$U ALL=(root) NOPASSWD: $SECURE *
$U ALL=(root) NOPASSWD: /sbin/poweroff, /usr/sbin/poweroff, /usr/bin/systemctl poweroff
EOF
sudo install -o root -g root -m 440 "$TMP" /etc/sudoers.d/solcam
rm -f "$TMP"
echo "[setup] 설치: /etc/sudoers.d/solcam"

# 3) 문법 검증
sudo visudo -c
echo
echo "완료. 이제 SolCam 아이콘 더블클릭하면 비번 안 물어봄."
echo "(되돌리려면: sudo rm /etc/sudoers.d/solcam $SECURE )"
