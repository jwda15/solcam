#!/usr/bin/env bash
# v4l2loopback 을 깨끗이 리로드해 /dev/videoN 꼬임(VIDIOC_G_FMT) 을 해제한다.
# root 권한 필요(sudo). phone_bridge 의 v4l2_reset_cmd 로도 호출됨.
#   reset_v4l2.sh [video_nr]   (기본 2)
set -e
NR="${1:-2}"
modprobe -r v4l2loopback 2>/dev/null || true
# ★exclusive_caps=0 필수. =1 이면 scrcpy 가 v4l2 sink 헤더를 못 써서
#  "VIDIOC_G_FMT: Invalid argument / Failed to write header" 로 즉시 죽고
#  워치독 재기동 무한루프가 된다(잿슨에서 확인). =0 이라야 scrcpy writer + cv2 reader 공존.
modprobe v4l2loopback video_nr="$NR" card_label=solcam_phone exclusive_caps=0
echo "v4l2loopback 리로드 완료: /dev/video${NR}"
