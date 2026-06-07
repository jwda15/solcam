#!/usr/bin/env bash
# v4l2loopback 을 깨끗이 리로드해 /dev/videoN 꼬임(VIDIOC_G_FMT) 을 해제한다.
# root 권한 필요(sudo). phone_bridge 의 v4l2_reset_cmd 로도 호출됨.
#   reset_v4l2.sh [video_nr]   (기본 2)
set -e
NR="${1:-2}"
modprobe -r v4l2loopback 2>/dev/null || true
modprobe v4l2loopback video_nr="$NR" card_label=solcam_phone exclusive_caps=1
echo "v4l2loopback 리로드 완료: /dev/video${NR}"
