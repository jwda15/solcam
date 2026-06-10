# ros2_phone_bridge

안드로이드 폰(촬영 카메라) ↔ Jetson **USB 브리지**. 폰을 USB로 Jetson에 꽂으면
폰 후면 카메라 영상과 배터리를 ROS2 토픽으로 올리고, 손동작 녹화 명령을
받아 촬영본을 폰에 저장한다. 별도 안드로이드 앱 개발 불필요(scrcpy/adb 사용).

## 파이프라인

```
폰 후면 카메라 ──scrcpy(USB)──▶ /dev/videoN(v4l2loopback) ──cv2──▶ /phone/image
폰 배터리      ──adb dumpsys battery──────────────────────────────▶ /phone/battery
/phone_cmd(record_toggle) ─▶ 캡처 프레임을 ffmpeg 파이프로 mp4 녹화(Jetson) ─(종료)─▶ adb push ─▶ 폰 DCIM
```

- 영상 미리보기(LCD)와 녹화를 **같은 v4l2 장치**에서 분리 처리 → 단일 카메라
  파이프라인이라 폰 카메라 앱과 충돌하지 않는다.
- 녹화는 Jetson에서 받아 저장한 뒤 **촬영 종료 시 폰으로 전송**(adb push)한다.
  → 결과 파일은 폰 `DCIM/solcam/` 에 남는다.

## 토픽

발행:
- `/phone/image` (sensor_msgs/Image, bgr8) — 폰 카메라 영상 (ui_node 배경)
- `/phone/battery` (sensor_msgs/BatteryState) — 상단바 배터리 %
- `/phone/recording` (std_msgs/Bool) — 녹화 중 여부 (상단 REC + Rec ON/OFF)

구독:
- `/phone_cmd` (std_msgs/String) — gesture_node 발행
  - `record_toggle` : 녹화 시작/종료 (구현됨)
  - `zoom_in` / `zoom_out` / `focus` : 자리만(로그) — 추후 구현

## 사전 준비 (Jetson, 최초 1회)

```bash
sudo apt install android-tools-adb scrcpy ffmpeg v4l2loopback-dkms v4l-utils python3-opencv
# scrcpy 가 2.0 미만이면 카메라 소스 미지원 → snap/공식 최신으로 설치
```

폰: 개발자 옵션 → **USB 디버깅 ON**, USB 연결 후 `adb devices` 에 기기 확인
(최초 1회 폰 화면의 "USB 디버깅 허용" 승인).

## 실행

```bash
# 1) 가상 카메라 장치 생성 (부팅마다)
ros2 run ... 대신 스크립트로:
sudo ros2_phone_bridge/scripts/setup_v4l2loopback.sh 2      # /dev/video2

# 2) 폰 카메라 스트림 시작 (별도 터미널, 계속 떠 있어야 함)
ros2_phone_bridge/scripts/start_scrcpy_camera.sh /dev/video2

# 3) 브리지 노드
ros2 launch ros2_phone_bridge phone.launch.py video_device:=/dev/video2

# 또는 노드가 scrcpy 까지 직접 관리:
ros2 launch ros2_phone_bridge phone.launch.py manage_scrcpy:=true
```

폰 없이 UI/배선만 점검:

```bash
ros2 launch ros2_phone_bridge phone.launch.py mock:=true
# 합성 영상 + 1초마다 배터리 1%↓ + record_toggle 로그
```

## 파라미터

| 이름 | 기본 | 설명 |
|------|------|------|
| `mock` | false | true면 폰/도구 없이 합성 영상·가짜 배터리 |
| `video_device` | /dev/video2 | scrcpy v4l2 sink 장치 |
| `publish_rate` | 20.0 | /phone/image 발행 Hz |
| `battery_period` | 15.0 | 배터리 폴링 주기(s) |
| `adb_serial` | "" | 기기 여러 대일 때 지정 |
| `record_dir` | /tmp/solcam_rec | Jetson 임시 녹화 폴더 |
| `phone_push_dir` | /sdcard/DCIM/solcam | 폰 저장 경로 |
| `manage_scrcpy` | false | 노드가 scrcpy 직접 실행 |

## 검증 (ROS 없이)

```bash
cd ros2_phone_bridge
python3 -m pytest test/test_battery_parse.py -v
```

## TODO

- [ ] 줌/포커스 실제 제어 (scrcpy 카메라 런타임 줌 한계 → Camera2 앱 또는
      adb input 경로 검토)
- [ ] scrcpy 끊김 자동 재연결(워치독)
- [ ] 녹화 중 미리보기 프레임드랍 시 해상도/fps 조정
