# ros2_yolo_oak

OAK-D S2 **온보드 YOLO**(SpatialDetectionNetwork)로 사람을 탐지하고,
기존 `ros2_tracking_node`(ByteTrack + OwnerTracker)에 그대로 물려
실시간 주인 추적을 돌리는 패키지. 기존 `ros2_yolo_d435i`를 대체한다.

**호스트 GPU에서 YOLO를 돌리지 않는다.** OAK-D 내부 VPU가 추론 + depth까지 계산.

> DepthAI **v3** 기준(2026-05). 모델은 `.blob` 파일이 아니라 HubAI **model zoo**에서
> 모델명으로 자동 다운로드(RVC2 SUPERBLOB). 별도 blob 변환 불필요.

## 파이프라인
```
[OAK-D S2]
  Camera(RGB preview 512x384) ┐
  MonoL+MonoR → StereoDepth    ┤
                SpatialDetectionNetwork (온보드, zoo yolov6-nano)
                        │ bbox(0~1) + spatialCoords XYZ(mm)
                        ▼
  [oak_detector] → /detections (픽셀 bbox + depth mm)
                   /camera/color/camera_info
                   /oak/rgb/image_raw (viz용)
                        ▼
  [tracking_node] (기존, 무수정) → /owner_pose
                        ▼
  [oak_viz] → OpenCV 창
```
핵심: tracking_node는 detection 소스에 독립적. 픽셀 bbox + depth(mm) + CameraInfo만
주면 D435i든 OAK든 동일 동작. **tracking_node는 한 줄도 안 고침.**

## 0. 사전 준비 (OAK 연결된 실제 PC)

### depthai 설치 (v3)
```bash
python3 -m pip install depthai     # v3.x (이 패키지는 v3 API 기준)
```

### udev 규칙 (필수 — 안 하면 권한 때문에 디바이스 못 잡음)
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# 그 후 OAK USB 케이블을 뽑았다 다시 꽂는다.
```

### 연결 확인
```bash
python3 -c "import depthai as dai; print([d.name for d in dai.Device.getAllAvailableDevices()])"
# 디바이스 찍히면 OK. USB3 케이블(USB-A 안쪽 파란색) 권장.
```

## 1. 모델 (zoo 자동 다운로드, 변환 불필요)

`oak_params.yaml`의 `model` 키에 zoo 모델명을 적으면 첫 실행 시 자동 다운로드된다.

- 기본: **`yolov6-nano`** (512x384, COCO 80클래스, person=0). RVC2 SUPERBLOB.
- 16:9 변형: `yolov6-nano:r2-coco-512x288`

> **왜 yolov6-nano인가**: zoo에 RVC2 blob 완성품으로 있는 detection은 v6 계열뿐
> (v8/v10/v11 detection은 zoo에 RVC2 blob이 없어 온라인 변환 필요).
> nano급 mAP 차이는 미미하고, 우리 과제(1~5m 사람 추적)에선 체감되지 않음.
> v6-large는 RVC2 미지원(RVC4 전용)이라 OAK-D S2에선 못 씀.

> **해상도 주의**: zoo yolov6-nano는 512x384(또는 512x288)만 제공. 416 정사각은 없음.
> preview를 NN 입력과 같은 종횡비(512x384)로 맞춰야 박스가 안 밀린다.

> **extended disparity/subpixel은 끈다**: zoo 모델은 8-shave 컴파일이라, stereo의
> extended/subpixel을 켜면 NN과 VPU shave 충돌로 크래시한다. depth 노이즈는
> tracking_node의 speed cap + EMA가 대응하므로 우리 용도(대략 거리)엔 기본 depth로 충분.

## 2. 빌드

repo 경로(`/media/jw/로컬 디스크/...`)에 **공백/한글**이 있으면 colcon의 rosidl
메시지 생성이 깨진다. → 홈에 공백 없는 워크스페이스를 만들고 **심볼릭 링크**로 빌드한다.
원본은 그대로 두므로 데이터셋/도커/git에 영향 없음.

```bash
source /opt/ros/humble/setup.bash      # 집:humble / 연구실:jazzy

# (최초 1회) 공백 없는 빌드 워크스페이스 + 심볼릭 링크
mkdir -p ~/solcam_ws/src && cd ~/solcam_ws/src
ln -sfn "/media/jw/로컬 디스크/32/capstone/0524solcam/ros2_yolo_oak"      ros2_yolo_oak
ln -sfn "/media/jw/로컬 디스크/32/capstone/0524solcam/ros2_tracking_node" ros2_tracking_node

# libbytetrack.so (순수 CMake)가 먼저 빌드되어 있어야 함:
#   cd "/media/jw/로컬 디스크/32/capstone/0524solcam/build" && cmake .. && cmake --build . -j

cd ~/solcam_ws
colcon build --packages-select ros2_tracking_node ros2_yolo_oak
source install/setup.bash
```
> ros2_tracking_node를 먼저(또는 함께) 빌드. oak가 그 msg에 의존.
> CMakeLists는 심볼릭 링크를 REALPATH로 해제해 원본 repo의 libbytetrack.so/include를 찾는다.

## 3. 실행
```bash
ros2 launch ros2_yolo_oak oak_tracking.launch.py
# viz 끄기: viz:=false / 종료: viz 창에서 q 또는 Ctrl-C
```
첫 실행 시 zoo 모델 다운로드로 몇 초 지연될 수 있음(이후 캐시).

### 토픽 확인
```bash
ros2 topic hz /detections      # ~19Hz (spatial+호스트 포함; NN 순수추론은 더 빠름)
ros2 topic echo /owner_pose    # is_detected, distance, azimuth, track_id
```

## 4. ★ 해상도 일치 (가장 흔한 실수)
`oak_params.yaml`의 `preview_width/height`(512x384)와
`tracking_params.yaml`의 `image_width/height`를 **반드시 일치**시킬 것.
```yaml
# tracking_params.yaml
image_width:  512
image_height: 384   # ← OAK preview(=NN 입력)와 일치
```
안 맞추면 (a) owner 초기화의 "화면 중앙"(cx,cy) 기준이 어긋나 초기 타겟이 빗나가고,
(b) NN crop/scale로 정규화 좌표가 어긋나 박스가 가로로 밀린다.

## 5. 좌표계 메모
- oak_detector는 spatialCoordinates.z(mm)를 Detection.depth에 그대로 넣음.
- tracking_node가 이 depth와 CameraInfo로 sx,sy,sz backproject (OAK의 X,Y 직접 안 씀,
  z만 → D435i 파이프라인과 동일 경로 유지).
- CameraInfo는 현재 FOV 69도 근사값(`_build_camera_info`). 정밀이 필요하면 추후
  실제 calibration(readCalibration)으로 대체.

## 6. 트러블슈팅
| 증상 | 원인 / 해결 |
|---|---|
| `No available devices` | udev 규칙 미적용/케이블. 재확인, 케이블 재연결 |
| `DEVICE_ALREADY_IN_USE` | 이전 detector/launch가 OAK 점유 중. `pkill -f oak_detector; pkill -f "ros2 launch"` |
| USB speed가 HIGH(=USB2) | USB2 케이블/포트. USB3(파란색)+USB3 포트 |
| `/detections` 안 나옴 | (1) setDepthAlign/setOutputSize를 Spatial NN에 걸면 NN out이 멈춤 → 빼야 함. (2) preview 종횡비 불일치 |
| `Blob compiled for 8 shaves...` 크래시 | stereo extended/subpixel을 켬 → 끄기(zoo 8-shave 모델과 충돌) |
| 박스가 가로로 밀림 | 4번 해상도/종횡비 일치 확인 |
| spatial z가 0/튐 | bbox_scale 키우기, depth_lower/upper 범위 확인 |
| `list index out of range` (빌드) | repo 경로의 공백/한글. 2번 심볼릭 링크 워크스페이스로 빌드 |

## 파일 구조
```
ros2_yolo_oak/
├── package.xml / setup.py / setup.cfg
├── config/oak_params.yaml          ← 모델명·해상도·spatial 파라미터 (v3)
├── launch/oak_tracking.launch.py   ← detector + tracking + viz 통합
└── ros2_yolo_oak/
    ├── oak_detector.py             ← OAK 온보드 YOLO → /detections
    └── oak_viz.py                  ← OpenCV 시각화
```
