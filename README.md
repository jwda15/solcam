# solcam

1인 크리에이터용 **팔로우 카메라 로봇**의 인지·추적 소프트웨어. OAK-D 카메라로
사용자(주인)를 실시간 추적하여 3D 위치를 추정하고, 그 위치를 로봇 제어에 넘긴다.

추적기는 [ByteTrack](https://github.com/Vertical-Beach/ByteTrack-cpp)을 기반으로,
RGB-D(깊이 포함) 환경과 "여러 사람 중 주인 한 명을 고정 추적"하는 용도에 맞게 확장했다.

---

## 시스템 구조

세 단계로 나뉘며, 각 단계는 ROS2 토픽으로만 연결된다. 그래서 **detector(입력 소스)를
갈아끼워도 추적 로직은 그대로** 쓸 수 있다 (OAK / RealSense / 데이터셋 공용).

```
┌─────────────┐   /detections        ┌──────────────┐   /owner_pose   ┌─────────────┐
│  detector   │ ───────────────────▶ │ tracking_node │ ──────────────▶ │ follow_node │
│ (사람 검출   │   DetectionArray     │ (주인 추적     │   OwnerPose     │  (로봇 제어  │
│  + depth)   │   /camera_info       │  + 3D 위치)    │                 │   - 후속)    │
└─────────────┘                      └──────────────┘                 └─────────────┘
```

- **detector**: 화면에서 사람을 검출하고, 각 사람의 bbox(픽셀) + depth(mm)를 발행한다.
  "누가 주인인지"는 모른다. 그냥 보이는 사람들을 다 넘긴다.
- **tracking_node**: 검출들을 ByteTrack으로 추적(각자 ID 부여)하고, 그 중 **주인 한 명**을
  특정·유지하여, 주인의 3D 위치(거리·방위각)를 발행한다. 이 저장소의 핵심.
- **follow_node**: 주인 위치를 받아 모터를 제어한다. (후속 단계, 골격만 있음)

### 데이터 계약 (메시지)

| 토픽 | 타입 | 내용 |
|------|------|------|
| `/detections` | `DetectionArray` | 사람들의 bbox(px) + depth(mm) + score + label |
| `/owner_pose` | `OwnerPose` | 주인의 `is_detected`, 3D 위치(`spatial_x/y/z`), `azimuth`, `distance`, `track_id` |

핵심은 `/detections`의 **bbox + depth만 있으면** tracking_node가 동작한다는 점이다.
이 계약만 지키면 어떤 detector든 붙는다.

---

## 패키지 구성

| 디렉터리 | 종류 | 역할 |
|----------|------|------|
| `src/`, `include/` | C++ (CMake) | ByteTrack 라이브러리 본체. 빌드 시 `libbytetrack.so` 생성 |
| `ros2_tracking_node/` | C++ (ament) | **추적 + 주인 특정 노드.** `libbytetrack.so`에 링크 |
| `ros2_yolo_oak/` | Python (ament) | OAK-D S2 온보드 YOLO detector (DepthAI v3) |
| `ros2_yolo_d435i/` | Python (ament) | RealSense D435i / 데이터셋용 YOLO detector |
| `ros2_follow_node/` | C++ (ament) | 모터 제어 노드 (후속) |
| `test/` | - | 검증 스크립트 (`test/oak_check/`에 시각화·진단 도구) |

> detector가 두 개인 이유: `oak_detector`는 OAK 하드웨어 전용(온보드 추론)이라
> 저장된 영상엔 못 쓴다. 영상 파일·RealSense로 검증할 땐 `yolo_detector`(d435i)를 쓴다.

---

## 추적 노드 동작 (ros2_tracking_node)

ByteTrack은 "모든 사람 추적 + ID 부여"만 한다. 그 위에 **OwnerTracker**가 얹혀
"누가 주인인가"를 관리한다. 매 프레임:

1. **초기화** — 아직 주인이 없으면, 화면 중앙에서 가장 가까운 트랙을 주인으로 지정.
2. **추적** — 현재 프레임 트랙들 중 주인 ID를 찾으면 위치 갱신.
3. **주인을 놓쳤을 때** — 두 메커니즘으로 대응:
   - **KF 외삽**: 주인이 ByteTrack의 lost 풀(약 3초)에 살아있는 동안, 칼만 필터
     예측 위치로 빈자리를 메운다. (짧은 깜빡임 대응)
   - **동적 재매핑**: 놓친 직후엔 좁은 거리만 허용하고, 오래 놓칠수록 허용 거리를
     점점 넓힌다(`base + growth × lost_count`). 짧은 깜빡임엔 옆 사람으로 잘못
     넘어가지 않고, 긴 가림 후엔 멀어진 주인도 회복한다.
4. **3D 변환** — 주인 bbox 중심 + depth + 카메라 intrinsic으로 `spatial_x/y/z`를
   역투영하고, 속도 제한 + EMA로 평활하여 발행.

> Kalman 상태는 10D `[x, y, z, a, h, vx, vy, vz, va, vh]` — 픽셀 bbox(중심·종횡비·높이)에
> metric depth(z)를 더한 형태. OAK가 z를 직접 주므로 관측이 선형이 된다.
> CNN Re-ID는 쓰지 않으므로, 외모로 사람을 구분하진 않는다(위치 기반).

---

## 빌드

> ⚠️ **경로에 공백·한글이 있으면** colcon의 메시지 생성이 깨진다(`list index out of range`).
> 그래서 저장소는 아무 데나 두되, **빌드는 공백·한글 없는 워크스페이스에 심볼릭 링크**로 한다.

```bash
source /opt/ros/humble/setup.bash      # 연구실은 jazzy

# 1) ByteTrack 라이브러리 (순수 CMake) — libbytetrack.so 생성
cd <저장소>/build && cmake .. && cmake --build . -j
#   (build 폴더가 없으면 mkdir build 먼저)

# 2) ROS2 패키지 — 심볼릭 링크 워크스페이스
mkdir -p ~/solcam_ws/src && cd ~/solcam_ws/src
ln -sfn <저장소>/ros2_tracking_node ros2_tracking_node
ln -sfn <저장소>/ros2_yolo_oak      ros2_yolo_oak
ln -sfn <저장소>/ros2_yolo_d435i    ros2_yolo_d435i

cd ~/solcam_ws
colcon build --packages-select ros2_tracking_node ros2_yolo_oak ros2_yolo_d435i
source install/setup.bash
```

> - `ros2_tracking_node`를 먼저(또는 함께) 빌드해야 한다. detector들이 그 메시지에 의존.
> - CMakeLists는 심볼릭 링크를 `REALPATH`로 풀어 원본 저장소의 `libbytetrack.so`/`include`를 찾는다.
> - `~/solcam_ws`는 코드가 아니라 빌드 작업장이다(심볼릭 링크 + 빌드 산출물). 다른 PC로
>   옮기지 말고 그 PC에서 새로 만든다.

---

## 실행

### A) OAK-D 실시간 추적
```bash
ros2 launch ros2_yolo_oak oak_tracking.launch.py        # viz 끄기: viz:=false
```
- 모델은 `ros2_yolo_oak/config/oak_params.yaml`의 `model` 키(HubAI zoo 모델명, 기본 `yolov6-nano`).
  첫 실행 시 자동 다운로드된다. 별도 `.blob` 변환 불필요.
- ⚠️ `oak_params.yaml`의 `preview_width/height`와 `tracking_params.yaml`의
  `image_width/height`를 **같게** 맞춰야 한다(불일치 시 bbox가 밀린다).

### B) 데이터셋·영상으로 검증 (OAK 없이)
```bash
# tum_publisher(RGB+depth) → yolo_detector(d435i) → tracking_node → viz
# 토픽: /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw,
#       /camera/color/camera_info, /detections, /owner_pose
```
TUM RGB-D 같은 RGB+depth 데이터셋을 `tum_publisher.py`로 재생하고 `yolo_detector`로
검출하면, OAK와 동일한 토픽 흐름으로 추적을 검증할 수 있다.
`test/oak_check/viz_dataset.py`는 검출(회색)·주인(초록)·발행 위치(노랑=검출/자홍=KF 외삽)·
재매핑 허용 범위(빨간 원)를 한 화면에 보여준다.

### 토픽 확인
```bash
ros2 topic hz /detections        # 검출 주기
ros2 topic echo /owner_pose      # 주인 위치 (is_detected, distance, azimuth, track_id)
```

---

## 환경

- **개발 PC**: Ubuntu 22.04 + ROS2 Humble
- **연구실 PC / Jetson(JetPack 6.2)**: Ubuntu 22.04 기반 → ROS2 Humble (배포판이 다르면
  `source` 경로를 맞출 것)
- **카메라**: OAK-D S2 (DepthAI v3) / 검증 시 RealSense D435i 또는 데이터셋
- **의존성**: Eigen 3.3+, C++17+, CMake 3.14+, DepthAI(OAK), ultralytics(d435i detector)

각 패키지의 상세 설정·트러블슈팅은 패키지별 README(예: `ros2_yolo_oak/README.md`) 참고.

---

## 기반 / 라이선스

추적 알고리즘은 [Vertical-Beach/ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp)
(ByteTrack의 C++ 구현)를 기반으로 하며, 그 위에 RGB-D 확장·주인 특정·ROS2 통합·OAK 연동을
추가했다. 원본과 동일하게 **MIT License**.
