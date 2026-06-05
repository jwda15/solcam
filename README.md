# solcam

1인 크리에이터용 팔로우 카메라 로봇. OAK-D 카메라로 주인을 인식·추적하고,
메카넘 휠 차체 + 리프트 + 회전 스테이지를 제어해 따라다니며 촬영한다.

추적기는 [Vertical-Beach/ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp)를
RGB-D와 주인 고정 추적 용도로 확장한 것이다. MIT License.

## 파이프라인

```
detector ──/detections──▶ tracking_node ──/owner_pose──▶ control_node ──/control_cmd──▶ driver
(사람 검출+depth)         (주인 추적+3D)                  (주행/리프트/회전 결정)        (모터 구동, 작업 중)
```

노드 사이는 토픽으로만 연결된다. detector를 갈아끼워도(OAK / RealSense / 데이터셋)
뒷단은 그대로 동작한다.

## detector — `ros2_yolo_oak/`

사람을 검출해 bbox(픽셀)와 depth(mm)를 발행한다. 누가 주인인지는 모른다.

- 발행: `/detections` (DetectionArray: bbox + depth + score), `/camera_info`
- `ros2_yolo_oak/oak_detector.py` — OAK-D 온보드 YOLO 추론(DepthAI v3). 실기 전용
- `ros2_yolo_d435i/yolo_detector.py` — ultralytics YOLO. RealSense나 저장된 영상 검증용
- `tum_publisher.py`(루트) — TUM RGB-D 데이터셋을 카메라 토픽으로 재생

## tracking_node — `ros2_tracking_node/`

검출들을 ByteTrack으로 추적하고 그중 주인 한 명을 특정해 3D 위치를 발행한다.

- 구독: `/detections`, `/camera_info`
- 발행: `/owner_pose` (OwnerPose: is_detected, spatial_x/y/z, azimuth, distance, track_id)

`src/`, `include/`는 ByteTrack 라이브러리 본체(빌드하면 `libbytetrack.so`).
원본 대비 수정: Kalman 상태를 8D→10D로 확장해 depth(z)를 직접 추정하고,
1차 매칭을 IoU 대신 3D Mahalanobis 거리로 한다.

`tracking_node.cpp`의 주인 관리 로직:

1. 초기화 — 주인이 없으면 화면 중앙에서 가장 가까운 트랙을 주인으로 지정
2. 추적 — 매 프레임 주인 ID의 트랙을 찾아 위치 갱신
3. 놓쳤을 때 — KF 예측으로 짧은 깜빡임을 메우고, 재매핑 허용 거리를
   놓친 시간에 비례해 넓혀가며 복귀를 기다린다
4. bbox 중심 + depth + intrinsic으로 3D 역투영, EMA 평활 후 발행

## control_node — `ros2_control_node/`

주인 위치와 오도메트리를 받아 차체 속도, 리프트 높이, 상단 회전각을 결정한다.
모터 개수나 추력은 모른다 — 자유도 명령까지만 내고, 모터 매핑은 driver 책임.

- 구독: `/owner_pose`, `/odom`, `/top_yaw_state`(상단 스테이지 현재각),
  `/control_mode`, `/gesture_active`, `/adjust_cmd`(손동작), `/proximity`(근접센서)
- 발행: `/control_cmd` (메카넘=속도, 리프트·상단yaw=위치 목표), `/control_debug`(튜닝용)

파일은 헤더(include/)에 선언, src/에 정의. 클래스별 역할:

| 파일 | 역할 |
|------|------|
| `control_node.cpp` | ROS 입출력과 50Hz 제어 루프(`controlStep`). 콜백은 값 저장만 하고 모든 결정은 루프에서 |
| `state_estimator.cpp` | 주인 글로벌 위치 합성: `bearing = 로봇yaw + 상단yaw각 − azimuth`. 미탐지 시 직전 위치 유지 |
| `follow_controller.cpp` | 모드1. 진입 순간 주인-로봇 선분(거리 D, 글로벌각 φ)을 캡처하고, 목표점 `주인위치 − D·(cosφ,sinφ)`를 추종 |
| `rotate_controller.cpp` | 모드2. 위치 고정, 몸체 yaw만 주인을 추종 |
| `controller_base.cpp` | 모드 공통: 상단yaw 주인 락온, 리프트, 헤딩 PD, 가속 제한. 새 모드는 이걸 상속 |
| `obstacle_field.cpp` | 근접센서 기준 막힌 방향 속도 성분만 감쇠 |
| `pid.cpp` | PD 제어기 부품 (ki=0으로 사용) |
| `params.hpp` | 튜닝 파라미터 전부. 런타임 값은 `config/control_params.yaml` |

제어 구조는 PD → 데드존 → 가속 제한 3단이고, 발행 직전에 속도 상한
(v_max 0.4 m/s)을 한 번 더 강제한다. 손동작은 `/adjust_cmd`로 선분 거리·각도,
촬영 헤딩 오프셋, 리프트 높이를 조정하고, `/gesture_active` 동안 몸체만 감속
정지한다(OAK-D 추적은 유지).

ROS 없이 제어 로직만 검증하는 시뮬 테스트가 있다:

```bash
cd ros2_control_node
g++ -std=c++17 -Iinclude src/types.cpp src/pid.cpp src/obstacle_field.cpp \
    src/state_estimator.cpp src/controller_base.cpp src/follow_controller.cpp \
    src/rotate_controller.cpp test/test_sim.cpp -o /tmp/test_sim
/tmp/test_sim
```

## driver (작업 중)

`/control_cmd`를 받아 메카넘 역기구학으로 4휠 속도를 분배하고(BTS7960),
리프트(NEMA23+DM542)·상단yaw(NEMA17+A4988)는 스텝 펄스로 변환한다.
`/odom`(휠 엔코더)과 `/top_yaw_state` 발행도 이쪽 담당.

## 빌드

경로에 공백·한글이 있으면 colcon 메시지 생성이 깨지므로, 빌드는 영문 경로
워크스페이스에 심볼릭 링크를 걸어서 한다.

```bash
source /opt/ros/humble/setup.bash

# ByteTrack 라이브러리
cd <저장소>/build && cmake .. && cmake --build . -j

# ROS2 패키지
mkdir -p ~/solcam_ws/src && cd ~/solcam_ws/src
ln -sfn <저장소>/ros2_tracking_node ros2_tracking_node
ln -sfn <저장소>/ros2_control_node  ros2_control_node
ln -sfn <저장소>/ros2_yolo_oak      ros2_yolo_oak
cd ~/solcam_ws
colcon build --packages-select ros2_tracking_node ros2_control_node ros2_yolo_oak
source install/setup.bash
```

`ros2_tracking_node`를 먼저(또는 함께) 빌드해야 한다. 다른 패키지가 메시지에 의존한다.

## 실행

```bash
ros2 launch ros2_yolo_oak oak_tracking.launch.py        # OAK 검출+추적
ros2 launch ros2_control_node control.launch.py mode:=1 # 제어 (1=팔로우 모드)

ros2 topic echo /owner_pose      # 주인 위치 확인
ros2 topic echo /control_debug   # 제어 내부 상태 (선분, 추정 위치, 발행 명령)
```

OAK 없이 검증할 때는 `tum_publisher.py`로 데이터셋을 재생하고 d435i detector를 쓴다.
시각화 도구는 `test/oak_check/`에 있다. 패키지별 상세는 각 디렉터리 README 참고.
