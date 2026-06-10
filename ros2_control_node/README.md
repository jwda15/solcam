# ros2_control_node

팔로잉 로봇 **제어 노드**. 트래킹 노드(`/owner_pose`)가 준 주인 위치와
휠 오도메트리(`/odom`)로 6자유도 명령을 산출해 드라이버 노드(팀원)로 보낸다.

> 설계 배경·제어 로직 전체는 저장소 루트의 `제어로직.md` 참조.

## 책임 경계

```
트래킹 노드        →  제어 노드(이 패키지)   →  드라이버 노드(팀원)
/owner_pose           6자유도 명령               모터 추력/펄스 매핑
/odom (펌웨어)        (ControlCmd.msg)           (메카넘4=속도,
/top_yaw_state                                    리프트1·상단yaw1=위치)
```

제어 노드는 **모터 개수를 모른다.** 자유도 명령까지만 책임진다.

## 파일 구조 (헤더=선언, src=정의)

```
include/control_node/
├── params.hpp           ★튜닝 파라미터 전부 (이 파일+yaml만 보면 됨)
├── types.hpp            공통 타입/좌표계 약속 (Vec2, ControlCommand, UserAdjust)
├── controller.hpp       IController 인터페이스 + ControlInput 묶음
├── controller_base.hpp  모드 공통 부품 (상단yaw 락온/리프트/헤딩/슬루)
├── follow_controller.hpp  모드1(FOLLOW) 선분 유지
├── rotate_controller.hpp  모드2(ROTATE) 제자리 회전 추적
├── state_estimator.hpp  주인 글로벌 위치 추정 (오도+상단yaw+카메라 합성)
├── obstacle_field.hpp   측면센서 간단 회피 (속도 깎기)
├── pid.hpp              PID/슬루 부품 (몸체 PD 제어에 사용, ki=0)
└── control_node.hpp     ROS 노드 선언 (전략 패턴: controllerFor)
src/                     위 헤더들의 정의(.cpp) + main.cpp
test/test_sim.cpp        ROS 없는 g++ 시뮬 검증 (T1~T12)
config/control_params.yaml  런타임 파라미터 (params.hpp와 기본값 일치 유지)
```

## 모드1(FOLLOW) — "선분 유지"

주인-로봇을 잇는 선분의 **길이 D**와 **글로벌 각도 φ**를 모드 진입 순간
캡처해 유지. 주인이 움직이면 선분이 평행이동하듯 로봇이 따라간다.

1. **상단 yaw (스텝, 위치)**: `azimuth → 0`. OAK-D를 주인에 락온.
2. **몸체 vx/vy (메카넘, 속도)**: `target = owner_global − D·(cosφ, sinφ)` PD 추종.
3. **몸체 yaw**: 주인 방향 + `heading_offset` 헤딩 PD 유지.

## 제어 구조 (발표용 요약)

```
[상위 — 이 노드, 50Hz]                    [하위 — 드라이버 노드(팀원)]
오차 → PD(kp·e + kd·ė) → 데드존 →  속도   → 휠 속도 PID(엔코더 피드백)
      → v_max 클램프 → 가속(슬루)제한      명령   → 모터 PWM
```

- **PD**: P가 끌고, D가 접근 속도를 보고 미리 감속(오버슈트·출렁임 억제)
- **데드존**: 도착/정렬 후 미세 떨림 방지 (pos_dead, byaw_dead)
- **가속 제한**: 급가감속 차단 — 메카넘 미끄럼·카메라 흔들림 방지
- I항은 안 씀(ki=0): 정상상태 오차보다 떨림 억제가 우선인 추종 문제라서.
- 실주행 튜닝 순서: kd=0(P만) → kp 맞추기 → 출렁이면 kd 올리기.

주인 글로벌 위치는 `StateEstimator`가 합성:
`bearing_g = robot.yaw + theta_head − azimuth`. 트래킹 끊기면 직전 위치 hold.

## 모드0(IDLE) — 대기 + 키보드 수동주행

자율 추적 정지 상태. 상단 yaw도 정지(현 위치 유지). 부팅 기본 모드.
`teleop_keyboard`로 수동 주행하고, 손동작(gesture_node)으로 모드를 바꾼다.
키보드 teleop은 모드0에서만 주행에 반영된다(다른 모드는 자율 제어).

```
ros2 run ros2_control_node teleop_keyboard   # pygame 창: ↑↓←→ 주행, a/d 회전,
                                             #   space 정지, m+숫자 모드변경
```

## 모드2(ROTATE) — "제자리 회전 추적"

차체 위치 고정(vx=vy=0), 메카넘으로 **몸체 yaw 회전만** 해서 주인을 따라
돈다. 상단 yaw도 주인 락온 유지 → 평상시 촬영 카메라와 OAK-D가 같은
방향(주인)을 본다. 게인은 모드1의 몸체 yaw 계열(kp_byaw 등)을 재사용.

## 손동작 조정 — /adjust_cmd (AdjustCmd.msg)

따봉 트리거 후 손동작 하나 = AdjustCmd 하나. `param` 상수로 대상 지정,
`delta` 로 증분/절대 선택. 새 기능은 msg 상수 + `adjustCallback` case 한 줄.

| param | 의미 | 적용 |
|-------|------|------|
| `PARAM_SEG_DISTANCE` | 선분 거리 D [m] | 모드1 |
| `PARAM_SEG_ANGLE` | 선분 글로벌각 φ [rad] | 모드1 |
| `PARAM_HEADING_OFFSET` | 몸체(촬영 카메라) 헤딩 오프셋 [rad] | 공통 |
| `PARAM_LIFT_HEIGHT` | 리프트 목표 높이 [m] | 공통 |

`HEADING_OFFSET ≠ 0` 이면 OAK-D는 주인 락온을 유지한 채 촬영 카메라만
다른 방향을 본다(주인 외 풍경 연출). 모드를 바꿔도 오프셋은 유지된다.

권장 시퀀스: 따봉 인식 → `/gesture_active=true`(몸체 정지, OAK-D는 계속
사용자 추적) → 손동작 → `/adjust_cmd` 발행 → `/gesture_active=false`
(주행 재개, 모드1은 선분 목표로 자연 복귀).

## 토픽

입력:
- `/owner_pose` (ros2_tracking_node/OwnerPose)
- `/odom` (nav_msgs/Odometry) — 휠 오도메트리 (팀원 펌웨어)
- `/top_yaw_state` (std_msgs/Float32) — 상단 yaw 현재 각 [rad]
- `/control_mode` (std_msgs/Int32) — supervisor(손동작)가 모드 결정
- `/gesture_active` (std_msgs/Bool) — true 동안 몸체 일시정지(손동작 세션)
- `/adjust_cmd` (ros2_control_node/AdjustCmd) — 손동작 조정 명령 (위 표)
- `/proximity` (ros2_control_node/ProximityArray) — 측면센서 6개
- `/teleop_cmd` (geometry_msgs/Twist) — 모드0 키보드 수동주행

출력:
- `/control_cmd` (ros2_control_node/ControlCmd) — 메카넘 속도 + 스텝 위치 목표
- `/control_debug` (ros2_control_node/ControlDebug) — 튜닝/모니터링용 내부 상태
  (선분 D·φ, 주인 글로벌 추정, 모드, hold, 발행 명령. rqt_plot 으로 보기)

## 빌드

```bash
# ~/solcam_ws/src 에 심볼릭 링크 (최초 1회)
ln -sfn <repo>/ros2_control_node ~/solcam_ws/src/ros2_control_node
cd ~/solcam_ws
colcon build --packages-select ros2_tracking_node ros2_control_node
source install/setup.bash
```

> `ros2_tracking_node` 를 먼저(또는 함께) 빌드해야 `OwnerPose` 를 찾는다.

## ROS 없는 시뮬 검증

```bash
cd ros2_control_node
g++ -std=c++17 -Iinclude src/types.cpp src/pid.cpp src/obstacle_field.cpp \
    src/state_estimator.cpp src/controller_base.cpp src/idle_controller.cpp \
    src/follow_controller.cpp \
    src/rotate_controller.cpp test/test_sim.cpp \
    -o /tmp/test_sim && /tmp/test_sim
```

## 실행

```bash
ros2 launch ros2_control_node control.launch.py            # IDLE로 시작
ros2 launch ros2_control_node control.launch.py mode:=1    # 바로 FOLLOW

# 모드 수동 전환(테스트)
ros2 topic pub /control_mode std_msgs/Int32 "data: 1" -1
```

## 현황 / TODO

- [x] 모드1(FOLLOW) 선분 유지 + StateEstimator
- [x] 모드2(ROTATE) 제자리 회전 추적 + ControllerBase 공통화 (0604)
- [x] 손동작 조정 인터페이스 /adjust_cmd (선분·헤딩 오프셋·리프트) (0604)
- [x] 손동작 세션 일시정지 /gesture_active (hold_body) (0605)
- [x] /control_debug 모니터링 토픽 + 선분 거리 클램프 + 지연보상 훅 (0605)
- [x] 몸체 PD 제어 (kd_pos/kd_byaw) + 데드존 + 가속 제한 (0605)
- [x] 측면센서 간단 장애물 회피
- [x] ControlCmd / ProximityArray / AdjustCmd 메시지
- [x] 헤더 선언/소스 정의 분리, params.hpp 파라미터 통합 (0604)
- [x] 모드3(FOLLOW2) leash 거리유지 (0607, odom 불필요)
- [x] 모드4(ORBIT) 공전 (0607, odom 불필요)
- [ ] 모드5(COMPOSE 자율 프레이밍 등)
- [ ] 상단 yaw 언와인딩(2바퀴 한계) — 지금은 파라미터 자리만
- [ ] 리프트 자동 프레이밍(spatial_y 기반)
- [ ] supervisor(손동작 상태기계, 미디어파이프) — AdjustCmd 발행 (팀원 분담)
- [ ] /odom, /top_yaw_state 토픽명·타입 팀원 펌웨어와 합의
