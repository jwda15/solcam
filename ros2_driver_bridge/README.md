# ros2_driver_bridge

`control_node`(ROS2)와 팀원의 **STM32F407 드라이버 보드** 사이를 잇는 UART 브리지.
STM32 펌웨어는 ROS를 모르고 USART1 바이너리 프레임만 알기 때문에, Jetson에서
이 노드가 토픽 ↔ 시리얼을 변환한다.

```
control_node ──/control_cmd──▶ driver_bridge ──UART(28B)──▶ STM32
   ◀── /odom, /top_yaw_state ── driver_bridge ◀──UART(19B)── STM32
```

## 토픽

| 방향 | 토픽 | 타입 | 비고 |
|------|------|------|------|
| 구독 | `/control_cmd` | `ros2_control_node/ControlCmd` | 6자유도 명령 |
| 발행 | `/odom` | `nav_msgs/Odometry` | 휠 엔코더 정기구학+적분 (SensorDataQoS) |
| 발행 | `/top_yaw_state` | `std_msgs/Float32` | 상단 yaw 현재각[rad] (SensorDataQoS) |

QoS는 `control_node`가 `/odom`·`/top_yaw_state`를 `SensorDataQoS`(BEST_EFFORT)로
구독하므로 거기에 맞췄다.

## UART 프로토콜 (펌웨어 main.c 기준)

**Jetson → STM32 (28바이트)** — 명령
```
[0]      0xAA
[1..4]   vx          float32 LE  (m/s, +전방)
[5..8]   vy          float32 LE  (m/s, +좌측)
[9..12]  wz          float32 LE  (rad/s, +CCW)
[13..16] lift_target float32 LE  (m)
[17]     lift_active uint8       (0=현위치 유지)
[18..21] yaw_target  float32 LE  (rad)
[22]     yaw_active  uint8       (0=현위치 유지)
[23..26] reserved    (0)
[27]     checksum    uint8 = sum(bytes[0..26]) & 0xFF
```

**STM32 → Jetson (19바이트)** — 상태
```
[0]      0xBB
[1..2]   enc1 int16 LE   (지난 주기 카운트 델타)
[3..4]   enc2 int16 LE
[5..6]   enc3 int16 LE
[7..8]   enc4 int16 LE
[9..12]  lift_height float32 LE (m)   ※현재 미사용(디버그)
[13..16] yaw_angle   float32 LE (rad) → /top_yaw_state
[17]     reserved (0)
[18]     checksum uint8 = sum(bytes[0..17]) & 0xFF
```

`ControlCmd.msg`와 펌웨어 필드는 1:1, 단위도 일치(m/s, rad/s, m, rad)라 변환이 없다.

## 오도메트리

펌웨어는 4휠 raw 엔코더 델타만 보낸다. 브리지가 메카넘 **정기구학**으로 환산한다
(`L = lx + ly`):

```
vx = r/4 · ( wFL + wFR + wRL + wRR)
vy = r/4 · (-wFL + wFR + wRL - wRR)
wz = r/(4L) · (-wFL + wFR - wRL + wRR)
```

각 `w_i = 2π · (counts_i / cpr) / dt`. 이를 odom 프레임에서 적분해 pose를 만든다.
휠 데드레커닝이라 누적 드리프트가 있으니, FOLLOW/ROTATE 모드처럼 odom을 쓰는
용도엔 충분하지만 장시간 절대측위엔 한계가 있다.

## ★ 실기 브링업 전 캘리브 (config/driver_params.yaml)

1. **port** — `ls /dev/ttyUSB* /dev/ttyTHS*` 로 확인. USB 어댑터면 보통 `ttyUSB0`,
   Jetson 40핀 UART면 `ttyTHS1`.
2. **encoder_cpr** — 휠 1회전당 카운트. 휠을 손으로 정확히 1바퀴 돌리고 누적 카운트
   측정해서 넣는다. 기본 `1320`은 추정 placeholder(JGB37-520, 11PPR×4×30:1).
3. **encoder_order / encoder_signs** — 로봇을 전진시켰을 때 모든 `w_i`가 +가 되도록,
   또 좌/우/회전 부호가 맞도록 매핑·부호를 조정. (`/odom` echo 보며 확인)
4. **wheel_radius / wheel_lx / wheel_ly** — 펌웨어 `WHEEL_R/LX/LY`와 동일하게.

## 실행

```bash
# 권한 (USB 시리얼): 최초 1회
sudo usermod -aG dialout $USER   # 재로그인 필요
sudo apt install python3-serial

ros2 launch ros2_driver_bridge driver.launch.py port:=/dev/ttyUSB0

# 시리얼 없이 로직만 점검
ros2 launch ros2_driver_bridge driver.launch.py mock:=true
```

## ⚠️ 펌웨어 측 필수 패치 (재플래시 필요)

브리지만으론 못 막는 문제가 펌웨어에 있어 함께 패치했다 (`D:/capstone/yaw`):

1. **명령 워치독** — STM32에 명령 타임아웃이 없어, Jetson이 죽거나 USB가 빠지면
   바퀴가 마지막 속도로 계속 돈다. `CMD_TIMEOUT_MS`(기본 300ms) 동안 유효 프레임이
   없으면 휠을 0으로 강제하는 워치독을 추가했다.
2. **상태프레임 체크섬/크기** — 원본 `UART_SendStatus`는 체크섬을 계산 전 위치에
   써넣고 마지막 바이트가 미초기화로 나가는 버그가 있었다. 위 19바이트 레이아웃
   (reserved[17]=0, checksum[18]=sum[0..17])으로 바로잡았다.

위 패치가 적용된 펌웨어를 플래시해야 `/odom` 체크섬 검증(`verify_rx_checksum`)이
정상 동작한다. 구버전 펌웨어로 테스트하려면 `verify_rx_checksum: false`로 두고
헤더 동기화만으로 받을 수 있다(권장하지 않음).
