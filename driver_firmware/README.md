# driver_firmware — STM32F407 드라이버 보드 펌웨어

차체의 모든 모터를 직접 구동하는 STM32CubeIDE 프로젝트(STM32F407VETx).
Jetson의 `ros2_driver_bridge` 노드와 **USART1 바이너리 프로토콜**로 통신한다.
ROS는 모른다 — 자유도 명령을 받아 모터로 바꾸는 것까지가 이쪽 책임.

```
control_node ─/control_cmd─▶ ros2_driver_bridge ─UART(28B)─▶ [이 펌웨어] ─▶ 모터
                            ◀──────────── UART(19B, 엔코더+yaw각) ──────────┘
```

## 담당 하드웨어

| 대상 | 모터/드라이버 | 타이머 | 명령 의미 |
|------|---------------|--------|-----------|
| 메카넘 4휠 | DC+엔코더 / BTS7960 | TIM5·9·10·11·12 (PWM) | 속도 |
| 리프트 | NEMA23 / TB6600 | TIM3 (스텝 IT) | 위치(높이) |
| 상단 yaw | NEMA17 / A4988 | TIM7 (스텝 IT) | 위치(각) |
| 휠 엔코더 ×4 | — | TIM1·2·4·8 (엔코더) | 카운트 |

기구 상수(`WHEEL_R=0.05`, `WHEEL_LX=0.36`, `WHEEL_LY=0.26`), 스텝 환산
(`LIFT_STEPS_PER_MM=38.55`, `YAW_STEPS_PER_RAD=636.6`)은 `Core/Src/main.c` 상단 참고.
브리지의 `driver_params.yaml` 값과 반드시 일치시킬 것.

## UART 프로토콜

`Core/Src/main.c` 상단 주석과 `ros2_driver_bridge/README.md`에 바이트 단위로 명시.
요약: 명령 28바이트(0xAA), 상태 19바이트(0xBB), 둘 다 LE + 단순합 체크섬.

## 원본 대비 패치 (이 레포에서 적용됨)

1. **명령 워치독** — STM32에 명령 타임아웃이 없어 Jetson이 죽거나 USB가 빠지면
   바퀴가 마지막 속도로 계속 돌았다. `CMD_TIMEOUT_MS`(300ms) 동안 유효 프레임이
   없으면 휠을 0으로 강제(리프트/yaw는 스텝 위치 유지). `main.c` 메인루프 참고.
2. **상태프레임 체크섬/크기** — `UART_SendStatus`가 체크섬을 계산 전 위치에 써넣고
   마지막 바이트가 미초기화로 전송되던 버그를 수정. reserved[17]=0, checksum[18]=
   sum(bytes[0..17]) 로 결정적이게 바로잡음.

## 빌드/플래시

STM32CubeIDE로 `yaw.ioc` 또는 프로젝트를 import해서 빌드 → ST-Link로 플래시.
`Debug/` 빌드 산출물은 `.gitignore`로 제외(IDE가 재생성).

## ⚠️ 실기 확인 사항

- yaw DIR 핀 부호: `Yaw_SetTarget`의 "CW/CCW" 주석이 ROS 좌표계(CCW+)와 맞는지
  실제 회전으로 확인. 반대면 부호 뒤집기.
- 엔코더 카운트 방향이 브리지 `encoder_signs`와 일관되는지(전진 시 +) 확인.
- 클럭이 HSI 16MHz(PLL 미설정)라 스텝 속도가 낮음 — 더 빠른 yaw/리프트가 필요하면
  PLL 설정 검토.
