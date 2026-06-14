# 드라이버 펌웨어 점검 — yaw "한 방향만 회전" 진단 (정정판)

## 1. 결론 (정정 — 클럭은 버그 아님)
- **yaw 방향 로직(committed main.c)은 정상.** +각/−각 번갈아 주면 정상 역회전(2 트레이스).
  이 커밋본대로면 "한 방향만"은 안 나온다.
- **클럭(HSI 16MHz)은 버그가 아니다.** 타이머 프리스케일러가 전부 `16-1`(=1MHz tick)로
  16MHz에 "일관되게" 맞춰져 있고 delay_us(TIM6 1MHz)도 정확하다. 16MHz로 도는 건
  사실이나 기능상 문제 없음(칩 최대성능 16/168 만 쓸 뿐). ★앞선 "클럭이 1순위 버그"
  기재는 오판이라 정정함.
- 따라서 **클럭을 고쳐도 한방향 증상은 안 고쳐진다.** 클럭은 속도에만 영향, 방향 무관.

## 2. yaw 방향 로직 검증 (왜 정상인가)
Yaw_SetTarget: target=rad*636.6; delta=target-current; delta>0→DIR(PC1)=SET(CW) 아니면 RESET.
ISR(TIM7): STEP(PC0) 토글, 상승엣지마다 DIR==SET?current++:current--; current==target면 정지.
트레이스: 0→+π(+2000): delta+ →DIR SET→ +2000(CW). 이어 −π(−2000): delta−4000 →DIR RESET
          → +2000→−2000(CCW, 역회전). ✅ 방향 전환 정상. (yaw 엔코더 없음=open-loop, 정상)

## 3. "한 방향만" 원인 후보 (커밋본엔 SW 버그 없음)
 (a) 팀원이 겪은 건 이전/테스트 버전 → 이 커밋본 플래시해 +180/−180 재현부터.
 (b) ★HW: DIR핀(PC1)↔A4988 DIR 배선 불량/플로팅 → 방향 한쪽 고정. "명령 뭘 줘도
     한 방향"이면 가장 유력. PC1이 명령에 따라 토글되는지 멀티미터/LED로 확인.
 (c) 명령 경로: 메인루프는 `if(uart_rx_ready)` 일 때만 Yaw_SetTarget 적용(매 루프 아님).
     테스트는 4번 절차로.

## 4. 깨끗한 +180/−180 테스트
 (A) UART: yaw_target=+3.14 프레임 → 2초 → yaw_target=−3.14 프레임. 왕복 확인.
 (B) 임시 테스트 블록(메인루프, uart 없이; 이때 UART 명령은 보내지 말 것):
     static uint32_t tk=0; static int d=0;
     if (HAL_GetTick()-tk>2000){ tk=HAL_GetTick(); Yaw_SetTarget(d?+3.14f:-3.14f,1); d^=1; }
 물리 방향이 반대(우 줬는데 좌)면 → Yaw_SetTarget 의 SET/RESET 두 줄 swap(배선 극성 보정).

## 5. yaw 속도가 느리면 (이게 실제 신경 쓸 점, 클럭 무관)
 현재 TIM7 Period=4999 → 200Hz ISR → ~100 full-step/s 로 느림.
 → MX_TIM7_Init 의 `htim7.Init.Period` 를 줄이면 빠라짐(예 4999→999 = 5배).
   클럭/프리스케일러 안 건드림. (스텝 누락 나면 다시 키우기)

## 6. (선택, 비권장) 168MHz 업그레이드
 더 빠른 칩 성능이 꼭 필요할 때만. 클럭만 바꾸면 안 되고 — 모든 타이머 프리스케일러
 (`16-1`)와 delay_us 가정을 84MHz/168MHz 기준으로 전부 다시 맞춰야 함(안 그러면 delay_us
 ·스텝레이트가 ~5배 빨라져 깨짐). 작업량·위험 크므로 지금 단계엔 권장하지 않음.

## 7. 적용 방법 (중요)
 STM32 펌웨어는 코드만 고친다고 적용 안 됨 → CubeIDE 재컴파일 + ST-Link/USB **재플래시** 필수.

## 8. 핀맵 (참고)
 yaw: STEP=PC0 DIR=PC1 ENA=PC2 (A4988) / lift: STEP=PA6 DIR=PA7 ENA=PC4 (DM542)
 yaw 상수 636.6 steps/rad, 소프트리밋 ±10000 steps.
