# 드라이버 펌웨어 점검 — yaw "한 방향만 회전" 진단 + 클럭 수정

## 1. 결론 요약
- **yaw 방향 로직(committed main.c)은 정상이다.** +각/−각을 번갈아 주면 정상적으로
  역회전한다(아래 2 트레이스). "한 방향만 돈다"는 이 코드대로면 안 나온다 →
  팀원이 겪은 건 이전 버전/테스트 코드일 가능성이 큼. 이 커밋본을 플래시해 재현 확인.
- **진짜 결함은 클럭이다.** SystemClock_Config 가 HSI 16MHz(PLL 미사용)로 둔 채
  타이머 프리스케일러는 168MHz(84MHz APB) 기준이라 불일치 → 스텝 펄스·delay_us
  타이밍이 전부 어긋난다(약 5~10배 느림/이상). ★1순위 수정.

## 2. yaw 방향 로직 검증 (왜 정상인가)
Yaw_SetTarget(target_rad, active):
  target_steps = target_rad * 636.6  (소프트리밋 ±10000 클램프)
  delta = target_steps - yaw_current_steps
  delta > 0 → DIR(PC1)=SET(CW) / 그 외 → RESET(CCW)
ISR(TIM7): STEP(PC0) 토글, 상승엣지마다  DIR==SET ? current++ : current--
           current == target 면 정지.
트레이스: current=0 에서 +π(=+2000) → delta=+2000 → DIR=SET → current 0→+2000(CW).
          이어서 −π(=−2000) → delta=−4000 → DIR=RESET → current +2000→−2000(CCW, 역회전).
→ 방향 전환 정상. (open-loop: yaw_current_steps 는 명령 누적값. yaw 엔코더 없음 — 정상)

## 3. ★클럭 수정 (1순위)
현재: HSI 16MHz, PLL_NONE, APB1/APB2 div1(=16MHz). → F407 정격 168MHz의 ~1/10.
타이머 프리스케일러(예: 83 → 84MHz/84 = 1MHz tick)·delay_us(TIM6 1MHz 가정)가
이 16MHz와 안 맞아 펄스레이트/지연이 전부 틀어짐.

### 권장: CubeMX(.ioc)에서 클럭 트리 수정 후 재생성 (★이게 정석 — .ioc도 같이 맞아야
###  다음 코드 생성 때 안 되돌아감)
  - HSE 25MHz 사용, PLL: PLLM=25, PLLN=336, PLLP=2 → SYSCLK 168MHz, PLLQ=7(48MHz)
  - AHB ÷1(168), APB1 ÷4(42 → 타이머클럭 84), APB2 ÷2(84 → 타이머클럭 168)
  - Flash latency 5 (자동)

### 스톱갭: main.c SystemClock_Config 를 아래로 교체 (보드에서 반드시 검증!)
```c
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;            // ★보드 HSE 가 25MHz 일 때
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 168MHz
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;  // 168
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;    // 42 (TIM3/6/7 클럭 84MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;    // 84 (TIM1/8 클럭 168MHz)
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}
```
★주의: 보드 HSE 크리스털이 실제 25MHz인지 확인(.ioc HSE_VALUE=25000000 기준). 다르면
  PLLM 을 (HSE_MHz)로. 잘못 넣으면 부팅 실패→ST-Link 재플래시 필요. 첫 플래시 후
  USART1 상태프레임이 정상 주기로 오는지로 클럭 정상 여부 확인.

수정 후 기대: delay_us 정확(TIM6=1MHz), 스텝레이트 의도대로 → yaw/리프트 정상 속도.

## 4. 깨끗한 +180/−180 테스트 절차
메인루프는 `if (uart_rx_ready)` 일 때만 Yaw_SetTarget 을 적용한다(매 루프 호출 아님).
→ 따라서 테스트는 둘 중 하나:
  (A) UART 로 명령: yaw_target=+3.14 프레임 → 2초 → yaw_target=−3.14 프레임. 왕복 확인.
  (B) 펌웨어에 임시 테스트 블록(메인루프 안, uart 없이):
      static uint32_t tk=0; static int dir=0;
      if (HAL_GetTick()-tk > 2000){ tk=HAL_GetTick();
        Yaw_SetTarget(dir? +3.14f : -3.14f, 1); dir^=1; }
      ★주의: 이 블록과 `if(uart_rx_ready) Yaw_SetTarget(cmd...)` 가 충돌하지 않게,
       테스트 중엔 UART 명령을 보내지 말 것(둘이 yaw_target 을 두고 다툼).
방향이 물리적으로 반대면(우로 줬는데 좌로) → Yaw_SetTarget 의 DIR SET/RESET 두 줄만
  서로 바꾸면 됨(A4988 DIR 배선 극성 보정). 로직 재설계 불필요.

## 5. 핀맵 (참고, main.c 기준)
  yaw  : STEP=PC0, DIR=PC1, ENA=PC2   (A4988)
  lift : STEP=PA6, DIR=PA7, ENA=PC4   (DM542)
  yaw 상수: 636.6 steps/rad, 소프트리밋 ±10000 steps(≈±15.7rad≈±2.5바퀴)
