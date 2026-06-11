/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SolCam Driver
  *                   휠모터4(메카넘) + 리프트(TIM3) + yaw(TIM7) + 엔코더4 + USART1
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);

/* USER CODE BEGIN 0 */

// ==========================================
// UART 프로토콜
// ==========================================
// Jetson → STM32 (28바이트):
// [0xAA][vx:4][vy:4][wz:4][lift_target:4][lift_active:1][yaw_target:4][yaw_active:1][checksum:1]
//
// STM32 → Jetson (19바이트):
// [0xBB][enc1:2][enc2:2][enc3:2][enc4:2][lift_height:4][yaw_angle:4][checksum:1]

#define UART_RX_SIZE    28
#define UART_TX_SIZE    19
#define UART_HEADER_RX  0xAA
#define UART_HEADER_TX  0xBB

uint8_t uart_rx_buf[UART_RX_SIZE];
uint8_t uart_tx_buf[UART_TX_SIZE];
volatile uint8_t uart_rx_ready = 0;

// ==========================================
// 제어 명령
// ==========================================
typedef struct {
    float   vx;
    float   vy;
    float   wz;
    float   lift_target;
    uint8_t lift_active;
    float   yaw_target;
    uint8_t yaw_active;
} ControlCmd;

volatile ControlCmd cmd = {0};

// ==========================================
// 안전 워치독
//   Jetson(브리지)이 죽거나 USB가 빠지면 마지막 PWM이 유지돼 바퀴가 계속
//   돈다. 유효 프레임을 마지막으로 받은 시각을 기록해, CMD_TIMEOUT_MS 동안
//   소식이 없으면 메카넘을 0으로 강제한다(리프트/yaw는 현위치 유지).
// ==========================================
#define CMD_TIMEOUT_MS  300
volatile uint32_t last_cmd_tick = 0;

// ==========================================
// 메카넘 상수
// ==========================================
#define V_MAX       0.4f    // m/s 최대속도
#define PWM_MAX     999     // PWM 최대값
#define WHEEL_R     0.05f   // 휠 반지름 5cm
#define WHEEL_LX    0.36f   // 좌우 거리 절반 (72cm/2)
#define WHEEL_LY    0.26f   // 전후 거리 절반 (52cm/2)

// ==========================================
// 리프트 상수 (NEMA23 + TB6600)
// ==========================================
#define LIFT_STEPS_PER_MM   38.55f
#define LIFT_MIN_MM         0.0f
#define LIFT_MAX_MM         850.0f
#define LIFT_MAX_STEPS      32768

// ==========================================
// yaw 상수 (NEMA17 + A4988)
// ==========================================
#define YAW_STEPS_PER_RAD   636.6f
#define YAW_MIN_STEPS       -10000
#define YAW_MAX_STEPS       10000

// ==========================================
// 리프트 상태
// ==========================================
volatile int32_t lift_current_steps = 0;
volatile int32_t lift_target_steps  = 0;
volatile uint8_t lift_running       = 0;
volatile uint8_t lift_pulse_state   = 0;

// ==========================================
// yaw 상태
// ==========================================
volatile int32_t yaw_current_steps  = 0;
volatile int32_t yaw_target_steps   = 0;
volatile uint8_t yaw_running        = 0;
volatile uint8_t yaw_pulse_state    = 0;

// ==========================================
// delay_us (TIM6)
// ==========================================
void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while (__HAL_TIM_GET_COUNTER(&htim6) < us);
}

// ==========================================
// 메카넘 역기구학
// 모터배치:
//   1(FL) --- 2(FR)
//   3(RL) --- 4(RR)
// ==========================================
void Mecanum_SetVelocity(float vx, float vy, float wz)
{
    float lxy = WHEEL_LX + WHEEL_LY;
    float w_max_limit = V_MAX / WHEEL_R;

    // 역기구학 계산
    float w_fl = (vx - vy - lxy * wz) / WHEEL_R;  // 모터1
    float w_fr = (vx + vy + lxy * wz) / WHEEL_R;  // 모터2
    float w_rl = (vx + vy - lxy * wz) / WHEEL_R;  // 모터3
    float w_rr = (vx - vy + lxy * wz) / WHEEL_R;  // 모터4

    // 정규화
    float max_w = fabsf(w_fl);
    if (fabsf(w_fr) > max_w) max_w = fabsf(w_fr);
    if (fabsf(w_rl) > max_w) max_w = fabsf(w_rl);
    if (fabsf(w_rr) > max_w) max_w = fabsf(w_rr);

    if (max_w > w_max_limit) {
        w_fl = w_fl / max_w * w_max_limit;
        w_fr = w_fr / max_w * w_max_limit;
        w_rl = w_rl / max_w * w_max_limit;
        w_rr = w_rr / max_w * w_max_limit;
    }

    // 모터1 FL (htim10 R_PWM, htim11 L_PWM)
    if (w_fl >= 0) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1,
            (uint32_t)(w_fl / w_max_limit * PWM_MAX));
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,
            (uint32_t)(-w_fl / w_max_limit * PWM_MAX));
    }

    // 모터2 FR (htim5 CH3 R_PWM, CH4 L_PWM)
    if (w_fr >= 0) {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,
            (uint32_t)(w_fr / w_max_limit * PWM_MAX));
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4,
            (uint32_t)(-w_fr / w_max_limit * PWM_MAX));
    }

    // 모터3 RL (htim9 CH1 R_PWM, CH2 L_PWM)
    if (w_rl >= 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,
            (uint32_t)(w_rl / w_max_limit * PWM_MAX));
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,
            (uint32_t)(-w_rl / w_max_limit * PWM_MAX));
    }

    // 모터4 RR (htim12 CH1 R_PWM, CH2 L_PWM)
    if (w_rr >= 0) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1,
            (uint32_t)(w_rr / w_max_limit * PWM_MAX));
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,
            (uint32_t)(-w_rr / w_max_limit * PWM_MAX));
    }
}

// ==========================================
// 리프트 제어
// ==========================================
int32_t lift_meter_to_steps(float height_m)
{
    float height_mm = height_m * 1000.0f;
    if (height_mm < LIFT_MIN_MM) height_mm = LIFT_MIN_MM;
    if (height_mm > LIFT_MAX_MM) height_mm = LIFT_MAX_MM;
    return (int32_t)(height_mm * LIFT_STEPS_PER_MM);
}

float lift_steps_to_meter(int32_t steps)
{
    return (float)steps / LIFT_STEPS_PER_MM / 1000.0f;
}

void Lift_SetTarget(float target_m, uint8_t active)
{
    if (!active) {
        lift_running = 0;
        HAL_TIM_Base_Stop_IT(&htim3);
        return;
    }

    int32_t target = lift_meter_to_steps(target_m);
    int32_t delta  = target - lift_current_steps;
    if (abs(delta) < 5) return;

    lift_target_steps = target;
    lift_running      = 1;

    if (delta > 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   // UP
    else
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // DOWN

    delay_us(5);
    HAL_TIM_Base_Start_IT(&htim3);
}

float Lift_GetHeight(void)
{
    return lift_steps_to_meter(lift_current_steps);
}

// ==========================================
// yaw 제어
// ==========================================
void Yaw_SetTarget(float target_rad, uint8_t active)
{
    if (!active) {
        yaw_running = 0;
        HAL_TIM_Base_Stop_IT(&htim7);
        return;
    }

    int32_t target = (int32_t)(target_rad * YAW_STEPS_PER_RAD);

    // 소프트 리밋
    if (target > YAW_MAX_STEPS) target = YAW_MAX_STEPS;
    if (target < YAW_MIN_STEPS) target = YAW_MIN_STEPS;

    int32_t delta = target - yaw_current_steps;
    if (abs(delta) < 2) return;

    yaw_target_steps = target;
    yaw_running      = 1;

    if (delta > 0)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);   // CW
    else
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // CCW

    delay_us(5);
    HAL_TIM_Base_Start_IT(&htim7);
}

float Yaw_GetAngle(void)
{
    return (float)yaw_current_steps / YAW_STEPS_PER_RAD;
}

// ==========================================
// TIM 인터럽트 콜백
// ==========================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 리프트 (TIM3)
    if (htim->Instance == TIM3)
    {
        if (!lift_running) return;

        // 소프트 리밋
        if (lift_current_steps <= 0 &&
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {
            lift_running = 0;
            HAL_TIM_Base_Stop_IT(&htim3);
            return;
        }
        if (lift_current_steps >= LIFT_MAX_STEPS &&
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) {
            lift_running = 0;
            HAL_TIM_Base_Stop_IT(&htim3);
            return;
        }

        // 목표 도달
        if (lift_current_steps == lift_target_steps) {
            lift_running = 0;
            HAL_TIM_Base_Stop_IT(&htim3);
            return;
        }

        // 펄스 토글
        lift_pulse_state = !lift_pulse_state;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
            lift_pulse_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // 스텝 카운터
        if (lift_pulse_state) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)
                lift_current_steps++;
            else
                lift_current_steps--;
        }
    }

    // yaw (TIM7)
    if (htim->Instance == TIM7)
    {
        if (!yaw_running) return;

        // 목표 도달
        if (yaw_current_steps == yaw_target_steps) {
            yaw_running = 0;
            HAL_TIM_Base_Stop_IT(&htim7);
            return;
        }

        // 소프트 리밋
        if (yaw_current_steps <= YAW_MIN_STEPS &&
            HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) {
            yaw_running = 0;
            HAL_TIM_Base_Stop_IT(&htim7);
            return;
        }
        if (yaw_current_steps >= YAW_MAX_STEPS &&
            HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) {
            yaw_running = 0;
            HAL_TIM_Base_Stop_IT(&htim7);
            return;
        }

        // 펄스 토글
        yaw_pulse_state = !yaw_pulse_state;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,
            yaw_pulse_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // 스텝 카운터
        if (yaw_pulse_state) {
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET)
                yaw_current_steps++;
            else
                yaw_current_steps--;
        }
    }
}

// ==========================================
// UART 체크섬
// ==========================================
uint8_t calc_checksum(uint8_t *buf, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += buf[i];
    return sum;
}

// ==========================================
// UART 수신 콜백
// ==========================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 헤더 확인
        if (uart_rx_buf[0] != UART_HEADER_RX) {
            HAL_UART_Receive_IT(&huart1, uart_rx_buf, UART_RX_SIZE);
            return;
        }

        // 체크섬 확인
        uint8_t checksum = calc_checksum(uart_rx_buf, UART_RX_SIZE - 1);
        if (checksum != uart_rx_buf[UART_RX_SIZE - 1]) {
            HAL_UART_Receive_IT(&huart1, uart_rx_buf, UART_RX_SIZE);
            return;
        }

        // 파싱
        uint8_t *p = &uart_rx_buf[1];
        memcpy((void*)&cmd.vx,          p, 4); p += 4;
        memcpy((void*)&cmd.vy,          p, 4); p += 4;
        memcpy((void*)&cmd.wz,          p, 4); p += 4;
        memcpy((void*)&cmd.lift_target, p, 4); p += 4;
        cmd.lift_active = *p++;
        memcpy((void*)&cmd.yaw_target,  p, 4); p += 4;
        cmd.yaw_active  = *p;

        uart_rx_ready = 1;
        last_cmd_tick = HAL_GetTick();   // 워치독 갱신 (유효 프레임 수신)

        HAL_UART_Receive_IT(&huart1, uart_rx_buf, UART_RX_SIZE);
    }
}

// ==========================================
// Jetson으로 상태 송신
// ==========================================
void UART_SendStatus(int16_t e1, int16_t e2,
                     int16_t e3, int16_t e4)
{
    uint8_t *p = uart_tx_buf;
    *p++ = UART_HEADER_TX;
    memcpy(p, &e1, 2); p += 2;
    memcpy(p, &e2, 2); p += 2;
    memcpy(p, &e3, 2); p += 2;
    memcpy(p, &e4, 2); p += 2;

    float lift_h = Lift_GetHeight();
    float yaw_a  = Yaw_GetAngle();
    memcpy(p, &lift_h, 4); p += 4;   // [9..12]
    memcpy(p, &yaw_a,  4); p += 4;   // [13..16], p → 17

    // [17] reserved 를 먼저 0으로 확정한 뒤 체크섬을 계산해야
    //  sum(bytes[0..17]) 이 결정적이다. (원본은 미초기화 바이트가 섞여 버그)
    *p++ = 0;                                            // [17] reserved
    *p   = calc_checksum(uart_tx_buf, UART_TX_SIZE - 1); // [18] = sum(bytes[0..17])

    HAL_UART_Transmit(&huart1, uart_tx_buf, UART_TX_SIZE, 10);
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM8_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_TIM12_Init();

    // 엔코더 시작
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    // 휠모터 PWM 시작
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5,  TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5,  TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,  TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    // 리프트 초기화
    // 수정
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   // 리프트 ENA
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET); // 리프트 STEP/DIR 초기화

    // yaw 초기화
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);  // ENA
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

    // TIM6 카운터 시작 (delay_us용)
    HAL_TIM_Base_Start(&htim6);

    // 휠모터 정지
    Mecanum_SetVelocity(0.0f, 0.0f, 0.0f);

    // UART 수신 시작
    HAL_UART_Receive_IT(&huart1, uart_rx_buf, UART_RX_SIZE);

    while (1)
    {
        if (uart_rx_ready)
        {
            uart_rx_ready = 0;

            // A. 휠모터
            Mecanum_SetVelocity(cmd.vx, cmd.vy, cmd.wz);

            // B. 리프트
            Lift_SetTarget(cmd.lift_target, cmd.lift_active);

            // C. yaw
            Yaw_SetTarget(cmd.yaw_target, cmd.yaw_active);
        }

        // 안전 워치독: 명령이 CMD_TIMEOUT_MS 이상 끊기면 휠 강제 정지.
        //  (리프트/yaw는 스텝 위치 유지 = 떨어지거나 풀리지 않음)
        if ((HAL_GetTick() - last_cmd_tick) > CMD_TIMEOUT_MS) {
            Mecanum_SetVelocity(0.0f, 0.0f, 0.0f);
        }

        // D. 엔코더 읽기 + Jetson 송신
        int16_t enc1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        int16_t enc2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t enc3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        int16_t enc4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);

        __HAL_TIM_SET_COUNTER(&htim1, 0);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        __HAL_TIM_SET_COUNTER(&htim8, 0);

        UART_SendStatus(enc1, enc2, enc3, enc4);

        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

// ======================================================================
// USART1
// ======================================================================
static void MX_USART1_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

// ======================================================================
// ENCODER TIMERS
// ======================================================================
static void MX_TIM1_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM8_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 65535;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

// ======================================================================
// BASE / INTERRUPT TIMERS
// ======================================================================
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16-1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 16-1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 0xFFFF;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM7_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 16-1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 4999;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

// ======================================================================
// PWM TIMERS
// ======================================================================
static void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 83;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 999;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
    HAL_TIM_MspPostInit(&htim5);
}

static void MX_TIM9_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim9.Instance = TIM9;
    htim9.Init.Prescaler = 83;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = 999;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim9) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    HAL_TIM_MspPostInit(&htim9);
}

static void MX_TIM10_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 83;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 999;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    HAL_TIM_MspPostInit(&htim10);
}

static void MX_TIM11_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 83;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 999;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim11) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    HAL_TIM_MspPostInit(&htim11);
}

static void MX_TIM12_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim12.Instance = TIM12;
    htim12.Init.Prescaler = 83;
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim12.Init.Period = 999;
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim12) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    HAL_TIM_MspPostInit(&htim12);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 |
                             GPIO_PIN_2 | GPIO_PIN_4, GPIO_PIN_RESET);

    // PA6 리프트STEP, PA7 리프트DIR
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC0 yawSTEP, PC1 yawDIR, PC2 yawENA, PC4 리프트ENA
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 |
                          GPIO_PIN_2 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
