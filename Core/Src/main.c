/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 모터 슬레이브 — CAN 수신 + 메카넘 PID 구동
  *                   ★ v2: 폭주(Runaway) 방지 안전 계층 추가
  *                   ★ v4: 4채널 독립 제어 및 TIM2 PWM 리맵 적용
  *
  *  CAN 수신 프로토콜 (Pi → STM32, ID 0x123):
  *    Byte 0-1: int16_t FL  (-9999 ~ +9999)
  *    Byte 2-3: int16_t FR
  *    Byte 4-5: int16_t RL
  *    Byte 6-7: int16_t RR
  *
  *  CAN 송신 프로토콜 (STM32 → Pi, ID 0x124):
  *    Byte 0-1: int16_t Left  누산 엔코더 틱 (TIM3)
  *    Byte 2-3: int16_t Right 누산 엔코더 틱 (TIM4)
  *
  *  메인 루프 타이밍:
  *    10ms: Motor_PID_Update()           — 엔코더 읽기 + 워치독 + PID + PWM
  *    20ms: Motor_Send_Feedback_CAN()   — 엔코더 틱 → Pi (오도메트리)
  *  매 루프: IWDG_Refresh()              — 독립 워치독 급이기
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"   /* PID 모터 제어 + CAN TX 피드백 + 명령 워치독 */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── [A] IWDG 설정 ────────────────────────────────────────────────
 *   LSI ≈ 40 kHz (개체차 30~60 kHz), 프리스케일러 /32 → 약 1250 Hz
 *   RLR = 1250 → 공칭 1.00 초 (최악 0.67 s ~ 최선 1.67 s)
 *   메인 루프 1회전은 수십 µs 이므로 여유가 1000배 이상입니다.
 *   ※ IWDG 는 한 번 시작하면 소프트웨어로 끌 수 없습니다.
 *   ※ 디버거로 정지시켜도 IWDG 는 계속 돕니다 → DBGMCU 로 동결시킵니다.
 * ──────────────────────────────────────────────────────────────── */
#define IWDG_PRESCALER_CODE   3U      /* 0=/4 1=/8 2=/16 3=/32 4=/64 ... */
#define IWDG_RELOAD_VALUE     1250U   /* 12비트(≤4095) */

/* [B] 상태 출력 주기. per-command printf 는 절대 부활시키지 마십시오. */
#define STATUS_PRINT_PERIOD_MS  1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ── [D] ISR 과 공유되는 변수는 반드시 volatile ──────────────────
 *   volatile 이 없으면 컴파일러가 "메인 루프에서 이 값을 바꾸는 코드가 없다"고
 *   판단해 레지스터에 캐싱하고, ISR 의 갱신을 메인이 영영 못 볼 수 있습니다.
 *   -O0 에서는 우연히 동작하지만 -O2 로 올리는 순간 깨지는 전형적인 버그입니다.
 * ──────────────────────────────────────────────────────────────── */
static volatile uint8_t  g_cmd_pending = 0U;
static volatile int16_t  g_rxFL = 0;
static volatile int16_t  g_rxFR = 0;
static volatile int16_t  g_rxRL = 0;
static volatile int16_t  g_rxRR = 0;

/* 진단 카운터 */
static volatile uint32_t g_can_rx_count = 0U;

/* 메인 루프 타이밍 변수 */
static uint32_t last_pid_tick    = 0;  /* PID 마지막 실행 시각 (ms) */
static uint32_t last_fb_tick     = 0;  /* CAN TX 마지막 실행 시각 (ms) */
static uint32_t last_status_tick = 0;  /* 상태 출력 마지막 시각 (ms) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_filter(void);
static void IWDG_Start(void);
static void IWDG_Refresh(void);
static void Report_Reset_Cause(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief [A] IWDG 기동 — HAL_IWDG_MODULE_ENABLED 없이 레지스터 직접 제어
 *
 * stm32f1xx_hal_conf.h 의 HAL_IWDG_MODULE_ENABLED 가 주석 처리되어 있어
 * HAL 드라이버를 쓰려면 CubeMX 설정을 건드려야 합니다. IWDG 는 레지스터가
 * 4개뿐이라 직접 제어가 더 간단하고 이식성도 좋습니다.
 *
 * 시퀀스 (RM0008 참조):
 *   KR = 0x5555  → PR/RLR 쓰기 잠금 해제
 *   PR, RLR 설정 (각각 SR 의 PVU/RVU 가 내려간 뒤에)
 *   KR = 0xAAAA  → 카운터 리로드
 *   KR = 0xCCCC  → 기동 (이후 정지 불가)
 */
static void IWDG_Start(void)
{
  /* 디버거로 코어를 멈춰도 IWDG 가 계속 돌면 브레이크포인트마다 리셋됩니다.
   * DBGMCU 로 동결시킵니다. 릴리스 빌드에서도 무해합니다. */
  DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

  IWDG->KR = 0x5555U;                       /* 쓰기 잠금 해제 */
  while ((IWDG->SR & IWDG_SR_PVU) != 0U) { }
  IWDG->PR = IWDG_PRESCALER_CODE;
  while ((IWDG->SR & IWDG_SR_RVU) != 0U) { }
  IWDG->RLR = IWDG_RELOAD_VALUE;
  IWDG->KR = 0xAAAAU;                       /* 리로드 */
  IWDG->KR = 0xCCCCU;                       /* 기동 */
}

/** @brief [A] IWDG 급이기. 메인 루프에서 매 회전 호출. */
static void IWDG_Refresh(void)
{
  IWDG->KR = 0xAAAAU;
}

/**
 * @brief 부팅 원인 보고 — IWDG 리셋이 실제로 걸렸는지 확인하는 진단
 *
 * 이 로그가 반복해서 찍히면 펌웨어가 어딘가에서 1초 이상 멈추고 있다는 뜻이며,
 * 그 자체가 폭주 위험 신호입니다. 원인을 반드시 추적하십시오.
 */
static void Report_Reset_Cause(void)
{
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    printf("[BOOT] !! IWDG RESET DETECTED !! 펌웨어가 1초 이상 멈췄습니다.\r\n");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    printf("[BOOT] software reset\r\n");
  }
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
  {
    printf("[BOOT] power-on reset\r\n");
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* printf 버퍼링 비활성화 — 즉시 출력 보장 */
  setvbuf(stdout, NULL, _IONBF, 0);

  /* v4 부팅 배너 및 시스템 정보 출력 */
  printf("\r\n=====================================================\r\n");
  printf("[BOOT] STM32 Motor v4  4ch-independent / TIM2 PWM x4\r\n");
  printf("  PWM   TIM2 CH1=PA0(RL) CH2=PA1(RR) CH3=PB10(FL) CH4=PB11(FR)\r\n");
  printf("  Enc   TIM3=FL  TIM4=FR  TIM1=%s\r\n",
         USE_RR_ENCODER ? "RR(active)" : "reserved");
  printf("  SAFE  PWM_MAX=%d  SLEW=%d/cycle  STALL=%lums\r\n",
         MOTOR_PWM_SAFE_MAX, MOTOR_PWM_SLEW, (unsigned long)STALL_HOLD_MS);
  printf("  CPR=%lu  MAX_RPM=%.0f  KP=%.1f KI=%.1f KD=%.2f\r\n",
         (unsigned long)ENCODER_CPR, MOTOR_MAX_RPM, MOTOR_KP, MOTOR_KI, MOTOR_KD);
  printf("  CAN RX: 0x123 | CAN TX: 0x124 | PID: 10ms\r\n");
  printf("  CMD watchdog: %u ms | IWDG: ~1000 ms\r\n", (unsigned)CMD_TIMEOUT_MS);
  printf("=====================================================\r\n");
  
  Report_Reset_Cause();

  /* CAN 수신 준비 */
  CAN_filter();

  if (HAL_CAN_Start(&hcan) != HAL_OK)
    Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();

  /* ── PWM 4채널 시작 — v4: 전부 TIM2 (partial remap 2) ──────────────
   *   TIM2_CH1 (PA0)  = RL
   *   TIM2_CH2 (PA1)  = RR
   *   TIM2_CH3 (PB10) = FL   ★ PA8 에서 이전
   *   TIM2_CH4 (PB11) = FR   ★ PA9 에서 이전
   *   TIM1 은 더 이상 PWM 이 아닙니다. Encoder Mode 로 전환됐습니다. */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* RL */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* RR */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   /* FL */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   /* FR */

  /* ── 엔코더 ────────────────────────────────────────────────────── */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   /* FL (PB4,PB5) */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   /* FR (PB6,PB7) */
#if USE_RR_ENCODER
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   /* RR (PA8,PA9) — Phase 3 */
#endif

  /* ★ Motor_Init() 은 반드시 PWM Start 이후에 호출하십시오.
   *   내부 s_hw_ready 게이트가 이 시점부터 PWM 레지스터 접근을 허용합니다. */
  Motor_Init();

  /* 타이밍 기준 초기화 */
  last_pid_tick    = HAL_GetTick();
  last_fb_tick     = HAL_GetTick();
  last_status_tick = HAL_GetTick();

  /* ★ [A] IWDG 기동 — 반드시 모든 초기화가 끝난 뒤에.
   *   초기화 도중 걸리면 부팅 루프에 빠집니다. */
  IWDG_Start();

  printf("[SYSTEM] Ready. Waiting for CAN 0x123... (no command = motors locked off)\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ── [A] IWDG 급이기 ─────────────────────────────────
     *   루프 최상단에서 1회. 아래 어떤 처리가 1초 이상 걸리면
     *   MCU 가 리셋되고 PWM 이 0 으로 떨어집니다. */
    IWDG_Refresh();

    /* ── CAN 수신: 목표 속도 갱신 ───────────────────────
     *   [D] 4개 값을 원자적으로 스냅샷.
     *   PRIMASK 를 저장/복원하는 형태를 쓰는 이유: 이 구간에 들어올 때
     *   인터럽트가 이미 꺼져 있을 수도 있으므로, 무조건 __enable_irq() 하면
     *   임계 구역을 잘못 열게 됩니다. */
    if (g_cmd_pending != 0U)
    {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      int16_t fl = g_rxFL;
      int16_t fr = g_rxFR;
      int16_t rl = g_rxRL;
      int16_t rr = g_rxRR;
      g_cmd_pending = 0U;
      __set_PRIMASK(primask);

      /* ★ [B] 여기에 있던 per-command printf 를 제거했습니다.
       *   45자 × 3.9ms 블로킹이 10ms PID 주기를 무너뜨리고,
       *   UART 이상 시 HAL_MAX_DELAY 로 영원히 갇혀 폭주로 직결됩니다. */
      Motor_Drive(fl, fr, rl, rr);
    }

    uint32_t now = HAL_GetTick();

    /* ── 10ms: 엔코더 + 명령 워치독 + PID ──────────────── */
    if (now - last_pid_tick >= PID_PERIOD_MS)
    {
      Motor_PID_Update();
      last_pid_tick = now;
    }

    /* ── 20ms: 엔코더 피드백 CAN TX ─────────────────────── */
    if (now - last_fb_tick >= CAN_FEEDBACK_PERIOD_MS)
    {
      Motor_Send_Feedback_CAN();
      last_fb_tick = now;
    }

    /* ── 1000ms: 상태 요약 1줄 ──────────────────────────
     *   per-command 출력 대신 저빈도 요약. 1초에 1회 × 약 4ms = 0.4% 부하.
     *   워치독 발동 여부를 여기서 눈으로 확인할 수 있습니다. */
    if (now - last_status_tick >= STATUS_PRINT_PERIOD_MS)
    {
      last_status_tick = now;
      if (Motor_Watchdog_IsTripped())
      {
        printf("[WDG] TRIPPED - motors forced off (no CAN cmd). rx=%lu\r\n",
               (unsigned long)g_can_rx_count);
      }
      else
      {
        printf("[OK ] rx=%lu  last_cmd=%lums\r\n",
               (unsigned long)g_can_rx_count,
               (unsigned long)Motor_Watchdog_ElapsedMs());
      }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* CAN 모든 메시지 통과 필터 */
void CAN_filter(void)
{
  CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

  if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* 라즈베리파이 CAN 데이터(0x123) 수신 콜백 함수
 *
 * [D] ISR 안에서는 값만 담고 즉시 빠져나옵니다.
 *     ISR 에서 Motor_Drive() 를 직접 부르지 않는 이유: 부동소수점 연산과
 *     GPIO/타이머 접근이 섞여 ISR 체류 시간이 길어지고, 메인 루프의
 *     임계 구역과 경합할 수 있기 때문입니다. (기존 구조 유지)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
  {
    if (rxHeader.StdId == 0x123) /* Pi -> STM32 */
    {
      g_rxFL = (int16_t)((rxData[0] << 8) | rxData[1]);
      g_rxFR = (int16_t)((rxData[2] << 8) | rxData[3]);
      g_rxRL = (int16_t)((rxData[4] << 8) | rxData[5]);
      g_rxRR = (int16_t)((rxData[6] << 8) | rxData[7]);
      g_can_rx_count++;
      g_cmd_pending = 1U;   /* 반드시 데이터 저장 '후'에 세울 것 */
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /* ★ [C] 무한 루프에 들어가기 '전에' 반드시 모터를 세웁니다.
   *
   *   기존 코드는 __disable_irq() 후 while(1){} 이었습니다.
   *   그런데 PWM 레지스터는 그대로 유지되므로, CAN 에러 한 번에
   *   '복구 불가능한 전속력 폭주'가 됩니다. 실제로 가장 위험한 경로입니다.
   *
   *   순서가 중요합니다:
   *     1) 인터럽트를 끄기 전에 모터 정지 (GPIO/타이머 접근이 필요)
   *     2) 그 다음 인터럽트 차단
   *     3) IWDG 를 일부러 급이지 않음 → 약 1초 뒤 MCU 리셋 → 정상 재기동 시도
   *        리셋 후에도 워치독이 '두절' 상태로 시작하므로 모터는 계속 정지 상태.
   */
  Motor_Stop();

  __disable_irq();
  while (1)
  {
    /* IWDG 를 급이지 않으므로 약 1초 뒤 리셋됩니다.
     * 만약 리셋을 원치 않고 그 자리에 정지시키고 싶다면 아래 주석을 해제하십시오.
     *   IWDG->KR = 0xAAAAU;
     * 단, 그 경우 사람이 전원을 내릴 때까지 로봇은 응답하지 않습니다. */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
