/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   STM32 모터 슬레이브 — CAN 수신 + 메카넘 4륜 독립 제어 (v4)
  ******************************************************************************
  *
  *  CAN 수신 (Pi -> STM32, ID 0x123, DLC 8):
  *    Byte 0-1: int16_t FL   Byte 2-3: int16_t FR
  *    Byte 4-5: int16_t RL   Byte 6-7: int16_t RR      (-9999 ~ +9999)
  *
  *  CAN 송신 (STM32 -> Pi, ID 0x124):
  *    USE_RR_ENCODER 0 -> DLC 4 : [FL ticks][FR ticks]
  *    USE_RR_ENCODER 1 -> DLC 6 : [FL][FR][RR]
  *
  *  메인 루프 타이밍:
  *    10ms : Motor_PID_Update()        엔코더 -> PID -> PWM
  *    20ms : Motor_Send_Feedback_CAN() 엔코더 틱 -> Pi
  *   1000ms: 상태 요약 1줄
  *
  *  ══════════════════════════════════════════════════════════════════════
  *  ★ v4 에서 고친 것 (구버전 대비)
  *  ══════════════════════════════════════════════════════════════════════
  *
  *  [1] PWM 시작을 TIM2 4채널로
  *      구: HAL_TIM_PWM_Start(&htim1, CH1/CH2) + (&htim2, CH1/CH2)
  *      신: HAL_TIM_PWM_Start(&htim2, CH1~CH4)
  *      TIM1 은 Encoder Mode 가 되었으므로 PWM_Start 를 호출하면
  *      엔코더 입력이 망가집니다. 반드시 지워야 합니다.
  *
  *  [2] ★ CAN 수신마다 찍던 printf 제거 — 이게 제일 중요합니다
  *      구: while(1) 안에서 CAN 이 올 때마다
  *          printf("[CAN RX] FL=%+6d ...") 를 호출
  *      문제: 약 45자 x 115200 baud = **3.9 ms 블로킹**.
  *            Pi 가 50 Hz(20 ms)로 보내므로 루프 시간의 20% 를 UART 가 먹습니다.
  *            10 ms PID 주기가 그만큼 밀리고, 최악에는 건너뜁니다.
  *            UART 가 물리적으로 막히면 HAL_MAX_DELAY 로 영원히 갇혀
  *            PID 도 워치독도 멈춥니다 = 마지막 PWM 유지 = 폭주.
  *      신: 완전 제거. 진단은 1 Hz 요약으로만.
  *
  *  [3] Error_Handler 에서 모터를 먼저 세움
  *      구: __disable_irq(); while(1){}   <- PWM 레지스터가 그대로 유지됨
  *          즉 에러가 나면 **마지막 속도로 계속 달립니다.**
  *      신: Motor_Stop() 을 먼저 호출한 뒤 정지.
  *
  *  [4] 부팅 배너에 버전/안전상한/이득을 출력
  *      "지금 보드에 어느 빌드가 올라가 있는가" 를 1초에 확인하기 위함.
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
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATUS_PRINT_PERIOD_MS   1000U

/**
 * ★ 디버그 UART 컴파일 스위치
 *
 *  1 : USART2(PA2/PA3) 사용. printf 동작.
 *  0 : printf 를 전부 무효화. USART2 를 ioc 에서 끈 경우 이걸로 두면
 *      usart.c/h 가 없어도 빌드가 통과합니다.
 *
 *  [왜 스위치를 두는가]
 *    이 프로젝트에서 ioc 와 코드가 어긋난 적이 여러 번 있었습니다.
 *    USART2 가 Disable 인 상태에서 재생성하면 MX_USART2_UART_Init 이
 *    사라져 링크 에러가 납니다. 그때 이 값을 0 으로 두면 즉시 빌드됩니다.
 */
#define DEBUG_UART               1

#if DEBUG_UART
  #include <stdio.h>
  #define DBG(...)   printf(__VA_ARGS__)
#else
  #define DBG(...)   ((void)0)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ★ volatile: 아래 4개는 CAN 수신 ISR 에서 쓰이고 메인 루프에서 읽힙니다.
 *   volatile 이 없으면 컴파일러가 "메인 루프에서 바뀌지 않는다" 고 판단해
 *   레지스터에 캐시해 버려, 새 명령을 영원히 못 보게 될 수 있습니다. */
volatile uint8_t dataReceived = 0;
volatile int16_t rxFL = 0;
volatile int16_t rxFR = 0;
volatile int16_t rxRL = 0;
volatile int16_t rxRR = 0;

/* 메인 루프 타이밍 */
static uint32_t last_pid_tick    = 0;
static uint32_t last_fb_tick     = 0;
static uint32_t last_status_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_filter(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();          /* ★ Encoder (RR). PWM 아님 */
  MX_TIM2_Init();          /* ★ PWM x4 */
  MX_CAN_Init();
#if DEBUG_UART
  MX_USART2_UART_Init();   /* PA2/PA3 디버그 UART */
#endif
  MX_TIM3_Init();          /* Encoder FL */
  MX_TIM4_Init();          /* Encoder FR */

  /* USER CODE BEGIN 2 */

  /* printf 버퍼링 비활성화 — 즉시 출력 보장 */
#if DEBUG_UART
  setvbuf(stdout, NULL, _IONBF, 0);
#endif

  /* ── 부팅 배너 ─────────────────────────────────────────────────────
   *  여기서만 길게 찍습니다. 주행 중에는 절대 이렇게 찍지 않습니다. */
  DBG("\r\n");
  DBG("========================================================\r\n");
  DBG("[BOOT] STM32 Motor v4  --  4ch independent mecanum\r\n");
  DBG("  PWM  TIM2  CH1=PA0(RL) CH2=PA1(RR) CH3=PB10(FL) CH4=PB11(FR)\r\n");
  DBG("  Enc  TIM3=FL  TIM4=FR  TIM1=%s\r\n",
         USE_RR_ENCODER ? "RR (active)" : "RR (reserved)");
  DBG("  CAN  RX 0x123 (DLC8)  TX 0x124 (DLC%d)\r\n",
         USE_RR_ENCODER ? 6 : 4);
  DBG("  SAFE PWM_MAX=%d  SLEW=%d/cycle  STALL=%lums  WD=%lums\r\n",
         (int)MOTOR_PWM_SAFE_MAX, (int)MOTOR_PWM_SLEW,
         (unsigned long)STALL_HOLD_MS, (unsigned long)CMD_TIMEOUT_MS);
  DBG("  CTRL CPR=%lu  MAX_RPM=%.0f  KP=%.2f KI=%.2f KD=%.2f\r\n",
         (unsigned long)ENCODER_CPR, (double)MOTOR_MAX_RPM,
         (double)MOTOR_KP, (double)MOTOR_KI, (double)MOTOR_KD);
  DBG("========================================================\r\n");

  /* ── CAN 준비 ─────────────────────────────────────────────────────── */
  CAN_filter();

  if (HAL_CAN_Start(&hcan) != HAL_OK)
    Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();

  /* ── PWM 4채널 시작 — 전부 TIM2 ────────────────────────────────────
   *   TIM2_CH1 (PA0)  = RL
   *   TIM2_CH2 (PA1)  = RR
   *   TIM2_CH3 (PB10) = FL     ★ 구버전은 TIM1_CH1 (PA8)
   *   TIM2_CH4 (PB11) = FR     ★ 구버전은 TIM1_CH2 (PA9)          */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* RL */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* RR */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   /* FL */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   /* FR */

  /* ── 엔코더 시작 ──────────────────────────────────────────────────── */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);   /* FL  PB4,PB5 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   /* FR  PB6,PB7 */
#if USE_RR_ENCODER
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   /* RR  PA8,PA9 */
#endif

  /* ★ Motor_Init() 은 반드시 PWM_Start 이후.
   *   내부 s_hw_ready 게이트가 이 시점부터 PWM 레지스터 접근을 허용합니다. */
  Motor_Init();

  last_pid_tick    = HAL_GetTick();
  last_fb_tick     = HAL_GetTick();
  last_status_tick = HAL_GetTick();

  DBG("[SYSTEM] Ready. Waiting for CAN 0x123 ...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    /* ── CAN 수신: 목표 갱신 ─────────────────────────────────────────
     *  ★ 여기에 printf 를 넣지 마십시오.
     *    45자 x 115200 baud = 3.9 ms 블로킹입니다. 50 Hz 로 들어오면
     *    루프의 20% 를 UART 가 먹어 10 ms PID 주기가 밀립니다.
     *    UART 가 막히면 HAL_MAX_DELAY 로 영원히 갇혀 PID/워치독이 멈추고
     *    마지막 PWM 이 그대로 유지됩니다 = 폭주.
     *    ISR 이 쓴 값을 지역 변수로 한 번에 복사해 원자성을 확보합니다. */
    if (dataReceived)
    {
      int16_t fl = rxFL, fr = rxFR, rl = rxRL, rr = rxRR;
      dataReceived = 0;
      Motor_Drive(fl, fr, rl, rr);
    }

    uint32_t now = HAL_GetTick();

    /* ── 10ms: 엔코더 + 워치독 + 스톨 + PID + PWM ────────────────────── */
    if ((now - last_pid_tick) >= PID_PERIOD_MS)
    {
      Motor_PID_Update();
      last_pid_tick = now;
    }

    /* ── 20ms: 엔코더 틱 -> Pi ───────────────────────────────────────── */
    if ((now - last_fb_tick) >= CAN_FEEDBACK_PERIOD_MS)
    {
      Motor_Send_Feedback_CAN();
      last_fb_tick = now;
    }

    /* ── 1000ms: 상태 요약 1줄 ────────────────────────────────────────
     *  1초에 1회 x 약 4 ms = 0.4% 부하. 이 정도는 안전합니다. */
    if ((now - last_status_tick) >= STATUS_PRINT_PERIOD_MS)
    {
      last_status_tick = now;
      if (Motor_Watchdog_IsTripped())
      {
        DBG("[WD] command timeout - motors held off\r\n");
      }
      if (Motor_Stall_IsTripped())
      {
        DBG("[STALL] wheel blocked - motors cut. "
               "release command (all zero) to reset\r\n");
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1 36MHz -> TIM2 클럭 72MHz */
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
  canFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;
  canFilterConfig.FilterBank           = 0;
  canFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh         = 0x0000;
  canFilterConfig.FilterIdLow          = 0x0000;
  canFilterConfig.FilterMaskIdHigh     = 0x0000;
  canFilterConfig.FilterMaskIdLow      = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

  if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief 라즈베리파이 CAN 명령(0x123) 수신 콜백
  *
  * ★ ISR 입니다. printf 나 긴 연산을 절대 넣지 마십시오.
  *   값만 복사하고 즉시 빠져나옵니다.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_h)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t             rxData[8];

  if (HAL_CAN_GetRxMessage(hcan_h, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    return;

  if (rxHeader.IDE != CAN_ID_STD)      return;
  if (rxHeader.RTR != CAN_RTR_DATA)    return;
  if (rxHeader.StdId != CAN_COMMAND_ID) return;
  /* ★ DLC 검사 — 8바이트가 아니면 우리 프로토콜이 아닙니다.
   *   이게 없으면 짧은 프레임에서 rxData 의 초기화 안 된 뒷부분을 읽습니다. */
  if (rxHeader.DLC != 8U)              return;

  rxFL = (int16_t)((rxData[0] << 8) | rxData[1]);
  rxFR = (int16_t)((rxData[2] << 8) | rxData[3]);
  rxRL = (int16_t)((rxData[4] << 8) | rxData[5]);
  rxRR = (int16_t)((rxData[6] << 8) | rxData[7]);
  dataReceived = 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* ★ 인터럽트를 끄기 **전에** 모터를 세웁니다.
   *   구버전은 곧바로 __disable_irq() + while(1) 이라, 에러가 난 순간의
   *   PWM 레지스터 값이 그대로 유지됐습니다 = 마지막 속도로 계속 주행.
   *   PWM 타이머는 CPU 가 멈춰도 하드웨어가 계속 돌립니다. */
  Motor_Stop();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */