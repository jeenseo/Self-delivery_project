/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 모터 슬레이브 — CAN 수신 + 스키드-스티어 PID 구동
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
  *    10ms: Motor_PID_Update()          — 엔코더 읽기 + PID + PWM
  *    20ms: Motor_Send_Feedback_CAN()   — 엔코더 틱 → Pi (오도메트리)
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
#include "motor.h"   /* PID 모터 제어 + CAN TX 피드백 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t dataReceived;    /* can.c에 정의: CAN 수신 플래그 */
extern int16_t rxFL;            /* can.c에 정의: Front Left  목표 속도 */
extern int16_t rxFR;            /* can.c에 정의: Front Right 목표 속도 */
extern int16_t rxRL;            /* can.c에 정의: Rear Left   목표 속도 */
extern int16_t rxRR;            /* can.c에 정의: Rear Right  목표 속도 */

/* 메인 루프 타이밍 변수 */
static uint32_t last_pid_tick = 0;   /* PID 마지막 실행 시각 (ms) */
static uint32_t last_fb_tick  = 0;   /* CAN TX 마지막 실행 시각 (ms) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* printf 버퍼링 비활성화 — 즉시 출력 보장 */
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\r\n[DEBUG] STM32 Motor Slave Online (PID + Encoder)\r\n");
  printf("=====================================================\r\n");
  printf(" CAN RX: 0x123 | CAN TX: 0x124 | PID: 10ms\r\n");
  printf("=====================================================\r\n");

  /* CAN 수신 준비 */
  CAN_filter();

  if (HAL_CAN_Start(&hcan) != HAL_OK)
    Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();

  /* PWM 4채널 시작
   *   TIM1_CH1 (PA8) = FL (좌측 전륜)
   *   TIM1_CH2 (PA9) = FR (우측 전륜)
   *   TIM2_CH1 (PA0) = RL (좌측 후륜)
   *   TIM2_CH2 (PA1) = RR (우측 후륜) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* FL */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* FR */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* RL */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* RR */

  /* 엔코더 2채널 시작
   *   TIM3 (PB4, PB5) = 좌측 엔코더
   *   TIM4 (PB6, PB7) = 우측 엔코더 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /* 모터 PID 모듈 초기화 (Motor_Drive 전 반드시 호출) */
  Motor_Init();

  /* 타이밍 기준 초기화 */
  last_pid_tick = HAL_GetTick();
  last_fb_tick  = HAL_GetTick();

  printf("[SYSTEM] Motor PID Ready. Waiting for CAN 0x123...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ── CAN 수신: 목표 속도 갱신 ───────────────────────── */
    if (dataReceived == 1)
    {
      printf("[CAN RX] FL=%+6d FR=%+6d RL=%+6d RR=%+6d\r\n",
             (int)rxFL, (int)rxFR, (int)rxRL, (int)rxRR);
      Motor_Drive(rxFL, rxFR, rxRL, rxRR);
      dataReceived = 0;
    }

    /* ── 10ms: PID 실행 ──────────────────────────────────
     *   엔코더 읽기 → RPM 계산 → PID 연산 → PWM 출력
     *   CAN TX 누산기에도 틱 누적
     * ─────────────────────────────────────────────────── */
    uint32_t now = HAL_GetTick();
    if (now - last_pid_tick >= PID_PERIOD_MS)
    {
      Motor_PID_Update();
      last_pid_tick = now;
    }

    /* ── 20ms: 엔코더 피드백 CAN TX ─────────────────────
     *   Payload: [Left_ticks(2B)][Right_ticks(2B)] Big-Endian
     *   Pi에서 /odom_wheel 토픽 계산에 사용
     * ─────────────────────────────────────────────────── */
    if (now - last_fb_tick >= CAN_FEEDBACK_PERIOD_MS)
    {
      Motor_Send_Feedback_CAN();
      last_fb_tick = now;
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
