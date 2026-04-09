/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 모터 슬레이브 — CAN 수신 + 모터 테스트 시퀀스
  *
  *  동작 흐름:
  *    1. USART2(115200) 초기화 → printf 리다이렉트
  *    2. CAN1(500kbps) 초기화 → Pass-All 필터 → Start → RX 인터럽트 활성화
  *    3. TIM1/TIM2 PWM 시작 (4채널)
  *    4. CAN 메시지 수신 시 모터 테스트 시퀀스 실행:
  *         전진 5000 (2초) → 후진 -5000 (2초) → 정지
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "main.h"
#include "motor.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
extern uint8_t dataReceived;   /* can.c에 정의 */
extern int16_t rxSpeed;        /* can.c에 정의 */
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
  /* USER CODE BEGIN 2 */

  printf("\r\n========================================\r\n");
  printf(" STM32 Motor Slave — Booting\r\n");
  printf(" CAN 500kbps | USART2 115200-8N1\r\n");
  printf("========================================\r\n");

  /* ── CAN 수신 준비 ────────────────────────────────────────── */
  CAN_filter();   /* Pass-All 필터 설정 */

  if (HAL_CAN_Start(&hcan) != HAL_OK)
    Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();

  /* ── PWM 채널 시작 (모터 4채널) ───────────────────────────── */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* 전면 우측 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* 전면 좌측 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* 후면 우측 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* 후면 좌측 */

  printf("[SYSTEM] Ready. Waiting for CAN message...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (dataReceived)
    {
      dataReceived = 0;

      printf("[MOTOR] CAN received (speed=%d). Starting test sequence...\r\n",
             (int)rxSpeed);

      /* ── 전진 2초 ── */
      Motor_Drive(5000);
      printf("[MOTOR] >> Forward  (PWM=5000)\r\n");
      HAL_Delay(1000);

      /* ── 후진 2초 ── */
      Motor_Drive(-5000);
      printf("[MOTOR] >> Backward (PWM=-5000)\r\n");
      HAL_Delay(1000);

      /* ── 정지 ── */
      Motor_Drive(0);
      printf("[MOTOR] >> Stop\r\n");
      printf("[MOTOR] Test sequence complete.\r\n\r\n");
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
  while (1)
  {
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
