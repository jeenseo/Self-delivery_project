/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 모터 슬레이브 — CAN 수신 + 스키드-스티어 구동
  *
  *  CAN 프로토콜 (Pi → STM32):
  *    ID     : 0x123
  *    Byte 0-1: int16_t Left  Speed (-9999 ~ +9999)
  *    Byte 2-3: int16_t Right Speed (-9999 ~ +9999)
  *
  *  동작 흐름:
  *    1. USART2 (115200) 초기화 → printf 리다이렉트
  *    2. CAN1 (500kbps) 초기화 → Pass-All 필터 → Start → RX 인터럽트
  *    3. TIM1/TIM2 PWM 4채널 시작
  *    4. while 루프: dataReceived 플래그 → printf → Motor_Drive(L, R)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

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
extern uint8_t dataReceived;    /* can.c에 정의 */
extern int16_t rxLeftSpeed;     /* can.c에 정의 */
extern int16_t rxRightSpeed;    /* can.c에 정의 */
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

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();     /* 먼저 초기화 → 이후 printf 사용 가능 */
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */

  /* printf 버퍼링 비활성화 — 즉시 출력 보장 */
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\r\n[DEBUG] STM32 Motor Slave Online\r\n");
  printf("========================================\r\n");
  printf(" CAN 500kbps | USART2 115200 | Skid-Steer\r\n");
  printf("========================================\r\n");

  /* CAN 수신 준비 */
  CAN_filter();

  if (HAL_CAN_Start(&hcan) != HAL_OK)
    Error_Handler();

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    Error_Handler();

  /* PWM 4채널 시작 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* R 앞 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* L 앞 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* R 뒤 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* L 뒤 */

  printf("[SYSTEM] Ready. Waiting for CAN...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (dataReceived == 1)
    {
      /* 콘솔 출력 후 모터 구동 */
      printf("[CAN RX] ID:0x123 | L=%+6d  R=%+6d\r\n",
             (int)rxLeftSpeed, (int)rxRightSpeed);
      Motor_Drive(rxLeftSpeed, rxRightSpeed);
      dataReceived = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        HSE 8MHz × PLL9 = 72MHz,  APB1=36MHz,  APB2=72MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* 36 MHz → CAN/USART2 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* 72 MHz              */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    Error_Handler();
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
