/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART2 (디버그 UART) — v4
  ******************************************************************************
  *
  *   USART2   PA2 = TX,  PA3 = RX,  115200 8N1
  *
  *  ★ 왜 USART2 인가 — 다른 선택지가 없습니다
  *     USART1 : PA9  = TIM1_CH2 (RR 엔코더)          -> 충돌
  *     USART3 : PB10 = TIM2_CH3 (FL PWM)             -> 충돌
  *     USART2 : PA2/PA3 만 유일하게 비어 있습니다.
  *
  *  ★ printf 재지향(_write)이 이 파일 아래쪽에 있습니다.
  *    HAL_MAX_DELAY 를 쓰지 않는 것이 핵심입니다 — 자세한 이유는 그 주석 참조.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */
    /* USER CODE END USART2_MspInit 0 */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* USER CODE BEGIN USART2_MspInit 1 */
    /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief printf 재지향
  *
  * ★★ 타임아웃을 반드시 유한하게 두십시오.
  *   HAL_MAX_DELAY 를 쓰면, UART 선이 빠지거나 수신 측이 XOFF 를 걸었을 때
  *   **영원히 블로킹**됩니다. 그러면 메인 루프가 멈추고 -> PID 도 워치독도
  *   멈추고 -> PWM 레지스터는 마지막 값을 유지 -> 로봇이 폭주합니다.
  *   "디버그 출력 하나가 로봇을 폭주시킨다" 는 실제로 일어나는 일입니다.
  *
  *   5 ms 면 115200 baud 에서 약 57자를 보낼 시간입니다. 그 안에 못 보내면
  *   그냥 버립니다. 디버그 문자열 몇 개보다 로봇이 서는 게 중요합니다.
  */
int _write(int file, char *ptr, int len)
{
  (void)file;
  if (len <= 0) return 0;
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 5);
  return len;   /* 실패해도 len 을 반환 — printf 가 재시도하지 않게 */
}

/* USER CODE END 1 */