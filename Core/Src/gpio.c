/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO 초기화 — v4 최소 구성
  ******************************************************************************
  *
  *  이 프로젝트가 GPIO 로 직접 쓰는 핀은 **모터 DIR 4개뿐**입니다.
  *  나머지(PWM/엔코더/CAN/UART)는 각 페리페럴의 MspInit 이 알아서 잡습니다.
  *
  *    PC0 = RL DIR      PC1 = RR DIR
  *    PC2 = FL DIR      PC3 = FR DIR
  *
  *  ★ 리셋 직후 4핀 모두 LOW 로 시작합니다.
  *    DIR 핀이 LOW 라도 PWM 듀티가 0 이면 모터는 돌지 않습니다.
  *    (motor.c 의 _set_wheel_raw 는 speed==0 일 때 DIR=RESET, PWM=0 을 씁니다)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gpio.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO 포트 클럭 활성화
   *   GPIOA : TIM2_CH1/CH2(PA0,PA1), USART2(PA2,PA3), TIM1 Enc(PA8,PA9),
   *           CAN(PA11,PA12), SWD(PA13,PA14)
   *   GPIOB : TIM2_CH3/CH4(PB10,PB11), TIM3 Enc(PB4,PB5), TIM4 Enc(PB6,PB7)
   *   GPIOC : 모터 DIR x4 (PC0~PC3)
   *   GPIOD : HSE 오실레이터(PD0,PD1) — 클럭만 켜두면 됩니다
   *   AFIO  : TIM2/TIM3 remap 매크로에 필요 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* ── 모터 DIR 4핀: 출력 LOW 로 초기화 ─────────────────────────────
   *  ★ HAL_GPIO_Init 보다 WritePin 을 **먼저** 호출합니다.
   *    출력 모드로 전환되는 순간 ODR 값이 그대로 핀에 나가므로,
   *    ODR 을 미리 0 으로 확정해 두어야 전환 순간의 글리치가 없습니다. */
  HAL_GPIO_WritePin(GPIOC,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */
/* USER CODE END 2 */