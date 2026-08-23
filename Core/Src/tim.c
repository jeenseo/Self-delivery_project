/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM 설정 — v4 핀맵 (TIM2 PWM x4 / TIM1·TIM3·TIM4 Encoder)
  ******************************************************************************
  *
  *  ★★ 이 파일은 CubeMX 가 생성한 내용을 v4 핀맵에 맞게 손으로 정리한 것입니다.
  *     **CubeMX 코드 재생성을 다시 돌리면 이 파일이 덮어써집니다.**
  *     .ioc 를 또 만지실 일이 있으면, 재생성 후 이 파일과 diff 하여
  *     아래 4가지가 살아 있는지 반드시 확인하십시오:
  *       1) MX_TIM1_Init 이 HAL_TIM_Encoder_Init 을 쓰는가 (PWM 아님)
  *       2) MX_TIM2_Init 이 CH1~CH4 네 개를 ConfigChannel 하는가
  *       3) HAL_TIM_MspPostInit 의 TIM2 가 PA0,PA1,PB10,PB11 을 잡는가
  *       4) __HAL_AFIO_REMAP_TIM2_PARTIAL_2() 가 호출되는가
  *
  *  [v4 타이머 배치]
  *    TIM1 : Encoder  RR   PA8(CH1)  PA9(CH2)     ← 구버전에서는 PWM 이었음
  *    TIM2 : PWM x4        PA0(CH1)=RL  PA1(CH2)=RR  PB10(CH3)=FL  PB11(CH4)=FR
  *    TIM3 : Encoder  FL   PB4(CH1)  PB5(CH2)     partial remap
  *    TIM4 : Encoder  FR   PB6(CH1)  PB7(CH2)
  *
  *  [TIM2 partial remap 2]
  *    PA2/PA3 는 USART2(디버그 UART) 이므로 쓰지 않습니다.
  *    REMAP[1:0] = 10  ->  CH1=PA0  CH2=PA1  CH3=PB10  CH4=PB11
  *
  *  [ARR 값이 다른 이유]
  *    TIM2 (PWM)     Period = 10000  -> MOTOR_PWM_MAX 9999 와 짝
  *    TIM1/3/4 (Enc) Period = 65535  -> 16bit 전체를 카운터로 사용
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* ══════════════════════════════════════════════════════════════════════════
 * TIM1 — ★ Encoder Mode (RR).  구버전의 PWM 설정을 전부 걷어냈습니다.
 * ══════════════════════════════════════════════════════════════════════════ */
void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;      /* x4 체배 */
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* ★ TIM1 은 이제 PWM 이 아닙니다.
   *   따라서 HAL_TIM_MspPostInit(&htim1) 을 호출하면 안 됩니다.
   *   (그 함수는 핀을 AF_PP 출력으로 만듭니다 = 엔코더 입력이 망가짐) */
  /* USER CODE END TIM1_Init 2 */
}

/* ══════════════════════════════════════════════════════════════════════════
 * TIM2 — ★ PWM 4채널 (partial remap 2)
 * ══════════════════════════════════════════════════════════════════════════ */
void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;                       /* ★ MOTOR_PWM_MAX 9999 와 짝 */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  /* ★ 네 채널 모두 설정 — 구버전은 CH1/CH2 두 개만 했습니다 */
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)   /* PA0  RL */
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)   /* PA1  RR */
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)   /* PB10 FL */
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)   /* PB11 FR */
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/* ══════════════════════════════════════════════════════════════════════════
 * TIM3 — Encoder (FL).  변경 없음
 * ══════════════════════════════════════════════════════════════════════════ */
void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/* ══════════════════════════════════════════════════════════════════════════
 * TIM4 — Encoder (FR).  변경 없음
 * ══════════════════════════════════════════════════════════════════════════ */
void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/* ══════════════════════════════════════════════════════════════════════════
 * MSP — PWM (TIM2 뿐)
 * ══════════════════════════════════════════════════════════════════════════ */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
  /* ★ TIM1 분기 삭제 — TIM1 은 더 이상 PWM 이 아닙니다 */
}

/* ══════════════════════════════════════════════════════════════════════════
 * MSP — Encoder (TIM1 / TIM3 / TIM4)
 * ══════════════════════════════════════════════════════════════════════════ */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (tim_encoderHandle->Instance == TIM1)
  {
    /* USER CODE BEGIN TIM1_MspInit 0 */

    /* USER CODE END TIM1_MspInit 0 */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    ★ 반드시 INPUT 입니다. 구버전은 여기가 AF_PP(출력)였습니다.
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* USER CODE BEGIN TIM1_MspInit 1 */

    /* USER CODE END TIM1_MspInit 1 */
  }
  else if (tim_encoderHandle->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB4     ------> TIM3_CH1
    PB5     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_TIM3_PARTIAL();
    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
  else if (tim_encoderHandle->Instance == TIM4)
  {
    /* USER CODE BEGIN TIM4_MspInit 0 */

    /* USER CODE END TIM4_MspInit 0 */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* USER CODE BEGIN TIM4_MspInit 1 */

    /* USER CODE END TIM4_MspInit 1 */
  }
}

/* ══════════════════════════════════════════════════════════════════════════
 * MSP PostInit — ★ TIM2 4채널 GPIO + partial remap 2
 * ══════════════════════════════════════════════════════════════════════════ */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (timHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspPostInit 0 */

    /* USER CODE END TIM2_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();      /* ★ remap 매크로를 쓰려면 AFIO 클럭 필수 */

    /**TIM2 GPIO Configuration  (partial remap 2)
    PA0-WKUP ------> TIM2_CH1   (RL)
    PA1      ------> TIM2_CH2   (RR)
    PB10     ------> TIM2_CH3   (FL)
    PB11     ------> TIM2_CH4   (FR)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ★★ REMAP[1:0] = 10  ->  CH1=PA0 CH2=PA1 CH3=PB10 CH4=PB11
     *   이 한 줄이 없으면 CH3/CH4 가 PA2/PA3 로 나갑니다(= USART2 와 충돌).
     *   반드시 GPIO 설정 뒤, PWM_Start 전에 호출되어야 합니다. */
    __HAL_AFIO_REMAP_TIM2_PARTIAL_2();

    /* USER CODE BEGIN TIM2_MspPostInit 1 */

    /* USER CODE END TIM2_MspPostInit 1 */
  }
  /* ★ TIM1 분기 삭제 — 엔코더 핀을 AF_PP 출력으로 만들면 안 됩니다 */
}

/* ══════════════════════════════════════════════════════════════════════════
 * MSP DeInit
 * ══════════════════════════════════════════════════════════════════════════ */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{
  if (tim_encoderHandle->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);
  }
  else if (tim_encoderHandle->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);
  }
  else if (tim_encoderHandle->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */