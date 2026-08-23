/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN1 초기화 — STM32 모터 슬레이브 (v4)
  ******************************************************************************
  *
  *  프로토콜
  *    Pi -> STM32  ID 0x123  DLC 8 : int16 FL, FR, RL, RR (Big-Endian, ±9999)
  *    STM32 -> Pi  ID 0x124  DLC 4 : int16 FL ticks, FR ticks
  *                           DLC 6 : + RR ticks   (USE_RR_ENCODER=1)
  *    ※ 구버전 주석에 "4바이트 좌/우" 라고 되어 있었는데 실제와 달랐습니다.
  *
  *  비트레이트 (PCLK1 = 36 MHz)
  *    Prescaler x (1 + BS1 + BS2) = 36 MHz / 500 kbps = 72
  *      PSC=6  BS1=10 BS2=1  -> 6x12 = 72 ✓  샘플포인트 91.7%   (구버전)
  *      PSC=4  BS1=15 BS2=2  -> 4x18 = 72 ✓  샘플포인트 88.9%   ★ 현재
  *
  *    [왜 바꿨는가]
  *      1) SocketCAN(Pi) 기본 샘플포인트가 87.5% 입니다. 91.7% 와 맞추면
  *         양쪽 비트 판정 시점이 가까워져 마진이 늘어납니다.
  *      2) 구버전은 BS2 = 1TQ 라서 **SJW 가 최대 1TQ** 였습니다.
  *         SJW 는 재동기화 시 늘리거나 줄일 수 있는 TQ 수인데, 1TQ 는
  *         클럭 편차 보정 여유가 최소입니다. BS2=2TQ 로 늘려 SJW=2TQ 확보.
  *      3) TQ 가 12개 -> 18개로 잘게 쪼개져 재동기화 해상도가 1.5배 좋아집니다.
  *
  *    ※ 비트레이트(500 kbps)는 두 설정 모두 정확히 같습니다.
  *      지금 통신이 되고 있다면 이 변경이 없어도 동작합니다.
  *      EMI 로 인한 ACK 에러/bus-off 마진을 넓히려는 목적입니다.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{
  /* USER CODE BEGIN CAN_Init 0 */
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;

  /* 500 kbps, 샘플포인트 88.9%, SJW 2TQ */
  hcan.Init.Prescaler     = 4;
  hcan.Init.Mode          = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1      = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2      = CAN_BS2_2TQ;

  hcan.Init.TimeTriggeredMode = DISABLE;

  /**
   * ★★ AutoBusOff = ENABLE  (구버전은 DISABLE 이었습니다)
   *
   *  [DISABLE 이면 무슨 일이 벌어지는가]
   *    TEC 가 256 을 넘어 BUS-OFF 에 빠지면, 소프트웨어가 명시적으로
   *    INRQ 를 토글해 복구시키지 않는 한 **영원히 통신 불가**입니다.
   *    이 펌웨어에는 그런 복구 루틴이 없습니다. 즉 한 번 빠지면 끝입니다.
   *
   *  [왜 이제야 문제가 되는가]
   *    Pi 쪽은 `ip link set can0 ... restart-ms 100` 으로 자동 복구하도록
   *    설정했습니다. 그런데 STM32 쪽은 자동 복구가 꺼져 있었습니다.
   *    **한쪽만 복구되는 비대칭 상태**였습니다.
   *    실제로 "CAN TX 100% 유실" 사건을 겪었고, Pi 를 복구해도 STM32 가
   *    bus-off 에 남아 있으면 통신이 살아나지 않습니다.
   *
   *  ENABLE 이면 128 x 11 연속 recessive 비트를 관측한 뒤 자동으로
   *  ERROR-ACTIVE 로 복귀합니다. 500 kbps 에서 약 2.8 ms 입니다.
   */
  hcan.Init.AutoBusOff = ENABLE;

  hcan.Init.AutoWakeUp = DISABLE;

  /**
   * AutoRetransmission = DISABLE (유지)
   *
   *  엔코더 틱 피드백은 '최신값이 의미 있는' 데이터입니다.
   *  재전송하느라 메일박스를 붙들고 있으면 다음 프레임이 밀립니다.
   *  실패하면 버리고, 다음 20 ms 프레임이 누산분까지 실어 나릅니다.
   *  (motor.c 의 Motor_Send_Feedback_CAN 이 성공 시에만 누산기를 비웁니다)
   */
  hcan.Init.AutoRetransmission  = DISABLE;
  hcan.Init.ReceiveFifoLocked   = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /* USER CODE END CAN_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspInit 0 */
    /* USER CODE END CAN1_MspInit 0 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin  = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 인터럽트
     *  ★ RX0 만 씁니다. 필터를 FIFO0 으로만 보내므로 RX1 은 절대 안 옵니다.
     *    구버전은 CAN1_RX1_IRQn 도 켜 두었는데, 쓰지 않는 인터럽트를
     *    켜 두면 스퓨리어스 진입 시 원인 추적만 어려워집니다.
     *  ★ TX 인터럽트도 쓰지 않습니다. Motor_Send_Feedback_CAN 은
     *    AddTxMessage 반환값만 보고 폴링 방식으로 동작합니다. */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    /* USER CODE BEGIN CAN1_MspInit 1 */
    /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if (canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */