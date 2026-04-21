/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN1 초기화 및 수신 처리 — STM32 모터 슬레이브
  *
  *  프로토콜 (Pi → STM32, CAN ID 0x123):
  *    Byte 0~1 : int16_t Left  Motor Speed  Big-Endian  (-9999 ~ +9999)
  *    Byte 2~3 : int16_t Right Motor Speed  Big-Endian  (-9999 ~ +9999)
  *    (총 4바이트)
  *
  *  비트레이트 계산:
  *    PCLK1 = 36 MHz,  Prescaler = 6  → TQ clock = 6 MHz
  *    1 + BS1(10) + BS2(1) = 12 TQ  → 6 MHz / 12 = 500 kbps ✓
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ── 전역 변수 ──────────────────────────────────────────────────────────── */
CAN_HandleTypeDef   hcan;

CAN_RxHeaderTypeDef rxHeader;
uint8_t             rxData[8];

uint8_t             dataReceived = 0;   /* CAN 수신 플래그 (main 루프용) */
int16_t             rxFL = 0;           /* Front Left  (-9999 ~ +9999)   */
int16_t             rxFR = 0;           /* Front Right (-9999 ~ +9999)   */
int16_t             rxRL = 0;           /* Rear Left   (-9999 ~ +9999)   */
int16_t             rxRR = 0;           /* Rear Right  (-9999 ~ +9999)   */

/* ── CAN_filter ─────────────────────────────────────────────────────────────
 *   Pass-All 필터: 모든 CAN ID 수신 허용
 * ──────────────────────────────────────────────────────────────────────── */
void CAN_filter(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;   /* Mask=0 → 모든 ID 통과 */
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();
}

/* ── HAL_CAN_RxFifo0MsgPendingCallback ─────────────────────────────────────
 *   ISR 내부: 데이터 파싱 + 플래그 세팅만 (printf/UART 금지)
 *   main 루프에서 dataReceived 확인 후 Motor_Drive 호출
 * ──────────────────────────────────────────────────────────────────────── */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        /* Byte 0~1: Front Left  (Big-Endian int16_t) */
        rxFL = (int16_t)((rxData[0] << 8) | rxData[1]);
        /* Byte 2~3: Front Right (Big-Endian int16_t) */
        rxFR = (int16_t)((rxData[2] << 8) | rxData[3]);
        /* Byte 4~5: Rear Left   (Big-Endian int16_t) */
        rxRL = (int16_t)((rxData[4] << 8) | rxData[5]);
        /* Byte 6~7: Rear Right  (Big-Endian int16_t) */
        rxRR = (int16_t)((rxData[6] << 8) | rxData[7]);

        dataReceived = 1;
    }
}

/* ── MX_CAN_Init ──────────────────────────────────────────────────────────
 *   CAN1 500 kbps 초기화
 *   PCLK1=36MHz / Prescaler=6 / BS1=10TQ / BS2=1TQ → 500kbps
 * ──────────────────────────────────────────────────────────────────────── */
void MX_CAN_Init(void)
{
    /* USER CODE BEGIN CAN_Init 0 */
    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */
    /* USER CODE END CAN_Init 1 */

    hcan.Instance                  = CAN1;
    hcan.Init.Prescaler            = 6;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_10TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_1TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = DISABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = DISABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK)
        Error_Handler();

    /* USER CODE BEGIN CAN_Init 2 */
    /* USER CODE END CAN_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(canHandle->Instance==CAN1)
    {
        /* USER CODE BEGIN CAN1_MspInit 0 */
        /* USER CODE END CAN1_MspInit 0 */
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA11 → CAN_RX */
        GPIO_InitStruct.Pin  = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PA12 → CAN_TX */
        GPIO_InitStruct.Pin   = GPIO_PIN_12;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* RX FIFO0 인터럽트 활성화 */
        HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
        /* USER CODE BEGIN CAN1_MspInit 1 */
        /* USER CODE END CAN1_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
    if(canHandle->Instance==CAN1)
    {
        /* USER CODE BEGIN CAN1_MspDeInit 0 */
        /* USER CODE END CAN1_MspDeInit 0 */
        __HAL_RCC_CAN1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
        /* USER CODE BEGIN CAN1_MspDeInit 1 */
        /* USER CODE END CAN1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
