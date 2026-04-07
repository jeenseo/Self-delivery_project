/* can.c — CAN 초기화 및 Rx 콜백 (모터 슬레이브) */
#include "can.h"
#include "motor.h"
#include "main.h"

/* ── 수신 버퍼 ─────────────────────────────────────────── */
CAN_RxHeaderTypeDef rxHeader;
uint8_t             rxData[8];

/* ── HAL 핸들 ──────────────────────────────────────────── */
CAN_HandleTypeDef hcan;

/* ── 필터 설정: ID 0x123 전용 수신 ──────────────────── */
#define MOTOR_CAN_ID  0x123U

void CAN_filter(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    /* 11-bit 표준 ID는 FilterIdHigh 상위 11bit에 배치 (<<5) */
    sFilterConfig.FilterIdHigh         = (MOTOR_CAN_ID << 5) & 0xFFFF;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = (0x7FFU << 5) & 0xFFFF; /* 11-bit 전체 마스크 */
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
        Error_Handler();
}

/**
 * HAL_CAN_RxFifo0MsgPendingCallback
 * -----------------------------------
 * CAN 프레임 파싱 → Motor_Drive(int speed) 직접 호출
 *
 * 프레임 포맷 (Raspberry Pi → STM32, ID: 0x123):
 *   Byte 0~1 : int16_t speed (Big-Endian, -9999 ~ +9999)
 *   Byte 2~7 : 예약
 *
 *   양수 = 전진, 음수 = 후진, 0 = 정지
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

    /* ID 이중 검증: 필터 통과 후에도 확인 */
    if (rxHeader.StdId != MOTOR_CAN_ID)
        return;

    int16_t speed = (int16_t)((rxData[0] << 8) | rxData[1]);

    Motor_Drive((int)speed);
}

/* ── CAN 주변장치 초기화 ────────────────────────────── */
void MX_CAN_Init(void)
{
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
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (canHandle->Instance == CAN1)
    {
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

        HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{
    if (canHandle->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    }
}
