/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for the can.c file
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern uint8_t           dataReceived;   /* CAN 수신 플래그                    */
extern int16_t           rxLeftSpeed;    /* 수신 좌측 속도 (-9999 ~ +9999)      */
extern int16_t           rxRightSpeed;   /* 수신 우측 속도 (-9999 ~ +9999)      */

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

/* Exported functions --------------------------------------------------------*/
void MX_CAN_Init(void);
void CAN_filter(void);

/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
