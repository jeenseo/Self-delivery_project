/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART2 헤더 — v4
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
int _write(int file, char *ptr, int len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /* __USART_H__ */