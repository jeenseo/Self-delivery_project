/* can.h — CAN 드라이버 헤더 (모터 슬레이브) */
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern CAN_HandleTypeDef hcan;

void MX_CAN_Init(void);
void CAN_filter(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
