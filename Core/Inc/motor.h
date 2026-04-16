/* motor.h — STM32 스키드-스티어 모터 슬레이브 인터페이스 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * Motor_Drive
 * -----------
 * 좌/우 모터 독립 속도 제어 (스키드-스티어)
 *
 * @param left   좌측 속도 (-9999 ~ +9999)  양수=전진, 음수=후진, 0=정지
 * @param right  우측 속도 (-9999 ~ +9999)
 *
 * PWM : htim2 CH1(R앞), htim2 CH2(L앞), htim1 CH1(R뒤), htim1 CH2(L뒤)
 * DIR : GPIOC  PC0/1(R앞), PC2/3(L앞), PC5/6(R뒤), PC8/9(L뒤)
 */
void Motor_Drive(int16_t left, int16_t right);

/** 모든 모터 즉시 정지 */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
