/* motor.h — STM32 Motor Slave Interface */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * Motor_Drive
 * -----------
 * speed > 0 : 전진  (htim2/htim1 CH1,CH2 = speed, 전진 GPIO)
 * speed < 0 : 후진  (htim2/htim1 CH1,CH2 = |speed|, 후진 GPIO)
 * speed = 0 : 정지
 *
 * PWM: htim2 TIM_CHANNEL_1/2, htim1 TIM_CHANNEL_1/2
 * DIR: GPIOC PIN 0/1/2/3/5/6/8/9
 * 범위: -9999 ~ +9999
 */
void Motor_Drive(int speed);

/** 전체 모터 즉시 정지 (방향 핀 초기화 + PWM = 0) */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
