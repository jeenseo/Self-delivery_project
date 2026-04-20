/* motor.h — Cytron MDD10A Rev 2.0 스키드-스티어 모터 드라이버 인터페이스 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * Motor_Drive
 * -----------
 * 좌/우 모터 독립 속도 제어 (스키드-스티어 + 전륜 우선 차동)
 *
 * @param left   좌측 속도 (-9999 ~ +9999)  양수=전진, 음수=후진, 0=정지
 * @param right  우측 속도 (-9999 ~ +9999)
 *
 * 드라이버: Cytron MDD10A Rev 2.0 (Sign-Magnitude: DIR + PWM)
 *
 * 핀 매핑:
 *   좌측 전륜: DIR=PC2, PWM=PA8  (htim1 TIM1_CH1)
 *   우측 전륜: DIR=PC3, PWM=PA9  (htim1 TIM1_CH2)
 *   좌측 후륜: DIR=PC0, PWM=PA0  (htim2 TIM2_CH1)
 *   우측 후륜: DIR=PC1, PWM=PA1  (htim2 TIM2_CH2)
 *
 * 스티어링:
 *   전륜 (htim1): 완전 차동 (left / right 원값)
 *   후륜 (htim2): 직진 성분 + 30% 차동
 */
void Motor_Drive(int16_t left, int16_t right);

/** 모든 모터 즉시 정지 (DIR=LOW, PWM=0) */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
