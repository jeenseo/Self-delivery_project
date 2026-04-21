/* motor.h — Cytron MDD10A Rev 2.0 4-휠 독립 제어 인터페이스 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * Motor_Drive
 * -----------
 * 4개 바퀴 독립 속도 제어 (8바이트 CAN 프로토콜 대응)
 *
 * @param fl  Front Left  속도 (-9999 ~ +9999)  양수=전진, 음수=후진
 * @param fr  Front Right 속도
 * @param rl  Rear Left   속도 (내부에서 하드웨어 역결선 보정 자동 적용)
 * @param rr  Rear Right  속도 (내부에서 하드웨어 역결선 보정 자동 적용)
 *
 * 드라이버: Cytron MDD10A Rev 2.0 (Sign-Magnitude: DIR + PWM)
 *
 * 핀 매핑:
 *   좌측 전륜: DIR=PC2, PWM=PA8  (htim1 TIM1_CH1)
 *   우측 전륜: DIR=PC3, PWM=PA9  (htim1 TIM1_CH2)
 *   좌측 후륜: DIR=PC0, PWM=PA0  (htim2 TIM2_CH1, 역결선 보정)
 *   우측 후륜: DIR=PC1, PWM=PA1  (htim2 TIM2_CH2, 역결선 보정)
 */
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr);

/** 모든 모터 즉시 정지 (DIR=LOW, PWM=0) */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
