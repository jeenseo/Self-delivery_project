/**
 * @file   motor.h
 * @brief  STM32 스키드-스티어 모터 제어 — 2단계 피드포워드 + PID + 엔코더 + CAN TX
 *
 * [하드웨어 핀 매핑]
 *   위치    │ DIR 핀  │ PWM 타이머/채널      │ PWM 핀
 *  ─────────┼────────┼────────────────────┼────────
 *  FL (좌전) │ PC2    │ TIM1_CH1 (htim1)   │ PA8
 *  FR (우전) │ PC3    │ TIM1_CH2 (htim1)   │ PA9
 *  RL (좌후) │ PC0    │ TIM2_CH1 (htim2)   │ PA0
 *  RR (우후) │ PC1    │ TIM2_CH2 (htim2)   │ PA1
 *  Left Enc  │  -     │ TIM3 (Encoder)     │ PB4, PB5
 *  Right Enc │  -     │ TIM4 (Encoder)     │ PB6, PB7
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────
 * 물리 상수 (Physical Robot Specifications)
 * ───────────────────────────────────────────────────────────── */
#define WHEEL_DIAMETER_M        0.12f           /**< 바퀴 직경 (m)         */
#define WHEEL_CIRCUMFERENCE_M   (WHEEL_DIAMETER_M * 3.14159265f)
#define TRACK_WIDTH_M           0.51f           /**< 좌우 트랙 폭 (m)      */
#define MOTOR_MAX_RPM           150.0f          /**< 최대 부하 속도 (RPM)   */
#define ENCODER_CPR             1404U           /**< 엔코더 해상도 (CPR)    */
#define MOTOR_PWM_MAX           9999U           /**< 최대 PWM 값            */
#define PID_PERIOD_MS           10U             /**< PID 갱신 주기 (ms)     */

/* ─────────────────────────────────────────────────────────────
 * 2단계 피드포워드 상수 (2-Stage Piecewise Feedforward)
 *
 * Zone 1 (Stiction Zone):  0 < RPM ≤ SMOOTH_RPM (31.8)
 *   → PWM: STICTION_PWM_BASE(1300) 선형 증가 → KINETIC_PWM_BASE(2000)
 *   → 정지마찰(스티션) 극복 구간 (Nav2 0.05~0.20 m/s)
 *
 * Zone 2 (Kinetic Zone):   RPM > SMOOTH_RPM (31.8)
 *   → PWM: KINETIC_PWM_BASE(2000) 선형 증가 → MOTOR_PWM_MAX(9999)
 *   → 동역학 구간 (속도에 비례)
 *
 * 수학적 연속성 검증 (PWM 2000에서 연결):
 *   Zone 1 끝: 1300 + (31.8/31.8)×700 = 2000 ✓
 *   Zone 2 시작: 2000 + (31.8-31.8)×slope = 2000 ✓
 * ───────────────────────────────────────────────────────────── */
#define STICTION_PWM_BASE       1300U           /**< 정지마찰 극복 Zone 1 시작 PWM */
#define KINETIC_PWM_BASE        2000U           /**< Zone 2 기저 PWM (연결점)      */
#define SMOOTH_RPM              31.8f           /**< Zone 전환 RPM (≈0.20 m/s)     */

/* ─────────────────────────────────────────────────────────────
 * 방향 반전 매크로 (Direction Inversion per Wheel)
 *   0 = 정상 (DIR HIGH=전진)
 *   1 = 반전 (DIR LOW=전진) — 물리적 역결선 바퀴에 사용
 * ───────────────────────────────────────────────────────────── */
#define FL_DIR_INVERT           1
#define FR_DIR_INVERT           0
#define RL_DIR_INVERT           1
#define RR_DIR_INVERT           0

/* ─────────────────────────────────────────────────────────────
 * PID 조정 파라미터 (Tunable PID Gains)
 *
 * 2단계 피드포워드가 약 95% 담당 → PID는 미세 보정(Sniper) 역할
 * KI는 낮게 유지하여 적분 윈드업 방지
 * ───────────────────────────────────────────────────────────── */
#define MOTOR_KP                2.0f    /**< 비례 이득 */
#define MOTOR_KI                1.0f    /**< 적분 이득 (부하 변동 대응) */
#define MOTOR_KD                0.05f   /**< 미분 이득 */

/* ─────────────────────────────────────────────────────────────
 * CAN 피드백 설정
 * ───────────────────────────────────────────────────────────── */
#define CAN_FEEDBACK_ID         0x124U  /**< 엔코더 피드백 CAN ID     */
#define CAN_FEEDBACK_PERIOD_MS  20U     /**< CAN TX 주기 (ms)         */

/* ─────────────────────────────────────────────────────────────
 * 공개 API (Public API)
 * ───────────────────────────────────────────────────────────── */

void Motor_Init(void);
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void Motor_PID_Update(void);
void Motor_Send_Feedback_CAN(void);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
