/**
 * @file   motor.h
 * @brief  STM32 스키드-스티어 모터 제어 — PID + 엔코더 + CAN TX
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
#define DEADBAND_PWM            2000U           /**< 최소 기동 PWM (실측값) */
#define MOTOR_PWM_MAX           9999U           /**< 최대 PWM 값            */
#define PID_PERIOD_MS           10U             /**< PID 갱신 주기 (ms)     */

/* ─────────────────────────────────────────────────────────────
 * 방향 반전 매크로 (Direction Inversion per Wheel)
 *   0 = 정상 (DIR HIGH=전진)
 *   1 = 반전 (DIR LOW=전진) — 물리적 역결선 바퀴에 사용
 * ───────────────────────────────────────────────────────────── */
#define FL_DIR_INVERT           0   /**< 좌측 전륜 — 필요 시 1로 변경 */
#define FR_DIR_INVERT           0   /**< 우측 전륜 — 필요 시 1로 변경 */
#define RL_DIR_INVERT           0   /**< 좌측 후륜 — 필요 시 1로 변경 */
#define RR_DIR_INVERT           0   /**< 우측 후륜 — 필요 시 1로 변경 */

/* ─────────────────────────────────────────────────────────────
 * PID 조정 파라미터 (Tunable PID Gains)
 * ───────────────────────────────────────────────────────────── */
#define MOTOR_KP                2.0f    /**< 비례 이득 */
#define MOTOR_KI                0.5f    /**< 적분 이득 */
#define MOTOR_KD                0.05f   /**< 미분 이득 */

/* ─────────────────────────────────────────────────────────────
 * CAN 피드백 설정
 * ───────────────────────────────────────────────────────────── */
#define CAN_FEEDBACK_ID         0x124U  /**< 엔코더 피드백 CAN ID     */
#define CAN_FEEDBACK_PERIOD_MS  20U     /**< CAN TX 주기 (ms)         */

/* ─────────────────────────────────────────────────────────────
 * 공개 API (Public API)
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief PID 상태, 엔코더 카운터 초기화
 *        Motor_Drive() / Motor_PID_Update() 사용 전 반드시 호출
 */
void Motor_Init(void);

/**
 * @brief CAN 수신값으로 4-휠 목표 속도 설정 (PID 입력)
 * @param fl  Front Left  (-9999 ~ +9999, 양수=전진)
 * @param fr  Front Right
 * @param rl  Rear Left
 * @param rr  Rear Right
 */
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr);

/**
 * @brief PID 1 사이클 실행 (10ms 주기로 호출)
 *        엔코더 읽기 → 속도 계산 → PID 연산 → PWM 출력
 *        CAN TX 누산기에도 엔코더 틱을 누적
 */
void Motor_PID_Update(void);

/**
 * @brief 엔코더 피드백을 CAN 0x124로 전송 (20ms 주기로 호출)
 *        Payload: [Left_delta(2B)][Right_delta(2B)] Big-Endian
 *        Pi에서 /odom_wheel 토픽 계산에 사용
 */
void Motor_Send_Feedback_CAN(void);

/**
 * @brief 모든 모터 즉시 정지 + PID 상태 리셋
 */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
