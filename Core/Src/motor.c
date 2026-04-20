/*
 * motor.c
 * ───────
 * STM32 스키드-스티어 — Cytron MDD10A Rev 2.0 드라이버
 *
 * 드라이버 인터페이스: Sign-Magnitude (DIR + PWM)
 *   DIR = HIGH → 전진 (Forward)
 *   DIR = LOW  → 후진 (Backward)
 *   PWM        → 속도 크기 (0 ~ 9999)
 *
 * 채널 / 핀 매핑
 * ─────────────────────────────────────────────────────────────────
 *  위치      │ DIR 핀 (GPIOC) │ PWM 채널          │ PWM 핀
 * ───────────┼───────────────┼───────────────────┼────────
 *  좌측 전륜 │ PC2           │ htim1 TIM1_CH1    │ PA8
 *  우측 전륜 │ PC3           │ htim1 TIM1_CH2    │ PA9
 *  좌측 후륜 │ PC0           │ htim2 TIM2_CH1    │ PA0
 *  우측 후륜 │ PC1           │ htim2 TIM2_CH2    │ PA1
 * ─────────────────────────────────────────────────────────────────
 *
 * 전륜 우선 스티어링 (Front-Wheel Bias):
 *   전륜 (htim1): 좌/우 완전 차동 (left / right 원값)
 *   후륜 (htim2): 직진 성분(base) + 차동 30% — 후방 항력(drag) 방지
 *
 * Motor_Drive(left, right):
 *   양수 = 전진, 음수 = 후진, 0 = 정지
 *   유효 범위: -9999 ~ +9999
 */

#include "motor.h"
#include "tim.h"

/* ── 상수 ────────────────────────────────────────────────── */
#define MOTOR_PWM_MAX    9999

/* 후륜 차동 비율 (30%) */
#define REAR_BIAS_NUM    3
#define REAR_BIAS_DEN    10

/* ── 내부 헬퍼: Sign-Magnitude 단일 채널 설정 ──────────────
 *
 * Cytron MDD10A Rev 2.0:
 *   DIR=HIGH + PWM → 전진
 *   DIR=LOW  + PWM → 후진
 *   DIR=LOW  + 0   → 정지
 *
 * @param pin_dir  GPIOC DIR 핀 (단일 핀)
 * @param tim      타이머 핸들 (htim1 또는 htim2)
 * @param ch       타이머 채널 (TIM_CHANNEL_x)
 * @param speed    속도 값 (-9999 ~ +9999)
 * ──────────────────────────────────────────────────────── */
static void _set_wheel(
    uint16_t          pin_dir,
    TIM_HandleTypeDef *tim,
    uint32_t          ch,
    int16_t           speed)
{
    /* 클램프 */
    if (speed >  (int16_t)MOTOR_PWM_MAX) speed =  (int16_t)MOTOR_PWM_MAX;
    if (speed < -(int16_t)MOTOR_PWM_MAX) speed = -(int16_t)MOTOR_PWM_MAX;

    uint32_t pwm = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);

    if (speed > 0)
    {
        /* 전진: DIR=HIGH, PWM=speed */
        HAL_GPIO_WritePin(GPIOC, pin_dir, GPIO_PIN_SET);
    }
    else if (speed < 0)
    {
        /* 후진: DIR=LOW, PWM=|speed| */
        HAL_GPIO_WritePin(GPIOC, pin_dir, GPIO_PIN_RESET);
    }
    else
    {
        /* 정지: DIR=LOW, PWM=0 */
        HAL_GPIO_WritePin(GPIOC, pin_dir, GPIO_PIN_RESET);
        pwm = 0U;
    }

    __HAL_TIM_SET_COMPARE(tim, ch, pwm);
}

/* ── 공개 API ────────────────────────────────────────────── */

/**
 * Motor_Drive
 * ──────────────────────────────────────────────────────────
 * 전륜 우선 스키드-스티어 구동.
 *
 * @param left   좌측 CAN 속도값 (-9999 ~ +9999)
 * @param right  우측 CAN 속도값 (-9999 ~ +9999)
 *
 * 전륜 (htim1): 완전 차동 (left, right 그대로)
 * 후륜 (htim2): 직진 성분 + 차동 30% (후방 항력 방지)
 */
void Motor_Drive(int16_t left, int16_t right)
{
    /* ── 전륜 속도 계산: 완전 차동 ──────────────────────── */
    int16_t L_front = left;
    int16_t R_front = right;

    /* ── 후륜 속도 계산: 직진 성분(base) + 30% 차동 ─────── */
    int16_t base   = (int16_t)((left + right) / 2);
    int16_t diff_L = (int16_t)((left  - base) * REAR_BIAS_NUM / REAR_BIAS_DEN);
    int16_t diff_R = (int16_t)((right - base) * REAR_BIAS_NUM / REAR_BIAS_DEN);
    int16_t L_rear = (int16_t)(base + diff_L);
    int16_t R_rear = (int16_t)(base + diff_R);

    /* ── 4채널 개별 적용 ─────────────────────────────────── */

    /* 좌측 전륜: DIR=PC2, PWM=PA8 (htim1 TIM1_CH1) */
    _set_wheel(GPIO_PIN_2, &htim1, TIM_CHANNEL_1, L_front);

    /* 우측 전륜: DIR=PC3, PWM=PA9 (htim1 TIM1_CH2) */
    _set_wheel(GPIO_PIN_3, &htim1, TIM_CHANNEL_2, R_front);

    /* 좌측 후륜: DIR=PC0, PWM=PA0 (htim2 TIM2_CH1) */
    _set_wheel(GPIO_PIN_0, &htim2, TIM_CHANNEL_1, L_rear);

    /* 우측 후륜: DIR=PC1, PWM=PA1 (htim2 TIM2_CH2) */
    _set_wheel(GPIO_PIN_1, &htim2, TIM_CHANNEL_2, R_rear);
}

/**
 * Motor_Stop
 * ──────────────────────────────────────────────────────────
 * 모든 모터 즉시 정지 (DIR=LOW, PWM=0).
 */
void Motor_Stop(void)
{
    Motor_Drive(0, 0);
}
