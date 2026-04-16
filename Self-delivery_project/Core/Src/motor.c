/*
 * motor.c
 * ───────
 * STM32 스키드-스티어 (탱크 구동) — 전륜 우선 차동 스티어링
 *
 * 채널 / 핀 매핑
 * ─────────────────────────────────────────────────────────────────
 *  위치        │ PWM 채널    │ 방향 핀 (IN1/IN2)
 * ─────────────┼────────────┼──────────────────────────────
 *  우측 전륜   │ htim2 CH1  │ PC0 / PC1
 *  우측 후륜   │ htim1 CH1  │ PC5 / PC6
 *  좌측 전륜   │ htim2 CH2  │ PC2 / PC3
 *  좌측 후륜   │ htim1 CH2  │ PC8 / PC9
 * ─────────────────────────────────────────────────────────────────
 *
 * 전륜 우선 스티어링 (Front-Wheel Bias):
 *   전륜: 좌/우 완전 차동 (left / right 원값)
 *   후륜: 평균 속도(직진) + 차동 30% — 후방 항력(drag) 방지
 *
 * Motor_Drive(left, right):
 *   ROS 2 motor_node 수정 믹싱 결과:
 *     left  = (linear + angular) × N
 *     right = (angular  − linear) × N
 *   → 양수 = 해당 방향 전진
 *   범위: -9999 ~ +9999
 */

#include "motor.h"
#include "tim.h"

/* ── 상수 ────────────────────────────────────────────────── */
#define MOTOR_PWM_MAX    9999

/* 후륜 차동 비율 (10분의 N = 30%) */
#define REAR_BIAS_NUM    3
#define REAR_BIAS_DEN    10

/* ── 내부 헬퍼: 단일 휠 방향 + PWM 설정 ─────────────────── */

/**
 * @brief 하나의 모터 드라이버 채널을 설정합니다.
 *
 * @param pin_fwd  IN1 핀 (전진 시 HIGH)
 * @param pin_rev  IN2 핀 (후진 시 HIGH)
 * @param tim      타이머 핸들
 * @param ch       타이머 채널
 * @param speed    속도 값 (-9999 ~ +9999)
 */
static void _set_wheel(
    uint16_t pin_fwd, uint16_t pin_rev,
    TIM_HandleTypeDef *tim, uint32_t ch,
    int16_t speed)
{
    /* 클램프 */
    if (speed >  (int16_t)MOTOR_PWM_MAX) speed =  (int16_t)MOTOR_PWM_MAX;
    if (speed < -(int16_t)MOTOR_PWM_MAX) speed = -(int16_t)MOTOR_PWM_MAX;

    uint32_t pwm = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);

    if (speed > 0)
    {
        /* 전진: IN1=HIGH, IN2=LOW */
        HAL_GPIO_WritePin(GPIOC, pin_fwd, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, pin_rev, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        /* 후진: IN1=LOW, IN2=HIGH */
        HAL_GPIO_WritePin(GPIOC, pin_fwd, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, pin_rev, GPIO_PIN_SET);
    }
    else
    {
        /* 정지: 모든 핀 LOW */
        HAL_GPIO_WritePin(GPIOC, pin_fwd | pin_rev, GPIO_PIN_RESET);
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
 * 전륜: 완전 차동 (left, right 그대로)
 * 후륜: 직진 성분 + 차동 30% (후방 항력 방지)
 *
 * 포인트 턴: left > 0 && right < 0  (또는 반대)
 */
void Motor_Drive(int16_t left, int16_t right)
{
    /* ── 전륜: 완전 차동 ──────────────────────────────────── */
    int16_t L_front = left;
    int16_t R_front = right;

    /* ── 후륜: 직진 성분(base) + 30% 차동 ────────────────── */
    int16_t base   = (int16_t)((left + right) / 2);
    int16_t diff_L = (int16_t)((left  - base) * REAR_BIAS_NUM / REAR_BIAS_DEN);
    int16_t diff_R = (int16_t)((right - base) * REAR_BIAS_NUM / REAR_BIAS_DEN);
    int16_t L_rear = (int16_t)(base + diff_L);
    int16_t R_rear = (int16_t)(base + diff_R);

    /* ── 4채널 개별 적용 ──────────────────────────────────── */

    /* 우측 전륜: htim2 CH1  / PC0(IN1) PC1(IN2) */
    _set_wheel(GPIO_PIN_0, GPIO_PIN_1, &htim2, TIM_CHANNEL_1, R_front);

    /* 우측 후륜: htim1 CH1  / PC5(IN1) PC6(IN2) */
    _set_wheel(GPIO_PIN_5, GPIO_PIN_6, &htim1, TIM_CHANNEL_1, R_rear);

    /* 좌측 전륜: htim2 CH2  / PC2(IN1) PC3(IN2) */
    _set_wheel(GPIO_PIN_2, GPIO_PIN_3, &htim2, TIM_CHANNEL_2, L_front);

    /* 좌측 후륜: htim1 CH2  / PC8(IN1) PC9(IN2) */
    _set_wheel(GPIO_PIN_8, GPIO_PIN_9, &htim1, TIM_CHANNEL_2, L_rear);
}

/**
 * Motor_Stop
 * ──────────────────────────────────────────────────────────
 * 모든 모터 즉시 정지.
 */
void Motor_Stop(void)
{
    Motor_Drive(0, 0);
}
