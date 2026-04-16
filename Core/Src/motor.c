/*
 * motor.c
 * ───────
 * STM32 스키드-스티어 (탱크 구동) 모터 슬레이브
 *
 * 채널 / 핀 매핑
 * ─────────────────────────────────────────────────────────────────
 *  사이드   │ PWM 채널             │ 방향 핀 (IN1/IN2)
 * ──────────┼─────────────────────┼─────────────────────────────
 *  우측(R)  │ htim2 CH1, htim1 CH1 │ PC0/PC1  (앞),  PC5/PC6  (뒤)
 *  좌측(L)  │ htim2 CH2, htim1 CH2 │ PC2/PC3  (앞),  PC8/PC9  (뒤)
 * ─────────────────────────────────────────────────────────────────
 *
 * Motor_Drive(left, right):
 *   양수 = 전진, 음수 = 후진, 0 = 정지
 *   유효 범위: -9999 ~ +9999
 */

#include "motor.h"
#include "tim.h"

/* PWM 최대값 (TIM ARR = 9999) */
#define MOTOR_PWM_MAX  9999

/* ─────────────────────── 내부 헬퍼 ────────────────────────── */

/**
 * @brief 한 사이드의 방향 핀과 PWM을 설정합니다.
 *
 * @param pin_fwd_a  IN1 핀 A (앞 모터 드라이버)
 * @param pin_rev_a  IN2 핀 A
 * @param pin_fwd_b  IN1 핀 B (뒤 모터 드라이버)
 * @param pin_rev_b  IN2 핀 B
 * @param tim_a      앞 드라이버 타이머 핸들
 * @param ch_a       앞 드라이버 채널
 * @param tim_b      뒤 드라이버 타이머 핸들
 * @param ch_b       뒤 드라이버 채널
 * @param speed      속도 값 (-9999 ~ +9999)
 */
static void _set_side(
    uint16_t pin_fwd_a, uint16_t pin_rev_a,
    uint16_t pin_fwd_b, uint16_t pin_rev_b,
    TIM_HandleTypeDef *tim_a, uint32_t ch_a,
    TIM_HandleTypeDef *tim_b, uint32_t ch_b,
    int16_t speed)
{
    /* 클램프 */
    if (speed >  MOTOR_PWM_MAX) speed =  MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    uint32_t pwm = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);

    if (speed > 0)
    {
        /* 전진: IN1=HIGH, IN2=LOW */
        HAL_GPIO_WritePin(GPIOC, pin_fwd_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, pin_rev_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, pin_fwd_b, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, pin_rev_b, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        /* 후진: IN1=LOW, IN2=HIGH */
        HAL_GPIO_WritePin(GPIOC, pin_fwd_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, pin_rev_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, pin_fwd_b, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, pin_rev_b, GPIO_PIN_SET);
    }
    else
    {
        /* 정지: 모든 핀 LOW */
        HAL_GPIO_WritePin(GPIOC,
            pin_fwd_a | pin_rev_a | pin_fwd_b | pin_rev_b,
            GPIO_PIN_RESET);
        pwm = 0;
    }

    __HAL_TIM_SET_COMPARE(tim_a, ch_a, pwm);
    __HAL_TIM_SET_COMPARE(tim_b, ch_b, pwm);
}

/* ─────────────────────── 공개 API ─────────────────────────── */

/**
 * Motor_Drive
 * ──────────────────────────────────────────────────────────────
 * 스키드-스티어 구동: 좌/우 독립 속도 제어
 *
 * @param left   좌측 모터 속도 (-9999 ~ +9999)
 * @param right  우측 모터 속도 (-9999 ~ +9999)
 *
 * 포인트 턴 (제자리 회전):
 *   좌 회전: left < 0,  right > 0
 *   우 회전: left > 0,  right < 0
 */
void Motor_Drive(int16_t left, int16_t right)
{
    /* 우측: htim2 CH1, htim1 CH1 / PC0(IN1) PC1(IN2) / PC5(IN1) PC6(IN2) */
    _set_side(
        GPIO_PIN_0, GPIO_PIN_1,   /* 우측 앞 IN1/IN2 */
        GPIO_PIN_5, GPIO_PIN_6,   /* 우측 뒤 IN1/IN2 */
        &htim2, TIM_CHANNEL_1,
        &htim1, TIM_CHANNEL_1,
        right);

    /* 좌측: htim2 CH2, htim1 CH2 / PC2(IN1) PC3(IN2) / PC8(IN1) PC9(IN2) */
    _set_side(
        GPIO_PIN_2, GPIO_PIN_3,   /* 좌측 앞 IN1/IN2 */
        GPIO_PIN_8, GPIO_PIN_9,   /* 좌측 뒤 IN1/IN2 */
        &htim2, TIM_CHANNEL_2,
        &htim1, TIM_CHANNEL_2,
        left);
}

/**
 * Motor_Stop
 * ──────────────────────────────────────────────────────────────
 * 모든 모터 즉시 정지.
 */
void Motor_Stop(void)
{
    Motor_Drive(0, 0);
}
