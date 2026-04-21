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
 * 하드웨어 특성:
 *   전륜 (htim1): 양수 값 = 전진
 *   후륜 (htim2): 하드웨어 역결선 → 음수 값 적용해야 전진
 *                Motor_Drive 내부에서 -rl, -rr 적용
 *
 * CAN 프로토콜 (Pi → STM32, 8바이트):
 *   Byte 0-1: FL (Front Left)
 *   Byte 2-3: FR (Front Right)
 *   Byte 4-5: RL (Rear Left)
 *   Byte 6-7: RR (Rear Right)
 *
 * Motor_Drive(fl, fr, rl, rr):
 *   양수 = 전진 의도, 음수 = 후진 의도
 *   범위: -9999 ~ +9999
 */

#include "motor.h"
#include "tim.h"

/* ── 상수 ────────────────────────────────────────────────── */
#define MOTOR_PWM_MAX    9999

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
 * 4-휠 독립 제어. CAN 수신값을 직접 각 채널에 적용.
 *
 * @param fl  Front Left  속도 (-9999 ~ +9999, 양수=전진)
 * @param fr  Front Right 속도
 * @param rl  Rear Left   속도 (하드웨어 반전: 내부에서 -rl 적용)
 * @param rr  Rear Right  속도 (하드웨어 반전: 내부에서 -rr 적용)
 *
 * 전륜 (htim1): 값 그대로 적용
 * 후륜 (htim2): 역결선 보정 — (-) 부호 반전 후 적용
 */
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    /* 전륜: DIR=PC2/PC3, PWM=PA8/PA9 (htim1) — 값 그대로 */
    _set_wheel(GPIO_PIN_2, &htim1, TIM_CHANNEL_1,  fl);   /* L Front */
    _set_wheel(GPIO_PIN_3, &htim1, TIM_CHANNEL_2,  fr);   /* R Front */

    /* 후륜: DIR=PC0/PC1, PWM=PA0/PA1 (htim2) — 하드웨어 반전 보정 */
    _set_wheel(GPIO_PIN_0, &htim2, TIM_CHANNEL_1, -rl);   /* L Rear  */
    _set_wheel(GPIO_PIN_1, &htim2, TIM_CHANNEL_2, -rr);   /* R Rear  */
}

/**
 * Motor_Stop
 * ──────────────────────────────────────────────────────────
 * 모든 모터 즉시 정지 (DIR=LOW, PWM=0).
 */
void Motor_Stop(void)
{
    Motor_Drive(0, 0, 0, 0);
}
