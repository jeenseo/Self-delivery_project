/*
 * motor.c
 * -------
 * STM32 순수 모터 슬레이브 구현
 *
 * PWM 채널:
 *   htim2 TIM_CHANNEL_1 → 우측 모터
 *   htim2 TIM_CHANNEL_2 → 좌측 모터
 *   htim1 TIM_CHANNEL_1 → 우측 모터 (보조)
 *   htim1 TIM_CHANNEL_2 → 좌측 모터 (보조)
 *
 * 방향 GPIO (GPIOC):
 *   전진: PC0=1,PC1=0 / PC2=1,PC3=0 / PC5=1,PC6=0 / PC8=1,PC9=0
 *   후진: PC0=0,PC1=1 / PC2=0,PC3=1 / PC5=0,PC6=1 / PC8=0,PC9=1
 *   정지: 모든 핀 LOW, PWM = 0
 */

#include "motor.h"
#include "tim.h"

/* PWM 최대값 (TIM2/TIM1 ARR = 9999) */
#define MOTOR_PWM_MAX  9999

/* ── 내부 헬퍼 ─────────────────────────────────────────── */

static void Direction_Forward(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

static void Direction_Backward(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

static void Direction_Stop(void)
{
    HAL_GPIO_WritePin(GPIOC,
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
        GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9,
        GPIO_PIN_RESET);
}

/* ── 공개 API ──────────────────────────────────────────── */

/**
 * Motor_Drive
 * -----------
 * CAN Rx 콜백에서 직접 호출됩니다.
 *
 * speed > 0 : 전진, htim2/htim1 CH1/CH2 = speed
 * speed < 0 : 후진, htim2/htim1 CH1/CH2 = |speed|
 * speed = 0 : 즉시 정지
 *
 * 유효 범위: -9999 ~ +9999
 */
void Motor_Drive(int speed)
{
    if (speed == 0)
    {
        Motor_Stop();
        return;
    }

    /* 클램프 */
    if (speed >  MOTOR_PWM_MAX) speed =  MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    /* 방향 핀 설정 */
    if (speed > 0)
        Direction_Forward();
    else
        Direction_Backward();

    int pwm = (speed > 0) ? speed : -speed;

    /* htim2: 전면 모터 드라이버 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);

    /* htim1: 후면(보조) 모터 드라이버 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
}

/**
 * Motor_Stop
 * ----------
 * 모든 모터 즉시 정지 (방향 핀 초기화 + PWM = 0)
 */
void Motor_Stop(void)
{
    Direction_Stop();

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}
