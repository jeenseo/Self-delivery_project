/**
 * @file   motor.c
 * @brief  STM32 스키드-스티어 4-휠 모터 제어
 *         PID 속도 제어 + 엔코더 피드백 + CAN TX (0x124)
 *
 * [아키텍처]
 *   CAN RX 0x123: Motor_Drive(fl, fr, rl, rr) → 목표 속도 설정
 *   10ms 주기:    Motor_PID_Update()          → 엔코더→RPM→PID→PWM
 *   20ms 주기:    Motor_Send_Feedback_CAN()   → 누산 틱 → Pi
 *
 * [좌/우 PID 분리 (2 엔코더, 4 바퀴)]
 *   TIM3 (Left Enc)  → FL + RL 공통 PID 제어
 *   TIM4 (Right Enc) → FR + RR 공통 PID 제어
 *   CAN 입력: left_target  = (rxFL + rxRL) / 2
 *             right_target = (rxFR + rxRR) / 2
 *
 * [데드밴드 보상]
 *   목표 ≠ 0 → base PWM = DEADBAND_PWM + PID 보정
 *   목표 = 0 → 즉시 정지 (PWM=0)
 */

#include "motor.h"
#include "tim.h"
#include "can.h"
#include <string.h>
#include <math.h>

/* ── 외부 핸들 선언 ──────────────────────────────────────────── */
extern TIM_HandleTypeDef htim1;   /* FL/FR PWM (TIM1_CH1, CH2) */
extern TIM_HandleTypeDef htim2;   /* RL/RR PWM (TIM2_CH1, CH2) */
extern TIM_HandleTypeDef htim3;   /* 좌측 엔코더               */
extern TIM_HandleTypeDef htim4;   /* 우측 엔코더               */
extern CAN_HandleTypeDef hcan;    /* CAN1 핸들                  */

/* ── 내부 상수 ───────────────────────────────────────────────── */
#define PID_PERIOD_S        (PID_PERIOD_MS * 0.001f)

/* RPM→PWM 선형 변환 스케일 (데드밴드 이후 구간) */
#define RPM_TO_PWM_SCALE    ((float)(MOTOR_PWM_MAX - DEADBAND_PWM) / MOTOR_MAX_RPM)

/* 클램프 매크로 */
#define CLAMP(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* 적분 누적 한계 (Anti-Windup) */
#define INTEGRAL_LIMIT      (MOTOR_MAX_RPM * 2.0f)

/* ── PID 상태 구조체 ─────────────────────────────────────────── */
typedef struct {
    float target_rpm;   /**< 목표 속도 (RPM, 양수=전진) */
    float integral;     /**< 적분 누적값                */
    float prev_error;   /**< 이전 오차 (미분 계산용)     */
} PID_t;

/* ── 모듈 내부 상태 ──────────────────────────────────────────── */
static PID_t   s_pid_left;            /* 좌측 PID 상태 */
static PID_t   s_pid_right;           /* 우측 PID 상태 */

static int32_t s_enc_left_last  = 0;  /* 이전 TIM3 카운터 */
static int32_t s_enc_right_last = 0;  /* 이전 TIM4 카운터 */

/* CAN TX 누산기: Motor_PID_Update() 마다 틱을 누적 */
static int32_t s_fb_left_accum  = 0;
static int32_t s_fb_right_accum = 0;

/* ════════════════════════════════════════════════════════════
 *  내부 헬퍼 함수
 * ════════════════════════════════════════════════════════════ */

/**
 * @brief Sign-Magnitude 방식으로 단일 채널 DIR+PWM 설정
 *
 * @param pin_dir  GPIOC DIR 핀 마스크 (예: GPIO_PIN_2)
 * @param tim      PWM 타이머 핸들 포인터
 * @param ch       타이머 채널 (TIM_CHANNEL_x)
 * @param speed    속도값 (-9999 ~ +9999, 양수=전진)
 * @param invert   방향 반전 플래그 (0=정상, 1=반전)
 */
static void _set_wheel(uint16_t             pin_dir,
                       TIM_HandleTypeDef   *tim,
                       uint32_t             ch,
                       int16_t              speed,
                       uint8_t              invert)
{
    /* 범위 클램프 */
    if (speed >  (int16_t)MOTOR_PWM_MAX) speed =  (int16_t)MOTOR_PWM_MAX;
    if (speed < -(int16_t)MOTOR_PWM_MAX) speed = -(int16_t)MOTOR_PWM_MAX;

    uint32_t     pwm = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);
    GPIO_PinState dir;

    if (speed > 0)
    {
        /* 전진: invert=0 → HIGH, invert=1 → LOW */
        dir = invert ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else if (speed < 0)
    {
        /* 후진: invert=0 → LOW, invert=1 → HIGH */
        dir = invert ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    else
    {
        /* 정지 */
        dir = GPIO_PIN_RESET;
        pwm = 0U;
    }

    HAL_GPIO_WritePin(GPIOC, pin_dir, dir);
    __HAL_TIM_SET_COMPARE(tim, ch, pwm);
}

/**
 * @brief 단일 PID 연산 (1 사이클)
 *
 * @param pid          PID 상태 포인터
 * @param measured_rpm 현재 측정 속도 (RPM)
 * @return             PWM 보정량 (RPM 오차 기반)
 */
static float _pid_compute(PID_t *pid, float measured_rpm)
{
    float error      = pid->target_rpm - measured_rpm;

    /* 적분 누적 + Anti-Windup 클램프 */
    pid->integral   += error * PID_PERIOD_S;
    pid->integral    = CLAMP(pid->integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    /* 미분 */
    float derivative = (error - pid->prev_error) / PID_PERIOD_S;
    pid->prev_error  = error;

    return (MOTOR_KP * error)
         + (MOTOR_KI * pid->integral)
         + (MOTOR_KD * derivative);
}

/**
 * @brief 목표 RPM → 피드포워드 PWM + PID 보정 → 모터 출력
 *
 * @param pid         PID 상태 포인터
 * @param measured    현재 측정 RPM
 * @param dir_pin_a   첫 번째 바퀴 DIR 핀
 * @param tim_a       첫 번째 바퀴 타이머
 * @param ch_a        첫 번째 바퀴 채널
 * @param invert_a    첫 번째 바퀴 반전 플래그
 * @param dir_pin_b   두 번째 바퀴 DIR 핀
 * @param tim_b       두 번째 바퀴 타이머
 * @param ch_b        두 번째 바퀴 채널
 * @param invert_b    두 번째 바퀴 반전 플래그
 */
static void _drive_side(PID_t             *pid,
                        float              measured,
                        uint16_t           dir_pin_a,
                        TIM_HandleTypeDef *tim_a,
                        uint32_t           ch_a,
                        uint8_t            invert_a,
                        uint16_t           dir_pin_b,
                        TIM_HandleTypeDef *tim_b,
                        uint32_t           ch_b,
                        uint8_t            invert_b)
{
    if (fabsf(pid->target_rpm) < 0.5f)
    {
        /* ── 목표=0: 즉시 정지 ────────────────────────────── */
        _set_wheel(dir_pin_a, tim_a, ch_a, 0, invert_a);
        _set_wheel(dir_pin_b, tim_b, ch_b, 0, invert_b);
    }
    else
    {
        /* ── 피드포워드(상수 DEADBAND) + PID 보정 ──────────── */
        float pid_correction = _pid_compute(pid, measured);

        /*
         * [복원] 상수 DEADBAND 피드포워드
         *
         * 이전의 "비례 데드밴드(soft-start)" 로직은 수학적으로 타당하지만,
         * 물리적으로 Mecanum 스키드-스티어 로봇에서 치명적 문제 발생:
         *   - 높은 정적 마찰 + 불균일 지면 접촉으로 인해
         *     PWM < DEADBAND_PWM(2000) 구간에서 모터 스톨, 좌우 지터 발생
         *   - DEADBAND_PWM(2000) ≈ 0.195 m/s가 실질적 물리 최소 속도
         *
         * 해결 전략: Nav2 파라미터(min_speed_xy=0.15 m/s)로 미속 명령 차단
         *   → Nav2가 0.15 m/s 이상만 요청하므로 soft-start 불필요
         *   → STM32는 항상 안정 구동 범위(≥DEADBAND_PWM)에서 동작
         */
        float abs_rpm       = fabsf(pid->target_rpm);
        float ff_magnitude  = (float)DEADBAND_PWM + abs_rpm * RPM_TO_PWM_SCALE;
        float ff_with_sign  = (pid->target_rpm > 0.0f) ? ff_magnitude : -ff_magnitude;

        /* 총 출력 = 피드포워드 + PID 보정 */
        float total = ff_with_sign + pid_correction;
        total       = CLAMP(total, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);

        int16_t cmd = (int16_t)total;
        _set_wheel(dir_pin_a, tim_a, ch_a, cmd, invert_a);
        _set_wheel(dir_pin_b, tim_b, ch_b, cmd, invert_b);
    }
}

/* ════════════════════════════════════════════════════════════
 *  공개 API 구현
 * ════════════════════════════════════════════════════════════ */

/**
 * @brief 모듈 초기화 — PID 상태 리셋 + 엔코더 기준값 설정
 *        Motor_Drive() / Motor_PID_Update() 호출 전 반드시 실행
 */
void Motor_Init(void)
{
    memset(&s_pid_left,  0, sizeof(PID_t));
    memset(&s_pid_right, 0, sizeof(PID_t));

    /* 현재 엔코더 카운터를 기준값으로 저장 */
    s_enc_left_last  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    s_enc_right_last = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    s_fb_left_accum  = 0;
    s_fb_right_accum = 0;

    /* 모든 모터 초기 정지 */
    _set_wheel(GPIO_PIN_2, &htim1, TIM_CHANNEL_1, 0, FL_DIR_INVERT);  /* FL */
    _set_wheel(GPIO_PIN_3, &htim1, TIM_CHANNEL_2, 0, FR_DIR_INVERT);  /* FR */
    _set_wheel(GPIO_PIN_0, &htim2, TIM_CHANNEL_1, 0, RL_DIR_INVERT);  /* RL */
    _set_wheel(GPIO_PIN_1, &htim2, TIM_CHANNEL_2, 0, RR_DIR_INVERT);  /* RR */
}

/**
 * @brief CAN 수신값 → 좌/우 PID 목표 속도 설정
 *
 *   좌측 목표 = (rxFL + rxRL) / 2  → RPM 변환
 *   우측 목표 = (rxFR + rxRR) / 2  → RPM 변환
 *
 *   목표가 0으로 전환되면 적분 누적을 즉시 리셋
 *   (빠른 정지 응답 및 오버슈트 방지)
 */
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    /* 좌/우 평균 (스키드-스티어: 동측 바퀴는 항상 동일 속도) */
    int32_t left_avg  = ((int32_t)fl + (int32_t)rl) / 2;
    int32_t right_avg = ((int32_t)fr + (int32_t)rr) / 2;

    /* CAN 범위(-9999~+9999) → RPM (-MOTOR_MAX_RPM ~ +MOTOR_MAX_RPM) */
    s_pid_left.target_rpm  = (float)left_avg  / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;
    s_pid_right.target_rpm = (float)right_avg / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;

    /* 목표=0 전환 시 적분/미분 상태 초기화 */
    if (left_avg == 0) {
        s_pid_left.integral   = 0.0f;
        s_pid_left.prev_error = 0.0f;
    }
    if (right_avg == 0) {
        s_pid_right.integral   = 0.0f;
        s_pid_right.prev_error = 0.0f;
    }
}

/**
 * @brief PID 1 사이클 실행 — 10ms 주기로 main 루프에서 호출
 *
 * 처리 순서:
 *   1. TIM3/TIM4 카운터 읽기
 *   2. 16비트 오버플로우 처리: delta = (int16_t)(current - last)
 *   3. RPM 변환: rpm = delta × 60 / (CPR × PID_PERIOD_S)
 *   4. 좌/우 PID 연산 → PWM + DIR 출력
 *   5. CAN TX 누산기에 틱 추가
 */
void Motor_PID_Update(void)
{
    /* ── 엔코더 카운터 읽기 ──────────────────────────────── */
    int32_t cur_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t cur_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    /*
     * int16_t 캐스트: 16비트 카운터 오버플로우 자동 처리
     *   예) last=65500, cur=50 → delta = (int16_t)(50-65500) = +86 ✓
     */
    int16_t delta_left  = (int16_t)((uint16_t)cur_left  - (uint16_t)s_enc_left_last);
    int16_t delta_right = (int16_t)((uint16_t)cur_right - (uint16_t)s_enc_right_last);

    s_enc_left_last  = cur_left;
    s_enc_right_last = cur_right;

    /* CAN TX 피드백 누산 (20ms마다 전송, 2사이클 누적) */
    s_fb_left_accum  += (int32_t)delta_left;
    s_fb_right_accum += (int32_t)delta_right;

    /* ── RPM 계산 ─────────────────────────────────────────
     *  RPM = (delta_counts / CPR) × (60 / PID_PERIOD_S)
     *      = delta × 60 / (1404 × 0.01) = delta × 4.2735
     * ─────────────────────────────────────────────────── */
    float rpm_scale = 60.0f / ((float)ENCODER_CPR * PID_PERIOD_S);
    float meas_left  = (float)delta_left  * rpm_scale;
    float meas_right = (float)delta_right * rpm_scale;

    /* ── 좌측 PID → FL + RL ──────────────────────────── */
    _drive_side(&s_pid_left,  meas_left,
                GPIO_PIN_2, &htim1, TIM_CHANNEL_1, FL_DIR_INVERT,  /* FL */
                GPIO_PIN_0, &htim2, TIM_CHANNEL_1, RL_DIR_INVERT); /* RL */

    /* ── 우측 PID → FR + RR ──────────────────────────── */
    _drive_side(&s_pid_right, meas_right,
                GPIO_PIN_3, &htim1, TIM_CHANNEL_2, FR_DIR_INVERT,  /* FR */
                GPIO_PIN_1, &htim2, TIM_CHANNEL_2, RR_DIR_INVERT); /* RR */
}

/**
 * @brief 엔코더 피드백 CAN 전송 — 20ms 주기로 main 루프에서 호출
 *
 * CAN ID  : 0x124
 * Payload : 4바이트 Big-Endian
 *   Byte 0-1: int16_t left_ticks  (좌측 누산 엔코더 틱)
 *   Byte 2-3: int16_t right_ticks (우측 누산 엔코더 틱)
 *
 * Raspberry Pi 오도메트리 계산:
 *   distance_m = ticks × (WHEEL_CIRCUMFERENCE_M / ENCODER_CPR)
 *              = ticks × (0.12π / 1404) ≈ ticks × 0.0002685 m
 */
void Motor_Send_Feedback_CAN(void)
{
    CAN_TxHeaderTypeDef tx_hdr  = {0};
    uint8_t             tx_data[4] = {0};
    uint32_t            tx_mailbox;

    /* 누산값 클램프 → int16_t */
    int16_t left_ticks  = (int16_t)CLAMP(s_fb_left_accum,  -32768, 32767);
    int16_t right_ticks = (int16_t)CLAMP(s_fb_right_accum, -32768, 32767);

    /* Big-Endian 직렬화 */
    tx_data[0] = (uint8_t)((left_ticks  >> 8) & 0xFF);
    tx_data[1] = (uint8_t)( left_ticks        & 0xFF);
    tx_data[2] = (uint8_t)((right_ticks >> 8) & 0xFF);
    tx_data[3] = (uint8_t)( right_ticks       & 0xFF);

    tx_hdr.StdId              = CAN_FEEDBACK_ID;
    tx_hdr.IDE                = CAN_ID_STD;
    tx_hdr.RTR                = CAN_RTR_DATA;
    tx_hdr.DLC                = 4U;
    tx_hdr.TransmitGlobalTime = DISABLE;

    /* 누산기 초기화 (전송 성공 여부 무관) */
    s_fb_left_accum  = 0;
    s_fb_right_accum = 0;

    /* CAN TX 요청 (메일박스 여유 없으면 무시됨) */
    (void)HAL_CAN_AddTxMessage(&hcan, &tx_hdr, tx_data, &tx_mailbox);
}

/**
 * @brief 모든 모터 즉시 정지 + PID 상태 리셋
 */
void Motor_Stop(void)
{
    /* PID 상태 완전 초기화 */
    s_pid_left.target_rpm  = 0.0f;
    s_pid_left.integral    = 0.0f;
    s_pid_left.prev_error  = 0.0f;
    s_pid_right.target_rpm = 0.0f;
    s_pid_right.integral   = 0.0f;
    s_pid_right.prev_error = 0.0f;

    /* 4채널 즉시 정지 */
    _set_wheel(GPIO_PIN_2, &htim1, TIM_CHANNEL_1, 0, FL_DIR_INVERT);
    _set_wheel(GPIO_PIN_3, &htim1, TIM_CHANNEL_2, 0, FR_DIR_INVERT);
    _set_wheel(GPIO_PIN_0, &htim2, TIM_CHANNEL_1, 0, RL_DIR_INVERT);
    _set_wheel(GPIO_PIN_1, &htim2, TIM_CHANNEL_2, 0, RR_DIR_INVERT);
}
