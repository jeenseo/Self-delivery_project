/**
 * @file   motor.c
 * @brief  STM32 메카넘 4-휠 모터 제어 — 하이브리드 PID/Open-Loop
 *
 * [아키텍처: 하이브리드 제어 전략]
 *
 *   2개 엔코더(TIM3=Left, TIM4=Right)로 메카넘 4-휠을 제어하는 근본적 문제:
 *   스트레이핑(Vy) 시 FL↑RL↓ 또는 FL↓RL↑ → 좌측 엔코더 평균 ≈ 0 → PID 오작동
 *
 *   해결: 스트레이핑 여부를 감지하여 두 모드로 동적 전환
 *
 *   [CTRL_CLOSED_LOOP] 순수 X/Yaw 이동:
 *     - 기존 좌/우 평균 PID 유지
 *     - left_avg_target  = (FL + RL) / 2
 *     - right_avg_target = (FR + RR) / 2
 *
 *   [CTRL_OPEN_LOOP] 스트레이핑 감지:
 *     - 4바퀴 독립 2단계 피드포워드 (PID 우회)
 *     - PID 적분 리셋으로 윈드업 방지
 *
 * [스트레이핑 감지 수식]
 *   메카넘 X-구성에서:
 *     FL = Vx - Vy - Wz·l,  FR = Vx + Vy + Wz·l
 *     RL = Vx + Vy - Wz·l,  RR = Vx - Vy + Wz·l
 *   implied_Vy = (-FL + FR + RL - RR) / 4  (정규화 공간)
 *   → Vx, Wz 항은 소거되고 Vy만 남음
 *   → |implied_Vy| > STRAFE_DETECT_THRESHOLD → CTRL_OPEN_LOOP
 *
 * [2단계 피드포워드]
 *   Zone 1 (0 < RPM ≤ SMOOTH_RPM): PWM 1300→2000 (스티션 극복)
 *   Zone 2 (RPM > SMOOTH_RPM):     PWM 2000→9999 (동역학 구간)
 */

#include "motor.h"
#include "tim.h"
#include "can.h"
#include <string.h>
#include <math.h>

/* ── 외부 핸들 ───────────────────────────────────────────────── */
extern TIM_HandleTypeDef htim1;   /* FL(CH1)/FR(CH2) PWM */
extern TIM_HandleTypeDef htim2;   /* RL(CH1)/RR(CH2) PWM */
extern TIM_HandleTypeDef htim3;   /* 좌측 엔코더 */
extern TIM_HandleTypeDef htim4;   /* 우측 엔코더 */
extern CAN_HandleTypeDef hcan;    /* CAN1 */

/* ── 내부 상수 ───────────────────────────────────────────────── */
#define PID_PERIOD_S        (PID_PERIOD_MS * 0.001f)

/* Zone 2 기울기: 2000→9999 / (MAX_RPM - SMOOTH_RPM) */
#define KINETIC_PWM_SLOPE   ((float)(MOTOR_PWM_MAX - KINETIC_PWM_BASE) \
                             / (MOTOR_MAX_RPM - SMOOTH_RPM))

/* 스트레이핑 감지 임계값 (정규화 속도 기준, 5% = 500/9999) */
#define STRAFE_DETECT_THRESHOLD  500.0f

/* 클램프 */
#define CLAMP(x, lo, hi)    ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))

/* 적분 Anti-Windup 한계 */
#define INTEGRAL_LIMIT      (MOTOR_MAX_RPM * 2.0f)

/* ── 제어 상태 열거형 ─────────────────────────────────────────── */
typedef enum {
    CTRL_CLOSED_LOOP = 0,  /**< PID 제어: 순수 X/Yaw 이동          */
    CTRL_OPEN_LOOP   = 1,  /**< 피드포워드: 스트레이핑 감지 시 사용  */
} ControlState_t;

/* ── PID 상태 구조체 ─────────────────────────────────────────── */
typedef struct {
    float target_rpm;
    float integral;
    float prev_error;
} PID_t;

/* ── 모듈 내부 상태 ──────────────────────────────────────────── */
static PID_t          s_pid_left;
static PID_t          s_pid_right;
static ControlState_t s_ctrl_state = CTRL_CLOSED_LOOP;

/* 4바퀴 독립 명령 (CAN 수신값 그대로 저장) */
static int16_t s_cmd_fl = 0;
static int16_t s_cmd_fr = 0;
static int16_t s_cmd_rl = 0;
static int16_t s_cmd_rr = 0;

/* 엔코더 카운터 */
static int32_t s_enc_left_last  = 0;
static int32_t s_enc_right_last = 0;

/* CAN TX 누산기 */
static int32_t s_fb_left_accum  = 0;
static int32_t s_fb_right_accum = 0;

/* ════════════════════════════════════════════════════════════════
 *  내부 헬퍼: DIR+PWM 단일 채널 설정 (Sign-Magnitude)
 * ════════════════════════════════════════════════════════════════ */
static void _set_wheel(uint16_t           pin_dir,
                       TIM_HandleTypeDef *tim,
                       uint32_t           ch,
                       int16_t            speed,
                       uint8_t            invert)
{
    if (speed >  (int16_t)MOTOR_PWM_MAX) speed =  (int16_t)MOTOR_PWM_MAX;
    if (speed < -(int16_t)MOTOR_PWM_MAX) speed = -(int16_t)MOTOR_PWM_MAX;

    uint32_t      pwm = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);
    GPIO_PinState dir;

    if      (speed > 0) dir = invert ? GPIO_PIN_RESET : GPIO_PIN_SET;
    else if (speed < 0) dir = invert ? GPIO_PIN_SET   : GPIO_PIN_RESET;
    else { dir = GPIO_PIN_RESET; pwm = 0U; }

    HAL_GPIO_WritePin(GPIOC, pin_dir, dir);
    __HAL_TIM_SET_COMPARE(tim, ch, pwm);
}

/* ════════════════════════════════════════════════════════════════
 *  내부 헬퍼: 2단계 피드포워드 PWM 계산
 * ════════════════════════════════════════════════════════════════ */
/**
 * @brief 목표 RPM → 2단계 피드포워드 PWM (부호 포함)
 *
 * Zone 1 (스티션): |RPM| ≤ SMOOTH_RPM → PWM 1300~2000 선형
 * Zone 2 (동역학): |RPM| >  SMOOTH_RPM → PWM 2000~9999 선형
 * 연결점: Zone1(SMOOTH_RPM) = Zone2(SMOOTH_RPM) = 2000 ✓
 */
static float _calc_feedforward(float target_rpm)
{
    float abs_rpm = fabsf(target_rpm);
    float ff_mag;

    if (abs_rpm <= SMOOTH_RPM)
    {
        /* Zone 1: 스티션 극복 (PWM 1300 → 2000) */
        ff_mag = (float)STICTION_PWM_BASE
                 + (abs_rpm / SMOOTH_RPM)
                 * (float)(KINETIC_PWM_BASE - STICTION_PWM_BASE);
    }
    else
    {
        /* Zone 2: 동역학 구간 (PWM 2000 → 9999) */
        ff_mag = (float)KINETIC_PWM_BASE
                 + (abs_rpm - SMOOTH_RPM) * KINETIC_PWM_SLOPE;
    }

    return (target_rpm >= 0.0f) ? ff_mag : -ff_mag;
}

/* ════════════════════════════════════════════════════════════════
 *  내부 헬퍼: Open-Loop 단일 채널 구동 (PID 없이 FF만)
 * ════════════════════════════════════════════════════════════════ */
static void _drive_wheel_open_loop(uint16_t           pin_dir,
                                    TIM_HandleTypeDef *tim,
                                    uint32_t           ch,
                                    float              target_rpm,
                                    uint8_t            invert)
{
    if (fabsf(target_rpm) < 0.5f)
    {
        _set_wheel(pin_dir, tim, ch, 0, invert);
        return;
    }
    float ff    = _calc_feedforward(target_rpm);
    float total = CLAMP(ff, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    _set_wheel(pin_dir, tim, ch, (int16_t)total, invert);
}

/* ════════════════════════════════════════════════════════════════
 *  내부 헬퍼: PID 연산
 * ════════════════════════════════════════════════════════════════ */
static float _pid_compute(PID_t *pid, float measured_rpm)
{
    float error = pid->target_rpm - measured_rpm;

    pid->integral += error * PID_PERIOD_S;
    pid->integral  = CLAMP(pid->integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float derivative = (error - pid->prev_error) / PID_PERIOD_S;
    pid->prev_error  = error;

    return (MOTOR_KP * error)
         + (MOTOR_KI * pid->integral)
         + (MOTOR_KD * derivative);
}

/* ════════════════════════════════════════════════════════════════
 *  내부 헬퍼: Closed-Loop 한 쪽(좌 또는 우) 구동
 *   — 두 바퀴가 공통 PID를 공유 (엔코더 평균 기반)
 * ════════════════════════════════════════════════════════════════ */
static void _drive_side_closed(PID_t             *pid,
                                float              measured,
                                uint16_t           pin_a,
                                TIM_HandleTypeDef *tim_a,
                                uint32_t           ch_a,
                                uint8_t            inv_a,
                                uint16_t           pin_b,
                                TIM_HandleTypeDef *tim_b,
                                uint32_t           ch_b,
                                uint8_t            inv_b)
{
    if (fabsf(pid->target_rpm) < 0.5f)
    {
        _set_wheel(pin_a, tim_a, ch_a, 0, inv_a);
        _set_wheel(pin_b, tim_b, ch_b, 0, inv_b);
        return;
    }

    float pid_correction = _pid_compute(pid, measured);
    float ff             = _calc_feedforward(pid->target_rpm);
    float total          = CLAMP(ff + pid_correction,
                                 -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    int16_t cmd          = (int16_t)total;

    _set_wheel(pin_a, tim_a, ch_a, cmd, inv_a);
    _set_wheel(pin_b, tim_b, ch_b, cmd, inv_b);
}

/* ════════════════════════════════════════════════════════════════
 *  공개 API
 * ════════════════════════════════════════════════════════════════ */

void Motor_Init(void)
{
    memset(&s_pid_left,  0, sizeof(PID_t));
    memset(&s_pid_right, 0, sizeof(PID_t));

    s_cmd_fl = s_cmd_fr = s_cmd_rl = s_cmd_rr = 0;
    s_ctrl_state = CTRL_CLOSED_LOOP;

    s_enc_left_last  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    s_enc_right_last = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    s_fb_left_accum  = 0;
    s_fb_right_accum = 0;

    _set_wheel(GPIO_PIN_2, &htim1, TIM_CHANNEL_1, 0, FL_DIR_INVERT);
    _set_wheel(GPIO_PIN_3, &htim1, TIM_CHANNEL_2, 0, FR_DIR_INVERT);
    _set_wheel(GPIO_PIN_0, &htim2, TIM_CHANNEL_1, 0, RL_DIR_INVERT);
    _set_wheel(GPIO_PIN_1, &htim2, TIM_CHANNEL_2, 0, RR_DIR_INVERT);
}

/**
 * @brief CAN 수신 → 4바퀴 독립 목표 저장 + 스트레이핑 감지
 *
 * 스트레이핑 감지 수식:
 *   implied_Vy = (-FL + FR + RL - RR) / 4  (정규화 CAN 값 기준)
 *   Vx, Wz 항은 소거되고 Vy 성분만 추출됨
 *   |implied_Vy| > STRAFE_DETECT_THRESHOLD → CTRL_OPEN_LOOP
 */
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    s_cmd_fl = fl;
    s_cmd_fr = fr;
    s_cmd_rl = rl;
    s_cmd_rr = rr;

    /* ── 스트레이핑 감지 ─────────────────────────────────────── */
    /* 메카넘 X-구성: implied_Vy = (-FL + FR + RL - RR) / 4      */
    float implied_vy = (float)(-fl + fr + rl - rr) / 4.0f;

    ControlState_t new_state =
        (fabsf(implied_vy) > STRAFE_DETECT_THRESHOLD)
        ? CTRL_OPEN_LOOP
        : CTRL_CLOSED_LOOP;

    if (new_state != s_ctrl_state)
    {
        /* ── 상태 전환: PID 적분 초기화 (윈드업 방지) ─────────── */
        s_pid_left.integral   = 0.0f;
        s_pid_left.prev_error = 0.0f;
        s_pid_right.integral  = 0.0f;
        s_pid_right.prev_error = 0.0f;
        s_ctrl_state = new_state;
    }

    /* ── Closed-Loop 시 PID 목표 설정 ───────────────────────── */
    if (s_ctrl_state == CTRL_CLOSED_LOOP)
    {
        /* 좌/우 평균으로 엔코더 기반 PID (Vy 소거 원리 활용) */
        int32_t left_avg  = ((int32_t)fl + (int32_t)rl) / 2;
        int32_t right_avg = ((int32_t)fr + (int32_t)rr) / 2;

        s_pid_left.target_rpm  = (float)left_avg
                                 / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;
        s_pid_right.target_rpm = (float)right_avg
                                 / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;

        if (left_avg == 0)
        {
            s_pid_left.integral   = 0.0f;
            s_pid_left.prev_error = 0.0f;
        }
        if (right_avg == 0)
        {
            s_pid_right.integral   = 0.0f;
            s_pid_right.prev_error = 0.0f;
        }
    }
}

/**
 * @brief PID 1 사이클 실행 (10ms 주기)
 *
 * CTRL_CLOSED_LOOP: 좌/우 평균 PID (기존 방식, 엔코더 신뢰 가능)
 * CTRL_OPEN_LOOP:   4바퀴 독립 2단계 피드포워드 (PID 우회)
 */
void Motor_PID_Update(void)
{
    /* ── 엔코더 읽기 ─────────────────────────────────────────── */
    int32_t cur_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t cur_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    int16_t delta_left  = (int16_t)((uint16_t)cur_left  - (uint16_t)s_enc_left_last);
    int16_t delta_right = (int16_t)((uint16_t)cur_right - (uint16_t)s_enc_right_last);

    s_enc_left_last  = cur_left;
    s_enc_right_last = cur_right;

    s_fb_left_accum  += (int32_t)delta_left;
    s_fb_right_accum += (int32_t)delta_right;

    /* ── RPM 계산 ────────────────────────────────────────────── */
    float rpm_scale   = 60.0f / ((float)ENCODER_CPR * PID_PERIOD_S);
    float meas_left   = (float)delta_left  * rpm_scale;
    float meas_right  = (float)delta_right * rpm_scale;

    /* ════════════════════════════════════════════════════════════
     *  제어 모드 분기
     * ════════════════════════════════════════════════════════════ */
    if (s_ctrl_state == CTRL_CLOSED_LOOP)
    {
        /* ── Closed-Loop: 좌/우 평균 PID ──────────────────────── */
        _drive_side_closed(&s_pid_left,  meas_left,
                           GPIO_PIN_2, &htim1, TIM_CHANNEL_1, FL_DIR_INVERT,
                           GPIO_PIN_0, &htim2, TIM_CHANNEL_1, RL_DIR_INVERT);

        _drive_side_closed(&s_pid_right, meas_right,
                           GPIO_PIN_3, &htim1, TIM_CHANNEL_2, FR_DIR_INVERT,
                           GPIO_PIN_1, &htim2, TIM_CHANNEL_2, RR_DIR_INVERT);
    }
    else
    {
        /* ── Open-Loop: 4바퀴 독립 피드포워드 ─────────────────── */
        /* 각 바퀴의 CAN 명령을 RPM으로 변환 후 FF 적용           */
        float fl_rpm = (float)s_cmd_fl / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;
        float fr_rpm = (float)s_cmd_fr / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;
        float rl_rpm = (float)s_cmd_rl / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;
        float rr_rpm = (float)s_cmd_rr / (float)MOTOR_PWM_MAX * MOTOR_MAX_RPM;

        _drive_wheel_open_loop(GPIO_PIN_2, &htim1, TIM_CHANNEL_1,
                               fl_rpm, FL_DIR_INVERT);    /* FL */
        _drive_wheel_open_loop(GPIO_PIN_3, &htim1, TIM_CHANNEL_2,
                               fr_rpm, FR_DIR_INVERT);    /* FR */
        _drive_wheel_open_loop(GPIO_PIN_0, &htim2, TIM_CHANNEL_1,
                               rl_rpm, RL_DIR_INVERT);    /* RL */
        _drive_wheel_open_loop(GPIO_PIN_1, &htim2, TIM_CHANNEL_2,
                               rr_rpm, RR_DIR_INVERT);    /* RR */
    }
}

/**
 * @brief 엔코더 피드백 CAN 전송 (20ms 주기)
 *
 * CAN ID 0x124, Payload 4B Big-Endian:
 *   Byte 0-1: int16 left_ticks
 *   Byte 2-3: int16 right_ticks
 *
 * 하이브리드 오도메트리 수학:
 *   Pi에서 Left/Right 평균이 Vy 항을 소거하므로
 *   X 및 Yaw는 2-엔코더로 정확하게 추정 가능.
 */
void Motor_Send_Feedback_CAN(void)
{
    CAN_TxHeaderTypeDef tx_hdr = {0};
    uint8_t             tx_data[4] = {0};
    uint32_t            tx_mailbox;

    int16_t lt = (int16_t)CLAMP(s_fb_left_accum,  -32768, 32767);
    int16_t rt = (int16_t)CLAMP(s_fb_right_accum, -32768, 32767);

    tx_data[0] = (uint8_t)((lt >> 8) & 0xFF);
    tx_data[1] = (uint8_t)( lt       & 0xFF);
    tx_data[2] = (uint8_t)((rt >> 8) & 0xFF);
    tx_data[3] = (uint8_t)( rt       & 0xFF);

    tx_hdr.StdId              = CAN_FEEDBACK_ID;
    tx_hdr.IDE                = CAN_ID_STD;
    tx_hdr.RTR                = CAN_RTR_DATA;
    tx_hdr.DLC                = 4U;
    tx_hdr.TransmitGlobalTime = DISABLE;

    s_fb_left_accum  = 0;
    s_fb_right_accum = 0;

    (void)HAL_CAN_AddTxMessage(&hcan, &tx_hdr, tx_data, &tx_mailbox);
}

void Motor_Stop(void)
{
    s_pid_left.target_rpm  = 0.0f;
    s_pid_left.integral    = 0.0f;
    s_pid_left.prev_error  = 0.0f;
    s_pid_right.target_rpm = 0.0f;
    s_pid_right.integral   = 0.0f;
    s_pid_right.prev_error = 0.0f;

    s_cmd_fl = s_cmd_fr = s_cmd_rl = s_cmd_rr = 0;
    s_ctrl_state = CTRL_CLOSED_LOOP;

    _set_wheel(GPIO_PIN_2, &htim1, TIM_CHANNEL_1, 0, FL_DIR_INVERT);
    _set_wheel(GPIO_PIN_3, &htim1, TIM_CHANNEL_2, 0, FR_DIR_INVERT);
    _set_wheel(GPIO_PIN_0, &htim2, TIM_CHANNEL_1, 0, RL_DIR_INVERT);
    _set_wheel(GPIO_PIN_1, &htim2, TIM_CHANNEL_2, 0, RR_DIR_INVERT);
}
