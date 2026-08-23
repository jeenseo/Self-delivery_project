/**
 * @file   motor.c
 * @brief  메카넘 4륜 독립 제어 — 단일 모드 (모드 전환 제거)
 *
 * ============================================================================
 * 설계 근거
 * ============================================================================
 *
 * [기존 구조의 문제 — 하이브리드 모드 전환]
 *   구버전은 implied_Vy 로 게걸음을 감지해 두 모드를 오갔습니다.
 *     CLOSED_LOOP : (fl+rl)/2 평균 목표 -> 좌/우 PID -> 앞뒤 같은 PWM
 *     OPEN_LOOP   : 4채널 독립 FF, PID 완전 우회
 *   세 가지가 문제였습니다.
 *     1) 전환 순간 PID 적분이 리셋되고 FF+PID -> FF only 로 바뀌어 PWM 이 튐
 *     2) OPEN_LOOP 구간은 **피드백이 0** — RL 부상으로 생기는 편차를 아무도 못 잡음
 *     3) 임계(500) 근처에서 모드가 떨릴 수 있음
 *   게걸음 중 관측된 '꿀렁임' 은 2번이 주원인입니다. 개루프로 달리니
 *   비대칭(좌측 접지 1바퀴 / 우측 2바퀴)이 그대로 궤적에 나타납니다.
 *
 * [v4 구조 — 단일 모드]
 *   항상 4채널 독립 피드포워드 + 측면 PID 트림.
 *     - 바퀴마다 자기 목표로 FF 계산       -> 게걸음이 성립 (fl 과 rl 부호가 달라도 됨)
 *     - PID 는 엔코더가 있는 앞바퀴에서만 계산
 *     - 그 보정을 같은 쪽 뒷바퀴에 **부호를 맞춰** 적용
 *
 *   [부호 맞춤이 왜 필요한가]
 *     corr 은 "부하를 이기려면 더 필요한 PWM" 입니다.
 *     전진처럼 앞뒤 목표 부호가 같으면 그대로 더하면 되지만,
 *     게걸음은 fl 과 rl 의 부호가 반대이므로 뒷바퀴에는 -corr 을 줘야
 *     '같은 방향으로 더 밀어주는' 의미가 됩니다.
 *
 *   [왜 이 근사가 충분한가]
 *     피드포워드가 PWM 의 약 97% 를 만듭니다(PID 권한 ~1/34).
 *     뒷바퀴가 개루프에 가깝다는 점은 구버전 OPEN_LOOP 와 같지만,
 *     **앞바퀴는 항상 폐루프**라는 점이 결정적으로 다릅니다.
 *
 * [메카넘 기구학 — 검산]
 *   X-구성 정기구학 (바퀴 접지면 속도, l = (track+wheelbase)/2 = 0.505):
 *     v_fl = vx - vy - l*wz      v_fr = vx + vy + l*wz
 *     v_rl = vx + vy - l*wz      v_rr = vx - vy + l*wz
 *
 *   ★ 각 식은 서로 독립입니다. 그래서 이 펌웨어는 역기구학을 **풀지 않습니다.**
 *     ROS 의 motor_node 가 이미 위 식으로 4개 값을 만들어 CAN 으로 보냅니다.
 *     펌웨어는 받은 4개를 각각 RPM 목표로 바꿔 추종하기만 하면 됩니다.
 *     -> 여기서 기구학이 꼬일 여지가 구조적으로 없습니다.
 *
 *   RL 이 떠도 마찬가지입니다. 접지 3륜(FL,FR,RR)은 구속 3개 / 자유도 3개로
 *   det = 4l = 2.020 != 0, 즉 정확히 결정계이며 **IK 공식이 바뀌지 않습니다.**
 *   RL 은 자기 몫의 올바른 속도로 돌다가, 떠 있으면 기여가 없을 뿐입니다.
 *   (구버전은 RL 에 FL 의 PWM 을 복사해 넣어서, RL 이 닿는 순간 충돌했습니다)
 *
 * [타이머 배치]
 *   TIM2 : PWM x4  (CH1=PA0 RL, CH2=PA1 RR, CH3=PB10 FL, CH4=PB11 FR)
 *   TIM3 : FL 엔코더 (PB4,PB5, partial remap)
 *   TIM4 : FR 엔코더 (PB6,PB7)
 *   TIM1 : ★ 해방됨 -> Phase 3 에서 RR 엔코더 (PA8,PA9)
 */

#include "motor.h"
#include "tim.h"
#include "can.h"
#include <string.h>
#include <math.h>

/* ── 외부 핸들 ─────────────────────────────────────────────────────────── */
extern TIM_HandleTypeDef htim2;   /* PWM x4 */
extern TIM_HandleTypeDef htim3;   /* FL 엔코더 */
extern TIM_HandleTypeDef htim4;   /* FR 엔코더 */
#if USE_RR_ENCODER
extern TIM_HandleTypeDef htim1;   /* RR 엔코더 (Phase 3) */
#endif
extern CAN_HandleTypeDef hcan;

/* ── 내부 상수 ─────────────────────────────────────────────────────────── */
#define PID_PERIOD_S        (PID_PERIOD_MS * 0.001f)
#define KINETIC_PWM_SLOPE   ((float)(MOTOR_PWM_MAX - KINETIC_PWM_BASE) \
                             / (MOTOR_MAX_RPM - SMOOTH_RPM))
#define INTEGRAL_LIMIT      (MOTOR_MAX_RPM * 2.0f)
#define CLAMP(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/* ── PID 상태 ──────────────────────────────────────────────────────────── */
typedef struct {
    float target_rpm;
    float integral;
    float prev_error;
} PID_t;

static PID_t s_pid_fl;
static PID_t s_pid_fr;
#if USE_RR_ENCODER
static PID_t s_pid_rr;
#endif

/* ── 4바퀴 명령 (CAN 원시값) ───────────────────────────────────────────── */
static volatile int16_t s_cmd_fl = 0;
static volatile int16_t s_cmd_fr = 0;
static volatile int16_t s_cmd_rl = 0;
static volatile int16_t s_cmd_rr = 0;

/* ── 엔코더 ────────────────────────────────────────────────────────────── */
static int32_t s_enc_fl_last = 0;
static int32_t s_enc_fr_last = 0;
static int32_t s_fb_fl_accum = 0;
static int32_t s_fb_fr_accum = 0;
#if USE_RR_ENCODER
static int32_t s_enc_rr_last = 0;
static int32_t s_fb_rr_accum = 0;
#endif

/* ── 안전 상태 ─────────────────────────────────────────────────────────── */
/* 슬루 제한용 직전 PWM (부호 포함) */
static int16_t  s_pwm_fl = 0, s_pwm_fr = 0, s_pwm_rl = 0, s_pwm_rr = 0;
/* 스톨 감지 타이머 */
static uint32_t s_stall_since_l = 0U;
static uint32_t s_stall_since_r = 0U;
static uint8_t  s_stall_tripped = 0U;
/* 명령 워치독 (volatile: ISR 에서 갱신될 수 있음) */
static volatile uint32_t s_last_cmd_tick     = 0U;
static volatile uint8_t  s_cmd_received_once = 0U;
static uint8_t  s_watchdog_tripped    = 1U;   /* 시작은 '두절' = 페일세이프 */
static uint32_t s_watchdog_trip_count = 0U;
/* PWM 타이머가 시작되기 전에는 레지스터를 만지지 않습니다 */
static uint8_t  s_hw_ready = 0U;

/* ═══════════════════════════════════════════════════════════════════════
 *  내부: DIR + PWM 단일 채널 (Sign-Magnitude). 슬루/상한 없음 = 즉시 반영
 * ═══════════════════════════════════════════════════════════════════════ */
static void _set_wheel_raw(uint16_t pin_dir, uint32_t ch,
                           int16_t speed, uint8_t invert)
{
    if (!s_hw_ready) return;

    if (speed >  (int16_t)MOTOR_PWM_MAX) speed =  (int16_t)MOTOR_PWM_MAX;
    if (speed < -(int16_t)MOTOR_PWM_MAX) speed = -(int16_t)MOTOR_PWM_MAX;

    uint32_t      pwm = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);
    GPIO_PinState dir;

    if      (speed > 0) dir = invert ? GPIO_PIN_RESET : GPIO_PIN_SET;
    else if (speed < 0) dir = invert ? GPIO_PIN_SET   : GPIO_PIN_RESET;
    else                { dir = GPIO_PIN_RESET; pwm = 0U; }

    HAL_GPIO_WritePin(MOTOR_DIR_PORT, pin_dir, dir);
    __HAL_TIM_SET_COMPARE(&htim2, ch, pwm);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  내부: 안전 경로 (절대 상한 -> 슬루 제한 -> 출력)
 *
 *  ★ 정상 주행 명령은 반드시 이 함수를 통과합니다.
 *    비상정지(_all_wheels_off / Motor_Stop)만 _set_wheel_raw 를 직접 씁니다.
 *    "멈추는 것"에 슬루를 걸면 안 되기 때문입니다.
 * ═══════════════════════════════════════════════════════════════════════ */
static void _set_wheel_safe(uint16_t pin_dir, uint32_t ch,
                            int16_t target, uint8_t invert, int16_t *prev)
{
    /* 1) 절대 상한 — 어떤 경로로도 이 값을 못 넘습니다 */
    if (target >  MOTOR_PWM_SAFE_MAX) target =  MOTOR_PWM_SAFE_MAX;
    if (target < -MOTOR_PWM_SAFE_MAX) target = -MOTOR_PWM_SAFE_MAX;

    /* 2) 슬루 제한 — 돌입 전류 차단 */
    int32_t d = (int32_t)target - (int32_t)(*prev);
    if      (d >  MOTOR_PWM_SLEW) target = (int16_t)((int32_t)(*prev) + MOTOR_PWM_SLEW);
    else if (d < -MOTOR_PWM_SLEW) target = (int16_t)((int32_t)(*prev) - MOTOR_PWM_SLEW);

    *prev = target;
    _set_wheel_raw(pin_dir, ch, target, invert);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  내부: 4채널 즉시 0 (워치독 / 스톨 / E-Stop 공용)
 * ═══════════════════════════════════════════════════════════════════════ */
static void _all_wheels_off(void)
{
    _set_wheel_raw(FL_DIR_PIN, FL_PWM_CH, 0, FL_DIR_INVERT);
    _set_wheel_raw(FR_DIR_PIN, FR_PWM_CH, 0, FR_DIR_INVERT);
    _set_wheel_raw(RL_DIR_PIN, RL_PWM_CH, 0, RL_DIR_INVERT);
    _set_wheel_raw(RR_DIR_PIN, RR_PWM_CH, 0, RR_DIR_INVERT);
    /* ★ 슬루 기준점도 함께 0 으로. 안 그러면 재출발 시 0 이 아닌 값에서
     *   슬루가 시작되어 갑자기 큰 PWM 이 나갈 수 있습니다. */
    s_pwm_fl = s_pwm_fr = s_pwm_rl = s_pwm_rr = 0;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  내부: 2단계 피드포워드 (목표 RPM -> PWM, 부호 포함)
 * ═══════════════════════════════════════════════════════════════════════ */
static float _calc_feedforward(float target_rpm)
{
    float abs_rpm = fabsf(target_rpm);
    float ff_mag;

    if (abs_rpm <= SMOOTH_RPM)
    {
        ff_mag = (float)STICTION_PWM_BASE
                 + (abs_rpm / SMOOTH_RPM)
                 * (float)(KINETIC_PWM_BASE - STICTION_PWM_BASE);
    }
    else
    {
        ff_mag = (float)KINETIC_PWM_BASE
                 + (abs_rpm - SMOOTH_RPM) * KINETIC_PWM_SLOPE;
    }
    return (target_rpm >= 0.0f) ? ff_mag : -ff_mag;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  내부: PID 1스텝
 * ═══════════════════════════════════════════════════════════════════════ */
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

static void _pid_reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  ★ 내부: 한 쪽(좌 또는 우) 앞/뒤 2바퀴를 독립 목표로 구동
 *
 *  앞바퀴 = 엔코더 있음 -> FF + PID
 *  뒷바퀴 = 엔코더 없음 -> FF + (같은 쪽 PID 보정을 부호 맞춰 적용)
 *
 *  [검산] 게걸음 +vy 일 때
 *    t_front(FL) = -a,  t_rear(RL) = +a
 *    FF(-a) = -|ff|,  FF(+a) = +|ff|          -> 앞뒤가 반대로 돈다 ✓
 *    corr 이 +c 라면 뒷바퀴에는 -c 를 적용
 *      cmd_f = -|ff| + c   (앞: 목표 -a 방향으로 c 만큼 덜 밈 = 부하 보상)
 *      cmd_r = +|ff| - c   (뒤: 목표 +a 방향으로 c 만큼 덜 밈 = 대칭)
 *    -> 두 바퀴가 '같은 물리적 의미' 의 보정을 받습니다 ✓
 * ═══════════════════════════════════════════════════════════════════════ */
static void _drive_pair(PID_t *pid, float measured_rpm,
                        float t_front, uint16_t pin_f, uint32_t ch_f,
                        uint8_t inv_f, int16_t *prev_f,
                        float t_rear,  uint16_t pin_r, uint32_t ch_r,
                        uint8_t inv_r, int16_t *prev_r)
{
    pid->target_rpm = t_front;

    float corr = 0.0f;
    if (fabsf(t_front) >= 0.5f)
    {
        corr = _pid_compute(pid, measured_rpm);
    }
    else
    {
        /* 목표가 0 이면 적분을 비웁니다. 안 그러면 정지 중에 쌓입니다. */
        _pid_reset(pid);
    }

    /* 뒷바퀴 보정 부호 맞춤 (위 검산 참조).
     * t_front 가 0 이면 corr 도 0 이므로 곱 판정은 안전합니다. */
    float corr_r = ((t_front * t_rear) >= 0.0f) ? corr : -corr;

    int16_t cmd_f = 0;
    int16_t cmd_r = 0;

    if (fabsf(t_front) >= 0.5f)
        cmd_f = (int16_t)CLAMP(_calc_feedforward(t_front) + corr,
                               -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    if (fabsf(t_rear) >= 0.5f)
        cmd_r = (int16_t)CLAMP(_calc_feedforward(t_rear) + corr_r,
                               -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);

    _set_wheel_safe(pin_f, ch_f, cmd_f, inv_f, prev_f);
    _set_wheel_safe(pin_r, ch_r, cmd_r, inv_r, prev_r);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  공개 API
 * ═══════════════════════════════════════════════════════════════════════ */

void Motor_Init(void)
{
    _pid_reset(&s_pid_fl);
    _pid_reset(&s_pid_fr);
    s_pid_fl.target_rpm = 0.0f;
    s_pid_fr.target_rpm = 0.0f;
#if USE_RR_ENCODER
    _pid_reset(&s_pid_rr);
    s_pid_rr.target_rpm = 0.0f;
#endif

    s_cmd_fl = s_cmd_fr = s_cmd_rl = s_cmd_rr = 0;
    s_pwm_fl = s_pwm_fr = s_pwm_rl = s_pwm_rr = 0;

    s_enc_fl_last = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    s_enc_fr_last = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    s_fb_fl_accum = s_fb_fr_accum = 0;
#if USE_RR_ENCODER
    s_enc_rr_last = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
    s_fb_rr_accum = 0;
#endif

    s_stall_since_l = s_stall_since_r = 0U;
    s_stall_tripped = 0U;

    /* 첫 CAN 명령을 받기 전에는 어떤 경우에도 PWM 이 나가지 않습니다 */
    s_last_cmd_tick       = 0U;
    s_cmd_received_once   = 0U;
    s_watchdog_tripped    = 1U;
    s_watchdog_trip_count = 0U;

    /* 이 시점 이후로만 PWM/GPIO 접근이 안전합니다 */
    s_hw_ready = 1U;
    _all_wheels_off();
}

/**
 * @brief CAN 수신 -> 4바퀴 목표 저장. **평균 내지 않습니다.**
 *
 * 구버전은 여기서 implied_Vy 를 계산해 모드를 전환했습니다. v4 는 하지 않습니다.
 * 모드 개념 자체를 없앴으므로 이 함수는 값 저장 + 워치독 갱신만 합니다.
 */
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    s_cmd_fl = fl;
    s_cmd_fr = fr;
    s_cmd_rl = rl;
    s_cmd_rr = rr;

    s_last_cmd_tick     = HAL_GetTick();
    s_cmd_received_once = 1U;
}

void Motor_PID_Update(void)
{
    /* ── [1] 엔코더 읽기 + 누산 (워치독 상태와 무관하게 항상) ──────────
     *   통신이 끊겨도 로봇은 관성으로 굴러갑니다. 그 이동량을 놓치면
     *   복구 후 오도메트리에 점프가 생겨 EKF 가 흔들립니다.
     *   정지시키는 것과 계측하는 것은 별개입니다. */
    int32_t cur_fl = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t cur_fr = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    int16_t d_fl = (int16_t)((uint16_t)cur_fl - (uint16_t)s_enc_fl_last);
    int16_t d_fr = (int16_t)((uint16_t)cur_fr - (uint16_t)s_enc_fr_last);
    s_enc_fl_last = cur_fl;
    s_enc_fr_last = cur_fr;
    s_fb_fl_accum += (int32_t)d_fl;
    s_fb_fr_accum += (int32_t)d_fr;

    const float rpm_scale = 60.0f / ((float)ENCODER_CPR * PID_PERIOD_S);
    float meas_fl = (float)d_fl * rpm_scale;
    float meas_fr = (float)d_fr * rpm_scale;

#if USE_RR_ENCODER
    int32_t cur_rr = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t d_rr   = (int16_t)((uint16_t)cur_rr - (uint16_t)s_enc_rr_last);
    s_enc_rr_last  = cur_rr;
    s_fb_rr_accum += (int32_t)d_rr;
    float meas_rr  = (float)d_rr * rpm_scale;
#endif

    /* ── [2] 명령 워치독 ────────────────────────────────────────────────
     *   volatile 은 한 번만 읽어 지역변수로 복사합니다.
     *   판정 도중 ISR 이 값을 바꿔도 판정 일관성이 깨지지 않도록. */
    uint32_t now       = HAL_GetTick();
    uint32_t last_tick = s_last_cmd_tick;
    uint8_t  ever      = s_cmd_received_once;

    uint8_t stale = (!ever) || ((now - last_tick) > CMD_TIMEOUT_MS);

    if (stale)
    {
        if (!s_watchdog_tripped)
        {
            s_watchdog_tripped = 1U;
            s_watchdog_trip_count++;
        }
        s_cmd_fl = s_cmd_fr = s_cmd_rl = s_cmd_rr = 0;
        _pid_reset(&s_pid_fl);
        _pid_reset(&s_pid_fr);
#if USE_RR_ENCODER
        _pid_reset(&s_pid_rr);
#endif
        _all_wheels_off();
        return;
    }
    s_watchdog_tripped = 0U;

    /* ── [3] 스톨 감지 ──────────────────────────────────────────────────
     *   PWM 은 크게 나가는데 바퀴가 안 도는 상태 = 물림 / 과부하.
     *   그대로 두면 모터와 드라이버가 열로 상합니다. */
    int16_t apwm_l = (s_pwm_fl >= 0) ? s_pwm_fl : (int16_t)(-s_pwm_fl);
    int16_t apwm_r = (s_pwm_fr >= 0) ? s_pwm_fr : (int16_t)(-s_pwm_fr);

    if ((apwm_l > STALL_PWM_THRESH) && (fabsf(meas_fl) < STALL_RPM_THRESH))
    {
        if (s_stall_since_l == 0U) s_stall_since_l = now;
    }
    else s_stall_since_l = 0U;

    if ((apwm_r > STALL_PWM_THRESH) && (fabsf(meas_fr) < STALL_RPM_THRESH))
    {
        if (s_stall_since_r == 0U) s_stall_since_r = now;
    }
    else s_stall_since_r = 0U;

    uint8_t stall_l = (s_stall_since_l != 0U) && ((now - s_stall_since_l) > STALL_HOLD_MS);
    uint8_t stall_r = (s_stall_since_r != 0U) && ((now - s_stall_since_r) > STALL_HOLD_MS);

    if (stall_l || stall_r)
    {
        s_stall_tripped = 1U;
        _pid_reset(&s_pid_fl);
        _pid_reset(&s_pid_fr);
#if USE_RR_ENCODER
        _pid_reset(&s_pid_rr);
#endif
        _all_wheels_off();
        /* 명령이 0 으로 돌아오면 자동 해제됩니다(아래 [4] 진입 시). */
        return;
    }

    /* ── [4] 4채널 독립 목표 -> 구동 ────────────────────────────────────
     *   ★ 평균 없음. 모드 전환 없음. 바퀴마다 자기 목표. */
    const float k = MOTOR_MAX_RPM / (float)MOTOR_PWM_MAX;
    float t_fl = (float)s_cmd_fl * k;
    float t_fr = (float)s_cmd_fr * k;
    float t_rl = (float)s_cmd_rl * k;
    float t_rr = (float)s_cmd_rr * k;

    /* 전 바퀴 목표가 0 이면 스톨 래치를 해제합니다 */
    if (s_stall_tripped &&
        (fabsf(t_fl) < 0.5f) && (fabsf(t_fr) < 0.5f) &&
        (fabsf(t_rl) < 0.5f) && (fabsf(t_rr) < 0.5f))
    {
        s_stall_tripped = 0U;
        s_stall_since_l = s_stall_since_r = 0U;
    }
    if (s_stall_tripped) { _all_wheels_off(); return; }

    /* 좌측: 앞 FL(엔코더) + 뒤 RL */
    _drive_pair(&s_pid_fl, meas_fl,
                t_fl, FL_DIR_PIN, FL_PWM_CH, FL_DIR_INVERT, &s_pwm_fl,
                t_rl, RL_DIR_PIN, RL_PWM_CH, RL_DIR_INVERT, &s_pwm_rl);

#if USE_RR_ENCODER
    /* Phase 3: RR 도 자기 엔코더로 독립 폐루프.
     *   FR 은 단독 폐루프(뒤에 딸린 바퀴 없음)이므로 t_rear 에 자기 자신을 넣어
     *   같은 함수를 재사용하지 않고 개별 처리합니다. */
    {
        s_pid_fr.target_rpm = t_fr;
        float corr_fr = 0.0f;
        if (fabsf(t_fr) >= 0.5f) corr_fr = _pid_compute(&s_pid_fr, meas_fr);
        else                     _pid_reset(&s_pid_fr);
        int16_t cmd_fr = 0;
        if (fabsf(t_fr) >= 0.5f)
            cmd_fr = (int16_t)CLAMP(_calc_feedforward(t_fr) + corr_fr,
                                    -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
        _set_wheel_safe(FR_DIR_PIN, FR_PWM_CH, cmd_fr, FR_DIR_INVERT, &s_pwm_fr);

        s_pid_rr.target_rpm = t_rr;
        float corr_rr = 0.0f;
        if (fabsf(t_rr) >= 0.5f) corr_rr = _pid_compute(&s_pid_rr, meas_rr);
        else                     _pid_reset(&s_pid_rr);
        int16_t cmd_rr = 0;
        if (fabsf(t_rr) >= 0.5f)
            cmd_rr = (int16_t)CLAMP(_calc_feedforward(t_rr) + corr_rr,
                                    -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
        _set_wheel_safe(RR_DIR_PIN, RR_PWM_CH, cmd_rr, RR_DIR_INVERT, &s_pwm_rr);
    }
#else
    /* 우측: 앞 FR(엔코더) + 뒤 RR */
    _drive_pair(&s_pid_fr, meas_fr,
                t_fr, FR_DIR_PIN, FR_PWM_CH, FR_DIR_INVERT, &s_pwm_fr,
                t_rr, RR_DIR_PIN, RR_PWM_CH, RR_DIR_INVERT, &s_pwm_rr);
#endif
}

/* ═══════════════════════════════════════════════════════════════════════
 *  CAN 피드백
 *
 *  ★ DLC 는 4(2채널) 또는 6(3채널). **8 을 쓰지 마십시오.**
 *    SocketCAN 에러 프레임이 항상 DLC=8 이라, ROS 측의
 *    "DLC 를 정확히 N 으로 요구" 방어층이 무력화됩니다.
 * ═══════════════════════════════════════════════════════════════════════ */
void Motor_Send_Feedback_CAN(void)
{
    CAN_TxHeaderTypeDef tx_hdr = {0};
    uint32_t            tx_mailbox;

#if USE_RR_ENCODER
    uint8_t tx_data[6] = {0};
    const uint32_t dlc = 6U;
#else
    uint8_t tx_data[4] = {0};
    const uint32_t dlc = 4U;
#endif

    int16_t t_fl = (int16_t)CLAMP(s_fb_fl_accum, -32768, 32767);
    int16_t t_fr = (int16_t)CLAMP(s_fb_fr_accum, -32768, 32767);

    tx_data[0] = (uint8_t)((t_fl >> 8) & 0xFF);
    tx_data[1] = (uint8_t)( t_fl       & 0xFF);
    tx_data[2] = (uint8_t)((t_fr >> 8) & 0xFF);
    tx_data[3] = (uint8_t)( t_fr       & 0xFF);
#if USE_RR_ENCODER
    int16_t t_rr = (int16_t)CLAMP(s_fb_rr_accum, -32768, 32767);
    tx_data[4] = (uint8_t)((t_rr >> 8) & 0xFF);
    tx_data[5] = (uint8_t)( t_rr       & 0xFF);
#endif

    tx_hdr.StdId              = CAN_FEEDBACK_ID;
    tx_hdr.IDE                = CAN_ID_STD;
    tx_hdr.RTR                = CAN_RTR_DATA;
    tx_hdr.DLC                = dlc;
    tx_hdr.TransmitGlobalTime = DISABLE;

    /* ★ 누산기는 '패킹 직후' 비웁니다.
     *   AddTxMessage 가 실패해도 비우는 것이 맞습니다 — 재전송하면 같은 틱을
     *   두 번 보내게 되어 오도메트리가 부풀려집니다. 손실이 중복보다 낫습니다. */
    s_fb_fl_accum = 0;
    s_fb_fr_accum = 0;
#if USE_RR_ENCODER
    s_fb_rr_accum = 0;
#endif

    (void)HAL_CAN_AddTxMessage(&hcan, &tx_hdr, tx_data, &tx_mailbox);
}

void Motor_Stop(void)
{
    s_cmd_fl = s_cmd_fr = s_cmd_rl = s_cmd_rr = 0;
    _pid_reset(&s_pid_fl);
    _pid_reset(&s_pid_fr);
#if USE_RR_ENCODER
    _pid_reset(&s_pid_rr);
#endif
    _all_wheels_off();
}

uint8_t Motor_Watchdog_IsTripped(void) { return s_watchdog_tripped; }
uint8_t Motor_Stall_IsTripped(void)    { return s_stall_tripped;    }