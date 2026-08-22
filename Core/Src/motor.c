/**
 * @file   motor.c
 * @brief  STM32 메카넘 4-휠 모터 제어 — 하이브리드 PID/Open-Loop
 *         ★ v2: 명령 워치독(Command Watchdog) 추가
 *
 * [아키텍처: 하이브리드 제어 전략]  ※ v1 그대로 유지
 *
 *   2개 엔코더(TIM3=Left, TIM4=Right)로 메카넘 4-휠을 제어하는 근본적 문제:
 *   스트레이핑(Vy) 시 FL↑RL↓ 또는 FL↓RL↑ → 좌측 엔코더 평균 ≈ 0 → PID 오작동
 *
 *   해결: 스트레이핑 여부를 감지하여 두 모드로 동적 전환
 *     [CTRL_CLOSED_LOOP] 순수 X/Yaw 이동 : 좌/우 평균 PID
 *     [CTRL_OPEN_LOOP]   스트레이핑 감지 : 4바퀴 독립 2단계 피드포워드
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ★ v2 변경점 — 명령 워치독
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   [폭주(Runaway)의 메커니즘]
 *     PWM 레지스터(__HAL_TIM_SET_COMPARE)는 한 번 쓰면 다시 쓸 때까지 그 값을
 *     유지합니다. 즉 명령 소스(Pi)가 사라져도 모터는 마지막 듀티로 계속 돕니다.
 *     "정지 명령이 오지 않으면 정지한다"가 아니라 "정지 명령이 와야 정지한다"가
 *     기존 동작이었고, 이것이 정확히 폭주의 정체입니다.
 *
 *   [해결 원리 — 페일세이프(Fail-Safe) 전환]
 *     명령의 '부재'를 곧 '정지 명령'으로 해석하도록 논리를 뒤집습니다.
 *       - Motor_Drive() 가 불릴 때마다 타임스탬프 갱신 (= 하트비트 수신)
 *       - Motor_PID_Update() 가 매 10ms 마다 경과 시간 검사
 *       - CMD_TIMEOUT_MS 초과 시 목표/명령/PWM 을 전부 0 으로 강제
 *
 *   [부팅 시 상태 = '두절'로 시작]
 *     s_cmd_received_once = 0 이므로 첫 CAN 명령을 받기 전에는 워치독이 발동
 *     상태입니다. 전원 인가 직후 노이즈로 PWM 이 튀는 상황을 원천 차단합니다.
 *
 *   [HAL_GetTick() 오버플로 안전]
 *     uint32_t 뺄셈 (now - last) 은 32비트 랩어라운드(약 49.7일) 시에도
 *     모듈러 산술에 의해 올바른 경과 시간을 반환합니다.
 *     ※ (now > last) 같은 대소 비교로 짜면 49.7일마다 워치독이 영구 발동합니다.
 *
 *   [주의 — 이 워치독의 한계]
 *     Motor_PID_Update() 는 메인 루프에서 호출됩니다. 메인 루프 자체가 블로킹
 *     되면(예: printf 의 HAL_UART_Transmit(HAL_MAX_DELAY)) 이 워치독도 함께
 *     멈춥니다. 그래서 main.c 에 IWDG(독립 워치독)를 별도로 추가했습니다.
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

#define KINETIC_PWM_SLOPE   ((float)(MOTOR_PWM_MAX - KINETIC_PWM_BASE) \
                             / (MOTOR_MAX_RPM - SMOOTH_RPM))

#define STRAFE_DETECT_THRESHOLD  500.0f

#define CLAMP(x, lo, hi)    ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))

#define INTEGRAL_LIMIT      (MOTOR_MAX_RPM * 2.0f)

/* ── 제어 상태 열거형 ─────────────────────────────────────────── */
typedef enum {
    CTRL_CLOSED_LOOP = 0,
    CTRL_OPEN_LOOP   = 1,
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

static int16_t s_cmd_fl = 0;
static int16_t s_cmd_fr = 0;
static int16_t s_cmd_rl = 0;
static int16_t s_cmd_rr = 0;

static int32_t s_enc_left_last  = 0;
static int32_t s_enc_right_last = 0;

static int32_t s_fb_left_accum  = 0;
static int32_t s_fb_right_accum = 0;

/* ── ★ v2: 워치독 상태 ──────────────────────────────────────── */
/* volatile: Motor_Drive() 는 메인 루프에서 불리지만, 향후 ISR 직결로 바꿔도
 * 안전하도록 미리 volatile 로 선언합니다. */
static volatile uint32_t s_last_cmd_tick     = 0U;
static volatile uint8_t  s_cmd_received_once = 0U;
/* 1 = 두절(강제 정지 중). 부팅 시 '두절'에서 시작 = 페일세이프 */
static uint8_t           s_watchdog_tripped  = 1U;
/* 통계: 워치독이 몇 번 발동했는가 (진단용) */
static uint32_t          s_watchdog_trip_count = 0U;

/* ★ v2: 하드웨어 준비 완료 플래그
 *
 *   [왜 필요한가]
 *   Error_Handler() 가 Motor_Stop() 을 호출하도록 바꿨는데, Error_Handler 는
 *   SystemClock_Config() 안에서도 불릴 수 있습니다. 그 시점에는 MX_TIM1_Init()
 *   이전이라 htim1.Instance 가 NULL 이고, __HAL_TIM_SET_COMPARE 가 NULL 을
 *   역참조해 HardFault 가 납니다. GPIOC 도 클럭이 아직 안 켜져 있습니다.
 *
 *   [안전한 이유]
 *   이 플래그가 0 인 시점에는 PWM 타이머가 아직 시작되지 않았으므로
 *   CCR 레지스터는 리셋값 0 = 모터 정지 상태입니다. 아무것도 안 해도 안전합니다. */
static uint8_t           s_hw_ready = 0U;

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
 *  ★ v2 내부 헬퍼: 4채널 PWM 즉시 0 (워치독/정지 공용)
 * ════════════════════════════════════════════════════════════════ */
static void _all_wheels_off(void)
{
    /* 페리페럴 초기화 전이면 아무것도 하지 않는다 (위 s_hw_ready 주석 참조).
     * 이 시점의 PWM CCR 은 리셋값 0 이므로 모터는 이미 정지 상태다. */
    if (s_hw_ready == 0U) { return; }

    _set_wheel(GPIO_PIN_2, &htim1, TIM_CHANNEL_1, 0, FL_DIR_INVERT);
    _set_wheel(GPIO_PIN_3, &htim1, TIM_CHANNEL_2, 0, FR_DIR_INVERT);
    _set_wheel(GPIO_PIN_0, &htim2, TIM_CHANNEL_1, 0, RL_DIR_INVERT);
    _set_wheel(GPIO_PIN_1, &htim2, TIM_CHANNEL_2, 0, RR_DIR_INVERT);
}

/* ════════════════════════════════════════════════════════════════
 *  ★ v2 내부 헬퍼: 제어 상태 전체 초기화 (적분 윈드업 제거 포함)
 * ════════════════════════════════════════════════════════════════ */
static void _reset_control_state(void)
{
    s_pid_left.target_rpm  = 0.0f;
    s_pid_left.integral    = 0.0f;
    s_pid_left.prev_error  = 0.0f;
    s_pid_right.target_rpm = 0.0f;
    s_pid_right.integral   = 0.0f;
    s_pid_right.prev_error = 0.0f;

    s_cmd_fl = s_cmd_fr = s_cmd_rl = s_cmd_rr = 0;
    s_ctrl_state = CTRL_CLOSED_LOOP;
}

/* ════════════════════════════════════════════════════════════════
 *  내부 헬퍼: 2단계 피드포워드 PWM 계산
 * ════════════════════════════════════════════════════════════════ */
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
 *
 *  ⚠ 구조적 주의사항 (분석 결과 기록)
 *    좌측 호출은 FL 과 RL 에 '같은 PWM' 을 내보내고, 피드백은 TIM3(=FL) 하나뿐입니다.
 *    RL 이 접지되지 않은 상태에서는 PID 가 접지된 FL 을 목표까지 끌어올리려 PWM 을
 *    올리고, 무부하 RL 이 그 PWM 을 그대로 받아 공중에서 과속 회전합니다.
 *    => 좌측 추진 = FL 1개 / 우측 추진 = FR+RR 2개 로 비대칭이 생기고 차체가 휩니다.
 *    => 기구(RL 접지) 수리가 근본 해결이며, 소프트웨어로는 보상만 가능합니다.
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

    /* ★ v2: 워치독을 '두절' 상태로 초기화.
     *   첫 CAN 명령을 받기 전에는 어떤 경우에도 PWM 이 나가지 않습니다. */
    s_last_cmd_tick       = 0U;
    s_cmd_received_once   = 0U;
    s_watchdog_tripped    = 1U;
    s_watchdog_trip_count = 0U;

    /* 이 시점 이후로만 PWM/GPIO 접근이 안전하다.
     * (main.c 에서 MX_GPIO_Init / MX_TIM*_Init / HAL_TIM_PWM_Start 가 모두 끝난 뒤
     *  Motor_Init 이 호출되는 순서를 전제로 한다) */
    s_hw_ready = 1U;

    _all_wheels_off();
}

/**
 * @brief CAN 수신 → 4바퀴 독립 목표 저장 + 스트레이핑 감지
 *        ★ v2: 호출될 때마다 워치독 타임스탬프를 갱신 (= 하트비트)
 */
void Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    /* ── ★ v2: 워치독 급이기(feed) ──────────────────────────── */
    s_last_cmd_tick     = HAL_GetTick();
    s_cmd_received_once = 1U;

    s_cmd_fl = fl;
    s_cmd_fr = fr;
    s_cmd_rl = rl;
    s_cmd_rr = rr;

    /* ── 스트레이핑 감지 ─────────────────────────────────────── */
    float implied_vy = (float)(-fl + fr + rl - rr) / 4.0f;

    ControlState_t new_state =
        (fabsf(implied_vy) > STRAFE_DETECT_THRESHOLD)
        ? CTRL_OPEN_LOOP
        : CTRL_CLOSED_LOOP;

    if (new_state != s_ctrl_state)
    {
        s_pid_left.integral    = 0.0f;
        s_pid_left.prev_error  = 0.0f;
        s_pid_right.integral   = 0.0f;
        s_pid_right.prev_error = 0.0f;
        s_ctrl_state = new_state;
    }

    if (s_ctrl_state == CTRL_CLOSED_LOOP)
    {
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
 *        ★ v2: 제어 전에 명령 워치독을 먼저 검사
 */
void Motor_PID_Update(void)
{
    /* ════════════════════════════════════════════════════════════
     *  [1] 엔코더 읽기 + 누산  ─ 워치독 상태와 무관하게 항상 수행
     *
     *  이유: 통신이 끊겨도 로봇은 관성으로 굴러갑니다. 그 이동량을 놓치면
     *        통신 복구 후 오도메트리에 '순간 점프'가 생겨 EKF 가 흔들립니다.
     *        정지시키는 것과 계측하는 것은 별개입니다.
     * ════════════════════════════════════════════════════════════ */
    int32_t cur_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t cur_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);

    int16_t delta_left  = (int16_t)((uint16_t)cur_left  - (uint16_t)s_enc_left_last);
    int16_t delta_right = (int16_t)((uint16_t)cur_right - (uint16_t)s_enc_right_last);

    s_enc_left_last  = cur_left;
    s_enc_right_last = cur_right;

    s_fb_left_accum  += (int32_t)delta_left;
    s_fb_right_accum += (int32_t)delta_right;

    float rpm_scale   = 60.0f / ((float)ENCODER_CPR * PID_PERIOD_S);
    float meas_left   = (float)delta_left  * rpm_scale;
    float meas_right  = (float)delta_right * rpm_scale;

    /* ════════════════════════════════════════════════════════════
     *  [2] ★ v2: 명령 워치독 판정
     *
     *  volatile 변수는 한 번만 읽어 지역 변수에 복사합니다.
     *  (판정 도중 ISR/다른 경로가 값을 바꿔도 판정 일관성이 깨지지 않도록)
     * ════════════════════════════════════════════════════════════ */
    uint32_t now       = HAL_GetTick();
    uint32_t last_tick = s_last_cmd_tick;
    uint8_t  ever      = s_cmd_received_once;

    /* uint32_t 뺄셈은 랩어라운드(49.7일)에서도 모듈러 산술로 올바르게 동작.
     * 절대 (now > last_tick + TIMEOUT) 형태로 쓰지 말 것 — 오버플로에 취약. */
    uint8_t timed_out = (ever == 0U) || ((now - last_tick) > CMD_TIMEOUT_MS);

    if (timed_out)
    {
        if (s_watchdog_tripped == 0U)
        {
            /* 정상 → 두절 전이: 이때 한 번만 상태를 완전 초기화 */
            s_watchdog_tripped = 1U;
            s_watchdog_trip_count++;
            _reset_control_state();
        }

        /* 매 주기 재확정.
         * 전이 시 1회만 끄지 않고 반복해서 0 을 쓰는 이유:
         *   - 어떤 경로로든 PWM 레지스터가 다시 쓰이면 즉시 덮어씁니다
         *   - 노이즈/글리치로 레지스터가 변조돼도 10ms 안에 복구됩니다
         * 비용은 GPIO 4회 + 레지스터 4회 = 무시할 수준입니다. */
        _all_wheels_off();
        return;
    }

    if (s_watchdog_tripped != 0U)
    {
        /* 두절 → 정상 복귀.
         * 적분항을 반드시 비웁니다. 두절 동안 쌓인 값이 남아 있으면
         * 복귀 순간 큰 PWM 이 튀어나가 로봇이 앞으로 튑니다(windup kick). */
        s_watchdog_tripped = 0U;
        s_pid_left.integral    = 0.0f;
        s_pid_left.prev_error  = 0.0f;
        s_pid_right.integral   = 0.0f;
        s_pid_right.prev_error = 0.0f;
    }

    /* ════════════════════════════════════════════════════════════
     *  [3] 제어 모드 분기 (v1 그대로)
     * ════════════════════════════════════════════════════════════ */
    if (s_ctrl_state == CTRL_CLOSED_LOOP)
    {
        _drive_side_closed(&s_pid_left,  meas_left,
                           GPIO_PIN_2, &htim1, TIM_CHANNEL_1, FL_DIR_INVERT,
                           GPIO_PIN_0, &htim2, TIM_CHANNEL_1, RL_DIR_INVERT);

        _drive_side_closed(&s_pid_right, meas_right,
                           GPIO_PIN_3, &htim1, TIM_CHANNEL_2, FR_DIR_INVERT,
                           GPIO_PIN_1, &htim2, TIM_CHANNEL_2, RR_DIR_INVERT);
    }
    else
    {
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
 *   Byte 0-1: int16 left_ticks   (실제로는 FL 한 바퀴)
 *   Byte 2-3: int16 right_ticks  (실제로는 FR 한 바퀴)
 *
 * ⚠ 정정 (분석 결과)
 *   기존 주석은 "Left/Right 평균이 Vy 항을 소거하므로 X 및 Yaw 를 2-엔코더로
 *   정확히 추정 가능"이라고 되어 있었으나, 이는 좌측 엔코더가 (FL+RL)/2 일 때만
 *   성립합니다. 실제로는 FL 한 바퀴뿐이므로 메카넘 IK 상
 *       (FL + FR)/2      = Vx              ← Vx 는 정확 (Vy, Wz 완전 소거)
 *       (FR − FL)/(2·l)  = Wz + Vy/l       ← Yaw 는 Vy 에 오염 (1/l = 1.98)
 *   따라서 Pi 측 EKF 는 이 피드백에서 Vx 만 사용해야 합니다.
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
    _reset_control_state();
    _all_wheels_off();
}

/* ── ★ v2 신규: 워치독 상태 조회 ───────────────────────────── */

uint8_t Motor_Watchdog_IsTripped(void)
{
    return s_watchdog_tripped;
}

uint32_t Motor_Watchdog_ElapsedMs(void)
{
    if (s_cmd_received_once == 0U)
    {
        return 0xFFFFFFFFU;
    }
    return HAL_GetTick() - s_last_cmd_tick;
}