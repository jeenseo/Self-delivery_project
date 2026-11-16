/**
 * @file   motor_diag.c
 * @brief  FF+PI 계측 스트림 — 링버퍼 + USART2 TX DMA(DMA1 Channel7) 폴링 드레인
 *
 *  설계 요약 (자세한 근거는 motor_diag.h 머리말)
 *   - 생산자: Motor_PID_Update(10 ms), printf(_write), MotorDiag_Poll(1 Hz health / 5 s info)
 *             전부 '메인 루프 문맥' 입니다. 그래서 잠금이 필요 없습니다.
 *   - 소비자: DMA1 Channel7. 링의 [tail, tail+inflight) 구간을 하드웨어가 읽는 동안
 *             생산자는 그 구간을 덮어쓰지 않습니다(used 에 inflight 포함).
 *   - 인터럽트 없음: DMA 완료(TCIF7)는 MotorDiag_Poll()/프레임 추가 시 폴링.
 *   - 손실은 '프레임 단위' + 카운터. 수신 측은 seq 갭과 CRC 로 손실을 검출합니다.
 */
#include "motor_diag.h"

#if MOTOR_DIAG_ENABLE

#include "motor.h"
#include "usart.h"
#include <string.h>

#if defined(__has_include)
#  if __has_include("fw_build_id.h")
#    include "fw_build_id.h"      /* tools/gen_build_id.sh 가 생성 (선택) */
#  endif
#endif
#ifndef FW_BUILD_ID
#define FW_BUILD_ID   "unset"     /* 플래시 이력 기록용. 빌드마다 git 해시로 바꾸십시오 */
#endif

extern UART_HandleTypeDef huart2;

/* ── CAN ISR 카운터 ───────────────────────────────────────────────────── */
volatile uint32_t g_diag_can_rx_any       = 0U;
volatile uint32_t g_diag_can_rx_cmd       = 0U;
volatile uint32_t g_diag_can_rx_overwrite = 0U;

/* ── 링버퍼 ───────────────────────────────────────────────────────────── */
#define RB_MASK  (DIAG_RING_SIZE - 1U)
static uint8_t  s_ring[DIAG_RING_SIZE];
static uint32_t s_head = 0U;        /* 다음 쓰기 위치 (자유 증가) */
static uint32_t s_tail = 0U;        /* DMA 가 아직 끝내지 않은 첫 바이트 */
static uint32_t s_inflight = 0U;    /* 현재 DMA 전송 중인 바이트 수 */
static uint16_t s_ring_hwm = 0U;

/* ── 카운터 ───────────────────────────────────────────────────────────── */
static uint32_t s_pid_cycles   = 0U;
static uint32_t s_rec_emitted  = 0U;
static uint32_t s_rec_dropped  = 0U;
static uint32_t s_text_dropped = 0U;
static uint32_t s_other_dropped = 0U;
static uint32_t s_dma_errors   = 0U;
static uint32_t s_fb_ok = 0U, s_fb_fail = 0U;
static uint32_t s_wd_trip_count = 0U;
static uint16_t s_seq = 0U;

/* ── DWT 시간 ─────────────────────────────────────────────────────────── */
static uint8_t  s_dwt_ok = 0U;
static uint32_t s_cyc_per_us = 72U;
static uint32_t s_cyc_last = 0U, s_cyc_rem = 0U, s_us = 0U;
static uint32_t s_cyc_start = 0U, s_cyc_prev_start = 0U;
static uint8_t  s_have_prev = 0U;
static uint16_t s_dt_us = 0U, s_exec_us = 0U, s_diag_us = 0U;
static uint32_t s_t_us_start = 0U;
/* health 창 통계 */
static uint16_t s_h_dt_max = 0U, s_h_dt_min = 0xFFFFU, s_h_exec_max = 0U, s_h_diag_max = 0U;
/* 데시메이션 창 */
static uint8_t  s_win_n = 0U;
static uint16_t s_win_lflags = 0U;
static uint8_t  s_win_offpath = 0U;
static uint16_t s_win_dt_max = 0U, s_win_exec_max = 0U;

static uint32_t s_last_health = 0U, s_last_info = 0U;
static MotorDiagInfo_t s_info;
static uint8_t  s_info_valid = 0U;

/* ═══════════════════════════════════════════════════════════════════════
 *  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, no xorout)
 *  니블 테이블 32 B. Python: binascii.crc_hqx(data, 0xFFFF)
 * ═══════════════════════════════════════════════════════════════════════ */
static const uint16_t s_crc_tab[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF };

static inline uint16_t _crc_step(uint16_t crc, uint8_t b)
{
    crc = (uint16_t)((crc << 4) ^ s_crc_tab[((crc >> 12) ^ (b >> 4)) & 0x0FU]);
    crc = (uint16_t)((crc << 4) ^ s_crc_tab[((crc >> 12) ^ (b & 0x0FU)) & 0x0FU]);
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  DWT 사이클 카운터 -> µs
 * ═══════════════════════════════════════════════════════════════════════ */
static void _dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t a = DWT->CYCCNT;
    for (volatile uint32_t k = 0U; k < 32U; k++) { }
    uint32_t b = DWT->CYCCNT;
    s_dwt_ok = (uint8_t)(b != a);   /* 일부 호환칩은 CYCCNT 가 없을 수 있음 -> 0 */
    s_cyc_per_us = SystemCoreClock / 1000000U;
    if (s_cyc_per_us == 0U) s_cyc_per_us = 1U;
    s_cyc_last = DWT->CYCCNT;
}

/** 단조 µs 타임스탬프 (10 ms 마다 호출되므로 CYCCNT 59.6 s 랩어라운드와 무관) */
static uint32_t _now_us(void)
{
    if (!s_dwt_ok) return 0U;
    uint32_t c = DWT->CYCCNT;
    s_cyc_rem += (c - s_cyc_last);
    s_cyc_last = c;
    uint32_t us = s_cyc_rem / s_cyc_per_us;
    s_cyc_rem  -= us * s_cyc_per_us;
    s_us       += us;
    return s_us;
}

static uint16_t _cyc_to_us16(uint32_t cyc)
{
    uint32_t us = cyc / s_cyc_per_us;
    return (us > 0xFFFEU) ? 0xFFFFU : (uint16_t)us;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  DMA1 Channel7 = USART2_TX (RM0008 Table 78). 레지스터 직접 제어, IRQ 없음.
 * ═══════════════════════════════════════════════════════════════════════ */
static void _dma_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    (void)RCC->AHBENR;
    DMA1_Channel7->CCR  = 0U;
    DMA1->IFCR          = DMA_IFCR_CGIF7;
    DMA1_Channel7->CPAR = (uint32_t)&USART2->DR;
    /* 메모리 -> 주변장치, 메모리 증가, 8 bit/8 bit, 우선순위 Low, 인터럽트 없음 */
    DMA1_Channel7->CCR  = DMA_CCR_DIR | DMA_CCR_MINC;
    USART2->CR3 |= USART_CR3_DMAT;
}

static void _dma_service(void)
{
    if (s_inflight != 0U)
    {
        uint32_t isr = DMA1->ISR;
        if ((isr & DMA_ISR_TEIF7) != 0U)
        {
            s_dma_errors++;                      /* 해당 청크는 손실(수신 측 CRC/seq 로 보임) */
        }
        else if ((isr & DMA_ISR_TCIF7) == 0U)
        {
            return;                              /* 전송 중 */
        }
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CGIF7;
        s_tail    += s_inflight;
        s_inflight = 0U;
    }
    uint32_t used = s_head - s_tail;
    if (used == 0U) return;
    uint32_t start = s_tail & RB_MASK;
    uint32_t n     = DIAG_RING_SIZE - start;     /* 연속 구간만 */
    if (n > used) n = used;
    DMA1_Channel7->CMAR  = (uint32_t)&s_ring[start];
    DMA1_Channel7->CNDTR = n;
    DMA1_Channel7->CCR  |= DMA_CCR_EN;
    s_inflight = n;
}

/** 프레임 1개를 통째로 넣거나(1) 통째로 버립니다(0). 부분 프레임은 없습니다. */
static uint8_t _frame_put(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint32_t need = (uint32_t)len + DIAG_FRAME_OVERHEAD;
    uint32_t used = s_head - s_tail;
    if ((DIAG_RING_SIZE - used) < need) return 0U;

    uint16_t crc = 0xFFFFU;
    s_ring[s_head++ & RB_MASK] = 0xA5U;
    s_ring[s_head++ & RB_MASK] = 0x5AU;
    s_ring[s_head++ & RB_MASK] = type; crc = _crc_step(crc, type);
    s_ring[s_head++ & RB_MASK] = len;  crc = _crc_step(crc, len);
    for (uint32_t k = 0U; k < len; k++)
    {
        s_ring[s_head++ & RB_MASK] = payload[k];
        crc = _crc_step(crc, payload[k]);
    }
    s_ring[s_head++ & RB_MASK] = (uint8_t)(crc & 0xFFU);
    s_ring[s_head++ & RB_MASK] = (uint8_t)(crc >> 8);

    used += need;
    if (used > s_ring_hwm) s_ring_hwm = (uint16_t)used;
    _dma_service();
    return 1U;
}

/* ── 직렬화 도우미 (정렬 무관, LE) ─────────────────────────────────────── */
static uint8_t *_u8 (uint8_t *p, uint8_t v)  { p[0] = v; return p + 1; }
static uint8_t *_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); return p + 2; }
static uint8_t *_i16(uint8_t *p, int16_t v)  { return _u16(p, (uint16_t)v); }
static uint8_t *_u32(uint8_t *p, uint32_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); return p + 4; }
static uint8_t *_f32(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); return _u32(p, u); }

/** float -> int16, 반올림(0 에서 멀어지는 쪽), 포화 */
static int16_t _sat16(float x)
{
    if (x != x)         return 0;          /* NaN 방어 */
    if (x >  32767.0f)  return  32767;
    if (x < -32768.0f)  return -32768;
    return (int16_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}
static uint8_t _sat8_10ms(uint16_t ms) { uint16_t v = (uint16_t)(ms / 10U); return (v > 255U) ? 255U : (uint8_t)v; }

/** 바퀴 1개의 제한 단계 판정. _set_wheel_safe 와 같은 순서(SAFE 클램프 -> 슬루)로 재현합니다.
 *  act=0 이면 cmd=0 이 들어가므로 SLEW(0 으로 감속 중)만 의미가 있습니다. */
static uint16_t _wheel_lflags(int16_t cmd, int16_t out, float pre, uint8_t act)
{
    uint16_t f = 0U;
    if (act) f |= DIAG_LF_ACTIVE;
    if (act && ((pre > (float)MOTOR_PWM_MAX) || (pre < -(float)MOTOR_PWM_MAX))) f |= DIAG_LF_CLAMP9999;
    int16_t safe = cmd;
    if (safe >  MOTOR_PWM_SAFE_MAX) { safe =  MOTOR_PWM_SAFE_MAX; f |= DIAG_LF_SAFE; }
    if (safe < -MOTOR_PWM_SAFE_MAX) { safe = -MOTOR_PWM_SAFE_MAX; f |= DIAG_LF_SAFE; }
    if (out != safe) f |= DIAG_LF_SLEW;
    return f;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  공개 API
 * ═══════════════════════════════════════════════════════════════════════ */
void MotorDiag_Init(void)
{
    if (DIAG_UART_BAUD != 115200U)
    {
        huart2.Init.BaudRate = DIAG_UART_BAUD;
        (void)HAL_UART_Init(&huart2);
    }
    _dwt_init();
    _dma_init();
    s_last_health = s_last_info = HAL_GetTick();
}

void MotorDiag_CycleBegin(void)
{
    s_pid_cycles++;
    if (!s_dwt_ok) { s_dt_us = 0U; s_t_us_start = 0U; return; }
    s_cyc_start  = DWT->CYCCNT;
    s_t_us_start = _now_us();
    s_dt_us      = s_have_prev ? _cyc_to_us16(s_cyc_start - s_cyc_prev_start) : 0U;
    s_cyc_prev_start = s_cyc_start;
    s_have_prev  = 1U;
}

void MotorDiag_CycleEnd(void)
{
    s_exec_us = s_dwt_ok ? _cyc_to_us16(DWT->CYCCNT - s_cyc_start) : 0U;
}

void MotorDiag_PushCycle(const MotorDiagCycle_t *c)
{
    uint32_t c0 = s_dwt_ok ? DWT->CYCCNT : 0U;

    /* ── 바퀴별 제한 플래그 ── */
    uint16_t lf = 0U;
    if (c->pair[0].valid)
    {
        lf |= (uint16_t)(_wheel_lflags(c->pair[0].cmd_f, c->pair[0].out_f, c->pair[0].pre_f, c->pair[0].act_f) << (4U * DIAG_FL));
        lf |= (uint16_t)(_wheel_lflags(c->pair[0].cmd_r, c->pair[0].out_r, c->pair[0].pre_r, c->pair[0].act_r) << (4U * DIAG_RL));
    }
    if (c->pair[1].valid)
    {
        lf |= (uint16_t)(_wheel_lflags(c->pair[1].cmd_f, c->pair[1].out_f, c->pair[1].pre_f, c->pair[1].act_f) << (4U * DIAG_FR));
        lf |= (uint16_t)(_wheel_lflags(c->pair[1].cmd_r, c->pair[1].out_r, c->pair[1].pre_r, c->pair[1].act_r) << (4U * DIAG_RR));
    }
    s_wd_trip_count = c->wd_trip_count;

    /* ── 창 누적 (DECIM>1) / health 통계 ── */
    s_win_lflags |= lf;
    if (c->path != DIAG_PATH_DRIVE) s_win_offpath = 1U;
    if (s_dt_us   > s_win_dt_max)   s_win_dt_max   = s_dt_us;
    if (s_exec_us > s_win_exec_max) s_win_exec_max = s_exec_us;
    if (s_have_prev && s_dt_us != 0U)
    {
        if (s_dt_us > s_h_dt_max) s_h_dt_max = s_dt_us;
        if (s_dt_us < s_h_dt_min) s_h_dt_min = s_dt_us;
    }
    if (s_exec_us > s_h_exec_max) s_h_exec_max = s_exec_us;
    if (++s_win_n < DIAG_DECIM) return;

    /* ── 직렬화 (86 B) ── */
    uint8_t  b[DIAG_CYCLE_PAYLOAD];
    uint8_t *p = b;
    uint8_t  sf = 0U;
    sf |= (uint8_t)(c->wd_tripped    ? 0x01U : 0U);
    sf |= (uint8_t)(c->stall_tripped ? 0x02U : 0U);
    sf |= (uint8_t)(c->cmd_ever      ? 0x04U : 0U);
    sf |= (uint8_t)(c->hw_ready      ? 0x08U : 0U);
    sf |= (uint8_t)(c->stall_released? 0x10U : 0U);
#if MOTOR_OUTPUT_INHIBIT
    sf |= 0x20U;
#endif
    sf |= (uint8_t)(s_win_offpath    ? 0x40U : 0U);
    uint8_t ifl = (uint8_t)((c->i_fl_hi ? 1U : 0U) | (c->i_fl_lo ? 2U : 0U) |
                            (c->i_fr_hi ? 4U : 0U) | (c->i_fr_lo ? 8U : 0U));

    p = _u16(p, s_seq);
    p = _u32(p, s_t_us_start);
    p = _u32(p, c->tick_ms);
    p = _u16(p, (DIAG_DECIM > 1U) ? s_win_dt_max   : s_dt_us);
    p = _u16(p, (DIAG_DECIM > 1U) ? s_win_exec_max : s_exec_us);
    p = _u16(p, s_diag_us);
    p = _u8 (p, c->path);
    p = _u8 (p, sf);
    p = _u16(p, s_win_lflags);
    p = _u8 (p, ifl);
    p = _u8 (p, _sat8_10ms(c->stall_l_ms));
    p = _u8 (p, _sat8_10ms(c->stall_r_ms));
    p = _u8 (p, 0U);
    p = _u16(p, c->cmd_age_ms);
    for (uint32_t w = 0U; w < 4U; w++) p = _i16(p, c->cmd[w]);
    p = _i16(p, c->d_fl);
    p = _i16(p, c->d_fr);
    p = _i16(p, _sat16(c->meas_fl * 100.0f));
    p = _i16(p, _sat16(c->meas_fr * 100.0f));
    for (uint32_t w = 0U; w < 4U; w++) p = _i16(p, _sat16(c->tgt[w] * 100.0f));
    /* FF: FL, FR, RL, RR */
    p = _i16(p, _sat16(c->pair[0].ff_f));
    p = _i16(p, _sat16(c->pair[1].ff_f));
    p = _i16(p, _sat16(c->pair[0].ff_r));
    p = _i16(p, _sat16(c->pair[1].ff_r));
    p = _i16(p, _sat16(c->pair[0].p));
    p = _i16(p, _sat16(c->pair[1].p));
    p = _i16(p, _sat16(c->pair[0].i));
    p = _i16(p, _sat16(c->pair[1].i));
    p = _i16(p, _sat16(c->pair[0].corr));
    p = _i16(p, _sat16(c->pair[1].corr));
    /* 제한 전: FL, FR, RL, RR */
    p = _i16(p, _sat16(c->pair[0].pre_f));
    p = _i16(p, _sat16(c->pair[1].pre_f));
    p = _i16(p, _sat16(c->pair[0].pre_r));
    p = _i16(p, _sat16(c->pair[1].pre_r));
    /* 최종 인가: FL, FR, RL, RR */
    p = _i16(p, c->pair[0].out_f);
    p = _i16(p, c->pair[1].out_f);
    p = _i16(p, c->pair[0].out_r);
    p = _i16(p, c->pair[1].out_r);

    if (_frame_put(DIAG_TYPE_CYCLE, b, (uint8_t)(p - b))) s_rec_emitted++;
    else                                                  s_rec_dropped++;
    s_seq++;                                  /* 버려져도 증가 -> 수신 측이 갭으로 손실을 셉니다 */

    s_win_n = 0U; s_win_lflags = 0U; s_win_offpath = 0U; s_win_dt_max = 0U; s_win_exec_max = 0U;

    if (s_dwt_ok)
    {
        s_diag_us = _cyc_to_us16(DWT->CYCCNT - c0);
        if (s_diag_us > s_h_diag_max) s_h_diag_max = s_diag_us;
    }
}

static void _emit_health(uint32_t now)
{
    uint8_t b[68]; uint8_t *p = b;
    p = _u32(p, now);
    p = _u32(p, _now_us());
    p = _u32(p, s_pid_cycles);
    p = _u32(p, s_rec_emitted);
    p = _u32(p, s_rec_dropped);
    p = _u32(p, s_text_dropped);
    p = _u32(p, s_other_dropped);
    p = _u16(p, s_ring_hwm);
    p = _u16(p, s_h_dt_max);
    p = _u16(p, (s_h_dt_min == 0xFFFFU) ? 0U : s_h_dt_min);
    p = _u16(p, s_h_exec_max);
    p = _u16(p, s_h_diag_max);
    p = _u16(p, (uint16_t)((s_dma_errors > 0xFFFFU) ? 0xFFFFU : s_dma_errors));
    p = _u32(p, g_diag_can_rx_cmd);
    p = _u32(p, g_diag_can_rx_any - g_diag_can_rx_cmd);
    p = _u32(p, g_diag_can_rx_overwrite);
    p = _u32(p, s_fb_ok);
    p = _u32(p, s_fb_fail);
    p = _u32(p, s_wd_trip_count);
    p = _u32(p, CAN1->ESR);
    if (!_frame_put(DIAG_TYPE_HEALTH, b, (uint8_t)(p - b))) s_other_dropped++;
    s_h_dt_max = 0U; s_h_dt_min = 0xFFFFU; s_h_exec_max = 0U; s_h_diag_max = 0U;
}

static void _emit_info(void)
{
    if (!s_info_valid) return;
    uint8_t b[120]; uint8_t *p = b;
    const MotorDiagInfo_t *f = &s_info;
    uint8_t fl = (uint8_t)((f->use_rr_encoder ? 0x01U : 0U) | (f->output_inhibit ? 0x02U : 0U) |
                           (s_dwt_ok ? 0x04U : 0U) | (f->inv_fl ? 0x08U : 0U) | (f->inv_fr ? 0x10U : 0U) |
                           (f->inv_rl ? 0x20U : 0U) | (f->inv_rr ? 0x40U : 0U));
    p = _u8 (p, DIAG_FMT_VERSION);
    p = _u8 (p, (uint8_t)DIAG_DECIM);
    p = _u8 (p, f->pid_period_ms);
    p = _u8 (p, fl);
    p = _u16(p, f->encoder_cpr);  p = _u16(p, f->pwm_max);   p = _u16(p, f->safe_max);
    p = _u16(p, f->slew);         p = _u16(p, f->stall_pwm); p = _u16(p, f->stall_hold_ms);
    p = _u16(p, f->cmd_timeout_ms); p = _u16(p, f->fb_period_ms);
    p = _u16(p, f->stiction_base);  p = _u16(p, f->kinetic_base);
    p = _f32(p, f->kp);  p = _f32(p, f->ki);  p = _f32(p, f->kd);  p = _f32(p, f->ff_gain);
    p = _f32(p, f->max_rpm); p = _f32(p, f->smooth_rpm); p = _f32(p, f->stall_rpm); p = _f32(p, f->integral_limit);
    p = _u32(p, DIAG_UART_BAUD);
    p = _u32(p, SystemCoreClock);
    p = _u16(p, f->can_cmd_id); p = _u16(p, f->can_fb_id);
    char id[48];
    memset(id, 0, sizeof(id));
    const char *parts[3] = { FW_BUILD_ID, __DATE__, __TIME__ };
    uint32_t k = 0U;
    for (uint32_t s = 0U; s < 3U; s++)
    {
        for (const char *q = parts[s]; *q && k < sizeof(id) - 1U; q++) id[k++] = *q;
        if (s < 2U && k < sizeof(id) - 1U) id[k++] = ' ';
    }
    memcpy(p, id, sizeof(id)); p += sizeof(id);
    if (!_frame_put(DIAG_TYPE_INFO, b, (uint8_t)(p - b))) s_other_dropped++;
}

void MotorDiag_SetInfo(const MotorDiagInfo_t *info)
{
    s_info = *info;
    s_info_valid = 1U;
    _emit_info();
}

void MotorDiag_Poll(void)
{
    _dma_service();
    uint32_t now = HAL_GetTick();
    if ((now - s_last_health) >= DIAG_HEALTH_PERIOD_MS)
    {
        s_last_health = now;
        _emit_health(now);
    }
    if ((now - s_last_info) >= DIAG_INFO_PERIOD_MS)
    {
        s_last_info = now;
        _emit_info();
    }
}

void MotorDiag_CountFeedbackTx(uint8_t ok)
{
    if (ok) s_fb_ok++; else s_fb_fail++;
}

/** printf 경로. 논블로킹: 공간이 없으면 버리고 센다. ISR 문맥이면 버린다. */
int MotorDiag_WriteText(const char *ptr, int len)
{
    if (len <= 0) return 0;
    if (__get_IPSR() != 0U) { s_text_dropped += (uint32_t)len; return len; }
    int done = 0;
    while (done < len)
    {
        int n = len - done;
        if (n > 120) n = 120;
        if (!_frame_put(DIAG_TYPE_TEXT, (const uint8_t *)ptr + done, (uint8_t)n))
        {
            s_text_dropped += (uint32_t)(len - done);
            break;
        }
        done += n;
    }
    return len;   /* printf 가 재시도하지 않도록 항상 len */
}

#endif /* MOTOR_DIAG_ENABLE */
