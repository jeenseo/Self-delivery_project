/**
 * @file   motor_diag.h
 * @brief  FF+PI 제어 루프 계측 (USART2 TX DMA, 논블로킹 바이너리 스트림)
 *
 * ============================================================================
 *  목적
 * ============================================================================
 *  Motor_PID_Update() 한 주기에서 펌웨어가 '실제로 계산한 값' 을 그대로 기록합니다.
 *    목표 RPM, 측정 RPM, 원시 엔코더 틱, FF, P, I, 보정량, 제한 전 PWM,
 *    최종 인가 PWM, 포화/슬루/워치독/스톨 상태, 주기 지터(dt), 실행 시간.
 *
 *  ★ 제어 동작은 바꾸지 않습니다.
 *    - motor.c 의 제어 문장은 그대로 두고, 값을 '읽기만' 하는 캡처 문장만 추가합니다.
 *    - MOTOR_DIAG_ENABLE = 0 으로 빌드하면 b57f7d2 와 동일한 동작입니다
 *      (계측 코드 전부 컴파일 제외, _write 도 원래의 블로킹 경로).
 *
 *  ★ 블로킹 printf 를 추가하지 않습니다.
 *    - 레코드는 링버퍼에 넣고 DMA1 Channel7 이 USART2 로 내보냅니다.
 *    - 링버퍼가 차면 '프레임 단위로' 버리고 카운터를 올립니다(부분 프레임 없음).
 *    - 계측이 켜진 빌드에서는 printf(_write) 도 같은 링버퍼로 들어갑니다(논블로킹).
 *
 *  ★ 인터럽트를 추가하지 않습니다.
 *    - DMA 완료는 메인 루프에서 MotorDiag_Poll() 이 폴링합니다.
 *    - NVIC 설정, CAN RX 우선순위, SysTick 은 손대지 않습니다.
 *
 * ============================================================================
 *  프레임 (리틀엔디언)
 * ============================================================================
 *    [0]   0xA5           sync0
 *    [1]   0x5A           sync1
 *    [2]   type           0x01 cycle / 0x02 text / 0x03 health / 0x04 info
 *    [3]   len            payload 바이트 수 N
 *    [4..]  payload[N]
 *    [4+N] crc16 (LE)     CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF),
 *                         범위 = type, len, payload  (sync 제외)
 *  필드 정의는 tools/stm32_diag_decode.py 와 1:1 로 맞춰져 있습니다.
 *
 *  호출 문맥: MotorDiag_* 는 메인 루프 문맥에서만 호출합니다(ISR 금지).
 *            예외: MOTOR_DIAG_CAN_RX_* 매크로(카운터 증가만)는 CAN RX ISR 용입니다.
 */
#ifndef __MOTOR_DIAG_H
#define __MOTOR_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── 빌드 스위치 ─────────────────────────────────────────────────────────── */
#ifndef MOTOR_DIAG_ENABLE
#define MOTOR_DIAG_ENABLE        1     /**< 1: 계측 스트림 ON, 0: b57f7d2 와 동일 */
#endif

/**
 * 레코드 데시메이션. 1 = 매 PID 주기(100 Hz), 2 = 2주기마다(50 Hz).
 *   DECIM>1 일 때: 값 필드 = 마지막 주기의 스냅샷,
 *                  lflags/sflags 의 '창' 비트 = 창 안 모든 주기의 OR,
 *                  dt_us/exec_us = 창 안 최댓값.
 */
#ifndef DIAG_DECIM
#define DIAG_DECIM               2U
#endif

/** USART2 보드레이트. 115200 은 기존 디버그 설정과 동일. 100 Hz 는 230400 이상 필요. */
#ifndef DIAG_UART_BAUD
#define DIAG_UART_BAUD           115200U
#endif

#define DIAG_RING_SIZE           2048U   /**< 2의 거듭제곱 */
#define DIAG_FMT_VERSION         1U

#define DIAG_TYPE_CYCLE          0x01U
#define DIAG_TYPE_TEXT           0x02U
#define DIAG_TYPE_HEALTH         0x03U
#define DIAG_TYPE_INFO           0x04U

#define DIAG_CYCLE_PAYLOAD       86U
#define DIAG_FRAME_OVERHEAD      6U
#define DIAG_HEALTH_PERIOD_MS    1000U
#define DIAG_INFO_PERIOD_MS      5000U

/* 링크 점유율 정적 검사: (프레임 x 10bit x 레코드율) / baud <= 65 % */
#define DIAG_RECORD_HZ           (100U / DIAG_DECIM)
#if ((DIAG_CYCLE_PAYLOAD + DIAG_FRAME_OVERHEAD) * 10U * DIAG_RECORD_HZ * 100U) > (65U * DIAG_UART_BAUD)
#error "motor_diag: link utilisation > 65 %. Raise DIAG_UART_BAUD or DIAG_DECIM."
#endif
#if (DIAG_RING_SIZE & (DIAG_RING_SIZE - 1U)) != 0U
#error "DIAG_RING_SIZE must be a power of two"
#endif

/* path (Motor_PID_Update 가 어느 출구로 나갔는가) */
#define DIAG_PATH_DRIVE          0U
#define DIAG_PATH_WD_OFF         1U   /**< 명령 두절 -> 전 바퀴 0 */
#define DIAG_PATH_STALL_TRIP     2U   /**< 이번 주기에 스톨 판정 -> 전 바퀴 0 */
#define DIAG_PATH_STALL_LATCHED  3U   /**< 스톨 래치 유지 중 -> 전 바퀴 0 */

/* 바퀴 인덱스 */
#define DIAG_FL 0U
#define DIAG_FR 1U
#define DIAG_RL 2U
#define DIAG_RR 3U

/* lflags: 바퀴마다 4비트 (bit = 4*wheel + k) */
#define DIAG_LF_ACTIVE           0x1U  /**< |목표| >= 0.5 RPM (앞: PID 동작 / 뒤: FF 동작) */
#define DIAG_LF_CLAMP9999        0x2U  /**< FF+corr 가 ±MOTOR_PWM_MAX 클램프에 걸림 */
#define DIAG_LF_SAFE             0x4U  /**< ±MOTOR_PWM_SAFE_MAX 상한에 걸림 */
#define DIAG_LF_SLEW             0x8U  /**< 슬루 제한이 값을 바꿈 */

/**
 * 한 쪽(좌: FL+RL, 우: FR+RR) _drive_pair 캡처.
 * motor.c 가 채우고 MotorDiag_PushCycle() 이 직렬화합니다.
 */
typedef struct {
    float   ff_f, ff_r;      /**< FF 출력 [PWM] (비활성 바퀴는 0)            */
    float   p, i, corr;      /**< 앞바퀴 P, I 기여분, 총 보정량 [PWM]         */
    float   pre_f, pre_r;    /**< FF + 보정, 어떤 제한도 걸기 전 [PWM]        */
    int16_t cmd_f, cmd_r;    /**< ±9999 클램프 후 _set_wheel_safe 입력 [PWM]  */
    int16_t prev_f, prev_r;  /**< 호출 직전 슬루 기준값 [PWM]                 */
    int16_t out_f, out_r;    /**< 최종 인가 PWM(부호 포함) [PWM]              */
    uint8_t act_f, act_r;    /**< 앞 PID 동작 / 뒤 FF 동작                    */
    uint8_t valid;           /**< 이번 주기에 _drive_pair 가 실행됨           */
} MotorDiagPair_t;

/** 한 PID 주기 캡처 (motor.c 내부에서 채움). */
typedef struct {
    uint32_t tick_ms;
    uint8_t  path;
    uint8_t  stall_released;
    int16_t  cmd[4];          /**< 이번 주기에 쓴 CAN 원시 명령 FL,FR,RL,RR */
    int16_t  d_fl, d_fr;      /**< 원시 엔코더 델타 [tick] (FR 부호 보정 전) */
    float    meas_fl, meas_fr;/**< 펌웨어 측정 RPM (FR 은 펌웨어 부호 처리 후) */
    float    tgt[4];          /**< 목표 RPM (DRIVE 경로에서만 유효)          */
    MotorDiagPair_t pair[2];  /**< 0 = 좌(FL/RL), 1 = 우(FR/RR)               */
    /* 주기 종료 시점 상태 */
    uint8_t  wd_tripped, stall_tripped, cmd_ever, hw_ready;
    uint8_t  i_fl_hi, i_fl_lo, i_fr_hi, i_fr_lo;
    uint16_t stall_l_ms, stall_r_ms;
    uint16_t cmd_age_ms;      /**< 0xFFFF = 명령을 한 번도 못 받음 */
    uint32_t wd_trip_count;
} MotorDiagCycle_t;

/** 상수 스냅샷 (부팅 시 1회 + 5 s 마다 info 레코드로 전송) */
typedef struct {
    uint8_t  use_rr_encoder, output_inhibit;
    uint8_t  inv_fl, inv_fr, inv_rl, inv_rr;
    uint8_t  pid_period_ms;
    uint16_t encoder_cpr, pwm_max, safe_max, slew, stall_pwm, stall_hold_ms;
    uint16_t cmd_timeout_ms, fb_period_ms, stiction_base, kinetic_base;
    uint16_t can_cmd_id, can_fb_id;
    float    kp, ki, kd, ff_gain, max_rpm, smooth_rpm, stall_rpm, integral_limit;
} MotorDiagInfo_t;

#if MOTOR_DIAG_ENABLE

/* CAN RX ISR 카운터 (ISR 에서 증가, 메인 루프에서 읽기) */
extern volatile uint32_t g_diag_can_rx_any;
extern volatile uint32_t g_diag_can_rx_cmd;
extern volatile uint32_t g_diag_can_rx_overwrite;
#define MOTOR_DIAG_CAN_RX_ANY()          do { g_diag_can_rx_any++; } while (0)
#define MOTOR_DIAG_CAN_RX_CMD(pending)   do { g_diag_can_rx_cmd++; \
                                              if (pending) g_diag_can_rx_overwrite++; } while (0)

void     MotorDiag_Init(void);                       /**< MX_USART2_UART_Init 이후, 첫 printf 이전 */
void     MotorDiag_Poll(void);                       /**< 메인 루프 매 반복 (DMA 서비스 + 1 Hz health) */
void     MotorDiag_CycleBegin(void);                 /**< Motor_PID_Update 시작 */
void     MotorDiag_CycleEnd(void);                   /**< 제어 코어 종료 직후 (exec_us 측정) */
void     MotorDiag_PushCycle(const MotorDiagCycle_t *c);
void     MotorDiag_SetInfo(const MotorDiagInfo_t *info);
void     MotorDiag_CountFeedbackTx(uint8_t ok);
int      MotorDiag_WriteText(const char *ptr, int len);

#else  /* !MOTOR_DIAG_ENABLE */

#define MOTOR_DIAG_CAN_RX_ANY()          do { } while (0)
/* 인자를 평가하지 않습니다(volatile 읽기도 생기지 않게) -> 비활성 빌드 = 원본과 같은 기계어 */
#define MOTOR_DIAG_CAN_RX_CMD(pending)   do { } while (0)

#endif /* MOTOR_DIAG_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DIAG_H */
