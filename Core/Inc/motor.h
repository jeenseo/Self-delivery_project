/**
 * @file   motor.h
 * @brief  STM32 메카넘 4륜 모터 제어 — 4채널 독립 피드포워드 + PID + 안전계층
 *
 * ============================================================================
 * v4 재설계 요약 (레거시 정리 + 타이머 재배치 + 4채널 단일모드)
 * ============================================================================
 *
 * [1] 타이머 재배치 — RR 엔코더용 타이머 확보
 *   변경 전: TIM1(PWM x2) TIM2(PWM x2) TIM3(Enc L) TIM4(Enc R)  -> 4개 전부 사용
 *   변경 후: TIM2(PWM x4)              TIM3(Enc L) TIM4(Enc R)  -> **TIM1 해방**
 *
 *   ★ TIM2 는 partial remap 2 를 씁니다 (PA2/PA3 를 쓰지 않습니다).
 *     __HAL_AFIO_REMAP_TIM2_PARTIAL_2()  ->  CH1=PA0  CH2=PA1  CH3=PB10  CH4=PB11
 *
 *     [왜 PA2/PA3 가 아닌가]
 *       main.c 가 MX_USART2_UART_Init() 를 호출하고 printf 디버그를 씁니다.
 *       USART2 = PA2(TX)/PA3(RX) 이므로, 거기에 PWM 을 얹으면 디버그 UART 가
 *       죽습니다. PB10/PB11 은 비어 있으므로 remap 2 가 무손실입니다.
 *
 *     [배선 변경 최소화]
 *       RL(PA0), RR(PA1) 은 그대로 두고 FL/FR 만 옮깁니다. 이사 가는 선은 2개뿐.
 *
 * [2] 하드웨어 핀 매핑 (v4 확정)
 *   위치       │ DIR 핀 │ PWM 타이머/채널        │ PWM 핀   │ 비고
 *  ────────────┼───────┼──────────────────────┼─────────┼──────────────
 *   FL (좌전)  │ PC2   │ TIM2_CH3 (htim2)     │ PB10    │ ★ PA8 에서 이전
 *   FR (우전)  │ PC3   │ TIM2_CH4 (htim2)     │ PB11    │ ★ PA9 에서 이전
 *   RL (좌후)  │ PC0   │ TIM2_CH1 (htim2)     │ PA0     │ 변경 없음
 *   RR (우후)  │ PC1   │ TIM2_CH2 (htim2)     │ PA1     │ 변경 없음
 *   Enc FL     │  -    │ TIM3 (Encoder,partial)│ PB4,PB5 │ 변경 없음
 *   Enc FR     │  -    │ TIM4 (Encoder)        │ PB6,PB7 │ 변경 없음
 *   Enc RR     │  -    │ TIM1 (Encoder)        │ PA8,PA9 │ ★ 신규 (Phase 3)
 *   CAN        │  -    │ CAN1                  │ PA11,PA12│
 *   Debug UART │  -    │ USART2                │ PA2,PA3 │ 유지
 *
 * [3] 제어 구조 — 단일 모드 4채널 독립
 *   변경 전: CTRL_CLOSED_LOOP((fl+rl)/2 평균 PID)  <->  CTRL_OPEN_LOOP(4채널 FF)
 *            를 implied_Vy 임계로 **전환**
 *   변경 후: **모드 전환 없음.** 항상 4채널 독립 피드포워드 + 측면 PID 트림
 *
 *   [왜 모드 전환을 없앴는가]
 *     - 전환 순간 PID 적분이 리셋되고 FF+PID -> FF only 로 바뀌어 PWM 이 튑니다.
 *     - OPEN_LOOP 구간에서는 **피드백이 전혀 없습니다.** 게걸음 중 RL 부상으로
 *       생기는 편차를 아무도 잡아주지 못해 그대로 흘러갑니다(= 꿀렁임).
 *     - 임계(500) 근처에서 모드가 떨릴 수 있습니다.
 *     4채널 독립 FF 는 게걸음에서도 성립하므로 굳이 나눌 이유가 없습니다.
 *
 * [4] 안전 계층 (신규)
 *   - PWM 절대 상한 (튜닝 중 저속 강제)
 *   - 슬루 제한 (전류 스파이크 차단)
 *   - 스톨 감지 (물림/과부하 시 자동 차단)
 *   - 기존 명령 워치독 / s_hw_ready 는 유지
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════
 * 물리 상수
 * ═════════════════════════════════════════════════════════════════════════ */
#define WHEEL_DIAMETER_M        0.123f    /**< 바퀴 직경 [m] (줄자 실측 12.3cm) */
#define WHEEL_CIRCUMFERENCE_M   (WHEEL_DIAMETER_M * 3.14159265f)
#define TRACK_WIDTH_M           0.51f     /**< 윤거 [m] (줄자 실측)           */
#define WHEELBASE_M             0.50f     /**< 축거 [m] (URDF)                */

/**
 * 최대 부하 속도 [RPM].
 * ★ 모터 사양이 146 입니다. 기존 150 은 사양 초과값이었습니다.
 *   이 값은 "CAN 명령 9999 = 몇 RPM 인가" 의 정의이므로, 사양보다 크게 두면
 *   도달 불가능한 목표를 주게 되어 PID 적분이 계속 쌓입니다.
 */
#define MOTOR_MAX_RPM           146.0f

/**
 * 엔코더 해상도 [counts / 바퀴 1회전].
 *
 * ★ 실측 확정 (encoder_probe, 2회 독립 측정)
 *     1차: FL 27837 / 10rev = 2783.7    FR 26971 / 10rev = 2697.1
 *     2차: FL  2747 /  1rev = 2747.0    FR  2826 /  1rev = 2826.0
 *     회전수 가중평균 = 60381 ticks / 22 rev = **2744.6**
 *
 * ★ 교차 검증 — 완전히 독립적인 두 방법이 0.76% 안에서 일치
 *     A) 손으로 바퀴를 돌려 직접 센 값              : 2744.6
 *     B) 주행 실측 7049.4 ticks/m x 둘레 0.38642 m : 2724.0
 *   서로 아무 관계 없는 측정이 만났으므로 확정입니다.
 *
 * ※ 라벨(GP36E13CPR, 1:27)의 13 x 4 x 27 = 1404 와는 1.955배 차이입니다.
 *   13/27/x4 중 무엇이 표기와 다른지는 여전히 미상이지만, 실측이 우선입니다.
 *
 * ★★ 이 값을 1404 -> 2745 로 바꾸면 측정 RPM 이 1.955배 작아집니다.
 *   오차가 그만큼 커지고 PID 가 세게 밀므로 **실효 이득이 1.955배**가 됩니다.
 *   그래서 아래 MOTOR_KP/KI 를 미리 나눠 두었습니다.
 */
#define ENCODER_CPR             2745U

#define MOTOR_PWM_MAX           9999U     /**< TIM2 ARR=10000 기준 최대 듀티  */
#define PID_PERIOD_MS           10U       /**< PID 갱신 주기 [ms]             */

/* ═════════════════════════════════════════════════════════════════════════
 * ★ 안전 계층 (튜닝 중 하드웨어 보호)
 * ═════════════════════════════════════════════════════════════════════════ */

/**
 * PWM 절대 상한. 어떤 경로로도 이 값을 넘는 듀티가 나가지 않습니다.
 *
 * 튜닝 단계별 권장값 (FF Zone2 = 2000 + 70.0*(RPM-31.8), 지름 123mm 기준):
 *   4000 -> 목표  60.4 RPM = 0.389 m/s   ← 1차 튜닝. 여기서 시작하십시오
 *   6000 -> 목표  88.9 RPM = 0.573 m/s   ← 2차 검증
 *   9999 -> 목표 146.0 RPM = 0.940 m/s   ← 최종 (= 제한 해제)
 */
#define MOTOR_PWM_SAFE_MAX      4000

/**
 * 1 PID 주기(10 ms) 당 허용 PWM 변화량.
 * 스텝 명령이 그대로 나가면 돌입 전류로 드라이버가 상합니다.
 * 800 이면 0 -> 4000 까지 50 ms. 제어 대역(1~2 Hz)에 비해 충분히 빠릅니다.
 * ※ 비상정지 경로(_all_wheels_off / Motor_Stop)는 슬루를 거치지 않습니다.
 */
#define MOTOR_PWM_SLEW          800

/** 스톨 감지: PWM 이 이만큼인데 RPM 이 이만큼 이하로 이 시간 지속되면 차단 */
#define STALL_PWM_THRESH        2500
#define STALL_RPM_THRESH        3.0f
#define STALL_HOLD_MS           700U

/** 명령 워치독 — 이 시간 동안 CAN 명령이 없으면 전 바퀴 정지 */
#define CMD_TIMEOUT_MS          500U

/* ═════════════════════════════════════════════════════════════════════════
 * 2단계 피드포워드 (2-Stage Piecewise Feedforward)
 *
 *   Zone 1 (스티션):  0 < |RPM| <= SMOOTH_RPM
 *       PWM = 1300 + (|RPM|/31.8) * 700          -> 1300 ~ 2000
 *   Zone 2 (동역학):  |RPM| > SMOOTH_RPM
 *       PWM = 2000 + (|RPM|-31.8) * SLOPE        -> 2000 ~ 9999
 *   연결점 검증: Zone1(31.8) = 2000 = Zone2(31.8) ✓  (C0 연속)
 *
 *   ★ SMOOTH_RPM 31.8 검산: 31.8/60 x pi x 0.123 = 0.2048 m/s ✓
 *     지름을 120 -> 123mm 로 정정해도 여전히 '약 0.20 m/s' 입니다.
 *
 *   ⚠ 다만 이 곡선은 **구 CPR 로 부풀려진 RPM** 을 보고 잡은 값입니다.
 *     실측 대조: CAN 9999(목표 146 RPM)에서 실제는 약 103 RPM (달성률 70%).
 *     즉 FF 가 낙관적이라 목표에 못 미치고, 그 차이를 적분기가 메웁니다.
 *     -> KI 의 역할이 전보다 훨씬 커집니다. STEP 4 에서 눈여겨보십시오.
 *     -> 응답이 굼뜨면 KINETIC_PWM_BASE/SLOPE 를 실측으로 다시 잡으십시오.
 * ═════════════════════════════════════════════════════════════════════════ */
#define STICTION_PWM_BASE       1300U
#define KINETIC_PWM_BASE        2000U
#define SMOOTH_RPM              31.8f

/* ═════════════════════════════════════════════════════════════════════════
 * 방향 반전 (0 = DIR HIGH 가 전진, 1 = DIR LOW 가 전진)
 * ═════════════════════════════════════════════════════════════════════════ */
#define FL_DIR_INVERT           1
#define FR_DIR_INVERT           0
#define RL_DIR_INVERT           1
#define RR_DIR_INVERT           0

/* DIR 핀 (전부 GPIOC) */
#define FL_DIR_PIN              GPIO_PIN_2
#define FR_DIR_PIN              GPIO_PIN_3
#define RL_DIR_PIN              GPIO_PIN_0
#define RR_DIR_PIN              GPIO_PIN_1
#define MOTOR_DIR_PORT          GPIOC

/* PWM 채널 (전부 TIM2, partial remap 2) */
#define FL_PWM_CH               TIM_CHANNEL_3   /* PB10 */
#define FR_PWM_CH               TIM_CHANNEL_4   /* PB11 */
#define RL_PWM_CH               TIM_CHANNEL_1   /* PA0  */
#define RR_PWM_CH               TIM_CHANNEL_2   /* PA1  */

/* ═════════════════════════════════════════════════════════════════════════
 * PID 이득
 *
 * ⚠ 피드포워드 Zone2 기울기가 70.0 PWM/RPM 입니다. KP 가 그 대비 몇 분의 일인지가
 *   곧 'PID 가 가진 권한' 입니다. KP=1.0 이면 1/70 — 아직 거의 개루프입니다.
 *   RL 부상으로 좌측 추진이 1바퀴가 되는 상황을 메우려면 결국 KP 를 올려야 합니다.
 *
 *   [튜닝 순서]  1.0 -> 3 -> 5 -> 10 -> 20 (진동 나면 직전 값의 60%)
 *   KD 는 엔코더 양자화 잡음을 증폭하므로 0 에서 시작하는 것을 권합니다.
 *
 * ★★ 이번 CPR 수정으로 **PID 가 실제로 일을 하기 시작합니다.**
 *   구 CPR 1404 는 측정 RPM 을 1.955배 부풀려 보여줬습니다. 즉 PID 는
 *   "이미 목표를 넘었다" 고 판단해 오히려 **뒤로 당기고** 있었습니다.
 *   이제는 진짜 RPM 을 보므로 목표를 향해 **밀어붙입니다.**
 *   -> 같은 명령에서 로봇이 전보다 빨라집니다.
 *   -> MOTOR_PWM_SAFE_MAX 4000 이 여기서 실제로 일을 합니다. 낮춰 두십시오.
 * ═════════════════════════════════════════════════════════════════════════ */
/* ★ CPR 1404 -> 2745 로 실효 이득이 1.955배 커지므로 미리 나눠 둔 값입니다.
 *   구 KP 2.0 은 새 CPR 기준 3.91 상당이라 그대로 두면 첫 주행에서 진동합니다.
 *      KP 2.0 / 1.955 = 1.02  -> 1.0
 *      KI 1.0 / 1.955 = 0.51  -> 0.5
 *   여기서 시작해 5 -> 10 -> 20 순으로 올리며 진동을 확인하십시오. */
#define MOTOR_KP                3.0f
#define MOTOR_KI                2.0f
#define MOTOR_KD                0.0f
#define FF_GAIN                 1.11f

/* ═════════════════════════════════════════════════════════════════════════
 * CAN
 * ═════════════════════════════════════════════════════════════════════════ */
#define CAN_COMMAND_ID          0x123U    /**< Pi -> STM32 (DLC 8, int16 x4)  */
#define CAN_FEEDBACK_ID         0x124U    /**< STM32 -> Pi                    */
#define CAN_FEEDBACK_PERIOD_MS  20U

/**
 * ★ Phase 3 스위치 — RR 엔코더(TIM1) 사용 여부
 *
 *   0 : FL, FR 2채널 피드백. CAN 피드백 DLC = 4  (현재)
 *   1 : FL, FR, RR 3채널.    CAN 피드백 DLC = 6  (Phase 3)
 *
 *   ※ DLC 를 8 로 만들지 마십시오.
 *     SocketCAN 에러 프레임은 항상 DLC=8 이라, ROS 측 3중 방어 중
 *     "DLC 를 정확히 N 으로 요구" 층이 무력화됩니다.
 *     3륜(FL,FR,RR)이면 (vx,vy,wz) 가 모두 관측되므로 6바이트로 충분합니다.
 *     RL 은 접지가 불안정해 추정식에 넣으면 안 되므로 보낼 이유가 없습니다.
 */
#define USE_RR_ENCODER          0

/* ═════════════════════════════════════════════════════════════════════════
 * 공개 API
 * ═════════════════════════════════════════════════════════════════════════ */
void    Motor_Init(void);
void    Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void    Motor_PID_Update(void);
void    Motor_Send_Feedback_CAN(void);
void    Motor_Stop(void);
uint8_t Motor_Watchdog_IsTripped(void);
uint8_t Motor_Stall_IsTripped(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
