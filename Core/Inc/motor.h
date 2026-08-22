/**
 * @file   motor.h
 * @brief  STM32 메카넘 4-휠 모터 제어 — 2단계 피드포워드 + PID + 엔코더 + CAN TX
 *         ★ v2: 명령 워치독(Command Watchdog) 추가
 *
 * [하드웨어 핀 매핑]
 *   위치    │ DIR 핀  │ PWM 타이머/채널      │ PWM 핀
 *  ─────────┼────────┼────────────────────┼────────
 *  FL (좌전) │ PC2    │ TIM1_CH1 (htim1)   │ PA8
 *  FR (우전) │ PC3    │ TIM1_CH2 (htim1)   │ PA9
 *  RL (좌후) │ PC0    │ TIM2_CH1 (htim2)   │ PA0
 *  RR (우후) │ PC1    │ TIM2_CH2 (htim2)   │ PA1
 *  Left Enc  │  -     │ TIM3 (Encoder)     │ PB4, PB5   ← 실제로는 FL 한 바퀴만
 *  Right Enc │  -     │ TIM4 (Encoder)     │ PB6, PB7   ← 실제로는 FR 한 바퀴만
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────
 * 물리 상수 (Physical Robot Specifications)
 * ───────────────────────────────────────────────────────────── */
#define WHEEL_DIAMETER_M        0.12f           /**< 바퀴 직경 (m)         */
#define WHEEL_CIRCUMFERENCE_M   (WHEEL_DIAMETER_M * 3.14159265f)
#define TRACK_WIDTH_M           0.51f           /**< 좌우 트랙 폭 (m)      */
#define MOTOR_MAX_RPM           150.0f          /**< 최대 부하 속도 (RPM)   */
#define ENCODER_CPR             1404U           /**< 엔코더 해상도 (CPR)    */
#define MOTOR_PWM_MAX           9999U           /**< 최대 PWM 값            */
#define PID_PERIOD_MS           10U             /**< PID 갱신 주기 (ms)     */

/* ═════════════════════════════════════════════════════════════
 * ★ 명령 워치독 (Command Watchdog) — v2 신규
 *
 * [무엇을 막는가]
 *   Pi(ROS 2)가 죽거나 CAN 이 끊겨도 STM32 는 마지막 PWM 을 무한히 유지합니다.
 *   PWM 레지스터는 한 번 쓰면 전원이 끊길 때까지 그대로이므로, 명령 소스가
 *   사라지면 로봇은 '마지막 속도로 영원히' 달립니다. 이것이 폭주의 정체입니다.
 *
 * [타임아웃 선택 근거]
 *   Pi 측 motor_node 가 50 Hz(20ms) 로 CAN 하트비트를 보내므로,
 *   타임아웃 500ms 는 연속 25 프레임 유실까지 견딥니다 (매우 넉넉).
 *
 *     타임아웃 | 허용 유실 프레임 | 0.95 m/s 에서 폭주 거리
 *     ─────────┼─────────────────┼────────────────────────
 *      500 ms  |      25         |   0.48 m
 *      300 ms  |      15         |   0.29 m
 *      200 ms  |      10         |   0.19 m   ← 실내 주행 권장
 *
 *   요청하신 500ms 를 기본값으로 두었습니다. 하트비트가 안정적으로 확인되면
 *   200~300ms 로 조이시길 권합니다. 폭주 거리가 절반 이하로 줄어듭니다.
 *
 * [계층 구조 — 이 워치독 하나로는 부족합니다]
 *   1층 (Pi)    : motor_node 하트비트 타이머     → cmd_vel 끊김 대응
 *   2층 (STM32) : 본 명령 워치독 (아래)          → Pi 사망 / CAN 두절 대응
 *   3층 (STM32) : IWDG 독립 워치독 (main.c)      → ★ 펌웨어 자체가 멈췄을 때
 *   4층 (물리)  : 모터 전원 차단 E-Stop 스위치   → 위 전부가 실패했을 때
 *
 *   2층은 '메인 루프가 돌고 있을 때'만 동작합니다. 메인 루프가 블로킹되면
 *   2층도 같이 멈추므로 3층 IWDG 가 반드시 필요합니다.
 *   소프트웨어는 4층을 대체할 수 없습니다. 물리 E-Stop 을 반드시 다십시오.
 * ═════════════════════════════════════════════════════════════ */
#define CMD_TIMEOUT_MS          500U    /**< 이 시간 동안 CAN 명령 없으면 전 모터 정지 */

/* ─────────────────────────────────────────────────────────────
 * 2단계 피드포워드 상수 (2-Stage Piecewise Feedforward)
 * ───────────────────────────────────────────────────────────── */
#define STICTION_PWM_BASE       1300U           /**< 정지마찰 극복 Zone 1 시작 PWM */
#define KINETIC_PWM_BASE        2000U           /**< Zone 2 기저 PWM (연결점)      */
#define SMOOTH_RPM              31.8f           /**< Zone 전환 RPM (≈0.20 m/s)     */

/* ─────────────────────────────────────────────────────────────
 * 방향 반전 매크로 (Direction Inversion per Wheel)
 * ───────────────────────────────────────────────────────────── */
#define FL_DIR_INVERT           1
#define FR_DIR_INVERT           0
#define RL_DIR_INVERT           1
#define RR_DIR_INVERT           0

/* ─────────────────────────────────────────────────────────────
 * PID 조정 파라미터 (Tunable PID Gains)
 *
 * ⚠ 분석 결과: 현재 KP=2.0 은 Zone2 피드포워드 기울기(67.7 PWM/RPM) 대비
 *   권한이 약 1/34 에 불과해 사실상 개루프로 동작합니다. 그 결과 부하가
 *   걸리면 목표 RPM 에 도달하지 못하고, 이것이 '속도에 비례하는 슬립'의 원인입니다.
 *   개선하려면 5 → 10 → 20 순으로 단계적으로 올리며 진동 여부를 확인하십시오.
 *   (본 커밋에서는 안전 기능만 다루므로 값은 그대로 둡니다)
 * ───────────────────────────────────────────────────────────── */
#define MOTOR_KP                2.0f    /**< 비례 이득 */
#define MOTOR_KI                1.0f    /**< 적분 이득 (부하 변동 대응) */
#define MOTOR_KD                0.05f   /**< 미분 이득 */

/* ─────────────────────────────────────────────────────────────
 * CAN 피드백 설정
 * ───────────────────────────────────────────────────────────── */
#define CAN_FEEDBACK_ID         0x124U  /**< 엔코더 피드백 CAN ID     */
#define CAN_FEEDBACK_PERIOD_MS  20U     /**< CAN TX 주기 (ms)         */

/* ─────────────────────────────────────────────────────────────
 * 공개 API (Public API)
 * ───────────────────────────────────────────────────────────── */

void    Motor_Init(void);
void    Motor_Drive(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void    Motor_PID_Update(void);
void    Motor_Send_Feedback_CAN(void);
void    Motor_Stop(void);

/* ── ★ v2 신규: 워치독 상태 조회 ───────────────────────────── */

/**
 * @brief  명령 워치독이 발동(통신 두절 판정)했는지 조회
 * @retval 1 = 두절 상태(모터 강제 정지 중), 0 = 정상
 * @note   부팅 직후 첫 CAN 명령을 받기 전까지는 1 을 반환합니다(페일세이프).
 */
uint8_t Motor_Watchdog_IsTripped(void);

/**
 * @brief  마지막 유효 명령 이후 경과 시간 [ms]
 * @note   진단/로깅용. 명령을 한 번도 못 받았으면 0xFFFFFFFF.
 */
uint32_t Motor_Watchdog_ElapsedMs(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */