"""
robot_control.py
================
Raspberry Pi 5 — 자율 장애물 회피 & 수동 원격 제어

사용법:
  python robot_control.py

키 바인딩:
  m          : MANUAL ↔ AUTO 모드 전환
  ↑ (위)     : 전진  (L+2000, R+2000)
  ↓ (아래)   : 후진  (L-2000, R-2000)
  ← (왼쪽)  : 좌 포인트 턴  (L-2000, R+2000)
  → (오른쪽) : 우 포인트 턴  (L+2000, R-2000)
  Ctrl+C     : 종료

CAN 프로토콜 (STM32 수신):
  ID    : 0x123 (표준 11-bit)
  Byte 0-1 : int16_t Left  Motor Speed  (-9999 ~ +9999)
  Byte 2-3 : int16_t Right Motor Speed  (-9999 ~ +9999)
  형식  : Big-Endian, 총 4바이트

의존성:
  pip install python-can rplidar
  (pynput 불필요 — 표준 라이브러리 termios/select 사용)
"""

import atexit
import os
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty
import logging

import can
from rplidar import RPLidar

# ──────────────────────────────────────────────────────────────────────────────
#  튜닝 상수 (실제 로봇에 맞게 조정)
# ──────────────────────────────────────────────────────────────────────────────
FOV_DEGREES       = 60       # 전방 감시 시야각 (도). 120 또는 180으로 쉽게 변경 가능
TURN_10DEG_SEC    = 0.2      # 약 10도 회전에 필요한 시간 (초)
BACKWARD_10CM_SEC = 0.5      # 약 10 cm 후진에 필요한 시간 (초)
LIDAR_OFFSET      = 0        # LiDAR 0° 보정값 (도). LiDAR가 정면 기준에서 틀어진 경우 조정
OBSTACLE_DIST_CM  = 50       # 장애물 감지 임계 거리 (cm)
FORWARD_SPEED     = 2000     # 전진 속도 (-9999 ~ +9999)
TURN_SPEED        = 2000     # 포인트 턴 속도
AUTO_LOOP_HZ      = 10       # AUTO 모드 최대 CAN 전송률 (Hz) — CAN 버스 포화 방지

# ──────────────────────────────────────────────────────────────────────────────
#  하드웨어 설정
# ──────────────────────────────────────────────────────────────────────────────
CAN_CHANNEL  = 'can0'
CAN_BUSTYPE  = 'socketcan'
CAN_ID       = 0x123
LIDAR_PORT   = '/dev/ttyUSB0'
LIDAR_BAUD   = 115200

# ──────────────────────────────────────────────────────────────────────────────
#  로깅
# ──────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s][%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('robot')

# ──────────────────────────────────────────────────────────────────────────────
#  공유 상태
# ──────────────────────────────────────────────────────────────────────────────
MANUAL_MODE = 'MANUAL'
AUTO_MODE   = 'AUTO'

_mode      = MANUAL_MODE
_mode_lock = threading.Lock()

# 현재 눌린 키 집합 — 문자열 식별자: 'UP', 'DOWN', 'LEFT', 'RIGHT'
_keys_pressed = set()
_key_lock     = threading.Lock()

# LiDAR 최신 전방 최소 거리 (cm) — 백그라운드 스레드가 갱신
_lidar_min_dist = float('inf')
_lidar_lock     = threading.Lock()
_lidar_ready    = threading.Event()  # 첫 스캔 수신 시 세팅


# ──────────────────────────────────────────────────────────────────────────────
#  CAN 전송 헬퍼
# ──────────────────────────────────────────────────────────────────────────────
def send_motor(bus: can.BusABC, left: int, right: int) -> None:
    """
    좌/우 모터 속도 명령을 CAN으로 전송.

    Parameters
    ----------
    left  : 좌측 모터 속도 (-9999 ~ +9999)
    right : 우측 모터 속도 (-9999 ~ +9999)
    """
    left  = max(-9999, min(9999, int(left)))
    right = max(-9999, min(9999, int(right)))
    # Big-Endian: Byte 0-1 = Left, Byte 2-3 = Right
    data = struct.pack('>hh', left, right)
    msg  = can.Message(arbitration_id=CAN_ID, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        log.info('[CAN TX] L=%+6d  R=%+6d', left, right)
    except can.CanError as e:
        log.error('[CAN ERROR] %s', e)


def motor_stop(bus: can.BusABC) -> None:
    """즉시 정지 명령 전송."""
    send_motor(bus, 0, 0)


# ──────────────────────────────────────────────────────────────────────────────
#  LiDAR 백그라운드 스레드
# ──────────────────────────────────────────────────────────────────────────────
def _lidar_thread(lidar: RPLidar) -> None:
    """
    백그라운드에서 LiDAR를 지속적으로 읽어 전방 최소 거리를 갱신합니다.
    FOV_DEGREES 와 LIDAR_OFFSET 을 적용합니다.
    """
    global _lidar_min_dist
    half_fov = FOV_DEGREES / 2.0

    log.info('[LIDAR] 스캐닝 시작 (FOV=±%.0f°, offset=%d°)', half_fov, LIDAR_OFFSET)

    try:
        for scan in lidar.iter_scans(max_buf_meas=500):
            min_d = float('inf')
            for (quality, angle_raw, dist_mm) in scan:
                if quality == 0 or dist_mm == 0:
                    continue

                # LIDAR_OFFSET 보정 후 [-180, 180] 범위로 정규화
                angle = (angle_raw - LIDAR_OFFSET) % 360.0
                if angle > 180.0:
                    angle -= 360.0

                if -half_fov <= angle <= half_fov:
                    dist_cm = dist_mm / 10.0
                    if dist_cm < min_d:
                        min_d = dist_cm

            with _lidar_lock:
                _lidar_min_dist = min_d

            # 첫 스캔 수신 완료 알림
            if not _lidar_ready.is_set():
                _lidar_ready.set()
                log.info('[LIDAR] 첫 스캔 수신 완료')

    except Exception as e:
        log.error('[LIDAR] 스레드 오류: %s', e)


def get_forward_distance() -> float:
    """현재 전방 최소 거리 (cm) 반환. 스캔 미수신 시 inf."""
    with _lidar_lock:
        return _lidar_min_dist


# ──────────────────────────────────────────────────────────────────────────────
#  인터럽트 가능한 sleep 헬퍼
# ──────────────────────────────────────────────────────────────────────────────
def _sleep_or_interrupt(seconds: float, check_interval: float = 0.05) -> bool:
    """
    duration 초 동안 대기합니다.
    중간에 모드가 AUTO 에서 변경되면 즉시 반환합니다.

    Returns
    -------
    True  : 정상 완료
    False : 모드 변경으로 인터럽트됨
    """
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        with _mode_lock:
            if _mode != AUTO_MODE:
                return False
        remaining = end - time.monotonic()
        time.sleep(min(check_interval, remaining))
    return True


# ──────────────────────────────────────────────────────────────────────────────
#  터미널 I/O — Wayland 호환 키보드 입력
#  (pynput 대체: 표준 라이브러리 termios + select 사용)
# ──────────────────────────────────────────────────────────────────────────────
_orig_terminal_attrs = None   # 원본 터미널 설정 저장


def _restore_terminal() -> None:
    """원본 터미널 상태를 복원합니다. atexit 에 등록됩니다."""
    global _orig_terminal_attrs
    if _orig_terminal_attrs is not None:
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _orig_terminal_attrs)
        except Exception:
            pass
        _orig_terminal_attrs = None


def _set_terminal_cbreak() -> None:
    """
    stdin을 cbreak 모드로 전환합니다.
    - 키 입력이 즉시 전달됩니다 (Enter 불필요).
    - Ctrl+C (ISIG) 는 여전히 SIGINT를 발생시킵니다.
    - atexit 에 _restore_terminal 을 등록합니다.
    """
    global _orig_terminal_attrs
    fd = sys.stdin.fileno()
    _orig_terminal_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    atexit.register(_restore_terminal)


def _keyboard_thread() -> None:
    """
    백그라운드 데몬 스레드: 터미널에서 논블로킹으로 키 입력을 읽습니다.

    처리 규칙:
      'm'        → MANUAL ↔ AUTO 모드 전환
      ESC[A (↑)  → _keys_pressed = {'UP'}
      ESC[B (↓)  → _keys_pressed = {'DOWN'}
      ESC[D (←)  → _keys_pressed = {'LEFT'}
      ESC[C (→)  → _keys_pressed = {'RIGHT'}
      0x03       → SIGINT 전송 (Ctrl+C 안전 종료)
      타임아웃   → _keys_pressed.clear() (키 릴리즈 처리)
    """
    global _mode
    _set_terminal_cbreak()

    # ANSI 방향키 이스케이프 코드 맵
    ESCAPE_MAP = {
        '[A': 'UP',
        '[B': 'DOWN',
        '[C': 'RIGHT',
        '[D': 'LEFT',
    }

    log.info('[KB] 터미널 키보드 스레드 시작 (cbreak 모드)')

    while True:
        try:
            # 0.1초 타임아웃으로 논블로킹 대기
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)

            if not ready:
                # ── 타임아웃: 키가 눌리지 않음 → 릴리즈 처리 ──────
                with _key_lock:
                    _keys_pressed.clear()
                continue

            ch = sys.stdin.read(1)

            # ── Ctrl+C 안전 처리 ─────────────────────────────────
            if ch == '\x03':
                log.info('[KB] Ctrl+C 감지 — SIGINT 전송')
                os.kill(os.getpid(), signal.SIGINT)
                break

            # ── 모드 전환 ────────────────────────────────────────
            elif ch == 'm':
                with _mode_lock:
                    if _mode == MANUAL_MODE:
                        _mode = AUTO_MODE
                        log.info('[MODE] MANUAL → AUTO')
                    else:
                        _mode = MANUAL_MODE
                        log.info('[MODE] AUTO → MANUAL')
                # 모드 전환 시 키 집합 초기화
                with _key_lock:
                    _keys_pressed.clear()

            # ── 방향키 (ANSI 이스케이프 시퀀스) ─────────────────
            elif ch == '\x1b':
                # ESC 다음 최대 0.05초 내 추가 바이트 읽기
                more_ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if more_ready:
                    seq = sys.stdin.read(2)   # '[A', '[B', '[C', '[D'
                    key_id = ESCAPE_MAP.get(seq)
                    if key_id:
                        with _key_lock:
                            _keys_pressed.clear()
                            _keys_pressed.add(key_id)
                # else: 단독 ESC → 무시

            # ── 그 외 키 → 무시 ──────────────────────────────────
            else:
                with _key_lock:
                    _keys_pressed.clear()

        except Exception as e:
            log.error('[KB] 스레드 오류: %s', e)
            break


# ──────────────────────────────────────────────────────────────────────────────
#  수동 모드 명령 계산
# ──────────────────────────────────────────────────────────────────────────────
def _get_manual_command() -> tuple:
    """현재 눌린 키로 (left, right) 속도 반환. 아무 키도 없으면 (0, 0)."""
    with _key_lock:
        pressed = set(_keys_pressed)

    if 'UP' in pressed:
        return FORWARD_SPEED, FORWARD_SPEED           # 전진
    elif 'DOWN' in pressed:
        return -FORWARD_SPEED, -FORWARD_SPEED          # 후진
    elif 'LEFT' in pressed:
        return -TURN_SPEED, TURN_SPEED                 # 좌 포인트 턴
    elif 'RIGHT' in pressed:
        return TURN_SPEED, -TURN_SPEED                 # 우 포인트 턴
    else:
        return 0, 0


# ──────────────────────────────────────────────────────────────────────────────
#  AUTO 모드 — 장애물 회피 상태머신
# ──────────────────────────────────────────────────────────────────────────────
def _auto_loop(bus: can.BusABC) -> None:
    """
    AUTO 모드에서 장애물 회피를 수행합니다.
    모드가 MANUAL 로 전환되면 즉시 반환합니다.

    알고리즘:
      1. 전방 거리 > 50cm  → 전진
      2. 전방 거리 ≤ 50cm  → 정지 → 좌 회전(TURN_10DEG_SEC) → 재확인
      3. 재확인 후 여전히 막힘 → 후진(BACKWARD_10CM_SEC) → 좌 회전 → 전진 재개
    """
    interval = 1.0 / AUTO_LOOP_HZ
    log.info('[AUTO] 장애물 회피 루프 시작 (%.0fHz)', AUTO_LOOP_HZ)

    while True:
        t0 = time.monotonic()

        # 모드 확인
        with _mode_lock:
            if _mode != AUTO_MODE:
                log.info('[AUTO] 모드 변경 감지 → 루프 종료')
                return

        dist = get_forward_distance()
        log.info('[AUTO] 전방 거리: %.1f cm', dist)

        if dist > OBSTACLE_DIST_CM:
            # ── 전진 ──────────────────────────────────────────
            send_motor(bus, FORWARD_SPEED, FORWARD_SPEED)

        else:
            # ── 장애물 감지 ────────────────────────────────────
            log.warning('[AUTO] 장애물 감지! dist=%.1fcm → 정지', dist)
            motor_stop(bus)

            if not _sleep_or_interrupt(0.2):
                return

            # 1차 좌 회전
            log.info('[AUTO] 좌 포인트 턴 (%.2fs)', TURN_10DEG_SEC)
            send_motor(bus, -TURN_SPEED, TURN_SPEED)
            if not _sleep_or_interrupt(TURN_10DEG_SEC):
                return
            motor_stop(bus)
            if not _sleep_or_interrupt(0.1):
                return

            # 재확인
            dist2 = get_forward_distance()
            log.info('[AUTO] 재확인 거리: %.1f cm', dist2)

            if dist2 <= OBSTACLE_DIST_CM:
                # ── 여전히 막힘: 후진 후 재회전 ────────────────
                log.warning('[AUTO] 여전히 막힘! 후진 (%.2fs)', BACKWARD_10CM_SEC)
                send_motor(bus, -FORWARD_SPEED, -FORWARD_SPEED)
                if not _sleep_or_interrupt(BACKWARD_10CM_SEC):
                    return
                motor_stop(bus)
                if not _sleep_or_interrupt(0.1):
                    return

                # 2차 좌 회전
                log.info('[AUTO] 2차 좌 포인트 턴 (%.2fs)', TURN_10DEG_SEC)
                send_motor(bus, -TURN_SPEED, TURN_SPEED)
                if not _sleep_or_interrupt(TURN_10DEG_SEC):
                    return
                motor_stop(bus)
                if not _sleep_or_interrupt(0.1):
                    return

            # 전진 재개
            log.info('[AUTO] 전진 재개')
            send_motor(bus, FORWARD_SPEED, FORWARD_SPEED)

        # ── 루프 레이트 제한 (최대 AUTO_LOOP_HZ Hz) ──────────
        elapsed = time.monotonic() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)


# ──────────────────────────────────────────────────────────────────────────────
#  메인
# ──────────────────────────────────────────────────────────────────────────────
def main() -> None:
    global _mode

    log.info('=' * 50)
    log.info(' robot_control.py 시작')
    log.info(' m 키: MANUAL ↔ AUTO  |  Ctrl+C: 종료')
    log.info('=' * 50)

    # ── CAN 버스 초기화 ───────────────────────────────────────
    log.info('[INIT] CAN 버스 초기화 (%s @ 500kbps)', CAN_CHANNEL)
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype=CAN_BUSTYPE)

    # ── LiDAR 초기화 (강화 시퀀스) ───────────────────────────
    lidar = None
    log.info('[INIT] LiDAR 초기화 (%s)', LIDAR_PORT)
    try:
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)

        # Step 1: 하드웨어 리셋 — 이전 세션의 junk 상태 제거
        log.info('[INIT] LiDAR 하드웨어 리셋 중...')
        lidar.reset()

        # Step 2: 센서 리부팅 완료 대기 (최소 2초)
        log.info('[INIT] 리부팅 대기 (2.0s)...')
        time.sleep(2.0)

        # Step 3: 시리얼 입력 버퍼 플러시 — 남은 junk 바이트 제거
        log.info('[INIT] 시리얼 버퍼 플러시...')
        lidar._serial.reset_input_buffer()

        # Step 4: 정상 순서로 모터 시작
        lidar.start_motor()
        log.info('[INIT] LiDAR 모터 시작 완료')

    except Exception as e:
        log.error('[INIT] LiDAR 초기화 실패: %s', e)
        log.warning('[INIT] LiDAR 없이 계속 진행합니다 (AUTO 모드 비활성화).')
        lidar = None

    # ── LiDAR 백그라운드 스레드 시작 ─────────────────────────
    if lidar is not None:
        lidar_t = threading.Thread(target=_lidar_thread, args=(lidar,), daemon=True)
        lidar_t.start()

        log.info('[INIT] LiDAR 첫 스캔 대기...')
        _lidar_ready.wait(timeout=10.0)
        if not _lidar_ready.is_set():
            log.warning('[INIT] LiDAR 스캔 타임아웃 — 계속 진행합니다.')
    else:
        # LiDAR 없음: 전방 거리를 무한대로 유지 (항상 전진)
        _lidar_ready.set()

    # ── 터미널 키보드 스레드 시작 (Wayland 호환) ─────────────
    kb_thread = threading.Thread(target=_keyboard_thread, daemon=True)
    kb_thread.start()
    log.info('[INIT] 키보드 스레드 시작 (termios/select)')

    motor_stop(bus)
    log.info('[INIT] 준비 완료. 현재 모드: %s', _mode)
    log.info('-' * 50)

    prev_mode = _mode

    try:
        while True:
            with _mode_lock:
                mode = _mode

            # ── 모드 전환 시 즉시 정지 ────────────────────────
            if mode != prev_mode:
                log.info('[MAIN] 모드 전환: %s → %s', prev_mode, mode)
                motor_stop(bus)
                prev_mode = mode

            # ── MANUAL 모드 ────────────────────────────────────
            if mode == MANUAL_MODE:
                left, right = _get_manual_command()
                send_motor(bus, left, right)
                time.sleep(1.0 / 20.0)  # 20Hz 갱신

            # ── AUTO 모드 ──────────────────────────────────────
            else:
                _auto_loop(bus)
                # _auto_loop 가 반환되면 모드가 MANUAL 로 바뀐 것
                motor_stop(bus)
                with _mode_lock:
                    prev_mode = _mode  # 전환 완료 반영

    except KeyboardInterrupt:
        log.info('[EXIT] Ctrl+C 감지 — 종료 중...')

    finally:
        motor_stop(bus)
        _restore_terminal()   # 터미널 상태 즉시 복원

        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass

        bus.shutdown()
        log.info('[EXIT] 정상 종료.')


if __name__ == '__main__':
    main()
