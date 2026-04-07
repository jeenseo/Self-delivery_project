"""
can_bridge.py
=============
Raspberry Pi CAN Bridge — STM32 모터 슬레이브 제어

프로토콜:
  CAN ID  : 0x123 (표준 11-bit)
  데이터  : 8 bytes, Big-Endian
  Byte 0~1: int16_t speed (-9999 ~ +9999)
  Byte 2~7: 예약 (0x00)

STM32 파싱:
  int16_t speed = (rxData[0] << 8) | rxData[1];
  Motor_Drive((int)speed);
  양수 = 전진, 음수 = 후진, 0 = 정지

의존성:
  pip install python-can
"""

import can
import struct
import time

# ─── 설정 ────────────────────────────────────────────────
CAN_CHANNEL = 'can0'      # SocketCAN 인터페이스 (Pi 기준)
CAN_BUSTYPE = 'socketcan'
CAN_ID      = 0x123       # STM32 수신 ID
CAN_BITRATE = 500000      # 500 kbps
# ─────────────────────────────────────────────────────────


def init_bus() -> can.BusABC:
    """SocketCAN 버스 초기화
    
    주의: bitrate는 OS 레벨에서 사전 설정 필요
      sudo ip link set can0 up type can bitrate 500000
    """
    return can.interface.Bus(
        channel=CAN_CHANNEL,
        bustype=CAN_BUSTYPE
    )


def send_motor_command(bus: can.BusABC, speed: int) -> None:
    """
    모터 명령 전송.

    Parameters
    ----------
    bus   : 초기화된 CAN 버스 객체
    speed : -9999 ~ +9999 (양수=전진, 음수=후진, 0=정지)
    """
    speed = max(-9999, min(9999, int(speed)))

    # '>h6x': Big-Endian int16, 6바이트 패딩 (총 8바이트)
    data = struct.pack('>h6x', speed)

    msg = can.Message(
        arbitration_id=CAN_ID,
        data=data,
        is_extended_id=False
    )
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"[CAN ERROR] {e}")


def motor_stop(bus: can.BusABC) -> None:
    """모터 정지 명령 전송"""
    send_motor_command(bus, 0)


# ─── 예제 실행 ────────────────────────────────────────────
if __name__ == '__main__':
    bus = init_bus()
    print("[CAN Bridge] 시작됨")

    try:
        # 전진 80% 속도로 3초
        print("[CMD] 전진 8000")
        send_motor_command(bus, 8000)
        time.sleep(3)

        # 후진 60% 속도로 2초
        print("[CMD] 후진 -6000")
        send_motor_command(bus, -6000)
        time.sleep(2)

        # 정지
        print("[CMD] 정지")
        motor_stop(bus)

    except KeyboardInterrupt:
        print("[CAN Bridge] 종료")
    finally:
        motor_stop(bus)
        bus.shutdown()
