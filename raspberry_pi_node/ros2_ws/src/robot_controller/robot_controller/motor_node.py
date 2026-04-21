"""
motor_node.py
=============
ROS 2 Node: /cmd_vel → CAN 8-바이트 4-휠 독립 모터 명령 변환기

CAN 출력 프로토콜 (ID 0x123, 8바이트):
  Byte 0-1 : int16_t FL (Front Left)   Big-Endian
  Byte 2-3 : int16_t FR (Front Right)  Big-Endian
  Byte 4-5 : int16_t RL (Rear Left)    Big-Endian
  Byte 6-7 : int16_t RR (Rear Right)   Big-Endian

믹싱 규약:
  Twist 입력: linear.x ∈ [-1,+1], angular.z ∈ [-1,+1]

  linear_v  = linear.x  × max_speed
  angular_v = angular.z × max_speed

  FL = clamp(linear_v + angular_v)   ← 전진 + 좌회전
  FR = clamp(linear_v - angular_v)   ← 전진 - 좌회전

  MANUAL 모드 (100% 전/후륜 동일):
    RL = FL,  RR = FR

  AUTO 모드 (후륜 angular 30% — mechanical binding 방지):
    RL = clamp(linear_v + angular_v × 0.3)
    RR = clamp(angular_v × 0.3 - linear_v)   ← STM32에서 -rr 반전 보정

파라미터:
  can_channel  str  'can0'
  can_id       int  0x123
  max_speed    int  9999
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import can


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_id',      0x123)
        self.declare_parameter('max_speed',   9999)

        channel         = self.get_parameter('can_channel').value
        self._can_id    = self.get_parameter('can_id').value
        self._max_speed = self.get_parameter('max_speed').value

        # ── 모드 상태 ─────────────────────────────────────────────
        self._mode      = 'MANUAL'   # 'MANUAL' | 'AUTO'
        self._mode_lock = threading.Lock()

        # ── CAN 버스 초기화 ───────────────────────────────────────
        try:
            self._bus = can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(
                f'CAN 버스 초기화 완료 ({channel}, ID=0x{self._can_id:03X})'
            )
        except Exception as exc:
            self.get_logger().fatal(f'CAN 버스 초기화 실패: {exc}')
            raise

        # ── 구독 ─────────────────────────────────────────────────
        self._sub_cmd  = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10
        )
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10
        )
        self.get_logger().info('MotorNode 준비 완료 (8바이트 CAN 프로토콜)')

    # ── /mode 콜백 ────────────────────────────────────────────────
    def _mode_cb(self, msg: String) -> None:
        with self._mode_lock:
            old         = self._mode
            self._mode  = msg.data
        if old != msg.data:
            self.get_logger().info(f'[MODE] {old} → {msg.data}')

    # ── /cmd_vel 콜백 ─────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist) -> None:
        """
        Twist → 4-휠 속도 계산 후 8바이트 CAN 전송.

        MANUAL: 전/후륜 동일 (100% 토크, 최대 제어력)
        AUTO  : 후륜 angular 30% (급선회 시 기계적 binding 방지)
        """
        linear  = max(-1.0, min(1.0, float(msg.linear.x)))
        angular = max(-1.0, min(1.0, float(msg.angular.z)))

        linear_v  = linear  * self._max_speed
        angular_v = angular * self._max_speed

        # ── 전륜: 100% 차동 ───────────────────────────────────────
        fl = int(linear_v + angular_v)
        fr = int(linear_v - angular_v)

        # ── 후륜: 모드에 따라 계산 ────────────────────────────────
        with self._mode_lock:
            mode = self._mode

        if mode == 'MANUAL':
            # MANUAL: 전/후륜 동일 (100%)
            rl = fl
            rr = fr
        else:
            # AUTO: 후륜 angular 30% (binding 방지)
            rl = int(linear_v + angular_v * 0.3)
            rr = int(angular_v * 0.3 - linear_v)  # STM32에서 -rr 반전

        # ── 클램프 ───────────────────────────────────────────────
        N  = self._max_speed
        fl = max(-N, min(N, fl))
        fr = max(-N, min(N, fr))
        rl = max(-N, min(N, rl))
        rr = max(-N, min(N, rr))

        self._send_can(fl, fr, rl, rr)
        self.get_logger().debug(
            f'[CAN TX] FL={fl:+6d} FR={fr:+6d} RL={rl:+6d} RR={rr:+6d}'
        )

    # ── CAN 전송 ──────────────────────────────────────────────────
    def _send_can(self, fl: int, fr: int, rl: int, rr: int) -> None:
        """8-바이트 Big-Endian CAN 프레임 전송."""
        data = struct.pack('>hhhh', fl, fr, rl, rr)
        msg  = can.Message(
            arbitration_id=self._can_id,
            data=data,
            is_extended_id=False
        )
        try:
            self._bus.send(msg)
        except can.CanError as exc:
            self.get_logger().error(f'[CAN ERROR] {exc}')

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self._send_can(0, 0, 0, 0)   # 긴급 정지
            self._bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
