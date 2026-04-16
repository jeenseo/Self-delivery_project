"""
motor_node.py
=============
ROS 2 Node: /cmd_vel → CAN 4-바이트 모터 명령 변환기

토픽 입력:
  /cmd_vel  (geometry_msgs/Twist)
    linear.x  : [-1.0, +1.0]  전진(+) / 후진(-)
    angular.z : [-1.0, +1.0]  좌회전(+) / 우회전(-)

CAN 출력:
  ID    : 0x123 (11-bit 표준)
  Byte 0-1 : int16_t Left  Motor Speed  Big-Endian  (-9999 ~ +9999)
  Byte 2-3 : int16_t Right Motor Speed  Big-Endian  (-9999 ~ +9999)

스키드-스티어 믹싱:
  left  = (linear - angular) × max_speed
  right = (linear + angular) × max_speed

파라미터:
  can_channel  str  'can0'
  can_id       int  0x123
  max_speed    int  9999
"""

import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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

        # ── CAN 버스 초기화 ───────────────────────────────────────
        try:
            self._bus = can.interface.Bus(channel=channel, bustype='socketcan')
            self.get_logger().info(f'CAN 버스 초기화 완료 ({channel}, ID=0x{self._can_id:03X})')
        except Exception as exc:
            self.get_logger().fatal(f'CAN 버스 초기화 실패: {exc}')
            raise

        # ── /cmd_vel 구독 ─────────────────────────────────────────
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10
        )
        self.get_logger().info('MotorNode 준비 완료')

    # ── /cmd_vel 콜백 ─────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist) -> None:
        """
        Twist → 수정된 스키드-스티어 믹싱 후 CAN 전송.

        물리 역분석 결과 (Up→전진, Left→좌회전 보장):
          Physical v = k*(L-R),  Physical ω = k*(L+R)
          역산:
            left  = (linear + angular) × max_speed
            right = (angular − linear) × max_speed

          Up   (linear=+1, ω=0) → L=+N, R=-N → 물리 전진 ✓
          Left (ω=+1, v=0)      → L=+N, R=+N → 물리 좌회전 ✓
        """
        linear  = max(-1.0, min(1.0, float(msg.linear.x)))
        angular = max(-1.0, min(1.0, float(msg.angular.z)))

        left  = int((linear + angular) * self._max_speed)
        right = int((angular - linear) * self._max_speed)
        left  = max(-self._max_speed, min(self._max_speed, left))
        right = max(-self._max_speed, min(self._max_speed, right))

        self._send_can(left, right)
        self.get_logger().debug(f'[CAN TX] L={left:+6d}  R={right:+6d}')

    # ── CAN 전송 ──────────────────────────────────────────────────
    def _send_can(self, left: int, right: int) -> None:
        """4-바이트 Big-Endian CAN 프레임 전송."""
        data = struct.pack('>hh', left, right)
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
            self._send_can(0, 0)   # 긴급 정지
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
