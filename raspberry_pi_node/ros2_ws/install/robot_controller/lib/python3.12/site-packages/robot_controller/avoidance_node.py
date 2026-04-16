"""
avoidance_node.py
=================
ROS 2 Node: 장애물 회피 자율 주행 (AUTO 모드)

토픽 입력:
  /forward_distance  (std_msgs/Float32)   — 전방 최소 거리 (cm), -1 = 미수신
  /mode              (std_msgs/String)    — "MANUAL" | "AUTO"

토픽 출력:
  /cmd_vel           (geometry_msgs/Twist) — AUTO 모드일 때만 게시

상태머신 (10Hz 타이머 기반):
  FORWARD        → 전진
  STOP_BEFORE_TURN → 0.2s 정지
  TURN_LEFT_1    → 좌 포인트 턴 (turn_sec)
  RECHECK        → 0.1s 대기 후 거리 재확인
  BACKWARD       → 후진 (back_sec)
  TURN_LEFT_2    → 2차 좌 포인트 턴 (turn_sec)

파라미터:
  obstacle_dist_cm   float  50.0   (장애물 감지 임계 거리)
  turn_10deg_sec     float  0.2    (10도 회전 시간)
  backward_10cm_sec  float  0.5    (10cm 후진 시간)
  forward_speed      float  0.2002 (전진 정규화 속도 = 2000/9999)
  turn_speed         float  0.2002 (회전 정규화 속도)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


class AvoidanceNode(Node):

    def __init__(self):
        super().__init__('avoidance_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('obstacle_dist_cm',   50.0)
        self.declare_parameter('turn_10deg_sec',     0.2)
        self.declare_parameter('backward_10cm_sec',  0.5)
        self.declare_parameter('forward_speed',      0.2002)
        self.declare_parameter('turn_speed',         0.2002)

        self._obs_dist  = self.get_parameter('obstacle_dist_cm').value
        self._turn_sec  = self.get_parameter('turn_10deg_sec').value
        self._back_sec  = self.get_parameter('backward_10cm_sec').value
        self._fwd_spd   = self.get_parameter('forward_speed').value
        self._trn_spd   = self.get_parameter('turn_speed').value

        # ── 공유 상태 ─────────────────────────────────────────────
        self._lock     = threading.Lock()
        self._mode     = 'MANUAL'
        self._dist_cm  = float('inf')

        # ── 상태머신 ──────────────────────────────────────────────
        self._state       = 'FORWARD'
        self._state_start = time.monotonic()

        # ── 토픽 게시자 / 구독자 ──────────────────────────────────
        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._sub_dist = self.create_subscription(
            Float32, '/forward_distance', self._dist_cb, 10
        )
        self._sub_mode = self.create_subscription(
            String, '/mode', self._mode_cb, 10
        )

        # ── 10Hz 상태머신 타이머 ──────────────────────────────────
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info('AvoidanceNode 준비 완료 (10Hz)')

    # ── 콜백 ──────────────────────────────────────────────────────
    def _dist_cb(self, msg: Float32) -> None:
        with self._lock:
            self._dist_cm = float(msg.data) if msg.data >= 0 else float('inf')

    def _mode_cb(self, msg: String) -> None:
        with self._lock:
            old        = self._mode
            self._mode = msg.data
        if old != msg.data:
            self.get_logger().info(f'모드 전환: {old} → {msg.data}')
            if msg.data == 'AUTO':
                self._state       = 'FORWARD'
                self._state_start = time.monotonic()

    # ── Twist 게시 헬퍼 ───────────────────────────────────────────
    def _pub(self, linear: float, angular: float) -> None:
        msg           = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    # ── 상태머신 틱 (10Hz 타이머) ─────────────────────────────────
    def _tick(self) -> None:
        with self._lock:
            mode = self._mode
            dist = self._dist_cm

        if mode != 'AUTO':
            return

        now = time.monotonic()
        s   = self._state

        # ── FORWARD ──────────────────────────────────────────────
        if s == 'FORWARD':
            if dist > self._obs_dist:
                self._pub(self._fwd_spd, 0.0)
            else:
                self.get_logger().warn(
                    f'[AUTO] 장애물 감지! dist={dist:.1f}cm → 정지'
                )
                self._pub(0.0, 0.0)
                self._state       = 'STOP_BEFORE_TURN'
                self._state_start = now

        # ── STOP_BEFORE_TURN (0.2s 정지) ─────────────────────────
        elif s == 'STOP_BEFORE_TURN':
            if now - self._state_start >= 0.2:
                self.get_logger().info(
                    f'[AUTO] 1차 좌 포인트 턴 ({self._turn_sec:.2f}s)'
                )
                self._state       = 'TURN_LEFT_1'
                self._state_start = now

        # ── TURN_LEFT_1 ───────────────────────────────────────────
        elif s == 'TURN_LEFT_1':
            self._pub(0.0, self._trn_spd)
            if now - self._state_start >= self._turn_sec:
                self._pub(0.0, 0.0)
                self._state       = 'RECHECK'
                self._state_start = now

        # ── RECHECK (0.1s 대기 후 재확인) ────────────────────────
        elif s == 'RECHECK':
            if now - self._state_start >= 0.1:
                if dist > self._obs_dist:
                    self.get_logger().info('[AUTO] 경로 확보 → 전진 재개')
                    self._state = 'FORWARD'
                else:
                    self.get_logger().warn(
                        f'[AUTO] 여전히 막힘! 후진 ({self._back_sec:.2f}s)'
                    )
                    self._state       = 'BACKWARD'
                    self._state_start = now

        # ── BACKWARD ─────────────────────────────────────────────
        elif s == 'BACKWARD':
            self._pub(-self._fwd_spd, 0.0)
            if now - self._state_start >= self._back_sec:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f'[AUTO] 2차 좌 포인트 턴 ({self._turn_sec:.2f}s)'
                )
                self._state       = 'TURN_LEFT_2'
                self._state_start = now

        # ── TURN_LEFT_2 ───────────────────────────────────────────
        elif s == 'TURN_LEFT_2':
            self._pub(0.0, self._trn_spd)
            if now - self._state_start >= self._turn_sec:
                self._pub(0.0, 0.0)
                self.get_logger().info('[AUTO] 회피 완료 → 전진 재개')
                self._state = 'FORWARD'


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
