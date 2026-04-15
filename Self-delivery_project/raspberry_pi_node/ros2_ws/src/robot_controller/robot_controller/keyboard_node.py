"""
keyboard_node.py
================
ROS 2 Node: 터미널 키보드 제어 (Wayland 호환)

pynput 없이 표준 라이브러리(termios + select)로 구현.
백그라운드 데몬 스레드가 키 입력을 읽고, 20Hz 타이머가 /cmd_vel 게시.

토픽 출력:
  /cmd_vel  (geometry_msgs/Twist) — MANUAL 모드일 때만 게시
  /mode     (std_msgs/String)     — "MANUAL" | "AUTO"

키 바인딩:
  m       : MANUAL ↔ AUTO 모드 전환
  ↑       : 전진
  ↓       : 후진
  ←       : 좌 포인트 턴
  →       : 우 포인트 턴
  Ctrl+C  : 종료

파라미터:
  forward_speed  float  0.2002  (전진 정규화 속도 = 2000/9999)
  turn_speed     float  0.2002  (회전 정규화 속도)
"""

import atexit
import os
import select
import signal
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('forward_speed', 0.2002)
        self.declare_parameter('turn_speed',    0.2002)

        self._fwd_spd = self.get_parameter('forward_speed').value
        self._trn_spd = self.get_parameter('turn_speed').value

        # ── 토픽 게시자 ───────────────────────────────────────────
        self._cmd_pub  = self.create_publisher(Twist,  '/cmd_vel', 10)
        self._mode_pub = self.create_publisher(String, '/mode',    10)

        # ── 공유 상태 ─────────────────────────────────────────────
        self._mode      = 'MANUAL'
        self._keys      = set()           # {'UP', 'DOWN', 'LEFT', 'RIGHT'}
        self._key_lock  = threading.Lock()
        self._orig_term = None            # 원본 터미널 설정

        # ── 키보드 백그라운드 스레드 ──────────────────────────────
        kb_t = threading.Thread(target=self._keyboard_loop, daemon=True)
        kb_t.start()

        # ── 20Hz 명령 게시 타이머 ─────────────────────────────────
        self._timer = self.create_timer(1.0 / 20.0, self._publish_cmd)

        self.get_logger().info(
            'KeyboardNode 준비 완료 (termios/select, Wayland 호환)\n'
            '  m: MANUAL↔AUTO  |  방향키: 이동  |  Ctrl+C: 종료'
        )

        # 시작 시 MANUAL 모드 게시
        self._publish_mode()

    # ── 터미널 복원 ────────────────────────────────────────────────
    def _restore_terminal(self) -> None:
        if self._orig_term is not None:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(), termios.TCSADRAIN, self._orig_term
                )
            except Exception:
                pass
            self._orig_term = None

    # ── 모드 게시 헬퍼 ───────────────────────────────────────────
    def _publish_mode(self) -> None:
        msg      = String()
        msg.data = self._mode
        self._mode_pub.publish(msg)

    # ── 키보드 루프 (백그라운드 데몬 스레드) ─────────────────────
    def _keyboard_loop(self) -> None:
        """
        cbreak 모드로 stdin을 읽어 키 이벤트를 처리.
        0.1s select 타임아웃 → 키 없음 = 릴리즈(clear)
        """
        ESCAPE_MAP = {
            '[A': 'UP',    # ↑
            '[B': 'DOWN',  # ↓
            '[C': 'RIGHT', # →
            '[D': 'LEFT',  # ←
        }

        fd = sys.stdin.fileno()
        self._orig_term = termios.tcgetattr(fd)
        atexit.register(self._restore_terminal)
        tty.setcbreak(fd)

        self.get_logger().debug('[KB] cbreak 모드 활성화')

        while rclpy.ok():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)

                if not ready:
                    # 타임아웃: 키 릴리즈 처리
                    with self._key_lock:
                        self._keys.clear()
                    continue

                ch = sys.stdin.read(1)

                # Ctrl+C 안전 처리
                if ch == '\x03':
                    self.get_logger().info('[KB] Ctrl+C 감지 — SIGINT 전송')
                    os.kill(os.getpid(), signal.SIGINT)
                    break

                # 'm' 키: 모드 토글
                elif ch == 'm':
                    self._mode = 'AUTO' if self._mode == 'MANUAL' else 'MANUAL'
                    self.get_logger().info(f'[MODE] → {self._mode}')
                    self._publish_mode()
                    with self._key_lock:
                        self._keys.clear()

                # 방향키 (ANSI 이스케이프 시퀀스 ESC[X)
                elif ch == '\x1b':
                    more_ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if more_ready:
                        seq    = sys.stdin.read(2)
                        key_id = ESCAPE_MAP.get(seq)
                        if key_id:
                            with self._key_lock:
                                self._keys.clear()
                                self._keys.add(key_id)
                    # else: 단독 ESC → 무시

                # 그 외 키 → 무시
                else:
                    with self._key_lock:
                        self._keys.clear()

            except Exception as exc:
                self.get_logger().error(f'[KB] 오류: {exc}')
                break

    # ── 20Hz 타이머: MANUAL 모드 명령 게시 ───────────────────────
    def _publish_cmd(self) -> None:
        if self._mode != 'MANUAL':
            return

        with self._key_lock:
            keys = set(self._keys)

        msg = Twist()
        if 'UP' in keys:
            msg.linear.x  =  self._fwd_spd    # 전진
        elif 'DOWN' in keys:
            msg.linear.x  = -self._fwd_spd    # 후진
        elif 'LEFT' in keys:
            msg.angular.z =  self._trn_spd    # 좌 포인트 턴
        elif 'RIGHT' in keys:
            msg.angular.z = -self._trn_spd    # 우 포인트 턴
        # else: 정지 (0, 0)

        self._cmd_pub.publish(msg)

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        self._restore_terminal()
        # 긴급 정지 명령 게시
        stop = Twist()
        self._cmd_pub.publish(stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
