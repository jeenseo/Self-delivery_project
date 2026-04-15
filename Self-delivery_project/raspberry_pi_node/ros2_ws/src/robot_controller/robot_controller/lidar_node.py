"""
lidar_node.py
=============
ROS 2 Node: RPLIDAR A1 스캔 게시자

토픽 출력:
  /scan              (sensor_msgs/LaserScan) — 360° 원시 스캔
  /forward_distance  (std_msgs/Float32)      — 전방 FOV 최소 거리 (cm)
                                               장애물 없으면 -1.0

파라미터:
  port           str    /dev/ttyUSB0
  baud           int    115200
  fov_degrees    float  60.0   (전방 감시 시야각, 쉽게 120/180 으로 변경 가능)
  lidar_offset   float  0.0    (LiDAR 0° 보정값)

LiDAR 초기화 강화:
  DTR/RTS 리셋 → 하드웨어 reset() → 버퍼 플러시 → start_motor()
  'Descriptor length mismatch' 오류 방지
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import serial
from rplidar import RPLidar


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        # ── 파라미터 선언 ─────────────────────────────────────────
        self.declare_parameter('port',          '/dev/ttyUSB0')
        self.declare_parameter('baud',          115200)
        self.declare_parameter('fov_degrees',   60.0)
        self.declare_parameter('lidar_offset',  0.0)

        self._port        = self.get_parameter('port').value
        self._baud        = self.get_parameter('baud').value
        self._fov_degrees = self.get_parameter('fov_degrees').value
        self._offset      = self.get_parameter('lidar_offset').value

        # ── 토픽 게시자 ───────────────────────────────────────────
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._dist_pub = self.create_publisher(Float32, '/forward_distance', 10)

        # ── LiDAR 초기화 ──────────────────────────────────────────
        self._lidar = self._init_lidar()

        if self._lidar is not None:
            scan_t = threading.Thread(target=self._scan_loop, daemon=True)
            scan_t.start()
            self.get_logger().info(
                f'LidarNode ready  port={self._port}  '
                f'FOV=±{self._fov_degrees/2:.0f}°  offset={self._offset:.0f}°'
            )
        else:
            self.get_logger().error('LiDAR 초기화 실패 — 노드는 계속 실행되지만 데이터 없음')

    # ── LiDAR 초기화 (Descriptor length mismatch 방지) ────────────
    def _init_lidar(self) -> RPLidar | None:
        """
        3단계 초기화:
          1) DTR/RTS 리셋 (serial 직접 제어)
          2) RPLidar.reset() 소프트웨어 리셋
          3) 시리얼 입력 버퍼 플러시
        """
        try:
            # Step 1: DTR 신호로 하드웨어 리셋 (junk 데이터 제거)
            self.get_logger().info('[LIDAR] DTR/RTS 리셋 중...')
            s = serial.Serial(self._port, self._baud, timeout=1)
            s.setDTR(False)
            time.sleep(0.1)
            s.reset_input_buffer()
            s.setDTR(True)
            s.close()
            time.sleep(0.5)

            # Step 2: RPLidar 인스턴스 생성 후 소프트웨어 리셋
            self.get_logger().info('[LIDAR] 소프트웨어 리셋 중...')
            lidar = RPLidar(self._port, baudrate=self._baud)
            lidar.reset()
            time.sleep(2.0)   # 센서 리부팅 대기

            # Step 3: 시리얼 버퍼 플러시 (남은 junk 제거)
            self.get_logger().info('[LIDAR] 시리얼 버퍼 플러시...')
            lidar._serial.reset_input_buffer()

            # Step 4: 모터 시작
            lidar.start_motor()
            self.get_logger().info('[LIDAR] 모터 시작 완료')
            return lidar

        except Exception as exc:
            self.get_logger().error(f'[LIDAR] 초기화 실패: {exc}')
            return None

    # ── 스캔 루프 (백그라운드 스레드) ────────────────────────────
    def _scan_loop(self) -> None:
        half_fov = self._fov_degrees / 2.0
        NUM_BINS = 360

        self.get_logger().info('[LIDAR] 스캐닝 시작')

        try:
            for scan in self._lidar.iter_scans(max_buf_meas=500):
                now = self.get_clock().now().to_msg()

                # ── LaserScan 메시지 구성 ───────────────────────
                ls = LaserScan()
                ls.header.stamp    = now
                ls.header.frame_id = 'lidar_link'
                ls.angle_min       = 0.0
                ls.angle_max       = 2.0 * math.pi
                ls.angle_increment = (2.0 * math.pi) / NUM_BINS
                ls.time_increment  = 0.0
                ls.scan_time       = 0.1
                ls.range_min       = 0.15
                ls.range_max       = 12.0
                ls.ranges          = [0.0] * NUM_BINS

                min_cm = float('inf')

                for (quality, angle_raw, dist_mm) in scan:
                    if quality == 0 or dist_mm == 0:
                        continue

                    dist_m = dist_mm / 1000.0
                    idx = int(angle_raw) % NUM_BINS
                    if ls.ranges[idx] == 0.0 or dist_m < ls.ranges[idx]:
                        ls.ranges[idx] = dist_m

                    # 전방 FOV 필터 (LIDAR_OFFSET 적용)
                    adj = (angle_raw - self._offset) % 360.0
                    if adj > 180.0:
                        adj -= 360.0

                    if -half_fov <= adj <= half_fov:
                        cm = dist_mm / 10.0
                        if cm < min_cm:
                            min_cm = cm

                # ── 게시 ────────────────────────────────────────
                self._scan_pub.publish(ls)

                dist_msg      = Float32()
                dist_msg.data = float(min_cm) if min_cm != float('inf') else -1.0
                self._dist_pub.publish(dist_msg)

        except Exception as exc:
            self.get_logger().error(f'[LIDAR] 스캔 루프 오류: {exc}')

    # ── 노드 소멸 ─────────────────────────────────────────────────
    def destroy_node(self):
        if self._lidar is not None:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
