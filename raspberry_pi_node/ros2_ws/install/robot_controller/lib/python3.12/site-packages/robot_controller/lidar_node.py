"""
lidar_node.py
=============
ROS 2 Node: RPLIDAR A1 스캔 게시자 (Auto-Recovery 기능 추가)
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
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('fov_degrees', 60.0)
        self.declare_parameter('lidar_offset', 0.0)

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baud').value
        self._fov_degrees = self.get_parameter('fov_degrees').value
        self._offset = self.get_parameter('lidar_offset').value

        # ── 토픽 게시자 ───────────────────────────────────────────
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._dist_pub = self.create_publisher(Float32, '/forward_distance', 10)

        # ── LiDAR 초기화 ──────────────────────────────────────────
        self._lidar = self._init_lidar()
        
        # 백그라운드 스레드 시작
        scan_t = threading.Thread(target=self._scan_loop, daemon=True)
        scan_t.start()
        
        self.get_logger().info(
            f'LidarNode ready  port={self._port} '
            f'FOV=±{self._fov_degrees/2:.0f}° offset={self._offset:.0f}°'
        )

    def _init_lidar(self) -> RPLidar | None:
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
            lidar = RPLidar(self._port, baudrate=self._baud)
            lidar.reset()
            time.sleep(2.0)  # 센서 리부팅 대기

            # Step 3: 시리얼 버퍼 플러시
            lidar._serial.reset_input_buffer()
            try:
                lidar.clean_input() # 추가 버퍼 청소 (rplidar 버전에 따라 다름)
            except:
                pass

            # Step 4: 모터 시작
            lidar.start_motor()
            self.get_logger().info('[LIDAR] 모터 시작 완료')
            return lidar
        except Exception as exc:
            self.get_logger().error(f'[LIDAR] 초기화 실패: {exc}')
            return None

    def _scan_loop(self) -> None:
        half_fov = self._fov_degrees / 2.0
        NUM_BINS = 360

        # 🚨 [핵심 변경점] 에러가 나도 죽지 않고 무한 재시도하는 Auto-Recovery 루프
        while rclpy.ok():
            if self._lidar is None:
                self.get_logger().warn('[LIDAR] 센서가 연결되지 않았습니다. 3초 후 재연결을 시도합니다...')
                time.sleep(3.0)
                self._lidar = self._init_lidar()
                continue

            try:
                self.get_logger().info('[LIDAR] 스캐닝 시작')
                for scan in self._lidar.iter_scans(max_buf_meas=500):
                    now = self.get_clock().now().to_msg()

                    # ── LaserScan 메시지 구성 ───────────────────────
                    ls = LaserScan()
                    ls.header.stamp = now
                    ls.header.frame_id = 'lidar_link'
                    ls.angle_min = 0.0
                    ls.angle_max = 2.0 * math.pi
                    ls.angle_increment = (2.0 * math.pi) / NUM_BINS
                    ls.time_increment = 0.0
                    ls.scan_time = 0.1
                    ls.range_min = 0.15
                    ls.range_max = 12.0
                    ls.ranges = [0.0] * NUM_BINS

                    min_cm = float('inf')

                    for (quality, angle_raw, dist_mm) in scan:
                        if quality == 0 or dist_mm == 0:
                            continue

                        dist_m = dist_mm / 1000.0
                        idx = int(angle_raw) % NUM_BINS
                        if ls.ranges[idx] == 0.0 or dist_m < ls.ranges[idx]:
                            ls.ranges[idx] = dist_m

                        # 전방 FOV 필터
                        adj = (angle_raw - self._offset) % 360.0
                        if adj > 180.0:
                            adj -= 360.0

                        if -half_fov <= adj <= half_fov:
                            cm = dist_mm / 10.0
                            if cm < min_cm:
                                min_cm = cm

                    # ── 게시 ────────────────────────────────────────
                    self._scan_pub.publish(ls)

                    dist_msg = Float32()
                    dist_msg.data = float(min_cm) if min_cm != float('inf') else -1.0
                    self._dist_pub.publish(dist_msg)

            except Exception as exc:
                # 🚨 Mismatch 에러가 발생하면 여기서 잡아서 스스로 초기화!
                self.get_logger().error(f'[LIDAR] 스캔 루프 오류 발생: {exc}')
                self.get_logger().info('[LIDAR] ⚙️ Auto-Recovery 작동: 2초 후 센서를 강제 재부팅합니다...')
                
                # 기존 연결 완전히 파기
                try:
                    self._lidar.stop()
                    self._lidar.stop_motor()
                    self._lidar.disconnect()
                except:
                    pass
                
                self._lidar = None
                time.sleep(2.0) # 센서가 진정할 시간 부여

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

if __name__ == '__main__':
    main()
