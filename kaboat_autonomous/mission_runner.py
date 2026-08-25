"""
미션 실행기
SeaNU_KABOAT2024 main.py 포팅 (ROS2)
- 센서 데이터 구독
- 웨이포인트 추종
- 미션 상태 관리
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import Quaternion, PointStamped
import numpy as np
import time
from typing import List, Optional
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from controllers.autonomous_module import Boat, pathplan, goal_passed, rotate, normalize_angle

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS


def quaternion_to_yaw(q: Quaternion) -> float:
    """쿼터니언을 yaw 각도(도)로 변환"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw_rad = np.arctan2(siny_cosp, cosy_cosp)
    return np.degrees(yaw_rad)


class MissionRunner(Node):
    """
    미션 실행기
    - GPS, IMU, LiDAR 구독
    - 경로 계획 및 명령 발행
    - 웨이포인트 관리
    """

    def __init__(self):
        super().__init__('mission_runner')

        # 보트 상태
        self.boat = Boat()

        # 기준점 (SETTINGS와 동일한 고정 기준점 사용)
        self.ref_utm_x = SETTINGS.REF_UTM_X
        self.ref_utm_y = SETTINGS.REF_UTM_Y
        self.ref_initialized = True  # 이미 초기화됨

        # 미션 상태
        self.waypoints: List[tuple] = []
        self.current_waypoint_idx = 0
        self.is_running = False
        self.mission_complete = False

        # Publishers
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/command', 10)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, '/waypoint', 10)

        # Subscribers
        self.create_subscription(
            NavSatFix,
            SETTINGS.TOPICS['gps'],
            self.gps_callback,
            10
        )
        self.create_subscription(
            Imu,
            SETTINGS.TOPICS['imu'],
            self.imu_callback,
            10
        )
        self.create_subscription(
            LaserScan,
            SETTINGS.TOPICS['lidar'],
            self.lidar_callback,
            10
        )

        # 클릭으로 웨이포인트 수신 (시각화에서)
        self.create_subscription(
            PointStamped,
            '/waypoint_goal',
            self.waypoint_goal_callback,
            10
        )

        # 시각화 트랙바에서 실시간 파라미터 조정 수신
        self.create_subscription(
            Float32MultiArray,
            '/tuning_params',
            self.tuning_params_callback,
            10
        )

        # 제어 루프 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Mission Runner initialized')
        self.get_logger().info(f'Reference UTM: ({SETTINGS.REF_UTM_X:.2f}, {SETTINGS.REF_UTM_Y:.2f})')

    def gps_callback(self, msg: NavSatFix):
        """GPS 데이터 수신"""
        if msg.latitude != 0 and msg.longitude != 0:
            utm_x, utm_y, _ = SETTINGS.latlon_to_utm(msg.latitude, msg.longitude)

            # SETTINGS 기준점 기준 상대 좌표 (시각화와 동일한 좌표계)
            self.boat.position[0] = utm_x - self.ref_utm_x
            self.boat.position[1] = utm_y - self.ref_utm_y

    def imu_callback(self, msg: Imu):
        """IMU 데이터 수신"""
        self.boat.psi = quaternion_to_yaw(msg.orientation)

    def lidar_callback(self, msg: LaserScan):
        """LiDAR 데이터 수신"""
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)

        # 디버그: RAW(전처리 전) 각도 정의 및 최단거리 반사의 raw 인덱스/각도 확인
        import time as _time
        if not hasattr(self, '_last_raw_debug') or _time.time() - self._last_raw_debug > 2:
            self._last_raw_debug = _time.time()
            valid_mask = (ranges > 0) & (ranges < SETTINGS.LIDAR_MAX_RANGE)
            if np.any(valid_mask):
                idxs = np.where(valid_mask)[0]
                dists = ranges[idxs]
                order = np.argsort(dists)[:5]
                lines = []
                for k in order:
                    ri = idxs[k]
                    ang_deg = np.degrees(msg.angle_min + ri * msg.angle_increment)
                    lines.append(f"raw_idx={ri} raw_angle={ang_deg:.1f} dist={dists[k]:.2f}")
                self.get_logger().info(
                    f'[RAWLIDAR] angle_min={np.degrees(msg.angle_min):.1f} '
                    f'angle_max={np.degrees(msg.angle_max):.1f} '
                    f'increment={np.degrees(msg.angle_increment):.4f} n={len(msg.ranges)} '
                    f'boat.psi={self.boat.psi:.1f} | ' + ' | '.join(lines)
                )

        # 360도로 리샘플링
        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]

        # 최대 거리 제한
        ranges[ranges > SETTINGS.LIDAR_MAX_RANGE] = 0

        # 자기반사 필터: 실측 결과 LiDAR 마운트 포스트가 뱃머리 기준 약
        # -137°~-44°(94도 연속) 구간에서 0.35~0.5m로 계속 잡힘. 실제 위험
        # 판정 임계값(dist_danger=1.5m)보다 충분히 낮은 MIN_VALID_RANGE
        # 미만은 자기 구조물로 보고 "장애물 없음(0)"으로 처리한다.
        ranges[(ranges > 0) & (ranges < SETTINGS.MIN_VALID_RANGE)] = 0

        # 180도 회전 보정: raw LaserScan은 angle_min=-180°부터 시작하고,
        # 실측 결과 자기반사(마운트 포스트)가 정확히 이 raw index 0(=-180°,
        # 정후방) 부근에서 잡힘 -> 정면(0°)은 raw 배열의 정중앙(360 리샘플
        # 기준 index≈180)에 있다는 뜻. index 0이 정면이 되려면 180도 롤 필요.
        # (기존 -90 롤은 잘못된 값이었음 - 실측으로 확인 후 수정)
        ranges = np.roll(ranges, 180)

        self.boat.scan = ranges.tolist()

    def tuning_params_callback(self, msg: Float32MultiArray):
        """시각화 트랙바에서 실시간 파라미터 조정 수신 -> 즉시 SETTINGS에 반영"""
        names = ['BOAT_WIDTH', 'AVOID_RANGE', 'GAIN_PSI', 'GAIN_DISTANCE', 'GOAL_RANGE']
        for name, value in zip(names, msg.data):
            setattr(SETTINGS, name, float(value))

    def waypoint_goal_callback(self, msg: PointStamped):
        """시각화에서 클릭한 웨이포인트 수신"""
        x, y = msg.point.x, msg.point.y
        self.waypoints = [(x, y)]
        self.current_waypoint_idx = 0
        self.mission_complete = False
        self.is_running = True

        # 현재 보트 위치와 거리 계산
        dx = x - self.boat.position[0]
        dy = y - self.boat.position[1]
        dist = (dx**2 + dy**2)**0.5

        self.get_logger().info(f'=== WAYPOINT RECEIVED ===')
        self.get_logger().info(f'  Target: ({x:.1f}, {y:.1f})')
        self.get_logger().info(f'  Boat position: ({self.boat.position[0]:.1f}, {self.boat.position[1]:.1f})')
        self.get_logger().info(f'  Distance: {dist:.1f}m')
        self.get_logger().info(f'  is_running={self.is_running}, mission_complete={self.mission_complete}')

    def control_loop(self):
        """10Hz 제어 루프"""
        if not self.is_running or self.mission_complete:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.mission_complete = True
            self.stop()
            self.get_logger().info('Mission Complete!')
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]

        # 웨이포인트 도착 확인
        if goal_passed(self.boat, goal_x, goal_y, SETTINGS.GOAL_RANGE):
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} reached!'
            )
            self.current_waypoint_idx += 1
            return

        # 경로 계획
        psi_error, tau_x = pathplan(self.boat, goal_x, goal_y)

        # 목표 방향/각도 (ENU: arctan2(dy, dx)) - 디버그 로그와 /command
        # 4번째 값(goal_psi, 시각화 Cost 그래프용)에 공용으로 사용
        dx = goal_x - self.boat.position[0]
        dy = goal_y - self.boat.position[1]
        dist = (dx**2 + dy**2)**0.5
        goal_heading = np.arctan2(dy, dx) * 180 / np.pi
        goal_psi = normalize_angle(goal_heading - self.boat.psi)

        # 디버그 출력 (2초마다)
        import time
        if not hasattr(self, '_last_debug') or time.time() - self._last_debug > 2:
            self._last_debug = time.time()
            self.get_logger().info(
                f'=== DEBUG ===\n'
                f'  Boat: pos=({self.boat.position[0]:.1f}, {self.boat.position[1]:.1f}) psi={self.boat.psi:.1f}°\n'
                f'  Goal: ({goal_x:.1f}, {goal_y:.1f}) dist={dist:.1f}m\n'
                f'  Direction: goal_heading={goal_heading:.1f}° goal_psi={goal_psi:.1f}°\n'
                f'  Command: psi_error={psi_error:.1f}° tau_x={tau_x:.1f}'
            )

        # 명령 발행 (4번째 값: goal_psi - 시각화 Cost 그래프용)
        cmd = Float32MultiArray()
        cmd.data = [float(psi_error), float(tau_x), float(SETTINGS.MAX_THRUST), float(goal_psi)]
        self.cmd_pub.publish(cmd)

        # 디버그: 1초마다 명령 발행 확인
        if not hasattr(self, '_last_cmd_log') or time.time() - self._last_cmd_log > 1:
            self._last_cmd_log = time.time()
            self.get_logger().info(
                f'/command published: psi_err={psi_error:.1f}, tau_x={tau_x:.1f}, max={SETTINGS.MAX_THRUST}'
            )

        # 웨이포인트 시각화용
        wp = Float32MultiArray()
        wp.data = [0.0, float(goal_x), float(goal_y)]
        self.waypoint_pub.publish(wp)

    def set_waypoints(self, waypoints: List[tuple]):
        """웨이포인트 설정"""
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.mission_complete = False
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')

    def start(self):
        """미션 시작"""
        self.is_running = True
        self.get_logger().info('Mission Started')

    def stop(self):
        """미션 정지"""
        self.is_running = False
        cmd = Float32MultiArray()
        cmd.data = [0.0, 0.0, 0.0]
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Mission Stopped')

    def wait(self, seconds: float):
        """대기 (호핑투어 3초 정지용)"""
        self.get_logger().info(f'Waiting {seconds}s...')
        self.stop()
        time.sleep(seconds)

    def rotate_to(self, target_heading: float):
        """특정 방향으로 회전"""
        self.get_logger().info(f'Rotating to {target_heading} deg...')
        rate = self.create_rate(10)

        while rclpy.ok():
            psi_error, _ = rotate(self.boat, target_heading)
            if abs(psi_error) < 5:
                break

            cmd = Float32MultiArray()
            cmd.data = [float(psi_error), 0.0, float(SETTINGS.MAX_THRUST)]
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()

    # 자동 시작 없음 - 시각화에서 클릭으로 웨이포인트 설정 대기
    # is_running = False 상태로 대기
    node.get_logger().info('Waiting for waypoint from visualizer (click on Global Map)')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
