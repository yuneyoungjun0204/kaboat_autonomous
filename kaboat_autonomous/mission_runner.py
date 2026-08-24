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
from geometry_msgs.msg import Quaternion
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

        # 기준점 (첫 GPS 수신 시 자동 설정)
        self.ref_utm_x = None
        self.ref_utm_y = None
        self.ref_initialized = False

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

        # 제어 루프 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Mission Runner initialized')
        self.get_logger().info(f'Reference UTM: ({SETTINGS.REF_UTM_X:.2f}, {SETTINGS.REF_UTM_Y:.2f})')

    def gps_callback(self, msg: NavSatFix):
        """GPS 데이터 수신"""
        if msg.latitude != 0 and msg.longitude != 0:
            utm_x, utm_y, _ = SETTINGS.latlon_to_utm(msg.latitude, msg.longitude)

            # 첫 GPS 수신 시 현재 위치를 기준점으로 설정
            if not self.ref_initialized:
                self.ref_utm_x = utm_x
                self.ref_utm_y = utm_y
                self.ref_initialized = True
                self.get_logger().info(
                    f'Reference point set: UTM ({utm_x:.2f}, {utm_y:.2f})'
                )

            # 기준점 기준 상대 좌표
            self.boat.position[0] = utm_x - self.ref_utm_x
            self.boat.position[1] = utm_y - self.ref_utm_y

    def imu_callback(self, msg: Imu):
        """IMU 데이터 수신"""
        self.boat.psi = quaternion_to_yaw(msg.orientation)

    def lidar_callback(self, msg: LaserScan):
        """LiDAR 데이터 수신"""
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)

        # 360도로 리샘플링
        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]

        # 최대 거리 제한
        ranges[ranges > SETTINGS.LIDAR_MAX_RANGE] = 0

        self.boat.scan = ranges.tolist()

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

        # 명령 발행
        cmd = Float32MultiArray()
        cmd.data = [float(psi_error), float(tau_x), float(SETTINGS.MAX_THRUST)]
        self.cmd_pub.publish(cmd)

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

    # 테스트 웨이포인트 (시뮬레이터용 상대 좌표)
    test_waypoints = [
        (10.0, 0.0),   # 전방 10m
        (10.0, 10.0),  # 우측 10m
        (0.0, 10.0),   # 후방 10m
        (0.0, 0.0),    # 시작점
    ]

    node.set_waypoints(test_waypoints)
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
