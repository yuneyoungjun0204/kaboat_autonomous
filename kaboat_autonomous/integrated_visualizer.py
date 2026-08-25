#!/usr/bin/env python3
"""
KABOAT 통합 시각화 시스템
- 왼쪽: Global Map (Cartesian) - 보트 이동 궤적, 클릭으로 목적지 설정
- 오른쪽: Local Map (Polar) - LiDAR, 명령 방향, 안전 구역

SeaNU_KABOAT2024 기반 ROS2 포팅
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import PointStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Polygon, FancyArrowPatch
import threading
from collections import deque
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

from kaboat_autonomous.controllers.autonomous_module import calculate_safe_zone, normalize_angle


class IntegratedVisualizer(Node):
    """통합 시각화 노드"""

    def __init__(self):
        super().__init__('integrated_visualizer')

        # 상태 변수
        self.gps_position = [0.0, 0.0]  # UTM 좌표 (로컬)
        self.heading_angle = 0.0  # 도 단위
        self.lidar_distances = np.zeros(360)
        # LiDAR 각도 (데이터 자체를 lidar_callback에서 90° 회전 보정)
        self.lidar_angles = np.radians(np.arange(360))
        self.waypoint = None  # 목표 지점
        self.psi_error = 0.0
        self.tau_x = 0.0
        self.trajectory = deque(maxlen=500)  # 이동 궤적

        # 스러스터 값
        self.thrust_left = 0.0
        self.thrust_right = 0.0

        # 기준점 (VRX 시작 위치)
        self.ref_position = [-532.0, 162.0]
        self.visual_range = 50.0  # 시각화 범위

        # Subscribers
        self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/fix',
            self.gps_callback, 10)
        self.create_subscription(
            Imu, '/wamv/sensors/imu/data',
            self.imu_callback, 10)
        self.create_subscription(
            LaserScan, '/wamv/sensors/lidar/scan',
            self.lidar_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/command',
            self.command_callback, 10)
        self.create_subscription(
            Float64, '/wamv/thrusters/left/thrust',
            self.thrust_left_callback, 10)
        self.create_subscription(
            Float64, '/wamv/thrusters/right/thrust',
            self.thrust_right_callback, 10)

        # Publisher (웨이포인트)
        self.waypoint_pub = self.create_publisher(
            PointStamped, '/waypoint_goal', 10)

        self.get_logger().info('Integrated Visualizer initialized')

    def gps_callback(self, msg: NavSatFix):
        """GPS 콜백 - UTM 좌표로 변환"""
        utm_x, utm_y, _ = SETTINGS.latlon_to_utm(msg.latitude, msg.longitude)
        # 로컬 좌표로 변환
        self.gps_position = [
            utm_x - SETTINGS.REF_UTM_X,
            utm_y - SETTINGS.REF_UTM_Y
        ]
        self.trajectory.append(self.gps_position.copy())

    def imu_callback(self, msg: Imu):
        """IMU 콜백 - 쿼터니언에서 yaw 추출"""
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = np.arctan2(siny_cosp, cosy_cosp)
        self.heading_angle = np.degrees(yaw_rad)

    def lidar_callback(self, msg: LaserScan):
        """LiDAR 콜백"""
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        # 360도로 리샘플링
        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]
        # 자기반사 필터: LiDAR 마운트 포스트가 뱃머리 기준 약 -137°~-44°
        # 구간에서 0.35~0.5m로 계속 잡힘. mission_runner.py와 동일하게 필터링.
        ranges[(ranges > 0) & (ranges < SETTINGS.MIN_VALID_RANGE)] = 0
        # 180도 회전 보정 (mission_runner.py와 동일 - 실측으로 확인됨:
        # raw index 0(=-180°, 정후방)이 자기반사 중심이라 정면은 배열
        # 정중앙에 있음. 기존 -90 롤은 틀린 값이었음)
        ranges = np.roll(ranges, 180)
        self.lidar_distances = ranges
        self.lidar_distances[self.lidar_distances > SETTINGS.LIDAR_MAX_RANGE] = 0

    def command_callback(self, msg: Float32MultiArray):
        """명령 콜백"""
        if len(msg.data) >= 2:
            self.psi_error = msg.data[0]
            self.tau_x = msg.data[1]

    def thrust_left_callback(self, msg: Float64):
        self.thrust_left = msg.data

    def thrust_right_callback(self, msg: Float64):
        self.thrust_right = msg.data

    def set_waypoint(self, x, y):
        """클릭으로 웨이포인트 설정"""
        self.waypoint = [x, y]
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.waypoint_pub.publish(msg)
        self.get_logger().info(f'Waypoint set: ({x:.1f}, {y:.1f})')


class VisualizerApp:
    """Matplotlib 시각화 앱"""

    def __init__(self, node: IntegratedVisualizer):
        self.node = node

        # Figure 설정 (왼쪽: Global, 오른쪽: Polar)
        self.fig = plt.figure(figsize=(14, 7))
        self.fig.suptitle('KABOAT Autonomous Navigation', fontsize=14, fontweight='bold')

        # 왼쪽: Global Map (Cartesian)
        self.ax_global = self.fig.add_subplot(1, 2, 1)
        self.ax_global.set_title('Global Map (Click to set waypoint)')
        self.ax_global.set_xlabel('X (East) [m]')
        self.ax_global.set_ylabel('Y (North) [m]')
        self.ax_global.set_aspect('equal')
        self.ax_global.grid(True, alpha=0.3)

        # 오른쪽: Local Map (Polar)
        self.ax_local = self.fig.add_subplot(1, 2, 2, projection='polar')
        self.ax_local.set_title('Local Map (LiDAR & Commands)')
        self.ax_local.set_theta_zero_location('N')  # 북쪽을 0도로
        self.ax_local.set_theta_direction(-1)  # 시계방향

        # 클릭 이벤트 연결
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        # 초기화
        self.setup_plots()

    def setup_plots(self):
        """플롯 초기 설정"""
        # Global map elements
        self.trajectory_line, = self.ax_global.plot([], [], 'b-', linewidth=1, alpha=0.5, label='Trajectory')
        self.boat_marker, = self.ax_global.plot([], [], 'ro', markersize=10, label='Boat')
        self.heading_line, = self.ax_global.plot([], [], 'g-', linewidth=2, label='Heading')
        self.waypoint_marker, = self.ax_global.plot([], [], 'r*', markersize=15, label='Waypoint')
        self.lidar_scatter_global = self.ax_global.scatter([], [], c='blue', s=2, alpha=0.5)

        # Local map elements (polar)
        self.lidar_scatter_local = self.ax_local.scatter([], [], c='blue', s=3)
        self.safe_zone_fill = None
        self.heading_arrow, = self.ax_local.plot([], [], 'g-', linewidth=3, label='Heading')
        self.command_arrow, = self.ax_local.plot([], [], 'r-', linewidth=2, label='Command')
        self.waypoint_local, = self.ax_local.plot([], [], 'r*', markersize=12)

    def update(self, frame):
        """애니메이션 업데이트"""
        # === Global Map 업데이트 ===
        pos = self.node.gps_position
        heading = self.node.heading_angle

        # 시각화 범위 설정 (보트 중심)
        margin = self.node.visual_range
        self.ax_global.set_xlim(pos[0] - margin, pos[0] + margin)
        self.ax_global.set_ylim(pos[1] - margin, pos[1] + margin)

        # 궤적
        if len(self.node.trajectory) > 1:
            traj = np.array(self.node.trajectory)
            self.trajectory_line.set_data(traj[:, 0], traj[:, 1])

        # 보트 위치
        self.boat_marker.set_data([pos[0]], [pos[1]])

        # 헤딩 방향 (화살표) - ENU: 0°=East, 90°=North
        line_length = 5.0
        heading_rad = np.radians(heading)
        end_x = pos[0] + line_length * np.cos(heading_rad)
        end_y = pos[1] + line_length * np.sin(heading_rad)
        self.heading_line.set_data([pos[0], end_x], [pos[1], end_y])

        # 웨이포인트
        if self.node.waypoint:
            self.waypoint_marker.set_data([self.node.waypoint[0]], [self.node.waypoint[1]])
        else:
            self.waypoint_marker.set_data([], [])

        # LiDAR 데이터 (글로벌 좌표로 변환) - ENU: cos→X, sin→Y
        lidar = self.node.lidar_distances
        angles = self.node.lidar_angles
        if np.any(lidar > 0):
            valid = lidar > 0
            # ENU 좌표계: X=East (cos), Y=North (sin)
            x_lidar = lidar[valid] * np.cos(angles[valid] + heading_rad) + pos[0]
            y_lidar = lidar[valid] * np.sin(angles[valid] + heading_rad) + pos[1]
            self.lidar_scatter_global.set_offsets(np.c_[x_lidar, y_lidar])
        else:
            self.lidar_scatter_global.set_offsets(np.c_[[], []])

        # === Local Map (Polar) 업데이트 ===
        self.ax_local.clear()
        self.ax_local.set_title('Local Map (LiDAR & Commands)')
        self.ax_local.set_theta_zero_location('N')
        self.ax_local.set_theta_direction(-1)
        self.ax_local.set_ylim(0, 37.5)

        # LiDAR 데이터 (polar) - Cmd/Waypoint와 동일하게 ENU(CCW+) -> polar(CW+) 부호 반전.
        # angles는 뱃머리 기준 CCW+ (pathplan의 atan2(dy,dx)와 동일 컨벤션)인데
        # 반전 없이 그리면 theta_direction=-1(화면 CW)과 어긋나 Cmd/Waypoint와
        # 좌우가 거울 대칭으로 나타난다.
        if np.any(lidar > 0):
            valid = lidar > 0
            self.ax_local.scatter(-angles[valid], lidar[valid], c='blue', s=3, label='LiDAR')

        # 안전 구역 표시
        if np.any(lidar > 0):
            safe_zone = np.array(calculate_safe_zone(lidar.tolist()))
            # 안전 구역을 반투명으로 표시 (LiDAR와 동일하게 부호 반전)
            theta = np.linspace(0, 2 * np.pi, 360)
            self.ax_local.fill(-theta, safe_zone, color='blue', alpha=0.1)

        # 현재 헤딩 (0도 = 전방)
        self.ax_local.plot([0, 0], [0, 30], 'g-', linewidth=3, label='Forward')

        # 명령 방향 (psi_error) - ENU(CCW+) → polar(CW+) 변환: 부호 반전
        psi_rad = np.radians(-self.node.psi_error)
        self.ax_local.plot([0, psi_rad], [0, 25], 'r-', linewidth=2, label=f'Cmd: {self.node.psi_error:.1f}°')

        # 웨이포인트 방향 (로컬 좌표계) - ENU(CCW+) → polar(CW+) 변환
        if self.node.waypoint:
            dx = self.node.waypoint[0] - pos[0]
            dy = self.node.waypoint[1] - pos[1]
            wp_angle = -(np.arctan2(dy, dx) - heading_rad)  # 부호 반전
            wp_dist = min(np.sqrt(dx**2 + dy**2), 30)
            self.ax_local.scatter([wp_angle], [wp_dist], c='red', s=100, marker='*', label='Waypoint')

        # 스러스터 상태 표시
        thrust_text = f'L: {self.node.thrust_left:.1f}  R: {self.node.thrust_right:.1f}'
        self.ax_local.text(0.5, -0.1, thrust_text, transform=self.ax_local.transAxes,
                          ha='center', fontsize=10, fontweight='bold')

        self.ax_local.legend(loc='upper right', fontsize=8)
        self.ax_local.grid(True, alpha=0.3)

        # Global legend
        self.ax_global.legend(loc='upper left', fontsize=8)

        return []

    def on_click(self, event):
        """마우스 클릭 이벤트 - 웨이포인트 설정"""
        if event.inaxes == self.ax_global and event.button == 1:  # 왼쪽 클릭
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                self.node.set_waypoint(x, y)
                print(f'\n=== Waypoint Set ===')
                print(f'Position: ({x:.1f}, {y:.1f})')
                print(f'Distance: {np.sqrt((x-self.node.gps_position[0])**2 + (y-self.node.gps_position[1])**2):.1f}m')

    def run(self):
        """애니메이션 시작"""
        self.ani = FuncAnimation(self.fig, self.update, interval=100, blit=False)
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedVisualizer()

    # ROS 스핀을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 시각화 앱 실행
    app = VisualizerApp(node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
