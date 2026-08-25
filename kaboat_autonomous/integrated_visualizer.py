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
from matplotlib.widgets import Slider
from math import ceil
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

from kaboat_autonomous.controllers.autonomous_module import (
    calculate_safe_zone, normalize_angle, goal_check, Boat, compute_cost_profile
)

# 트랙바로 조정할 파라미터: (SETTINGS 속성명, 최소값, 최대값)
TUNABLE_PARAMS = [
    ('BOAT_WIDTH', 0.5, 5.0),
    ('AVOID_RANGE', 5.0, 60.0),
    ('GAIN_PSI', 0.0, 5.0),
    ('GAIN_DISTANCE', 0.0, 20.0),
    ('GOAL_RANGE', 1.0, 10.0),
]


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

        # Publisher (트랙바 파라미터 -> mission_runner의 SETTINGS에 실시간 반영)
        self.tuning_pub = self.create_publisher(
            Float32MultiArray, '/tuning_params', 10)

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
        self._sliders = {}  # matplotlib이 GC하지 않도록 참조 유지

        # Figure 레이아웃: 제일 왼쪽에 길게 기존 Global Map, 그 다음 Local
        # Polar Map(둘 다 기존과 동일하게 위아래로 긴 패널), 오른쪽에 Cost
        # 구성/총합 그래프, 맨 아래 트랙바
        self.fig = plt.figure(figsize=(18, 9))
        self.fig.suptitle('KABOAT Autonomous Navigation', fontsize=14, fontweight='bold')

        gs = self.fig.add_gridspec(
            3, 3, height_ratios=[3, 3, 1.8], width_ratios=[1.3, 1, 1],
            hspace=0.6, wspace=0.3, left=0.05, right=0.97, top=0.90, bottom=0.04)

        # 제일 왼쪽: Global Map (Cartesian) - 기존과 동일, 위아래로 길게
        self.ax_global = self.fig.add_subplot(gs[0:2, 0])
        self.ax_global.set_title('Global Map (Click to set waypoint)')
        self.ax_global.set_xlabel('X (East) [m]')
        self.ax_global.set_ylabel('Y (North) [m]')
        self.ax_global.set_aspect('equal')
        self.ax_global.grid(True, alpha=0.3)

        # 가운데: Local Map (Polar) - 기존과 동일, 위아래로 길게
        self.ax_local = self.fig.add_subplot(gs[0:2, 1], projection='polar')
        self.ax_local.set_title('Local Map (LiDAR & Commands)')
        self.ax_local.set_theta_zero_location('N')  # 북쪽을 0도로
        self.ax_local.set_theta_direction(-1)  # 시계방향

        # 오른쪽: Cost 구성 요소 / 총 Cost 그래프 (디버그용, 신규)
        self.ax_cost_comp = self.fig.add_subplot(gs[0, 2])
        self.ax_cost_total = self.fig.add_subplot(gs[1, 2])

        # 맨 아래: BOAT_WIDTH / AVOID_RANGE / GAIN_PSI / GAIN_DISTANCE /
        # GOAL_RANGE 실시간 트랙바 (신규) - 조정 즉시 이 프로세스 계산 +
        # mission_runner의 실제 알고리즘(SETTINGS)에 /tuning_params로 반영
        slider_gs = gs[2, :].subgridspec(1, len(TUNABLE_PARAMS), wspace=0.4)
        for i, (name, vmin, vmax) in enumerate(TUNABLE_PARAMS):
            sax = self.fig.add_subplot(slider_gs[0, i])
            slider = Slider(sax, name, vmin, vmax, valinit=getattr(SETTINGS, name))
            slider.on_changed(self._make_slider_callback(name))
            self._sliders[name] = slider

        # 클릭 이벤트 연결
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        # 초기화
        self.setup_plots()

    def _make_slider_callback(self, name):
        def _callback(val):
            setattr(SETTINGS, name, val)
            msg = Float32MultiArray()
            msg.data = [float(getattr(SETTINGS, n)) for n, _, _ in TUNABLE_PARAMS]
            self.node.tuning_pub.publish(msg)
        return _callback

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

        # Corridor(BOAT_WIDTH 폭 직선 경로) 클리어 판정 시각화
        # goal_check()가 True를 반환하면 pathplan()이 목표 방향으로 풀
        # 스피드 직진한다 - 이 상태가 실제로 언제 발동하는지 확인용.
        self.corridor_patch = Polygon(
            [[0, 0], [0, 0], [0, 0], [0, 0]], closed=True,
            alpha=0.25, facecolor='gray', edgecolor='gray', linewidth=1, zorder=1
        )
        self.ax_global.add_patch(self.corridor_patch)
        self.corridor_patch.set_visible(False)
        self.corridor_status_text = self.ax_global.text(
            0.5, 1.05, '', transform=self.ax_global.transAxes,
            ha='center', fontsize=10, fontweight='bold'
        )

    def update(self, frame):
        """애니메이션 업데이트"""
        # === Global Map 업데이트 ===
        pos = self.node.gps_position
        heading = self.node.heading_angle
        heading_rad = np.radians(heading)
        lidar = self.node.lidar_distances
        angles = self.node.lidar_angles

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
        end_x = pos[0] + line_length * np.cos(heading_rad)
        end_y = pos[1] + line_length * np.sin(heading_rad)
        self.heading_line.set_data([pos[0], end_x], [pos[1], end_y])

        # 웨이포인트
        if self.node.waypoint:
            self.waypoint_marker.set_data([self.node.waypoint[0]], [self.node.waypoint[1]])
        else:
            self.waypoint_marker.set_data([], [])

        # === 직선 경로(Corridor) 클리어 판정 - goal_check()와 동일 조건 ===
        # BOAT_WIDTH 폭 직선 통로에 장애물이 없으면 pathplan()이 목표
        # 방향으로 풀 스피드 직진(is_clear=True)한다. 이 판정이 실제로
        # 언제 발동하는지 화면에서 바로 확인하기 위한 시각화.
        is_clear = None
        goal_distance = 0.0
        wp_angle = 0.0
        theta_deg = 0.0
        dx = dy = 0.0
        if self.node.waypoint:
            dx = self.node.waypoint[0] - pos[0]
            dy = self.node.waypoint[1] - pos[1]
            goal_distance = np.sqrt(dx ** 2 + dy ** 2)
            wp_angle = -(np.arctan2(dy, dx) - heading_rad)  # polar 표시용 (CW+)

            if np.any(lidar > 0) and goal_distance > 1e-6:
                goal_heading = np.degrees(np.arctan2(dy, dx))
                goal_psi = normalize_angle(goal_heading - heading)
                boat_state = Boat(position=list(pos), psi=heading, scan=lidar.tolist())
                is_clear = goal_check(boat_state, goal_distance, goal_psi)
                theta_deg = ceil(np.degrees(np.arctan2(SETTINGS.BOAT_WIDTH / 2, goal_distance)))

        corridor_color = 'green' if is_clear else ('red' if is_clear is False else 'gray')

        # Corridor 표시 (Global Map): 보트-목적지 직선을 BOAT_WIDTH 폭 띠로 표시
        if self.node.waypoint and goal_distance > 1e-6:
            ux, uy = dx / goal_distance, dy / goal_distance
            px, py = -uy, ux
            half_w = SETTINGS.BOAT_WIDTH / 2
            corners = [
                (pos[0] + px * half_w, pos[1] + py * half_w),
                (self.node.waypoint[0] + px * half_w, self.node.waypoint[1] + py * half_w),
                (self.node.waypoint[0] - px * half_w, self.node.waypoint[1] - py * half_w),
                (pos[0] - px * half_w, pos[1] - py * half_w),
            ]
            self.corridor_patch.set_xy(corners)
            self.corridor_patch.set_facecolor(corridor_color)
            self.corridor_patch.set_edgecolor(corridor_color)
            self.corridor_patch.set_visible(True)
        else:
            self.corridor_patch.set_visible(False)

        if is_clear is None:
            self.corridor_status_text.set_text('')
        else:
            status_str = 'PATH CLEAR -> FULL SPEED' if is_clear else 'PATH BLOCKED -> AVOIDING'
            self.corridor_status_text.set_text(status_str)
            self.corridor_status_text.set_color(corridor_color)

        # LiDAR 데이터 (글로벌 좌표로 변환) - ENU: cos→X, sin→Y
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
        safe_zone = None
        if np.any(lidar > 0):
            safe_zone = np.array(calculate_safe_zone(lidar.tolist()))
            # 안전 구역을 반투명으로 표시 (LiDAR와 동일하게 부호 반전)
            theta = np.linspace(0, 2 * np.pi, 360)
            self.ax_local.fill(-theta, safe_zone, color='blue', alpha=0.1)

        # === Cost 구성 요소 / 총 Cost 그래프 (디버그용) ===
        self.ax_cost_comp.clear()
        self.ax_cost_total.clear()
        if safe_zone is not None and self.node.waypoint and goal_distance > 1e-6:
            angs, angle_costs, dist_costs, total_costs = compute_cost_profile(
                lidar.tolist(), safe_zone.tolist(), goal_psi)
            if angs:
                self.ax_cost_comp.plot(angs, angle_costs, color='tab:blue',
                                        label=f'angle cost x GAIN_PSI({SETTINGS.GAIN_PSI:.2f})')
                self.ax_cost_comp.plot(angs, dist_costs, color='tab:orange',
                                        label=f'dist cost x GAIN_DISTANCE({SETTINGS.GAIN_DISTANCE:.2f})')
                self.ax_cost_comp.axvline(goal_psi, color='gray', linestyle='--', linewidth=1,
                                           label=f'goal_psi={goal_psi:.0f}°')
                self.ax_cost_comp.set_title('Cost components')
                self.ax_cost_comp.set_xlabel('candidate angle (deg)')
                self.ax_cost_comp.set_xlim(-180, 180)
                self.ax_cost_comp.legend(loc='upper right', fontsize=7)
                self.ax_cost_comp.grid(True, alpha=0.3)

                best_idx = int(np.argmin(total_costs))
                best_angle = angs[best_idx]
                total_title = 'Total cost (lower is better)'
                if is_clear:
                    total_title += ' - PATH CLEAR, cost profile bypassed'
                self.ax_cost_total.plot(angs, total_costs, color='tab:green', label='total cost')
                self.ax_cost_total.axvline(goal_psi, color='gray', linestyle='--', linewidth=1,
                                            label=f'goal_psi={goal_psi:.0f}°')
                self.ax_cost_total.axvline(best_angle, color='red', linestyle='-', linewidth=1.5,
                                            label=f'argmin={best_angle}°')
                self.ax_cost_total.axvline(self.node.psi_error, color='green', linestyle=':', linewidth=1.5,
                                            label=f'actual cmd={self.node.psi_error:.0f}°')
                self.ax_cost_total.set_title(total_title, fontsize=9)
                self.ax_cost_total.set_xlabel('candidate angle (deg)')
                self.ax_cost_total.set_xlim(-180, 180)
                self.ax_cost_total.legend(loc='upper right', fontsize=7)
                self.ax_cost_total.grid(True, alpha=0.3)
            else:
                self.ax_cost_comp.set_title('Cost components (all angles blocked)')
                self.ax_cost_total.set_title('Total cost (all angles blocked)')
        else:
            self.ax_cost_comp.set_title('Cost components (no waypoint/LiDAR)')
            self.ax_cost_total.set_title('Total cost (no waypoint/LiDAR)')

        # Corridor 표시 (Local Map): goal_check가 검사하는 ±theta 전방 부채꼴
        if self.node.waypoint and is_clear is not None:
            wedge_half = np.radians(theta_deg)
            r_cap = min(goal_distance, 35)
            n_pts = 20
            edge = np.linspace(wp_angle - wedge_half, wp_angle + wedge_half, n_pts)
            theta_fill = np.concatenate(([wp_angle - wedge_half], edge, [wp_angle + wedge_half]))
            r_fill = np.concatenate(([0], np.full(n_pts, r_cap), [0]))
            self.ax_local.fill(theta_fill, r_fill, color=corridor_color, alpha=0.25, label='Corridor')

        # 현재 헤딩 (0도 = 전방)
        self.ax_local.plot([0, 0], [0, 30], 'g-', linewidth=3, label='Forward')

        # 명령 방향 (psi_error) - ENU(CCW+) → polar(CW+) 변환: 부호 반전
        psi_rad = np.radians(-self.node.psi_error)
        self.ax_local.plot([0, psi_rad], [0, 25], 'r-', linewidth=2, label=f'Cmd: {self.node.psi_error:.1f}°')

        # 웨이포인트 방향 (로컬 좌표계) - ENU(CCW+) → polar(CW+) 변환
        if self.node.waypoint:
            wp_dist = min(goal_distance, 30)
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
        # gridspec에서 이미 여백을 수동 지정했으므로 tight_layout은 쓰지 않음
        # (트랙바 Axes 배치가 tight_layout과 충돌해 찌그러짐)
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
