#!/usr/bin/env python3
"""
로컬 맵 시각화 (Polar 좌표) + Cost 디버그 뷰
SeaNU_KABOAT2024 VisualizeLocalMap.py 포팅 (ROS2)
- LiDAR 데이터 시각화
- 안전 구역 (불가능 영역) 표시
- 웨이포인트 및 명령 각도 표시
- 오른쪽: 각도/거리 Cost 구성 및 총 Cost 그래프
- 하단: BOAT_WIDTH / AVOID_RANGE / GAIN_PSI / GAIN_DISTANCE / GOAL_RANGE
  실시간 트랙바 (조정 즉시 이 프로세스의 계산 + mission_runner의 실제
  알고리즘(SETTINGS)에 /tuning_params 토픽으로 반영됨)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
import threading
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from controllers import autonomous_module as AM

# 전역 변수
distances = np.zeros(360)
angles = np.radians(np.arange(360))
waypoint = [0.0, 0.0]
psi_error = 0.0
goal_psi_deg = 0.0
gps_position = [0.0, 0.0]
heading_angle = 0.0
threshold = 50.0

tuning_pub = None  # main()에서 생성, 슬라이더 콜백에서 사용
_sliders = {}      # matplotlib이 GC하지 않도록 참조 유지

# 트랙바로 조정할 파라미터: (SETTINGS 속성명, 최소값, 최대값)
TUNABLE_PARAMS = [
    ('BOAT_WIDTH', 0.5, 5.0),
    ('AVOID_RANGE', 5.0, 60.0),
    ('GAIN_PSI', 0.0, 5.0),
    ('GAIN_DISTANCE', 0.0, 20.0),
    ('GOAL_RANGE', 1.0, 10.0),
]


class LocalMapVisualizer(Node):
    def __init__(self):
        super().__init__('local_map_visualizer')

        # Subscribers
        self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidar/scan',
            self.lidar_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/command',
            self.command_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/waypoint',
            self.waypoint_callback,
            10
        )

        self.get_logger().info('Local Map Visualizer initialized')

    def lidar_callback(self, msg):
        global distances, angles
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)

        # 360도로 리샘플링
        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]

        ranges[ranges > threshold] = 0

        # mission_runner.py와 동일한 자기반사 필터 (실제 알고리즘 입력과
        # 동일한 전처리를 거쳐야 Cost 그래프가 실제 계산과 일치함)
        ranges[(ranges > 0) & (ranges < AM.SETTINGS.MIN_VALID_RANGE)] = 0

        # mission_runner.py와 동일한 180도 회전 보정 (실측 확인: 자기반사가
        # raw index 0=-180°(정후방) 중심이라 정면은 배열 정중앙에 위치)
        ranges = np.roll(ranges, 180)

        distances = ranges
        angles = np.radians(np.arange(360))

    def command_callback(self, msg):
        global psi_error, goal_psi_deg
        if len(msg.data) > 0:
            psi_error = msg.data[0]
        if len(msg.data) > 3:
            goal_psi_deg = msg.data[3]

    def waypoint_callback(self, msg):
        global waypoint
        if len(msg.data) >= 3:
            waypoint = [msg.data[1], msg.data[2]]


def update(frame):
    """시각화 업데이트"""
    global distances, angles, waypoint, psi_error, goal_psi_deg

    # --- Polar LiDAR 맵 ---
    ax_polar.clear()
    ax_polar.set_title('LiDAR Local Map (Polar)', va='bottom')
    ax_polar.set_ylim(0, 37.5)
    ax_polar.set_theta_zero_location("N")
    ax_polar.set_theta_direction(-1)

    valid = distances > 0
    ax_polar.scatter(angles[valid], distances[valid], color='blue', s=3, label='LiDAR')

    safe_zone = None
    if np.any(distances > 0):
        safe_zone = AM.calculate_safe_zone(distances.tolist())
        unsafe_angles = []
        unsafe_distances = []
        for i, sz in enumerate(safe_zone):
            if sz == 0:  # 불가능 영역
                unsafe_angles.append(angles[i])
                unsafe_distances.append(5.0)  # 고정 거리로 표시
        if unsafe_angles:
            ax_polar.scatter(unsafe_angles, unsafe_distances, color='red', s=5, alpha=0.3, label='Unsafe')

    # 명령 각도 (초록색 선) - ENU(CCW+) → polar(CW+) 변환: 부호 반전
    ax_polar.plot([0, np.radians(-psi_error)], [0, 30], color='green', linewidth=2, label=f'Cmd: {psi_error:.1f}°')

    # 웨이포인트 (있다면) - ENU(CCW+) → polar(CW+) 변환
    if waypoint[0] != 0 or waypoint[1] != 0:
        wp_angle = -np.arctan2(waypoint[1], waypoint[0])
        wp_dist = min(np.sqrt(waypoint[0] ** 2 + waypoint[1] ** 2), 35)
        ax_polar.scatter([wp_angle], [wp_dist], color='red', s=100, marker='*', label='Waypoint')

    ax_polar.grid(True)
    ax_polar.legend(loc='upper right', fontsize=8)

    # --- Cost 구성 요소 그래프 (오른쪽 위) ---
    ax_cost_comp.clear()
    ax_cost_total.clear()

    if safe_zone is not None:
        angs, angle_costs, dist_costs, total_costs = AM.compute_cost_profile(
            distances.tolist(), safe_zone, goal_psi_deg)

        if angs:
            ax_cost_comp.plot(angs, angle_costs, color='tab:blue',
                               label=f'angle cost x GAIN_PSI({AM.SETTINGS.GAIN_PSI:.2f})')
            ax_cost_comp.plot(angs, dist_costs, color='tab:orange',
                               label=f'dist cost x GAIN_DISTANCE({AM.SETTINGS.GAIN_DISTANCE:.2f})')
            ax_cost_comp.axvline(goal_psi_deg, color='gray', linestyle='--', linewidth=1,
                                  label=f'goal_psi={goal_psi_deg:.0f}°')
            ax_cost_comp.set_title('Cost components (per candidate angle)')
            ax_cost_comp.set_xlabel('candidate angle (deg)')
            ax_cost_comp.set_xlim(-180, 180)
            ax_cost_comp.legend(loc='upper right', fontsize=7)
            ax_cost_comp.grid(True, alpha=0.3)

            best_idx = int(np.argmin(total_costs))
            best_angle = angs[best_idx]
            ax_cost_total.plot(angs, total_costs, color='tab:green', label='total cost')
            ax_cost_total.axvline(goal_psi_deg, color='gray', linestyle='--', linewidth=1,
                                   label=f'goal_psi={goal_psi_deg:.0f}°')
            ax_cost_total.axvline(best_angle, color='red', linestyle='-', linewidth=1.5,
                                   label=f'argmin={best_angle}°')
            ax_cost_total.axvline(psi_error, color='green', linestyle=':', linewidth=1.5,
                                   label=f'actual cmd={psi_error:.0f}°')
            ax_cost_total.set_title('Total cost = angle cost + dist cost (lower is better)')
            ax_cost_total.set_xlabel('candidate angle (deg)')
            ax_cost_total.set_xlim(-180, 180)
            ax_cost_total.legend(loc='upper right', fontsize=7)
            ax_cost_total.grid(True, alpha=0.3)
        else:
            ax_cost_comp.set_title('Cost components (all angles blocked)')
            ax_cost_total.set_title('Total cost (all angles blocked)')
    else:
        ax_cost_comp.set_title('Cost components (no LiDAR data)')
        ax_cost_total.set_title('Total cost (no LiDAR data)')


def make_slider_callback(name):
    def _callback(val):
        setattr(AM.SETTINGS, name, val)
        if tuning_pub is not None:
            msg = Float32MultiArray()
            msg.data = [float(getattr(AM.SETTINGS, n)) for n, _, _ in TUNABLE_PARAMS]
            tuning_pub.publish(msg)
    return _callback


def ros_spin(node):
    """ROS2 스핀 스레드"""
    rclpy.spin(node)


def main(args=None):
    global fig, ax_polar, ax_cost_comp, ax_cost_total, tuning_pub

    rclpy.init(args=args)
    node = LocalMapVisualizer()
    tuning_pub = node.create_publisher(Float32MultiArray, '/tuning_params', 10)

    # ROS2를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Matplotlib 레이아웃: 왼쪽 Polar 맵 / 오른쪽 Cost 그래프 2단 / 하단 트랙바
    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(3, 2, height_ratios=[3, 3, 1.8], width_ratios=[1.3, 1],
                           hspace=0.6, wspace=0.3, left=0.06, right=0.97, top=0.95, bottom=0.04)
    ax_polar = fig.add_subplot(gs[0:2, 0], projection='polar')
    ax_cost_comp = fig.add_subplot(gs[0, 1])
    ax_cost_total = fig.add_subplot(gs[1, 1])

    slider_gs = gs[2, :].subgridspec(len(TUNABLE_PARAMS), 1, hspace=1.8)
    for i, (name, vmin, vmax) in enumerate(TUNABLE_PARAMS):
        sax = fig.add_subplot(slider_gs[i, 0])
        slider = Slider(sax, name, vmin, vmax, valinit=getattr(AM.SETTINGS, name))
        slider.on_changed(make_slider_callback(name))
        _sliders[name] = slider

    ani = FuncAnimation(fig, update, interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
