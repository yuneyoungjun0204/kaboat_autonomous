#!/usr/bin/env python3
"""
글로벌 맵 시각화 (Cartesian 좌표)
SeaNU_KABOAT2024 VisualizeGlobalMap.py 포팅 (ROS2)
- GPS 위치 및 LiDAR 데이터 시각화
- Cost 함수 시각화
- 헤딩 방향 표시
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix, Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import threading
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from controllers.autonomous_module import calculate_safe_zone, cost_func_angle, cost_func_distance, normalize_angle

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

# 전역 변수
distances = np.zeros(360)
gps_position = [0.0, 0.0]
heading_angle = 0.0
waypoint = [0.0, 0.0]
psi_error = 0.0
tau_x = 0.0
threshold = 50.0
visual_size = 30

# 기준점 (첫 GPS 시 자동 설정)
ref_utm_x = None
ref_utm_y = None
ref_initialized = False


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.degrees(np.arctan2(siny_cosp, cosy_cosp))


class GlobalMapVisualizer(Node):
    def __init__(self):
        super().__init__('global_map_visualizer')

        # Subscribers
        self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidar/scan',
            self.lidar_callback,
            10
        )
        self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/fix',
            self.gps_callback,
            10
        )
        self.create_subscription(
            Imu,
            '/wamv/sensors/imu/data',
            self.imu_callback,
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

        self.get_logger().info('Global Map Visualizer initialized')

    def lidar_callback(self, msg):
        global distances
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)

        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]

        ranges[ranges > threshold] = 0
        distances = ranges

    def gps_callback(self, msg):
        global gps_position, ref_utm_x, ref_utm_y, ref_initialized
        if msg.latitude != 0 and msg.longitude != 0:
            utm_x, utm_y, _ = SETTINGS.latlon_to_utm(msg.latitude, msg.longitude)

            # 첫 GPS 수신 시 현재 위치를 기준점으로 설정
            if not ref_initialized:
                ref_utm_x = utm_x
                ref_utm_y = utm_y
                ref_initialized = True
                self.get_logger().info(f'Viz ref point: ({utm_x:.2f}, {utm_y:.2f})')

            gps_position[0] = utm_x - ref_utm_x
            gps_position[1] = utm_y - ref_utm_y

    def imu_callback(self, msg):
        global heading_angle
        heading_angle = quaternion_to_yaw(msg.orientation)

    def command_callback(self, msg):
        global psi_error, tau_x
        if len(msg.data) >= 2:
            psi_error = msg.data[0]
            tau_x = msg.data[1]

    def waypoint_callback(self, msg):
        global waypoint
        if len(msg.data) >= 3:
            waypoint = [msg.data[1], msg.data[2]]


def update(frame):
    """시각화 업데이트"""
    global distances, gps_position, heading_angle, waypoint, psi_error, tau_x

    ax.clear()
    ax.set_title(f'Global Map | Heading: {heading_angle:.1f}° | Cmd: {psi_error:.1f}° | Thrust: {tau_x:.0f}')

    # 뷰 범위 설정 (보트 중심)
    ax.set_xlim(gps_position[0] - visual_size, gps_position[0] + visual_size)
    ax.set_ylim(gps_position[1] - visual_size, gps_position[1] + visual_size)
    ax.set_aspect('equal')

    # LiDAR 데이터를 Cartesian 좌표로 변환
    angles = np.radians(np.arange(360))
    x_positions = distances * np.sin(angles + np.radians(heading_angle)) + gps_position[0]
    y_positions = distances * np.cos(angles + np.radians(heading_angle)) + gps_position[1]

    valid = distances > 0
    ax.scatter(x_positions[valid], y_positions[valid], color='blue', s=3, label='LiDAR')

    # 불가능 영역 표시
    if np.any(distances > 0):
        safe_zone = calculate_safe_zone(distances.tolist())
        for i, sz in enumerate(safe_zone):
            if sz == 0 and distances[i] > 0:
                ux = distances[i] * np.sin(angles[i] + np.radians(heading_angle)) + gps_position[0]
                uy = distances[i] * np.cos(angles[i] + np.radians(heading_angle)) + gps_position[1]
                ax.scatter(ux, uy, color='red', s=10, alpha=0.5)

    # GPS 위치 (빨간 원)
    boat = Circle((gps_position[0], gps_position[1]), 1.5, color='red', fill=True, alpha=0.7)
    ax.add_patch(boat)

    # 헤딩 방향 (초록색 선)
    line_length = 8
    line_x = gps_position[0] + line_length * np.sin(np.radians(heading_angle))
    line_y = gps_position[1] + line_length * np.cos(np.radians(heading_angle))
    ax.plot([gps_position[0], line_x], [gps_position[1], line_y],
            color='green', linewidth=3, label='Heading')

    # 명령 방향 (파란색 점선)
    cmd_angle = heading_angle + psi_error
    cmd_x = gps_position[0] + line_length * np.sin(np.radians(cmd_angle))
    cmd_y = gps_position[1] + line_length * np.cos(np.radians(cmd_angle))
    ax.plot([gps_position[0], cmd_x], [gps_position[1], cmd_y],
            color='cyan', linewidth=2, linestyle='--', label='Command')

    # 웨이포인트 (노란 별)
    if waypoint[0] != 0 or waypoint[1] != 0:
        ax.scatter(waypoint[0], waypoint[1], color='yellow', s=200, marker='*',
                   edgecolors='black', linewidths=1, label='Waypoint', zorder=5)
        # 웨이포인트까지 선
        ax.plot([gps_position[0], waypoint[0]], [gps_position[1], waypoint[1]],
                color='orange', linewidth=1, linestyle=':', alpha=0.7)

    # Cost 시각화 (360도 방사형)
    if np.any(distances > 0):
        cost_lines = []
        for i in range(-180, 180, 10):  # 10도 간격
            idx = i % 360
            dist = distances[idx] if distances[idx] > 0 else 10
            cost = SETTINGS.GAIN_PSI * cost_func_angle(i - psi_error) + \
                   SETTINGS.GAIN_DISTANCE * cost_func_distance(dist)
            # Cost가 낮을수록 선이 길어짐
            line_len = 5 / (cost + 0.5)
            angle_rad = np.radians(heading_angle + i)
            cx = gps_position[0] + line_len * np.sin(angle_rad)
            cy = gps_position[1] + line_len * np.cos(angle_rad)
            ax.plot([gps_position[0], cx], [gps_position[1], cy],
                    color='purple', linewidth=0.5, alpha=0.4)

    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)


def ros_spin(node):
    """ROS2 스핀 스레드"""
    rclpy.spin(node)


def main(args=None):
    global fig, ax

    rclpy.init(args=args)
    node = GlobalMapVisualizer()

    # ROS2를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Matplotlib 설정
    fig, ax = plt.subplots(figsize=(10, 10))
    ani = FuncAnimation(fig, update, interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
