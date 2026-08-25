#!/usr/bin/env python3
"""
로컬 맵 시각화 (Polar 좌표)
SeaNU_KABOAT2024 VisualizeLocalMap.py 포팅 (ROS2)
- LiDAR 데이터 시각화
- 안전 구역 (불가능 영역) 표시
- 웨이포인트 및 명령 각도 표시
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from controllers.autonomous_module import calculate_safe_zone

# 전역 변수
distances = np.zeros(360)
angles = np.radians(np.arange(360))
waypoint = [0.0, 0.0]
psi_error = 0.0
gps_position = [0.0, 0.0]
heading_angle = 0.0
threshold = 50.0


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

        # mission_runner.py와 동일한 90도 회전 보정 (VRX LiDAR 0도가 오른쪽 -> 전방)
        ranges = np.roll(ranges, -90)

        distances = ranges
        angles = np.radians(np.arange(360))

    def command_callback(self, msg):
        global psi_error
        if len(msg.data) > 0:
            psi_error = msg.data[0]

    def waypoint_callback(self, msg):
        global waypoint
        if len(msg.data) >= 3:
            waypoint = [msg.data[1], msg.data[2]]


def update(frame):
    """시각화 업데이트"""
    global distances, angles, waypoint, psi_error

    ax.clear()
    ax.set_title('LiDAR Local Map (Polar)', va='bottom')
    ax.set_ylim(0, 15)

    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)

    # LiDAR 데이터 (파란 점)
    valid = distances > 0
    ax.scatter(angles[valid], distances[valid], color='blue', s=3, label='LiDAR')

    # 안전 구역 (불가능 영역 - 빨간색 채움)
    if np.any(distances > 0):
        safe_zone = calculate_safe_zone(distances.tolist())
        unsafe_angles = []
        unsafe_distances = []
        for i, sz in enumerate(safe_zone):
            if sz == 0:  # 불가능 영역
                unsafe_angles.append(angles[i])
                unsafe_distances.append(5.0)  # 고정 거리로 표시
        if unsafe_angles:
            ax.scatter(unsafe_angles, unsafe_distances, color='red', s=5, alpha=0.3, label='Unsafe')

    # 명령 각도 (초록색 선) - ENU(CCW+) → polar(CW+) 변환: 부호 반전
    ax.plot([0, np.radians(-psi_error)], [0, 12], color='green', linewidth=2, label=f'Cmd: {psi_error:.1f}°')

    # 웨이포인트 (있다면) - ENU(CCW+) → polar(CW+) 변환
    if waypoint[0] != 0 or waypoint[1] != 0:
        wp_angle = -np.arctan2(waypoint[1], waypoint[0])
        wp_dist = min(np.sqrt(waypoint[0]**2 + waypoint[1]**2), 14)
        ax.scatter([wp_angle], [wp_dist], color='red', s=100, marker='*', label='Waypoint')

    ax.grid(True)
    ax.legend(loc='upper right', fontsize=8)


def ros_spin(node):
    """ROS2 스핀 스레드"""
    rclpy.spin(node)


def main(args=None):
    global fig, ax

    rclpy.init(args=args)
    node = LocalMapVisualizer()

    # ROS2를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Matplotlib 설정
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
    ani = FuncAnimation(fig, update, interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
