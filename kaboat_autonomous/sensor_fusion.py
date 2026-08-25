#!/usr/bin/env python3
"""
센서 데이터 통합 노드 - LLM 멀티모달 입력용

모든 센서 데이터를 하나의 JSON 토픽으로 통합하여 발행.
LLM이 /sensor_fusion 구독 + 카메라 이미지로 상황을 종합 판단.

발행 토픽:
  /sensor_fusion (String): 통합 센서 데이터 (JSON, 5Hz)

데이터 포함:
  - GPS (위경도, 로컬 좌표)
  - IMU (헤딩, 롤, 피치, 각속도)
  - LiDAR 요약 (방향별 장애물, 게이트 감지)
  - 액션 상태 (현재 액션, 웨이포인트 진행)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import json
import time
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

from kaboat_autonomous.controllers.autonomous_module import Boat
from kaboat_autonomous.controllers import maneuvers


class SensorFusion(Node):
    """센서 데이터 통합 노드"""

    def __init__(self):
        super().__init__('sensor_fusion')

        # 센서 데이터 저장
        self.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'local_x': 0.0,
            'local_y': 0.0
        }
        self.imu_data = {
            'heading_deg': 0.0,
            'roll_deg': 0.0,
            'pitch_deg': 0.0,
            'angular_z': 0.0
        }
        self.lidar_summary = {}
        self.action_status = {
            'current_action': None,
            'action_elapsed_sec': 0.0
        }
        self.command_data = {
            'psi_error': 0.0,
            'tau_x': 0.0
        }

        # Boat 객체 (LiDAR 분석용)
        self.boat = Boat()
        self.ref_utm_x = SETTINGS.REF_UTM_X
        self.ref_utm_y = SETTINGS.REF_UTM_Y

        # === Publishers ===
        self.fusion_pub = self.create_publisher(String, '/sensor_fusion', 10)

        # === Subscribers ===
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/wamv/sensors/lidar/scan', self.lidar_callback, 10)
        self.create_subscription(String, '/action_status', self.action_callback, 10)
        self.create_subscription(Float32MultiArray, '/command', self.command_callback, 10)

        # 발행 타이머 (5Hz)
        self.timer = self.create_timer(0.2, self.publish_fusion)

        self.get_logger().info('=== Sensor Fusion Started ===')
        self.get_logger().info('Publishing to /sensor_fusion (5Hz)')

    def gps_callback(self, msg: NavSatFix):
        utm_x, utm_y, _ = SETTINGS.latlon_to_utm(msg.latitude, msg.longitude)
        self.gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'local_x': round(utm_x - self.ref_utm_x, 2),
            'local_y': round(utm_y - self.ref_utm_y, 2)
        }
        self.boat.position[0] = utm_x - self.ref_utm_x
        self.boat.position[1] = utm_y - self.ref_utm_y

    def imu_callback(self, msg: Imu):
        q = msg.orientation

        # Quaternion to Euler
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.imu_data = {
            'heading_deg': round(np.degrees(yaw), 1),
            'roll_deg': round(np.degrees(roll), 1),
            'pitch_deg': round(np.degrees(pitch), 1),
            'angular_z': round(msg.angular_velocity.z, 3)
        }
        self.boat.psi = np.degrees(yaw)

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)

        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]

        ranges = np.roll(ranges, 180)
        ranges[(ranges > 0) & (ranges < SETTINGS.MIN_VALID_RANGE)] = 0
        ranges[ranges > SETTINGS.LIDAR_MAX_RANGE] = 0

        self.boat.scan = ranges.tolist()
        self.lidar_summary = maneuvers.analyze_lidar(self.boat)

    def action_callback(self, msg: String):
        try:
            self.action_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def command_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.command_data = {
                'psi_error': round(msg.data[0], 1),
                'tau_x': round(msg.data[1], 0)
            }

    def publish_fusion(self):
        """통합 센서 데이터 발행"""
        fusion = {
            'timestamp': round(time.time(), 2),
            'gps': self.gps_data,
            'imu': self.imu_data,
            'lidar': self.lidar_summary,
            'command': self.command_data,
            'action': {
                'current': self.action_status.get('current_action'),
                'elapsed_sec': self.action_status.get('action_elapsed_sec', 0),
                'waypoint_progress': self.action_status.get('waypoints', {})
            }
        }

        msg = String()
        msg.data = json.dumps(fusion)
        self.fusion_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
