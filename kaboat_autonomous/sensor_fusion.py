#!/usr/bin/env python3
"""
센서 데이터 통합 노드 - LLM 멀티모달 입력용

모든 센서 데이터를 하나의 JSON 토픽으로 통합하여 발행.
LLM이 /sensor_fusion 구독 + 카메라 이미지로 상황을 종합 판단.
LiDAR는 자체 계산하지 않고 action_dispatcher가 발행하는 /lidar_summary를
그대로 구독한다 (rejected_clusters 등 상태 기반 결과의 단일 소스 유지 -
action_dispatcher 노드가 실행 중이어야 lidar 필드가 채워진다).

발행 토픽:
  /sensor_fusion (String): 통합 센서 데이터 (JSON, 5Hz)

데이터 포함:
  - GPS (위경도, 로컬 좌표)
  - IMU (헤딩, 롤, 피치, 각속도)
  - LiDAR 요약 (방향별 장애물, 게이트 감지, 클러스터+rejected 여부) - action_dispatcher 발행분 그대로
  - 액션 상태 (현재 액션, 웨이포인트 진행, decision_needed)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
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

        self.ref_utm_x = SETTINGS.REF_UTM_X
        self.ref_utm_y = SETTINGS.REF_UTM_Y

        # === Publishers ===
        self.fusion_pub = self.create_publisher(String, '/sensor_fusion', 10)

        # === Subscribers ===
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/data', self.imu_callback, 10)
        # LiDAR는 직접 처리하지 않고 action_dispatcher가 발행하는 요약을 그대로
        # 구독한다 (rejected_clusters 등 상태 기반 주석이 그쪽에만 있으므로
        # 단일 소스로 유지 - 중복 계산/불일치 방지).
        self.create_subscription(String, '/lidar_summary', self.lidar_summary_callback, 10)
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

    def lidar_summary_callback(self, msg: String):
        try:
            self.lidar_summary = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

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
                'waypoint_progress': self.action_status.get('waypoints', {}),
                'last_action': self.action_status.get('last_action'),
                'last_action_result': self.action_status.get('last_action_result'),
                'retry_count': self.action_status.get('retry_count', {})
            },
            # action_dispatcher가 계산한 값을 그대로 전달 (진행 중인 액션이 없을 때만 true)
            'decision_needed': self.action_status.get('decision_needed', True)
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
