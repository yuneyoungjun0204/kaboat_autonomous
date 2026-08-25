#!/usr/bin/env python3
"""
LLM 인터페이스 노드
- LLM(ros-mcp)이 장애물 회피 모듈과 연동하여 고수준 제어
- 상태 요약, 미션 명령, 예외 감지 (stuck 등)

사용법:
1. rosbridge 실행: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
2. ros-mcp로 연결 후:
   - /boat_status 구독: 보트 상태 JSON
   - /llm_waypoint 발행: 목표 설정
   - /llm_command 발행: 직접 제어 (긴급 시)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PointStamped
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


class LLMInterfaceNode(Node):
    """
    LLM을 위한 고수준 인터페이스 노드

    LLM이 ros-mcp를 통해 사용할 토픽:

    [구독 (LLM이 읽기)]
    - /boat_status: 보트 상태 요약 (JSON, 1Hz)

    [발행 (LLM이 쓰기)]
    - /llm_waypoint: 목표 지점 설정 (geometry_msgs/PointStamped)
    - /llm_command: 직접 모터 명령 (std_msgs/Float32MultiArray)
    - /llm_override: 장애물 회피 모듈 오버라이드 (std_msgs/String)
    """

    def __init__(self):
        super().__init__('llm_interface')

        # 상태 저장
        self.position = [0.0, 0.0]
        self.heading = 0.0
        self.current_waypoint = None
        self.lidar_summary = {}
        self.command_status = {'psi_error': 0.0, 'tau_x': 0.0}
        self.thrust_left = 0.0
        self.thrust_right = 0.0

        # Stuck 감지
        self.is_stuck = False
        self.stuck_duration = 0.0
        self.position_history = []
        self.position_check_interval = 2.0
        self.last_position_check = time.time()

        # 오버라이드 모드
        self.override_active = False
        self.override_reason = ""

        # 기준점
        self.ref_utm_x = SETTINGS.REF_UTM_X
        self.ref_utm_y = SETTINGS.REF_UTM_Y

        # === Publishers ===
        self.status_pub = self.create_publisher(String, '/boat_status', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/waypoint_goal', 10)
        self.command_pub = self.create_publisher(Float32MultiArray, '/command', 10)

        # === Subscribers ===
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/wamv/sensors/lidar/scan', self.lidar_callback, 10)
        self.create_subscription(Float32MultiArray, '/command', self.command_callback, 10)

        # LLM 명령 수신
        self.create_subscription(PointStamped, '/llm_waypoint', self.llm_waypoint_callback, 10)
        self.create_subscription(Float32MultiArray, '/llm_command', self.llm_command_callback, 10)
        self.create_subscription(String, '/llm_override', self.llm_override_callback, 10)

        # 타이머
        self.create_timer(1.0, self.publish_status)
        self.create_timer(0.5, self.check_stuck)

        self.get_logger().info('=== LLM Interface Node Started ===')
        self.get_logger().info('LLM can use these topics via ros-mcp:')
        self.get_logger().info('  [READ]  /boat_status - JSON status (1Hz)')
        self.get_logger().info('  [WRITE] /llm_waypoint - Set goal (PointStamped)')
        self.get_logger().info('  [WRITE] /llm_command - Direct motor cmd')
        self.get_logger().info('  [WRITE] /llm_override - Override mode')

    def gps_callback(self, msg: NavSatFix):
        utm_x, utm_y, _ = SETTINGS.latlon_to_utm(msg.latitude, msg.longitude)
        self.position = [utm_x - self.ref_utm_x, utm_y - self.ref_utm_y]

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.heading = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)

        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]

        ranges = np.roll(ranges, 180)
        ranges[(ranges > 0) & (ranges < SETTINGS.MIN_VALID_RANGE)] = 0

        # 방향별 요약 (LLM이 이해하기 쉽게)
        def sector_min(start, end):
            sector = ranges[start:end]
            valid = sector[(sector > 0) & (sector < SETTINGS.LIDAR_MAX_RANGE)]
            return float(np.min(valid)) if len(valid) > 0 else 999.0

        self.lidar_summary = {
            'front': sector_min(350, 360) if sector_min(350, 360) < sector_min(0, 10) else sector_min(0, 10),
            'front_left': sector_min(10, 60),
            'left': sector_min(60, 120),
            'back_left': sector_min(120, 150),
            'back': sector_min(150, 210),
            'back_right': sector_min(210, 240),
            'right': sector_min(240, 300),
            'front_right': sector_min(300, 350),
            'closest': float(np.min(ranges[(ranges > 0) & (ranges < SETTINGS.LIDAR_MAX_RANGE)])) if np.any(ranges > 0) else 999.0
        }

    def command_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.command_status = {'psi_error': msg.data[0], 'tau_x': msg.data[1]}

    def llm_waypoint_callback(self, msg: PointStamped):
        """LLM이 설정한 웨이포인트 → mission_runner로 전달"""
        self.current_waypoint = [msg.point.x, msg.point.y]
        self.waypoint_pub.publish(msg)
        self.is_stuck = False
        self.stuck_duration = 0.0
        self.get_logger().info(f'[LLM] Waypoint set: ({msg.point.x:.1f}, {msg.point.y:.1f})')

    def llm_command_callback(self, msg: Float32MultiArray):
        """LLM 직접 모터 명령 (오버라이드 시에만 사용)"""
        if self.override_active:
            self.command_pub.publish(msg)
            self.get_logger().info(f'[LLM] Direct command: {msg.data}')
        else:
            self.get_logger().warn('[LLM] Direct command ignored - override not active')

    def llm_override_callback(self, msg: String):
        """
        오버라이드 모드 제어

        JSON 형식:
        {"action": "enable", "reason": "stuck recovery"}
        {"action": "disable"}
        """
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')

            if action == 'enable':
                self.override_active = True
                self.override_reason = data.get('reason', 'LLM override')
                self.get_logger().warn(f'[LLM] Override ENABLED: {self.override_reason}')
            elif action == 'disable':
                self.override_active = False
                self.override_reason = ""
                self.get_logger().info('[LLM] Override DISABLED')
        except json.JSONDecodeError:
            self.get_logger().error(f'[LLM] Invalid override JSON: {msg.data}')

    def check_stuck(self):
        """Stuck 감지: 웨이포인트가 있는데 5초간 3m 미만 이동"""
        now = time.time()
        if now - self.last_position_check < self.position_check_interval:
            return

        self.last_position_check = now
        self.position_history.append(self.position.copy())

        if len(self.position_history) > 5:
            self.position_history.pop(0)

        if len(self.position_history) >= 3 and self.current_waypoint:
            old_pos = self.position_history[0]
            dx = self.position[0] - old_pos[0]
            dy = self.position[1] - old_pos[1]
            moved = np.sqrt(dx**2 + dy**2)

            wp_dx = self.current_waypoint[0] - self.position[0]
            wp_dy = self.current_waypoint[1] - self.position[1]
            wp_dist = np.sqrt(wp_dx**2 + wp_dy**2)

            if moved < 3.0 and wp_dist > SETTINGS.GOAL_RANGE:
                if not self.is_stuck:
                    self.is_stuck = True
                    self.stuck_duration = 0.0
                    self.get_logger().warn('[STUCK] Boat not progressing!')
                else:
                    self.stuck_duration += self.position_check_interval * len(self.position_history)
            else:
                self.is_stuck = False
                self.stuck_duration = 0.0

    def publish_status(self):
        """상태 발행 (1Hz) - LLM이 /boat_status로 구독"""
        wp_info = None
        if self.current_waypoint:
            dx = self.current_waypoint[0] - self.position[0]
            dy = self.current_waypoint[1] - self.position[1]
            dist = np.sqrt(dx**2 + dy**2)
            bearing = np.degrees(np.arctan2(dy, dx))
            relative_bearing = bearing - self.heading
            relative_bearing = (relative_bearing + 180) % 360 - 180
            wp_info = {
                'x': round(self.current_waypoint[0], 1),
                'y': round(self.current_waypoint[1], 1),
                'distance_m': round(dist, 1),
                'bearing_deg': round(bearing, 1),
                'relative_bearing_deg': round(relative_bearing, 1)
            }

        status = {
            'timestamp': round(time.time(), 1),
            'position': {
                'x': round(self.position[0], 1),
                'y': round(self.position[1], 1),
                'heading_deg': round(self.heading, 1)
            },
            'obstacles': {k: round(v, 1) for k, v in self.lidar_summary.items()},
            'command': {
                'psi_error_deg': round(self.command_status['psi_error'], 1),
                'thrust': round(self.command_status['tau_x'], 0)
            },
            'waypoint': wp_info,
            'status': {
                'is_stuck': self.is_stuck,
                'stuck_duration_s': round(self.stuck_duration, 1),
                'override_active': self.override_active,
                'override_reason': self.override_reason
            }
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LLMInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
