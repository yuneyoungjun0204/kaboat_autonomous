#!/usr/bin/env python3
"""
Action Dispatcher - LLM 명령을 모듈로 실행하는 브릿지 노드

LLM(ros-mcp)이 /llm_action 토픽으로 JSON 명령을 보내면
적절한 maneuvers.py 함수를 호출하고 /command로 모터 명령 발행.

토픽:
  [구독]
  - /llm_action (String): LLM JSON 명령
  - /wamv/sensors/gps/fix: GPS
  - /wamv/sensors/imu/data: IMU
  - /wamv/sensors/lidar/scan: LiDAR

  [발행]
  - /command (Float32MultiArray): 모터 명령
  - /action_status (String): 현재 액션 상태 (JSON)
  - /lidar_summary (String): LiDAR 요약 (LLM 판단용)
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

from kaboat_autonomous.controllers.autonomous_module import Boat, normalize_angle
from kaboat_autonomous.controllers import maneuvers


class ActionDispatcher(Node):
    """LLM 명령 → 모듈 실행 디스패처"""

    def __init__(self):
        super().__init__('action_dispatcher')

        # Boat 상태
        self.boat = Boat()
        self.ref_utm_x = SETTINGS.REF_UTM_X
        self.ref_utm_y = SETTINGS.REF_UTM_Y

        # 현재 액션 상태
        self.current_action = None
        self.action_start_time = None
        self.action_params = {}
        self.waypoint_queue = []
        self.current_waypoint_idx = 0

        # 타이머 기반 상태 (dorodori 등)
        self.action_elapsed = 0.0

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/command', 10)
        self.status_pub = self.create_publisher(String, '/action_status', 10)
        self.lidar_pub = self.create_publisher(String, '/lidar_summary', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/waypoint_goal', 10)

        # === Subscribers ===
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/wamv/sensors/lidar/scan', self.lidar_callback, 10)
        self.create_subscription(String, '/llm_action', self.action_callback, 10)

        # 제어 루프 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        # 상태 발행 (2Hz)
        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info('=== Action Dispatcher Started ===')
        self.get_logger().info('Waiting for LLM commands on /llm_action')

    # === Sensor Callbacks ===

    def gps_callback(self, msg: NavSatFix):
        utm_x, utm_y, _ = SETTINGS.latlon_to_utm(msg.latitude, msg.longitude)
        self.boat.position[0] = utm_x - self.ref_utm_x
        self.boat.position[1] = utm_y - self.ref_utm_y

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.boat.psi = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

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

        # LiDAR 요약 발행 (LLM 판단용)
        summary = maneuvers.analyze_lidar(self.boat)
        msg_out = String()
        msg_out.data = json.dumps(summary)
        self.lidar_pub.publish(msg_out)

    # === LLM Action Handler ===

    def action_callback(self, msg: String):
        """LLM JSON 명령 파싱 및 액션 시작"""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '')
            self.get_logger().info(f'[LLM] Action received: {action}')

            # 액션 초기화
            self.current_action = action
            self.action_start_time = time.time()
            self.action_elapsed = 0.0
            self.action_params = cmd

            # 액션별 초기화
            if action == 'orbit':
                self._init_orbit(cmd)
            elif action == 'gate_pass':
                self._init_gate_pass(cmd)
            elif action == 'waypoints':
                self._init_waypoints(cmd)
            elif action == 'stop':
                self._stop()
            elif action == 'analyze':
                self._publish_analysis()
                self.current_action = None

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')

    def _init_orbit(self, cmd):
        """궤도 웨이포인트 생성"""
        idx = cmd.get('lidar_idx', 0)
        radius = cmd.get('radius', SETTINGS.ORBIT_DEFAULT_RADIUS)
        direction = cmd.get('direction', 'cw')
        laps = cmd.get('laps', 1.0)

        waypoints = maneuvers.plan_orbit(self.boat, idx, radius, direction, laps=laps)
        if waypoints:
            self.waypoint_queue = waypoints
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Orbit: {len(waypoints)} waypoints generated')
        else:
            self.get_logger().warn(f'Orbit failed: no valid lidar at idx {idx}')
            self.current_action = None

    def _init_gate_pass(self, cmd):
        """게이트 중점 웨이포인트 생성"""
        left_idx = cmd.get('left_idx', 45)
        right_idx = cmd.get('right_idx', 315)

        wp = maneuvers.midpoint_waypoint_from_scan(self.boat, left_idx, right_idx)
        if wp:
            self.waypoint_queue = [wp]
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Gate pass: midpoint at ({wp[0]:.1f}, {wp[1]:.1f})')
        else:
            self.get_logger().warn('Gate pass failed: invalid lidar indices')
            self.current_action = None

    def _init_waypoints(self, cmd):
        """웨이포인트 리스트 설정"""
        wps = cmd.get('waypoints', [])
        if wps:
            self.waypoint_queue = [(w['x'], w['y']) for w in wps]
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Waypoints: {len(wps)} loaded')

    def _stop(self):
        """정지"""
        self.current_action = None
        self.waypoint_queue = []
        cmd = Float32MultiArray()
        cmd.data = [0.0, 0.0, float(SETTINGS.MAX_THRUST)]
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Stopped')

    def _publish_analysis(self):
        """LiDAR 분석 결과 발행"""
        analysis = maneuvers.analyze_lidar(self.boat)
        analysis['position'] = {
            'x': round(self.boat.position[0], 1),
            'y': round(self.boat.position[1], 1),
            'heading': round(self.boat.psi, 1)
        }
        msg = String()
        msg.data = json.dumps(analysis, indent=2)
        self.status_pub.publish(msg)

    # === Control Loop ===

    def control_loop(self):
        """10Hz 제어 루프"""
        if self.current_action is None:
            return

        self.action_elapsed = time.time() - self.action_start_time
        psi_error, tau_x = 0.0, 0.0

        # 액션별 처리
        if self.current_action == 'navigate_avoid':
            psi_error, tau_x = self._exec_navigate_avoid()
        elif self.current_action == 'navigate_direct':
            psi_error, tau_x = self._exec_navigate_direct()
        elif self.current_action == 'backward':
            psi_error, tau_x = self._exec_backward()
        elif self.current_action == 'dorodori':
            psi_error, tau_x = self._exec_dorodori()
        elif self.current_action == 'hover':
            psi_error, tau_x = self._exec_hover()
        elif self.current_action in ('orbit', 'gate_pass', 'waypoints'):
            psi_error, tau_x = self._exec_waypoint_follow()

        # 명령 발행
        cmd = Float32MultiArray()
        cmd.data = [float(psi_error), float(tau_x), float(SETTINGS.MAX_THRUST)]
        self.cmd_pub.publish(cmd)

    def _exec_navigate_avoid(self):
        """장애물 회피 항법"""
        goal_x = self.action_params.get('goal_x', self.boat.position[0])
        goal_y = self.action_params.get('goal_y', self.boat.position[1])

        if maneuvers.is_goal_reached(self.boat, goal_x, goal_y):
            self.get_logger().info('navigate_avoid: goal reached')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.navigate_avoid(self.boat, goal_x, goal_y)

    def _exec_navigate_direct(self):
        """직진 항법"""
        goal_x = self.action_params.get('goal_x', self.boat.position[0])
        goal_y = self.action_params.get('goal_y', self.boat.position[1])

        if maneuvers.is_goal_reached(self.boat, goal_x, goal_y):
            self.get_logger().info('navigate_direct: goal reached')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.navigate_direct(self.boat, goal_x, goal_y)

    def _exec_backward(self):
        """후진"""
        duration = self.action_params.get('duration', 3.0)
        hold_heading = self.action_params.get('hold_heading', self.boat.psi)

        if self.action_elapsed >= duration:
            self.get_logger().info('backward: complete')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.backward(self.boat, hold_heading=hold_heading)

    def _exec_dorodori(self):
        """좌우 스캔"""
        duration = self.action_params.get('duration', SETTINGS.DORODORI_PERIOD_SEC)
        center = self.action_params.get('center_heading', self.boat.psi)
        half_range = self.action_params.get('half_range', SETTINGS.DORODORI_HALF_RANGE_DEG)

        if self.action_elapsed >= duration:
            self.get_logger().info('dorodori: complete')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.dorodori(self.boat, center, half_range, self.action_elapsed)

    def _exec_hover(self):
        """호버링"""
        duration = self.action_params.get('duration', 5.0)
        hold_x = self.action_params.get('x', self.boat.position[0])
        hold_y = self.action_params.get('y', self.boat.position[1])

        if self.action_elapsed >= duration:
            self.get_logger().info('hover: complete')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.hover(self.boat, hold_x, hold_y)

    def _exec_waypoint_follow(self):
        """웨이포인트 순차 추종 (orbit, gate_pass, waypoints)"""
        if self.current_waypoint_idx >= len(self.waypoint_queue):
            self.get_logger().info(f'{self.current_action}: all waypoints complete')
            self.current_action = None
            return 0.0, 0.0

        goal_x, goal_y = self.waypoint_queue[self.current_waypoint_idx]

        if maneuvers.is_goal_reached(self.boat, goal_x, goal_y):
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'{self.current_action}: waypoint {self.current_waypoint_idx}/{len(self.waypoint_queue)}'
            )
            return 0.0, 0.0

        # 장애물 회피 사용 여부
        use_avoidance = self.action_params.get('use_avoidance', True)
        if use_avoidance:
            return maneuvers.navigate_avoid(self.boat, goal_x, goal_y)
        else:
            return maneuvers.navigate_direct(self.boat, goal_x, goal_y)

    # === Status Publishing ===

    def publish_status(self):
        """현재 상태 발행 (LLM 모니터링용)"""
        status = {
            'timestamp': round(time.time(), 1),
            'current_action': self.current_action,
            'action_elapsed_sec': round(self.action_elapsed, 1) if self.current_action else 0,
            'position': {
                'x': round(self.boat.position[0], 1),
                'y': round(self.boat.position[1], 1),
                'heading_deg': round(self.boat.psi, 1)
            },
            'waypoints': {
                'total': len(self.waypoint_queue),
                'current': self.current_waypoint_idx,
                'remaining': len(self.waypoint_queue) - self.current_waypoint_idx
            }
        }

        # 현재 웨이포인트까지 거리
        if self.waypoint_queue and self.current_waypoint_idx < len(self.waypoint_queue):
            gx, gy = self.waypoint_queue[self.current_waypoint_idx]
            info = maneuvers.get_goal_info(self.boat, gx, gy)
            status['current_goal'] = info

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
