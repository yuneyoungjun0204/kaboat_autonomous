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
from datetime import datetime
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append('/home/yune/ros-mcp-server/history')

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

from kaboat_autonomous.controllers.autonomous_module import Boat, normalize_angle
from kaboat_autonomous.controllers import maneuvers

try:
    from mission_logger import MissionLogger
    LOGGING_ENABLED = True
except ImportError:
    LOGGING_ENABLED = False
    print("[ActionDispatcher] mission_logger not found, logging disabled")


class ActionDispatcher(Node):
    """LLM 명령 → 모듈 실행 디스패처"""

    def __init__(self):
        super().__init__('action_dispatcher')

        # 미션 로거 초기화
        if LOGGING_ENABLED:
            self.logger = MissionLogger()
            self.get_logger().info(f'Mission logging to: {self.logger.get_session_path()}')
        else:
            self.logger = None

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
        self.align_settle_count = 0  # align이 tolerance 이내에 연속으로 머문 틱 수
        self.yaw_rate_deg = 0.0      # IMU 기반 실제 요(yaw) 각속도 (도/초)

        # === 압축 상태(blackboard) ===
        # LLM 대화를 매 판단마다 짧게 끊어도(bounded-context) "방금 뭘 했는지,
        # 뭐가 실패했는지, 이미 아니라고 확인된 방향"을 잃지 않도록 이 노드
        # 프로세스(미션 내내 살아있음)가 최소 정보만 요약해서 들고 있는다.
        self.last_action = None
        self.last_action_result = None
        self.action_retry_count = {}   # {action_name: 연속 실패 횟수}
        self.rejected_clusters = []    # [{'x':.., 'y':.., 'reason':..}, ...] 전역 좌표

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/command', 10)
        self.status_pub = self.create_publisher(String, '/action_status', 10)
        self.lidar_pub = self.create_publisher(String, '/lidar_summary', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/waypoint_goal', 10)
        self.reasoning_pub = self.create_publisher(String, '/llm_reasoning', 10)
        self.vision_pub = self.create_publisher(String, '/vision_analysis', 10)

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
        # 실제 요(yaw) 각속도 - align 정착 판정에 사용. heading이 순간적으로
        # tolerance 안에 들어온 것만으로는 "회전이 멈췄다"를 보장 못 한다
        # (빠르게 지나치는 중에도 위치 조건은 잠깐 만족될 수 있음 - 2026-08-26
        # 실측: 목표 116°에서 141.8°까지 밀려난 뒤에야 정지).
        self.yaw_rate_deg = np.degrees(msg.angular_velocity.z)

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

        # LiDAR 요약 발행 (LLM 판단용) - 거부 기록된 클러스터는 rejected:true로 표시
        summary = maneuvers.analyze_lidar(self.boat)
        summary['clusters'] = maneuvers.annotate_cluster_rejection(
            self.boat, summary['clusters'], self.rejected_clusters
        )
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

            # LLM 추론 내용 발행 (웹 시각화용)
            if 'reasoning' in cmd:
                reasoning_msg = String()
                reasoning_msg.data = json.dumps({
                    'timestamp': time.time(),
                    'action': action,
                    'reasoning': cmd['reasoning'],
                    'confidence': cmd.get('confidence', 1.0)
                })
                self.reasoning_pub.publish(reasoning_msg)

            # 이미지 분석 결과 발행 (웹 시각화용)
            if 'vision_analysis' in cmd:
                vision_msg = String()
                vision_msg.data = json.dumps({
                    'timestamp': time.time(),
                    'detections': cmd['vision_analysis']
                })
                self.vision_pub.publish(vision_msg)

            # 미션 로깅
            if self.logger:
                sensor_state = {
                    'position': {'x': self.boat.position[0], 'y': self.boat.position[1]},
                    'heading': self.boat.psi
                }
                self.logger.log_action(action, cmd, "started")
                self.logger.log_sensor_state(sensor_state)

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
            elif action == 'align':
                self.align_settle_count = 0  # 정착 카운터 리셋
            elif action == 'align_to_cluster':
                self.align_settle_count = 0
                self._init_align_to_cluster(cmd)
            elif action == 'analyze':
                self._publish_analysis()
                self.current_action = None
            elif action == 'reject_cluster':
                self._reject_cluster(cmd)
                self.current_action = None

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')

    # === 압축 상태(blackboard) 기록 ===

    FAILURE_RESULTS = {'timeout', 'failed_no_lidar'}

    def _record_result(self, action: str, result: str):
        """액션이 끝날 때 결과를 요약 기록. 실패면 재시도 카운트 증가, 성공이면 리셋."""
        self.last_action = action
        self.last_action_result = result
        if result in self.FAILURE_RESULTS:
            self.action_retry_count[action] = self.action_retry_count.get(action, 0) + 1
        else:
            self.action_retry_count[action] = 0

    def _reject_cluster(self, cmd):
        """카메라로 확인해 타깃이 아니라고 판정된 클러스터를 전역 좌표로 기억
        (같은 곳으로 재탐색/재정렬하지 않도록). 대화가 짧게 끊겨도 이 노드
        프로세스가 살아있는 동안은 유지됨."""
        angle = cmd.get('angle')
        if angle is None:
            self.get_logger().warn('reject_cluster: angle missing')
            return

        distance = cmd.get('distance')
        if distance is None:
            distance = self.boat.scan[int(round(angle)) % 360]
        if not distance or distance <= 0:
            self.get_logger().warn('reject_cluster: no valid distance, skip')
            return

        x, y = maneuvers.reject_cluster_point(self.boat, float(angle), float(distance))
        self.rejected_clusters.append({
            'x': round(x, 1), 'y': round(y, 1),
            'reason': cmd.get('reason', ''),
        })
        if len(self.rejected_clusters) > SETTINGS.REJECT_CLUSTER_MAX_COUNT:
            self.rejected_clusters.pop(0)
        self.get_logger().info(f"Cluster rejected @ ({x:.1f}, {y:.1f}): {cmd.get('reason', '')}")

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
            self._record_result('orbit', 'failed_no_lidar')
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
            self._record_result('gate_pass', 'failed_no_lidar')
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
        elif self.current_action == 'align':
            psi_error, tau_x = self._exec_align()
        elif self.current_action == 'align_to_cluster':
            psi_error, tau_x = self._exec_align_to_cluster()
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
            if self.logger:
                self.logger.log_waypoint_reached('navigate_avoid_goal', {
                    'x': self.boat.position[0], 'y': self.boat.position[1]
                })
            self._record_result('navigate_avoid', 'goal_reached')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.navigate_avoid(self.boat, goal_x, goal_y)

    def _exec_navigate_direct(self):
        """직진 항법 (hold_heading 지정 시 목표 방향 대신 해당 헤딩을 유지하며 직진)"""
        goal_x = self.action_params.get('goal_x', self.boat.position[0])
        goal_y = self.action_params.get('goal_y', self.boat.position[1])
        hold_heading = self.action_params.get('hold_heading')

        if maneuvers.is_goal_reached(self.boat, goal_x, goal_y):
            self.get_logger().info('navigate_direct: goal reached')
            if self.logger:
                self.logger.log_waypoint_reached('navigate_direct_goal', {
                    'x': self.boat.position[0], 'y': self.boat.position[1]
                })
            self._record_result('navigate_direct', 'goal_reached')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.navigate_direct(self.boat, goal_x, goal_y, hold_heading=hold_heading)

    def _exec_backward(self):
        """후진"""
        duration = self.action_params.get('duration', 3.0)
        hold_heading = self.action_params.get('hold_heading', self.boat.psi)

        if self.action_elapsed >= duration:
            self.get_logger().info('backward: complete')
            self._record_result('backward', 'completed')
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
            self._record_result('dorodori', 'completed')
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
            self._record_result('hover', 'completed')
            self.current_action = None
            return 0.0, 0.0

        return maneuvers.hover(self.boat, hold_x, hold_y)

    def _exec_align(self):
        """헤딩 정렬 - tolerance 이내 + 실제 회전(yaw rate)이 거의 멎은 상태가
        ALIGN_SETTLE_TICKS만큼 연속돼야 완료.
        위치(psi_error)만 보고 끝내면, 빠르게 회전하며 tolerance 구간을
        스쳐 지나가는 순간에도 조건이 잠깐 만족돼 제어가 끊기고 - 그 시점의
        회전 관성 때문에 목표를 훨씬 넘어가버린다 (2026-08-26 실측: 목표
        116°에서 141.8°까지 밀려난 뒤에야 정지). yaw_rate_deg를 함께 봐야
        진짜로 멈췄는지 알 수 있다."""
        target_heading = self.action_params.get('heading', 0.0)
        tolerance = self.action_params.get('tolerance', 5.0)
        timeout = self.action_params.get('timeout', 10.0)

        # 타임아웃 체크
        if self.action_elapsed >= timeout:
            self.get_logger().info('align: timeout')
            self._record_result('align', 'timeout')
            self.current_action = None
            self.align_settle_count = 0
            return 0.0, 0.0

        psi_error, tau_x, in_tolerance = maneuvers.align_to_heading(
            self.boat, target_heading, tolerance
        )

        settled_now = in_tolerance and abs(self.yaw_rate_deg) < SETTINGS.ALIGN_SETTLE_MAX_YAW_RATE_DEG
        self.align_settle_count = self.align_settle_count + 1 if settled_now else 0

        if self.align_settle_count >= SETTINGS.ALIGN_SETTLE_TICKS:
            self.get_logger().info(f'align: aligned to {target_heading:.1f}° (settled)')
            self._record_result('align', 'aligned')
            self.current_action = None
            self.align_settle_count = 0
            return 0.0, 0.0

        return psi_error, tau_x

    def _init_align_to_cluster(self, cmd):
        """클러스터 기준 정렬 초기화 - 클러스터 ID로 해당 물체 방향을 찾아 저장"""
        cluster_id = cmd.get('cluster_id', 0)
        clusters = maneuvers.detect_lidar_clusters(self.boat)

        if cluster_id < len(clusters):
            cluster = clusters[cluster_id]
            self.action_params['target_angle'] = cluster['center_angle']
            self.get_logger().info(
                f'align_to_cluster: id={cluster_id} → angle={cluster["center_angle"]:.1f}°'
            )
        else:
            self.get_logger().warn(f'align_to_cluster: id={cluster_id} not found')
            self.action_params['target_angle'] = None

    def _exec_align_to_cluster(self):
        """클러스터 기준 정렬 실행 - 저장된 클러스터 각도로 정렬"""
        target_angle = self.action_params.get('target_angle')
        if target_angle is None:
            self._record_result('align_to_cluster', 'failed_no_cluster')
            self.current_action = None
            return 0.0, 0.0

        # 클러스터 각도를 절대 헤딩으로 변환
        target_heading = (self.boat.psi + target_angle) % 360
        tolerance = self.action_params.get('tolerance', 5.0)
        timeout = self.action_params.get('timeout', 10.0)

        if self.action_elapsed >= timeout:
            self.get_logger().info('align_to_cluster: timeout')
            self._record_result('align_to_cluster', 'timeout')
            self.current_action = None
            self.align_settle_count = 0
            return 0.0, 0.0

        psi_error, tau_x, in_tolerance = maneuvers.align_to_heading(
            self.boat, target_heading, tolerance
        )

        settled_now = in_tolerance and abs(self.yaw_rate_deg) < SETTINGS.ALIGN_SETTLE_MAX_YAW_RATE_DEG
        self.align_settle_count = self.align_settle_count + 1 if settled_now else 0

        if self.align_settle_count >= SETTINGS.ALIGN_SETTLE_TICKS:
            self.get_logger().info(f'align_to_cluster: aligned to cluster angle {target_angle:.1f}°')
            self._record_result('align_to_cluster', 'aligned')
            self.current_action = None
            self.align_settle_count = 0
            return 0.0, 0.0

        return psi_error, tau_x

    def _exec_waypoint_follow(self):
        """웨이포인트 순차 추종 (orbit, gate_pass, waypoints)"""
        if self.current_waypoint_idx >= len(self.waypoint_queue):
            self.get_logger().info(f'{self.current_action}: all waypoints complete')
            if self.logger:
                self.logger.log_action(self.current_action, {}, "completed")
            self._record_result(self.current_action, 'completed')
            self.current_action = None
            return 0.0, 0.0

        goal_x, goal_y = self.waypoint_queue[self.current_waypoint_idx]

        if maneuvers.is_goal_reached(self.boat, goal_x, goal_y):
            self.current_waypoint_idx += 1
            self.get_logger().info(
                f'{self.current_action}: waypoint {self.current_waypoint_idx}/{len(self.waypoint_queue)}'
            )
            if self.logger:
                self.logger.log_waypoint_reached(
                    f'{self.current_action}_wp{self.current_waypoint_idx}',
                    {'x': self.boat.position[0], 'y': self.boat.position[1]}
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
            # 진행 중인 액션이 없을 때만 true. 폴링 루프가 이 플래그로
            # "지금 LLM 판단이 필요한가"를 판단해 불필요한 LLM 호출을 건너뛸 수 있다
            # (액션 실행 중엔 control_loop가 LLM 없이 자율 진행하므로 새 판단이 불필요).
            'decision_needed': self.current_action is None,
            'current_action': self.current_action,
            'action_elapsed_sec': round(self.action_elapsed, 1) if self.current_action else 0,
            # 압축 상태(blackboard) - 대화가 짧게 끊겨도 이전 판단 이력을 알 수 있게
            'last_action': self.last_action,
            'last_action_result': self.last_action_result,
            'retry_count': self.action_retry_count,
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
