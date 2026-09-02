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
from sensor_msgs.msg import NavSatFix, Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
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

        # 3D 포인트클라우드 (클러스터링 우선 소스) - 신선하지 않으면(끊김 등)
        # lidar_callback에서 자동으로 2D LaserScan 클러스터링으로 폴백
        self.points_3d = None
        self.points_3d_time = None
        self._points_3d_fallback_logged = False

        # === 압축 상태(blackboard) ===
        # LLM 대화를 매 판단마다 짧게 끊어도(bounded-context) "방금 뭘 했는지,
        # 뭐가 실패했는지, 이미 아니라고 확인된 방향"을 잃지 않도록 이 노드
        # 프로세스(미션 내내 살아있음)가 최소 정보만 요약해서 들고 있는다.
        self.last_action = None
        self.last_action_result = None
        self.action_retry_count = {}   # {action_name: 연속 실패 횟수}
        self.rejected_clusters = []    # [{'x':.., 'y':.., 'reason':..}, ...] 전역 좌표

        # === 미션 단계 자동 전환(FSM) ===
        # settings.MISSION_SEQUENCE를 따라 순수 이동 구간(requires_llm=False)은
        # 도착 즉시 스스로 다음 지점으로 navigate_avoid를 발행한다. 비전이
        # 필요한 지점(requires_llm=True)에 도착하면 멈추고 LLM에게 넘기며,
        # LLM은 그 작업을 마치면 'mission_phase_done'을 호출해야 다음 자동
        # 구간이 재개된다. 'mission_auto_pause'/'mission_auto_resume'으로
        # 언제든 자동 진행을 멈추고 수동 개입할 수 있다.
        self.mission_phase_idx = 0
        self.mission_auto_enabled = True
        self._mission_waypoints_local = SETTINGS.get_mission_waypoints_local()  # [(x,y,name,desc,requires_llm), ...]

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/command', 10)
        self.status_pub = self.create_publisher(String, '/action_status', 10)
        self.lidar_pub = self.create_publisher(String, '/lidar_summary', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, '/waypoint_goal', 10)

        # === Subscribers ===
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/wamv/sensors/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/wamv/sensors/lidar/scan', self.lidar_callback, 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidar/points', self.points_callback, 10)
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

    def points_callback(self, msg: PointCloud2):
        """3D 포인트클라우드 수신 - 클러스터링 우선 소스로 저장.
        analyze_lidar()가 boat.scan(2D)과 별개로 이 데이터를 우선 사용한다
        (신선할 때만 - lidar_callback에서 staleness 판단)."""
        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if len(points) == 0:
            return
        self.points_3d = np.column_stack([
            points['x'].astype(np.float64).ravel(),
            points['y'].astype(np.float64).ravel(),
            points['z'].astype(np.float64).ravel(),
        ])
        self.points_3d_time = time.time()
        if self._points_3d_fallback_logged:
            self.get_logger().info('[Cluster] PointCloud2 복구됨 - 3D 클러스터링으로 복귀')
            self._points_3d_fallback_logged = False

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

        # 3D 포인트클라우드가 신선하면 클러스터링에 우선 사용, 아니면 2D 폴백
        # (cluster_visualizer.py의 3D 우선/2D 폴백 방식을 실제 미션 파이프라인에도 적용)
        points_3d = None
        if self.points_3d is not None and (time.time() - self.points_3d_time) <= SETTINGS.CLUSTER3D_STALE_SEC:
            points_3d = self.points_3d
        elif not self._points_3d_fallback_logged:
            self.get_logger().warn(
                f'[Cluster] PointCloud2가 {SETTINGS.CLUSTER3D_STALE_SEC}초 이상 안 들어옴 - '
                '2D LaserScan 클러스터링으로 폴백'
            )
            self._points_3d_fallback_logged = True

        # LiDAR 요약 발행 (LLM 판단용) - 거부 기록된 클러스터는 rejected:true로 표시
        summary = maneuvers.analyze_lidar(self.boat, points_3d=points_3d)
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
            elif action == 'pass_between_clusters':
                self._init_pass_between_clusters(cmd)
            elif action == 'waypoints':
                self._init_waypoints(cmd)
            elif action == 'stop':
                self._stop()
            elif action == 'dorodori':
                self._init_dorodori(cmd)
            elif action == 'align':
                self.align_settle_count = 0  # 정착 카운터 리셋
            elif action == 'align_to_cluster':
                self.align_settle_count = 0
                self._init_align_to_cluster(cmd)
            elif action == 'advance_bearing':
                self._init_advance_bearing(cmd)
            elif action == 'analyze':
                self._publish_analysis()
                self.current_action = None
            elif action == 'reject_cluster':
                self._reject_cluster(cmd)
                self.current_action = None
            elif action == 'mission_phase_done':
                self._mission_phase_done()
                self.current_action = None
            elif action == 'mission_auto_pause':
                self.mission_auto_enabled = False
                self.get_logger().info('[Mission] auto-transit paused')
                self.current_action = None
            elif action == 'mission_auto_resume':
                self.mission_auto_enabled = True
                self.get_logger().info('[Mission] auto-transit resumed')
                self.current_action = None
            elif action == 'set_goal_range':
                new_range = float(cmd.get('value', 5.0))
                new_range = max(1.0, min(25.0, new_range))  # 1-25 범위 제한
                SETTINGS.GOAL_RANGE = new_range
                self.get_logger().info(f'[Settings] GOAL_RANGE = {new_range}m')
                self.current_action = None
            elif action == 'set_speed':
                new_speed = float(cmd.get('value', 5.0))
                new_speed = max(1.0, min(10.0, new_speed))  # 1-10 범위 제한
                SETTINGS.SPEED_MULTIPLIER = new_speed
                self.get_logger().info(f'[Settings] SPEED_MULTIPLIER = {new_speed}')
                self.current_action = None
            elif action == 'get_settings':
                self.get_logger().info(
                    f'[Settings] GOAL_RANGE={SETTINGS.GOAL_RANGE}m, '
                    f'SPEED_MULTIPLIER={SETTINGS.SPEED_MULTIPLIER}'
                )
                self.current_action = None

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')

    # === 미션 단계 자동 전환(FSM) ===

    def _current_mission_phase(self):
        """현재 미션 단계 정보. 시퀀스를 다 마쳤으면 None."""
        if self.mission_phase_idx >= len(self._mission_waypoints_local):
            return None
        return self._mission_waypoints_local[self.mission_phase_idx]

    def _mission_phase_status(self) -> dict:
        """/action_status용 미션 단계 요약."""
        phase = self._current_mission_phase()
        return {
            'index': self.mission_phase_idx,
            'total': len(self._mission_waypoints_local),
            'name': phase[2] if phase else None,
            'requires_llm': phase[4] if phase else None,
            'auto_enabled': self.mission_auto_enabled,
        }

    def _mission_waypoint_distance(self) -> float:
        """현재 미션 웨이포인트까지의 거리 (m)."""
        phase = self._current_mission_phase()
        if phase is None:
            return 0.0
        x, y = phase[0], phase[1]
        dx = x - self.boat.position[0]
        dy = y - self.boat.position[1]
        return round((dx**2 + dy**2)**0.5, 1)

    def _mission_phase_done(self):
        """LLM이 비전이 필요한 단계(게이트 통과/부표선회/도킹)를 마쳤을 때
        호출 - 다음 자동 구간이 재개되도록 인덱스를 전진시킨다."""
        phase = self._current_mission_phase()
        if phase is None:
            self.get_logger().warn('mission_phase_done: 이미 미션 시퀀스 끝')
            return
        name, requires_llm = phase[2], phase[4]
        if not requires_llm:
            self.get_logger().warn(
                f"mission_phase_done: 현재 단계 '{name}'는 requires_llm=False라 "
                "이미 자동 진행 대상임 - 호출 불필요했지만 그대로 전진시킴"
            )
        self.mission_phase_idx += 1
        self.get_logger().info(f"[Mission] phase done: '{name}' → 다음 자동 구간 재개")

    def _check_waypoint_arrival(self):
        """액션 실행 중에도 웨이포인트 도착 체크 - 도착 시 미션 단계 자동 전환.
        _maybe_auto_advance_mission은 current_action is None일 때만 호출되므로,
        외부 LLM 명령 실행 중에는 미션 전환이 안 됨. 이 함수로 보완."""
        if not self.mission_auto_enabled:
            return

        phase = self._current_mission_phase()
        if phase is None:
            return

        x, y, name, _desc, requires_llm = phase

        if maneuvers.is_goal_reached(self.boat, x, y):
            if requires_llm:
                return  # LLM 필요한 단계는 자동 전환 안 함
            self.get_logger().info(f"[Auto] ✓ Waypoint '{name}' reached during action - advancing phase")
            self.mission_phase_idx += 1

    def _start_simple_action(self, action: str, params: dict):
        """init 단계가 필요 없는 액션(navigate_avoid 등)을 코드에서 직접 시작.
        LLM이 /llm_action으로 보낼 때와 동일한 상태 초기화를 거친다."""
        self.current_action = action
        self.action_start_time = time.time()
        self.action_elapsed = 0.0
        self.action_params = dict(params, action=action)

    def _maybe_auto_advance_mission(self):
        """구간 전환 자동화: current_action이 비어있을 때(=decision_needed)
        매 tick 호출된다. 순수 이동 구간이면 도착할 때까지 스스로
        navigate_avoid를 발행하고, 이미 도착했으면 다음 구간으로 인덱스를
        전진시켜 연쇄 진행한다(여러 구간이 연속으로 이미 도착해 있는 경우도
        한 번에 처리). 비전이 필요한 지점에 도착하면 멈추고 LLM에게 넘긴다."""
        if not self.mission_auto_enabled:
            return

        while True:
            phase = self._current_mission_phase()
            if phase is None:
                return  # 미션 시퀀스 끝 - 자동 진행 없음

            x, y, name, _desc, requires_llm = phase

            # 도착 거리 계산
            dx = x - self.boat.position[0]
            dy = y - self.boat.position[1]
            dist = (dx**2 + dy**2)**0.5

            if not maneuvers.is_goal_reached(self.boat, x, y):
                self._start_simple_action('navigate_avoid', {'goal_x': x, 'goal_y': y})
                self.get_logger().info(f"[Auto] transit → '{name}' dist={dist:.1f}m (goal={SETTINGS.GOAL_RANGE}m)")
                return

            # 이미 도착함
            self.get_logger().info(f"[Auto] ✓ ARRIVED at '{name}' dist={dist:.1f}m < {SETTINGS.GOAL_RANGE}m")
            if requires_llm:
                self.get_logger().info(f"[Auto] arrived at '{name}' (vision phase) - LLM에게 넘김")
                return  # decision_needed=true, LLM이 mission_phase_done 호출할 때까지 대기

            self.mission_phase_idx += 1  # 순수 이동 지점이면 바로 다음 구간 확인 (루프 계속)

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

    def _init_pass_between_clusters(self, cmd):
        """2D LiDAR 스캔 두 점 사이 중점 웨이포인트 생성

        ★ 부표길 유지: 직선 통과 (use_avoidance=False)
        ★ 중점 + 연장점으로 확실히 통과

        Parameters:
            left_idx: 왼쪽 물체의 LiDAR 스캔 인덱스 (0-360)
            right_idx: 오른쪽 물체의 LiDAR 스캔 인덱스 (0-360)
            extend_dist: 통과 후 연장 거리 (기본 10m)
        """
        left_idx = cmd.get('left_idx', 315)   # 기본: 왼쪽 315° (= -45°)
        right_idx = cmd.get('right_idx', 45)  # 기본: 오른쪽 45°
        extend_dist = cmd.get('extend_dist', 10.0)

        # LiDAR 스캔에서 두 점의 중점 계산
        wp = maneuvers.midpoint_waypoint_from_scan(self.boat, left_idx, right_idx)

        if wp is None:
            self.get_logger().warn(
                f'pass_between: invalid LiDAR idx {left_idx}, {right_idx}'
            )
            self._record_result('pass_between_clusters', 'failed_no_lidar')
            self.current_action = None
            return

        mid_x, mid_y = wp

        # 보트 → 중점 방향으로 연장점 계산 (확실히 통과하도록)
        import math
        boat_x, boat_y = self.boat.position[0], self.boat.position[1]
        dx = mid_x - boat_x
        dy = mid_y - boat_y
        dist_to_mid = math.sqrt(dx*dx + dy*dy)

        if dist_to_mid > 0.1:
            ux, uy = dx / dist_to_mid, dy / dist_to_mid
            ext_x = mid_x + ux * extend_dist
            ext_y = mid_y + uy * extend_dist
            self.waypoint_queue = [(mid_x, mid_y), (ext_x, ext_y)]
        else:
            self.waypoint_queue = [(mid_x, mid_y)]

        self.current_waypoint_idx = 0

        # ★ 부표길 유지: 직선 통과 (장애물 회피 비활성화)
        self.action_params['use_avoidance'] = False

        self.get_logger().info(
            f'pass_between: idx[{left_idx}] ↔ idx[{right_idx}] → DIRECT path ({mid_x:.1f}, {mid_y:.1f})'
        )

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
        # 액션 실행 중에도 웨이포인트 도착 체크 (미션 단계 자동 전환)
        self._check_waypoint_arrival()

        if self.current_action is None:
            self._maybe_auto_advance_mission()
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
        elif self.current_action in ('orbit', 'gate_pass', 'pass_between_clusters', 'waypoints'):
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

    def _init_dorodori(self, cmd):
        """도리도리 시작 시 스윕 기준각(center_heading)을 한 번만 고정한다.

        _exec_dorodori가 매 틱 center_heading을 다시 조회하면서 기본값으로
        self.boat.psi(회전 중인 현재 헤딩)를 썼던 과거 구현은, LLM이
        center_heading을 생략할 때마다 기준각이 매 틱 최신 헤딩으로 다시
        잡혀버려 실제로는 좌우로 훑지 않고 한쪽으로 계속 드리프트하는
        버그가 있었다 (2026-08-28 확인). 시작 시점에 한 번만 계산해
        action_params에 고정해둔다.

        cmd:
            center_heading: 스윕 기준각(도, 절대 헤딩). 지정하면 최우선 사용.
            center_bearing_to_goal: true면 center_heading 대신 현재 미션
                구간 목표 지점 방향(bearing)을 기준각으로 사용 - 탐색
                방향을 진행 방향 쪽으로 맞추고 싶을 때.
            (둘 다 없으면 시작 시점의 현재 헤딩을 기준각으로 사용)
        """
        if 'center_heading' in cmd:
            center = float(cmd['center_heading'])
        elif cmd.get('center_bearing_to_goal'):
            phase = self._current_mission_phase()
            if phase is not None:
                goal_x, goal_y = phase[0], phase[1]
                center = maneuvers.get_goal_info(self.boat, goal_x, goal_y)['bearing']
            else:
                center = self.boat.psi
        else:
            center = self.boat.psi
        self.action_params['center_heading'] = center
        self.get_logger().info(f'dorodori: center_heading={center:.1f}° (시작 시점 고정)')

    def _exec_dorodori(self):
        """좌우 스캔 (여러 번 왕복하며 탐색)"""
        duration = self.action_params.get('duration', SETTINGS.DORODORI_DEFAULT_DURATION)
        center = self.action_params['center_heading']
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

    def _init_advance_bearing(self, cmd):
        """카메라에서 타깃(부표 등)이 살짝이라도 보이는데 LiDAR 클러스터가
        아직 안 잡힐 때(사거리 밖, 얇은 물체, 각도 미세 오차 등) 쓰는 액션.
        그 자리에서 align/dorodori로 계속 재탐색만 반복하면 제자리를 맴도는
        것처럼 보인다 (2026-08-26 사용자 피드백: 부표 탐색 중 실제로 이 문제
        발생). 카메라로 추정한 상대 방위각만으로 일단 거리를 좁혀 클러스터
        탐지 범위 안으로 들어오게 한 뒤 재평가한다.

        cmd:
            bearing_deg: 보트 기준 상대 방위각(도, 카메라/LiDAR 컨벤션과 동일
                - 0=정면, 양수=좌현, 음수=우현)
            distance: 전진 거리(m), 기본 SETTINGS.VISUAL_ADVANCE_DISTANCE
        """
        bearing = cmd.get('bearing_deg', 0.0)
        distance = cmd.get('distance', SETTINGS.VISUAL_ADVANCE_DISTANCE)
        target_heading = normalize_angle(self.boat.psi + bearing)
        rad = np.radians(target_heading)
        goal_x = self.boat.position[0] + distance * np.cos(rad)
        goal_y = self.boat.position[1] + distance * np.sin(rad)

        self.current_action = 'navigate_direct'
        self.action_params = {
            'action': 'navigate_direct',
            'goal_x': goal_x, 'goal_y': goal_y,
            'hold_heading': target_heading,
        }
        self.get_logger().info(
            f'[Visual] advance_bearing: bearing={bearing:+.1f}° dist={distance:.1f}m '
            f'-> ({goal_x:.1f}, {goal_y:.1f})'
        )

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
            'mission_phase': self._mission_phase_status(),
            'waypoint_distance': self._mission_waypoint_distance(),
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
