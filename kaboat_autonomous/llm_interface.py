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
from sensor_msgs.msg import NavSatFix, Imu, LaserScan, Image
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import json
import time
import sys
import os
import base64
import cv2
import threading

# HSV 색상 감지기
sys.path.insert(0, '/home/yune/ros-mcp-server/kaboat_llm/perception')
try:
    from color_detector import ColorBuoyDetector, compute_navigation_error
    HSV_AVAILABLE = True
except ImportError:
    HSV_AVAILABLE = False
    print("[WARN] color_detector not found, HSV mode disabled")

# .env 파일에서 API 키 로드
from pathlib import Path
for env_path in [
    Path('/home/yune/ros-mcp-server/kaboat_llm/web/.env'),
    Path.home() / '.env',
]:
    if env_path.exists():
        for line in env_path.read_text().strip().split('\n'):
            if '=' in line and not line.startswith('#'):
                key, val = line.split('=', 1)
                os.environ.setdefault(key.strip(), val.strip())
        break

# Gemini API
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    print("[WARN] google-generativeai not installed, autonomous mode disabled")

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

        # === 자율 모드 ===
        self.autonomous_mode = False
        self.autonomous_interval = 0.5  # 초 (최대 2Hz - Gemini API 제한 고려)
        self.last_autonomous_time = 0.0
        self.cv_bridge = CvBridge()
        self.current_image = None
        self.current_image_base64 = None
        self.gemini_model = None
        self.autonomous_log = []  # 최근 판단 로그
        self.max_log_entries = 20

        # 미션 상태
        self.mission_sequence = ['gate_search', 'buoy_orbit', 'hopping_tour', 'docking']
        self.current_mission_index = 0
        self.mission_status = 'idle'  # idle, running, completed, error

        # HSV 감지기 초기화
        self.hsv_detector = None
        self.use_hsv_mode = False  # HSV 비활성화, Gemini 비전만 사용
        if HSV_AVAILABLE:
            self.hsv_detector = ColorBuoyDetector(min_area=300, enabled_colors=['red', 'green', 'yellow', 'blue'])
            self.get_logger().info('[AUTONOMOUS] HSV detector initialized')

        # Gemini API 설정 (HSV 실패 시 폴백)
        if GEMINI_AVAILABLE:
            api_key = os.environ.get('GOOGLE_API_KEY') or os.environ.get('GEMINI_API_KEY')
            if api_key:
                genai.configure(api_key=api_key)
                self.gemini_model = genai.GenerativeModel('gemini-3.5-flash-lite')
                self.get_logger().info('[AUTONOMOUS] Gemini API configured (fallback)')

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

        # 카메라 구독
        self.create_subscription(Image, '/wamv/sensors/camera/image_raw', self.camera_callback, 10)

        # 자율 모드 제어
        self.create_subscription(String, '/autonomous_control', self.autonomous_control_callback, 10)

        # === Publishers 추가 ===
        self.autonomous_log_pub = self.create_publisher(String, '/autonomous_log', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission_status', 10)
        self.action_pub = self.create_publisher(String, '/llm_action', 10)  # action_dispatcher 연동

        # 타이머
        self.create_timer(1.0, self.publish_status)
        self.create_timer(0.5, self.check_stuck)
        self.create_timer(0.5, self.autonomous_loop)  # 자율 루프

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
            'closest': float(np.min(ranges[(ranges > 0) & (ranges < SETTINGS.LIDAR_MAX_RANGE)])) if np.any((ranges > 0) & (ranges < SETTINGS.LIDAR_MAX_RANGE)) else 999.0
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

    def camera_callback(self, msg: Image):
        """카메라 이미지 수신 및 Base64 변환"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image

            # 리사이즈 후 JPEG 압축
            resized = cv2.resize(cv_image, (320, 240))
            _, buffer = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 70])
            self.current_image_base64 = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')

    def autonomous_control_callback(self, msg: String):
        """자율 모드 제어: {"action": "start"} or {"action": "stop"}"""
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')

            if action == 'start':
                if not self.gemini_model:
                    self.get_logger().error('[AUTONOMOUS] Gemini not configured')
                    return
                self.autonomous_mode = True
                self.mission_status = 'running'
                self.current_mission_index = 0
                self.get_logger().info('[AUTONOMOUS] Mode STARTED')
                self._log_autonomous('자율 모드 시작', 'system')

            elif action == 'stop':
                self.autonomous_mode = False
                self.mission_status = 'idle'
                self.get_logger().info('[AUTONOMOUS] Mode STOPPED')
                self._log_autonomous('자율 모드 중지', 'system')

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid autonomous control: {msg.data}')

    def autonomous_loop(self):
        """자율 판단 루프 - HSV 우선, Gemini 폴백"""
        if not self.autonomous_mode:
            return

        now = time.time()
        if now - self.last_autonomous_time < self.autonomous_interval:
            return

        self.last_autonomous_time = now

        # HSV 모드 (빠름, 로컬)
        if self.use_hsv_mode and self.hsv_detector and self.current_image is not None:
            self._run_hsv_analysis()
            return

        # Gemini 폴백 (느림, API)
        if self.gemini_model and self.current_image_base64:
            threading.Thread(target=self._run_autonomous_analysis, daemon=True).start()

    def _run_hsv_analysis(self):
        """HSV 기반 규칙 판단 (빠름, 로컬)"""
        try:
            detections = self.hsv_detector.detect(self.current_image)
            current_mission = self.mission_sequence[self.current_mission_index] if self.current_mission_index < len(self.mission_sequence) else 'completed'

            # 색상별 감지 결과 정리
            red_det = next((d for d in detections if d.color == 'red'), None)
            green_det = next((d for d in detections if d.color == 'green'), None)

            red_x = red_det.normalized_x if red_det else None
            green_x = green_det.normalized_x if green_det else None

            # LiDAR 정보
            front_dist = self.lidar_summary.get('front', 999)
            closest_dist = self.lidar_summary.get('closest', 999)

            action_cmd = None
            analysis = ""

            # 미션별 규칙
            if current_mission == 'gate_search':
                if red_det and green_det:
                    # 게이트 감지됨 - 중점으로 이동
                    error, desc = compute_navigation_error(red_x, green_x)
                    if abs(error) < 0.1 and front_dist > 5:
                        action_cmd = {'action': 'navigate_direct', 'goal_x': self.position[0] + 10, 'goal_y': self.position[1]}
                        analysis = f"게이트 정렬됨, 직진 ({desc})"
                    else:
                        heading_adj = -error * 30  # 오차를 헤딩 조정으로 변환
                        action_cmd = {'action': 'align', 'heading': self.heading + heading_adj, 'tolerance': 5}
                        analysis = f"게이트 정렬 중: error={error:.2f}"
                elif red_det or green_det:
                    # 한쪽만 보임 - 탐색
                    action_cmd = {'action': 'dorodori', 'duration': 3, 'half_range': 30}
                    analysis = f"부분 감지: red={red_det is not None}, green={green_det is not None}"
                else:
                    # 아무것도 안 보임 - 전방 탐색
                    action_cmd = {'action': 'dorodori', 'duration': 5, 'half_range': 45}
                    analysis = "게이트 탐색 중"

            elif current_mission == 'buoy_orbit':
                if red_det:
                    if red_det.area > 5000 and front_dist < 8:
                        # 부표 가까움 - 선회 시작
                        action_cmd = {'action': 'orbit', 'lidar_idx': int(red_x * 360), 'radius': 5, 'direction': 'cw', 'laps': 1}
                        analysis = f"빨간 부표 근접, 선회 시작 (area={red_det.area})"
                    else:
                        # 부표 방향으로 접근
                        heading_to_buoy = self.heading + (red_x - 0.5) * 60
                        action_cmd = {'action': 'align', 'heading': heading_to_buoy, 'tolerance': 10}
                        analysis = f"빨간 부표 접근 중 (x={red_x:.2f})"
                else:
                    action_cmd = {'action': 'dorodori', 'duration': 5, 'half_range': 60}
                    analysis = "빨간 부표 탐색 중"

            elif current_mission == 'hopping_tour':
                # 웨이포인트 순회 - action_dispatcher의 미션 FSM 사용
                action_cmd = {'action': 'mission_auto_resume'}
                analysis = "호핑 투어 자동 진행"

            elif current_mission == 'docking':
                # 도킹 - 노란색 또는 파란색 마커 찾기
                yellow_det = next((d for d in detections if d.color == 'yellow'), None)
                if yellow_det:
                    if yellow_det.area > 3000:
                        action_cmd = {'action': 'hover', 'duration': 5}
                        analysis = "도킹 완료"
                        self.current_mission_index += 1
                    else:
                        heading_to_dock = self.heading + (yellow_det.normalized_x - 0.5) * 40
                        action_cmd = {'action': 'navigate_direct', 'goal_x': self.position[0] + 5, 'goal_y': self.position[1]}
                        analysis = f"도킹 스테이션 접근 (x={yellow_det.normalized_x:.2f})"
                else:
                    action_cmd = {'action': 'dorodori', 'duration': 5, 'half_range': 45}
                    analysis = "도킹 스테이션 탐색 중"

            else:
                analysis = "미션 완료"
                self.autonomous_mode = False

            # 장애물 회피 오버라이드
            if closest_dist < 3 and action_cmd and action_cmd.get('action') not in ['backward', 'stop', 'hover']:
                action_cmd = {'action': 'backward', 'duration': 2}
                analysis = f"긴급 회피! 장애물 {closest_dist:.1f}m"

            # 명령 실행
            if action_cmd:
                self._log_autonomous(f'[HSV] {analysis}', 'decision')
                self._execute_decision({'action': action_cmd['action'], 'params': action_cmd, 'analysis': analysis})

        except Exception as e:
            self._log_autonomous(f'HSV 분석 오류: {e}', 'error')
            self.get_logger().error(f'[HSV] Analysis error: {e}')

    def _run_autonomous_analysis(self):
        """Gemini로 상황 분석 및 명령 결정 (action_dispatcher 활용) - 폴백용"""
        try:
            # 현재 상태 요약
            current_mission = self.mission_sequence[self.current_mission_index] if self.current_mission_index < len(self.mission_sequence) else 'completed'

            prompt = f"""KABOAT 자율주행 보트의 현재 상황을 분석하고 다음 행동을 결정하세요.

현재 미션: {current_mission}
위치: x={self.position[0]:.1f}m, y={self.position[1]:.1f}m
헤딩: {self.heading:.1f}도
장애물 (LiDAR):
  - 전방: {self.lidar_summary.get('front', 999):.1f}m
  - 좌측: {self.lidar_summary.get('left', 999):.1f}m
  - 우측: {self.lidar_summary.get('right', 999):.1f}m
  - 가장 가까운: {self.lidar_summary.get('closest', 999):.1f}m
웨이포인트: {self.current_waypoint}
Stuck 상태: {self.is_stuck}

미션 목표:
- gate_search: 빨간/초록 게이트 찾아서 통과
- buoy_orbit: 빨간 부표 주변을 시계방향으로 선회
- hopping_tour: 웨이포인트 순회
- docking: 도킹 스테이션에 정박

사용 가능한 액션 (action_dispatcher 명령):
1. navigate_avoid: 장애물 회피하며 목표로 이동 - {{"action":"navigate_avoid","goal_x":숫자,"goal_y":숫자}}
2. navigate_direct: 직선 이동 - {{"action":"navigate_direct","goal_x":숫자,"goal_y":숫자}}
3. orbit: 부표 선회 - {{"action":"orbit","lidar_idx":각도(0-359),"radius":반경,"direction":"cw"또는"ccw","laps":바퀴수}}
4. gate_pass: 게이트 통과 - {{"action":"gate_pass","left_idx":왼쪽각도,"right_idx":오른쪽각도}}
5. dorodori: 좌우 스캔 탐색 - {{"action":"dorodori","duration":초,"half_range":각도범위}}
6. align: 특정 방향 정렬 - {{"action":"align","heading":목표헤딩}}
7. align_to_cluster: LiDAR 클러스터 방향 정렬 - {{"action":"align_to_cluster","cluster_id":번호}}
8. hover: 현재 위치 유지 - {{"action":"hover","duration":초}}
9. backward: 후진 - {{"action":"backward","duration":초}}
10. stop: 정지 - {{"action":"stop"}}
11. mission_phase_done: 현재 미션 단계 완료 - {{"action":"mission_phase_done"}}

JSON 형식으로 응답:
{{"analysis": "상황 분석 (1-2문장)", "action": "액션명", "params": {{액션파라미터들}}, "mission_complete": true/false}}"""

            # 이미지와 함께 API 호출
            image_data = base64.b64decode(self.current_image_base64)
            response = self.gemini_model.generate_content([
                prompt,
                {"mime_type": "image/jpeg", "data": image_data}
            ])

            # 응답 파싱
            response_text = response.text
            self._log_autonomous(f'Gemini: {response_text[:100]}...', 'gemini')

            # JSON 추출
            try:
                # JSON 부분만 추출
                json_start = response_text.find('{')
                json_end = response_text.rfind('}') + 1
                if json_start >= 0 and json_end > json_start:
                    json_str = response_text[json_start:json_end]
                    decision = json.loads(json_str)
                    self._execute_decision(decision)
            except json.JSONDecodeError:
                self._log_autonomous(f'JSON 파싱 실패: {response_text[:50]}', 'error')

        except Exception as e:
            self._log_autonomous(f'분석 오류: {str(e)}', 'error')
            self.get_logger().error(f'[AUTONOMOUS] Analysis error: {e}')

    def _execute_decision(self, decision: dict):
        """LLM 판단 결과 실행 - action_dispatcher로 명령 전달"""
        action = decision.get('action', '')
        analysis = decision.get('analysis', '')
        params = decision.get('params', {})

        self._log_autonomous(f'판단: {analysis}', 'decision')
        self._log_autonomous(f'행동: {action}', 'action')

        # action_dispatcher가 처리하는 액션들
        dispatcher_actions = [
            'navigate_avoid', 'navigate_direct', 'orbit', 'gate_pass',
            'dorodori', 'align', 'align_to_cluster', 'hover', 'backward',
            'stop', 'waypoints', 'mission_phase_done', 'analyze',
            'reject_cluster', 'advance_bearing'
        ]

        if action in dispatcher_actions:
            # /llm_action 토픽으로 JSON 명령 발행
            action_cmd = {'action': action}
            action_cmd.update(params)
            msg = String()
            msg.data = json.dumps(action_cmd)
            self.action_pub.publish(msg)
            self._log_autonomous(f'명령 발행: {action_cmd}', 'command')

        elif action == 'waypoint':
            # 레거시 호환: 단순 웨이포인트
            x = params.get('x', decision.get('waypoint_x', 0))
            y = params.get('y', decision.get('waypoint_y', 0))
            wp_msg = PointStamped()
            wp_msg.header.stamp = self.get_clock().now().to_msg()
            wp_msg.point.x = float(x)
            wp_msg.point.y = float(y)
            self.waypoint_pub.publish(wp_msg)
            self.current_waypoint = [x, y]
            self._log_autonomous(f'웨이포인트: ({x}, {y})', 'command')

        elif action in ('forward', 'turn_left', 'turn_right'):
            # 레거시 호환: 단순 모터 명령
            if action == 'forward':
                cmd = Float32MultiArray()
                cmd.data = [0.0, 200.0]
                self.command_pub.publish(cmd)
            elif action == 'turn_left':
                cmd = Float32MultiArray()
                cmd.data = [30.0, 150.0]
                self.command_pub.publish(cmd)
            elif action == 'turn_right':
                cmd = Float32MultiArray()
                cmd.data = [-30.0, 150.0]
                self.command_pub.publish(cmd)

        # 미션 완료 체크
        if decision.get('mission_complete', False):
            self.current_mission_index += 1
            if self.current_mission_index >= len(self.mission_sequence):
                self.mission_status = 'completed'
                self.autonomous_mode = False
                self._log_autonomous('모든 미션 완료!', 'system')
            else:
                next_mission = self.mission_sequence[self.current_mission_index]
                self._log_autonomous(f'다음 미션: {next_mission}', 'system')

        # 미션 상태 발행
        self._publish_mission_status()

    def _log_autonomous(self, message: str, level: str = 'info'):
        """자율 모드 로그 추가 및 발행"""
        log_entry = {
            'timestamp': time.time(),
            'level': level,
            'message': message
        }
        self.autonomous_log.append(log_entry)
        if len(self.autonomous_log) > self.max_log_entries:
            self.autonomous_log.pop(0)

        # 로그 발행
        msg = String()
        msg.data = json.dumps(log_entry)
        self.autonomous_log_pub.publish(msg)

    def _publish_mission_status(self):
        """미션 상태 발행"""
        current_mission = self.mission_sequence[self.current_mission_index] if self.current_mission_index < len(self.mission_sequence) else 'completed'
        status = {
            'autonomous_mode': self.autonomous_mode,
            'mission_status': self.mission_status,
            'current_mission': current_mission,
            'mission_index': self.current_mission_index,
            'total_missions': len(self.mission_sequence),
            'recent_logs': self.autonomous_log[-5:]
        }
        msg = String()
        msg.data = json.dumps(status)
        self.mission_status_pub.publish(msg)

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
