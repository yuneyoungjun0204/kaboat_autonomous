"""
자율 기동(Maneuver) 모듈 - LLM 도구용
autonomous_module.py와 동일한 스타일: ROS 의존성 없는 순수 함수.
직접 python으로 Boat 상태를 구성해서 각 함수를 개별 테스트할 수 있다.
추력 등 튜닝 가능한 기본값은 모두 config/settings.py를 공용 기준으로 삼는다
(장애물회피 pathplan()과 동일한 MAX_FORWARD_THRUST 기반 비율).

mission_runner.py에 /maneuver_cmd(JSON) 토픽으로 배선되어 있다:
start_backward()/start_dorodori()/start_hover()/start_orbit()/start_midpoint().

구현된 모듈 (난이도 순):
1. backward()            - 헤딩 유지 후진
2. dorodori()            - 지정 각도 중심 ±범위 좌우 스윕
3. midpoint_waypoint()   - LiDAR 두 점의 중점에 웨이포인트
4. plan_orbit()          - LiDAR 한 점 주위를 반경 x(m)로 궤도(로이터링)
5. hover()               - 위치 유지(호버링), 목표가 후방이면 후진으로 보정
6. navigate_avoid()      - 장애물 회피 항법 (pathplan 래핑)
7. navigate_direct()     - 직진 항법 (장애물 회피 없음)

LLM 사용 예시:
  - "장애물이 많다" → navigate_avoid(boat, goal_x, goal_y)
  - "경로 깨끗하다" → navigate_direct(boat, goal_x, goal_y)
  - "stuck됐다"    → backward(boat)
  - "게이트 통과"  → midpoint_waypoint(boat, idx1, idx2) → navigate_direct()
  - "부표 선회"    → plan_orbit(boat, idx) → 웨이포인트 리스트
"""
import numpy as np
from typing import List, Optional, Tuple
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

try:
    from .autonomous_module import Boat, normalize_angle
except ImportError:
    from autonomous_module import Boat, normalize_angle


# ============================================================
# 공용 유틸: LiDAR 극좌표 -> 전역(UTM 상대) 좌표 변환
# ============================================================

def lidar_point_to_global(boat: Boat, angle_deg: float, distance: float) -> Tuple[float, float]:
    """
    LiDAR 각도(도, 보트 기준 ENU CCW+, boat.scan 인덱스와 동일 컨벤션)와
    거리를 전역(보트 position과 같은 상대 UTM) 좌표로 변환.
    """
    angle_rad = np.radians(angle_deg) + np.radians(boat.psi)
    x = boat.position[0] + distance * np.cos(angle_rad)
    y = boat.position[1] + distance * np.sin(angle_rad)
    return (float(x), float(y))


def scan_point_to_global(boat: Boat, idx: int) -> Optional[Tuple[float, float]]:
    """boat.scan[idx]가 유효(>0)하면 전역 좌표를, 아니면 None을 반환."""
    dist = boat.scan[idx % 360]
    if dist <= 0:
        return None
    return lidar_point_to_global(boat, float(idx % 360), dist)


# ============================================================
# 1. 후진 (Backward) - 가장 단순: 헤딩을 유지하며 역추진
# ============================================================

def backward(boat: Boat, thrust: float = None,
             hold_heading: Optional[float] = None) -> Tuple[float, float]:
    """
    후진 명령 생성.

    Args:
        boat: 보트 상태
        thrust: 역추진 크기 (양수, 기본 SETTINGS.BACKWARD_THRUST)
        hold_heading: 유지할 헤딩(도). None이면 회전 보정 없이 그대로 후진
                      (호출자가 매 tick마다 최초 헤딩을 넘겨주는 방식을 권장)

    Returns:
        (psi_error, tau_x): tau_x는 음수(역추진)
    """
    if thrust is None:
        thrust = SETTINGS.BACKWARD_THRUST
    thrust = abs(thrust)

    if hold_heading is None:
        psi_error = 0.0
    else:
        psi_error = normalize_angle(hold_heading - boat.psi)

    return (float(psi_error), -float(thrust))


# ============================================================
# 2. 도리도리 (Heading Sweep) - 지정 각도 중심 ±half_range를 사인파로 스윕
# ============================================================

def dorodori_target_heading(center_heading: float, half_range_deg: float,
                             t: float, period_sec: float = None) -> float:
    """
    center_heading을 기준으로 ±half_range_deg 사이를 사인파로 왕복하는
    목표 헤딩을 시간 t(초)에 대해 계산.
    """
    if period_sec is None:
        period_sec = SETTINGS.DORODORI_PERIOD_SEC
    phase = 2 * np.pi * (t % period_sec) / period_sec
    offset = half_range_deg * np.sin(phase)
    return normalize_angle(center_heading + offset)


def dorodori(boat: Boat, center_heading: float, half_range_deg: float = None,
             t: float = 0.0, period_sec: float = None) -> Tuple[float, float]:
    """
    제자리에서 center_heading ± half_range_deg 범위를 천천히 좌우로 훑는다.
    추력은 0 (전진 없이 회전만).

    Args:
        boat: 보트 상태
        center_heading: 스윕 중심 각도(도)
        half_range_deg: 중심 기준 좌우 스윕 범위(도) - 즉 -half~+half
                        (기본 SETTINGS.DORODORI_HALF_RANGE_DEG)
        t: 기동 시작 후 경과 시간(초) - 호출자가 관리
        period_sec: 한 왕복(중심->한쪽 끝->반대쪽 끝->중심)에 걸리는 시간
                    (기본 SETTINGS.DORODORI_PERIOD_SEC)

    Returns:
        (psi_error, tau_x=0)
    """
    if half_range_deg is None:
        half_range_deg = SETTINGS.DORODORI_HALF_RANGE_DEG
    target = dorodori_target_heading(center_heading, half_range_deg, t, period_sec)
    psi_error = normalize_angle(target - boat.psi)
    return (float(psi_error), 0.0)


# ============================================================
# 3. LiDAR 두 점의 중점에 웨이포인트
# ============================================================

def midpoint_waypoint(boat: Boat, angle1_deg: float, dist1: float,
                       angle2_deg: float, dist2: float) -> Tuple[float, float]:
    """LiDAR 두 점(각도/거리)의 중점을 전역 좌표 웨이포인트로 계산."""
    x1, y1 = lidar_point_to_global(boat, angle1_deg, dist1)
    x2, y2 = lidar_point_to_global(boat, angle2_deg, dist2)
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def midpoint_waypoint_from_scan(boat: Boat, idx1: int, idx2: int) -> Optional[Tuple[float, float]]:
    """boat.scan 인덱스 두 개를 직접 지정해 중점 웨이포인트를 계산."""
    p1 = scan_point_to_global(boat, idx1)
    p2 = scan_point_to_global(boat, idx2)
    if p1 is None or p2 is None:
        return None
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)


# ============================================================
# 4. 궤도/로이터링 (Orbit) - LiDAR 한 점 주위를 반경 radius(m)로 회전
# ============================================================

def generate_orbit_waypoints(center: Tuple[float, float], radius: float,
                              entry_bearing_deg: float, direction: str = 'cw',
                              n_points: int = None, laps: float = 1.0) -> List[Tuple[float, float]]:
    """
    center 주위를 radius(m) 반경으로 도는 웨이포인트 리스트 생성.
    entry_bearing_deg에서 시작해 direction 방향으로 원을 근사하는
    n_points개의 점을 laps 바퀴만큼 생성한다.

    Args:
        center: 궤도 중심 (x, y)
        radius: 궤도 반경 (m)
        entry_bearing_deg: 궤도 진입 각도(도, center 기준 ENU CCW+)
        direction: 'cw'(시계) 또는 'ccw'(반시계)
        n_points: 한 바퀴를 근사할 웨이포인트 개수 (기본 SETTINGS.ORBIT_N_POINTS)
        laps: 돌 바퀴 수 (1.0 = 한 바퀴)

    Returns:
        [(x, y), ...] 웨이포인트 리스트 (mission_runner.set_waypoints()에 바로 사용 가능)
    """
    if direction not in ('cw', 'ccw'):
        raise ValueError("direction must be 'cw' or 'ccw'")
    if n_points is None:
        n_points = SETTINGS.ORBIT_N_POINTS

    sign = -1.0 if direction == 'cw' else 1.0  # ENU는 CCW+이므로 시계방향은 각도 감소
    step_deg = sign * (360.0 / n_points)
    total_steps = max(int(round(n_points * laps)), 1)

    waypoints = []
    for i in range(1, total_steps + 1):
        angle_rad = np.radians(entry_bearing_deg + step_deg * i)
        wx = center[0] + radius * np.cos(angle_rad)
        wy = center[1] + radius * np.sin(angle_rad)
        waypoints.append((float(wx), float(wy)))
    return waypoints


def plan_orbit(boat: Boat, idx: int, radius: float = None, direction: str = 'cw',
               n_points: int = None, laps: float = 1.0) -> Optional[List[Tuple[float, float]]]:
    """
    boat.scan[idx]에 잡힌 LiDAR 점을 중심으로, 보트의 현재 위치에서 가장
    가까운 원 위의 지점을 진입점 삼아 궤도 웨이포인트를 생성한다.

    radius가 None이면 SETTINGS.ORBIT_DEFAULT_RADIUS 사용.

    Returns:
        웨이포인트 리스트, 해당 인덱스에 유효한 LiDAR 반사가 없으면 None
    """
    if radius is None:
        radius = SETTINGS.ORBIT_DEFAULT_RADIUS

    center = scan_point_to_global(boat, idx)
    if center is None:
        return None

    dx = center[0] - boat.position[0]
    dy = center[1] - boat.position[1]
    bearing_to_center = np.degrees(np.arctan2(dy, dx))
    # 원 위에서 보트와 가장 가까운 지점 = 중심에서 보트 반대 방향(180도 반대편)
    entry_bearing = normalize_angle(bearing_to_center + 180)

    return generate_orbit_waypoints(center, radius, entry_bearing, direction, n_points, laps)


# ============================================================
# 5. 호버링 (Hovering / Station Keeping) - 가장 복잡: 위치 유지 + 후방이면 후진
# ============================================================

def hover(boat: Boat, hold_x: float, hold_y: float,
          hold_heading: Optional[float] = None,
          deadband: float = None, max_thrust: float = None) -> Tuple[float, float]:
    """
    지정된 지점(hold_x, hold_y)에서 위치를 유지한다 (station keeping).

    - deadband(m) 이내로 드리프트하면 추력 없이 헤딩(지정 시)만 유지
    - deadband를 벗어나면 목표 지점 방향으로 천천히 복귀
    - 목표가 후방(|psi_error|>90도)이면 제자리에서 180도 도는 대신
      backward()와 동일한 방식으로 짧게 후진해 보정한다

    Args:
        boat: 보트 상태
        hold_x, hold_y: 유지할 위치 (전역 좌표)
        hold_heading: 유지할 헤딩(도), None이면 헤딩은 신경쓰지 않음
        deadband: 이 거리(m) 이내 드리프트는 무시 (기본 SETTINGS.HOVER_DEADBAND)
        max_thrust: 복귀에 사용할 최대 추력 (기본 SETTINGS.HOVER_MAX_THRUST)

    Returns:
        (psi_error, tau_x)
    """
    if deadband is None:
        deadband = SETTINGS.HOVER_DEADBAND
    if max_thrust is None:
        max_thrust = SETTINGS.HOVER_MAX_THRUST

    dx = hold_x - boat.position[0]
    dy = hold_y - boat.position[1]
    dist = float(np.sqrt(dx ** 2 + dy ** 2))

    if dist < deadband:
        psi_error = normalize_angle(hold_heading - boat.psi) if hold_heading is not None else 0.0
        return (float(psi_error), 0.0)

    bearing = np.degrees(np.arctan2(dy, dx))
    psi_error = normalize_angle(bearing - boat.psi)
    # deadband 밖으로 벗어난 정도에 비례해 서서히 복귀 (deadband*4에서 최대 추력)
    tau_x = min(max_thrust, max_thrust * (dist - deadband) / (deadband * 4))

    if abs(psi_error) > 90:
        # 목표가 후방에 있으면 제자리 선회 대신 후진으로 보정
        psi_error_rev = normalize_angle(psi_error - 180) if psi_error > 0 else normalize_angle(psi_error + 180)
        return (float(psi_error_rev), -float(tau_x))

    return (float(psi_error), float(tau_x))


# ============================================================
# 6. 장애물 회피 항법 (Navigate with Avoidance)
# ============================================================

def navigate_avoid(boat: Boat, goal_x: float, goal_y: float) -> Tuple[float, float]:
    """
    장애물 회피를 적용한 웨이포인트 항법.
    autonomous_module.pathplan()을 래핑.

    LLM 사용 시나리오:
    - "장애물이 많다" / "LiDAR에 물체가 보인다" → 이 함수 사용
    - stuck 상태에서 벗어난 후 정상 항법 재개

    Args:
        boat: 보트 상태 (position, psi, scan 필요)
        goal_x, goal_y: 목표 좌표 (전역 좌표)

    Returns:
        (psi_error, tau_x): 조향 오차(도), 추진력
    """
    try:
        from .autonomous_module import pathplan
    except ImportError:
        from autonomous_module import pathplan

    return pathplan(boat, goal_x, goal_y)


def navigate_direct(boat: Boat, goal_x: float, goal_y: float,
                    thrust: float = None) -> Tuple[float, float]:
    """
    장애물 회피 없이 목표 방향으로 직진.

    LLM 사용 시나리오:
    - "경로가 깨끗하다" / "장애물 없다"
    - 게이트 중간점으로 짧은 거리 이동
    - 정밀 접근 (회피 알고리즘이 오히려 방해될 때)

    Args:
        boat: 보트 상태
        goal_x, goal_y: 목표 좌표
        thrust: 추진력 (기본 BASE_CRUISE_THRUST)

    Returns:
        (psi_error, tau_x)
    """
    if thrust is None:
        thrust = SETTINGS.BASE_CRUISE_THRUST

    dx = goal_x - boat.position[0]
    dy = goal_y - boat.position[1]
    dist = np.sqrt(dx ** 2 + dy ** 2)

    # 목표 방향 계산 (ENU: arctan2(dy, dx))
    goal_heading = np.degrees(np.arctan2(dy, dx))
    psi_error = normalize_angle(goal_heading - boat.psi)

    # 거리에 따른 추력 조절 (가까워지면 감속)
    if dist < SETTINGS.GOAL_RANGE * 2:
        thrust = thrust * (dist / (SETTINGS.GOAL_RANGE * 2))

    # 큰 각도 오차 시 회전 우선 (추력 감소)
    if abs(psi_error) > 30:
        thrust = thrust * 0.3

    return (float(psi_error), float(thrust))


# ============================================================
# 7. 유틸리티: 목표 도달 확인
# ============================================================

def is_goal_reached(boat: Boat, goal_x: float, goal_y: float,
                    threshold: float = None) -> bool:
    """목표 지점 도착 여부 확인."""
    if threshold is None:
        threshold = SETTINGS.GOAL_RANGE

    dx = goal_x - boat.position[0]
    dy = goal_y - boat.position[1]
    return (dx ** 2 + dy ** 2) < threshold ** 2


def get_goal_info(boat: Boat, goal_x: float, goal_y: float) -> dict:
    """
    목표까지의 거리/방향 정보 반환 (LLM 상황 판단용).

    Returns:
        {
            'distance': 거리(m),
            'bearing': 절대 방향(도),
            'relative_bearing': 상대 방향(도, 양수=좌현),
            'is_reached': 도착 여부
        }
    """
    dx = goal_x - boat.position[0]
    dy = goal_y - boat.position[1]
    dist = float(np.sqrt(dx ** 2 + dy ** 2))
    bearing = float(np.degrees(np.arctan2(dy, dx)))
    relative = float(normalize_angle(bearing - boat.psi))

    return {
        'distance': round(dist, 1),
        'bearing': round(bearing, 1),
        'relative_bearing': round(relative, 1),
        'is_reached': dist < SETTINGS.GOAL_RANGE
    }


# ============================================================
# 8. LiDAR 분석 유틸리티 (LLM 상황 판단용)
# ============================================================

def analyze_lidar(boat: Boat) -> dict:
    """
    LiDAR 데이터 요약 (LLM이 상황 판단에 사용).

    Returns:
        {
            'front_clear': 전방 10m 이내 장애물 없음,
            'left_clear': 좌현 클리어,
            'right_clear': 우현 클리어,
            'closest_obstacle': {'angle': 도, 'distance': m},
            'gate_detected': 게이트 패턴 감지 여부,
            'obstacle_count': 장애물 개수
        }
    """
    scan = np.array(boat.scan)
    valid = (scan > 0) & (scan < SETTINGS.LIDAR_MAX_RANGE)

    # 섹터별 분석
    def sector_clear(start, end, threshold=10.0):
        indices = np.arange(start, end) % 360
        sector = scan[indices]
        sector_valid = sector[(sector > 0) & (sector < SETTINGS.LIDAR_MAX_RANGE)]
        return len(sector_valid) == 0 or np.min(sector_valid) > threshold

    # 가장 가까운 장애물
    closest_angle = -1
    closest_dist = 999.0
    if np.any(valid):
        valid_indices = np.where(valid)[0]
        valid_dists = scan[valid_indices]
        min_idx = np.argmin(valid_dists)
        closest_angle = int(valid_indices[min_idx])
        closest_dist = float(valid_dists[min_idx])

    # 게이트 패턴 감지 (양쪽에 대칭적 장애물)
    gate_detected = False
    left_obs = scan[20:70]
    right_obs = scan[290:340]
    left_valid = left_obs[(left_obs > 0) & (left_obs < 20)]
    right_valid = right_obs[(right_obs > 0) & (right_obs < 20)]
    if len(left_valid) > 0 and len(right_valid) > 0:
        if abs(np.min(left_valid) - np.min(right_valid)) < 5:
            gate_detected = True

    return {
        'front_clear': bool(sector_clear(350, 370, 10.0) and sector_clear(0, 10, 10.0)),
        'left_clear': bool(sector_clear(60, 120, 8.0)),
        'right_clear': bool(sector_clear(240, 300, 8.0)),
        'closest_obstacle': {
            'angle': int(closest_angle),
            'distance': round(float(closest_dist), 1)
        },
        'gate_detected': bool(gate_detected),
        'obstacle_count': int(np.sum(valid))
    }
