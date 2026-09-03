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
# 2.5. 헤딩 정렬 (Align to Heading) - 특정 각도로 정렬
# ============================================================

def align_to_heading(boat: Boat, target_heading: float,
                      tolerance: float = 5.0) -> Tuple[float, float, bool]:
    """
    특정 헤딩으로 정렬 (전진 없이 회전만).

    LLM 사용 시나리오:
    - "북쪽을 향해" → align_to_heading(boat, 90)
    - "게이트 방향으로 정렬" → align_to_heading(boat, gate_heading)
    - 부표 접근 전 방향 정렬

    Args:
        boat: 보트 상태
        target_heading: 목표 헤딩(도, ENU: 0=동, 90=북)
        tolerance: 허용 오차(도), 이내면 정렬 완료

    Returns:
        (psi_error, tau_x, aligned):
        - psi_error: 조향 오차(도)
        - tau_x: 0 (전진 없음)
        - aligned: True면 정렬 완료
    """
    psi_error = normalize_angle(target_heading - boat.psi)
    aligned = abs(psi_error) < tolerance
    return (float(psi_error), 0.0, aligned)


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
                    thrust: float = None,
                    hold_heading: Optional[float] = None) -> Tuple[float, float]:
    """
    장애물 회피 없이 목표 방향으로 직진.

    LLM 사용 시나리오:
    - "경로가 깨끗하다" / "장애물 없다"
    - 게이트 중간점으로 짧은 거리 이동
    - 정밀 접근 (회피 알고리즘이 오히려 방해될 때)
    - "각도를 유지한 채 직진" → hold_heading 지정
      (게이트/도킹처럼 목표점 방향이 아니라 특정 헤딩을 그대로 유지하며
      전진해야 할 때. 예: 게이트 중심선을 따라 똑바로 진입)

    Args:
        boat: 보트 상태
        goal_x, goal_y: 목표 좌표 (도달 판정 및 감속 거리 계산용으로는 항상 사용됨)
        thrust: 추진력 (기본 BASE_CRUISE_THRUST)
        hold_heading: 유지할 기준 헤딩(도). None이면 기존처럼 목표 좌표 방향으로
                      조향한다. 지정하면 목표 방향 대신 이 헤딩을 그대로
                      유지하며 직진한다(호출자가 정렬 완료 후 헤딩을 넘겨주는
                      방식을 권장).

    Returns:
        (psi_error, tau_x)
    """
    if thrust is None:
        thrust = SETTINGS.BASE_CRUISE_THRUST

    dx = goal_x - boat.position[0]
    dy = goal_y - boat.position[1]
    dist = np.sqrt(dx ** 2 + dy ** 2)

    if hold_heading is None:
        # 목표 방향 계산 (ENU: arctan2(dy, dx))
        goal_heading = np.degrees(np.arctan2(dy, dx))
        psi_error = normalize_angle(goal_heading - boat.psi)
    else:
        # 목표 방향 대신 지정된 헤딩을 그대로 유지
        psi_error = normalize_angle(hold_heading - boat.psi)

    # 거리에 따른 추력 조절 (가까워지면 감속)
    if dist < SETTINGS.GOAL_RANGE * 2:
        thrust = thrust * (dist / (SETTINGS.GOAL_RANGE * 2))

    # 큰 각도 오차 시 회전 우선 (추력 감소)
    if abs(psi_error) > 30:
        thrust = thrust * 0.3

    return (float(psi_error), float(thrust))


# ============================================================
# 7-1. advance_bearing: 지정 방위(절대 헤딩, ENU)로 일정 거리 전진할
#      목표 좌표를 계산 (카메라엔 보이는데 LiDAR 클러스터가 안 잡힐 때,
#      "그 방향으로 일단 접근" 용도 - mission_prompt.py 참고)
# ============================================================

def advance_bearing_target(boat: Boat, bearing_deg: float,
                            distance: float = 20.0) -> Tuple[float, float]:
    """
    현재 위치 기준으로 절대 방위(bearing_deg, ENU: 0°=동쪽/CCW+)를 따라
    distance(m) 앞의 좌표를 계산한다. 호출 시점의 위치를 기준으로 한 번만
    계산되는 고정 목표점이어야 하므로(매 틱 재계산하면 계속 전진만 하며
    도착 판정이 안 됨), action_dispatcher가 액션 시작 시 1회만 호출해
    goal_x/goal_y로 저장하고, 이후엔 navigate_direct 실행 로직을 그대로
    재사용한다(hold_heading=bearing_deg로 방위 유지).
    """
    bearing_rad = np.radians(bearing_deg)
    goal_x = boat.position[0] + distance * np.cos(bearing_rad)
    goal_y = boat.position[1] + distance * np.sin(bearing_rad)
    return (float(goal_x), float(goal_y))


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

def detect_lidar_clusters(boat: Boat, max_range: float = None, gap_deg: int = None,
                           min_points: int = None, max_clusters: int = None) -> List[dict]:
    """
    인접한 유효 반사점들을 각도 기준으로 묶어 물체 클러스터 후보를 찾는다
    (부표가 모인 곳, 도킹 스테이션 등). 탐색 시 dorodori(광역 스윕)보다
    먼저 확인해 클러스터 방향으로 align_to_heading을 우선 시도하기 위함.

    Returns:
        가까운 순 정렬된 클러스터 리스트. 각 원소:
        {'center_angle': 보트 기준 각도(도, LiDAR idx 컨벤션),
         'distance': 최근접 거리(m), 'width_deg': 각폭, 'point_count': 포인트 수}
    """
    if max_range is None:
        max_range = SETTINGS.CLUSTER_MAX_RANGE
    if gap_deg is None:
        gap_deg = SETTINGS.CLUSTER_GAP_DEG
    if min_points is None:
        min_points = SETTINGS.CLUSTER_MIN_POINTS
    if max_clusters is None:
        max_clusters = SETTINGS.CLUSTER_MAX_COUNT

    scan = np.array(boat.scan)
    n = len(scan)
    valid_idx = np.where((scan > 0) & (scan < max_range))[0]
    if len(valid_idx) == 0:
        return []

    groups = [[int(valid_idx[0])]]
    for idx in valid_idx[1:]:
        idx = int(idx)
        if idx - groups[-1][-1] <= gap_deg:
            groups[-1].append(idx)
        else:
            groups.append([idx])

    # 0°/360° 경계에서 갈라진 첫/마지막 그룹을 하나로 병합 (각도 랩어라운드)
    if len(groups) > 1:
        wrap_gap = (groups[0][0] + n) - groups[-1][-1]
        if wrap_gap <= gap_deg:
            groups[-1] = groups[-1] + [i + n for i in groups[0]]
            groups = groups[1:]

    clusters = []
    for g in groups:
        if len(g) < min_points:
            continue
        dists = scan[[i % n for i in g]]
        clusters.append({
            'center_angle': round(float(normalize_angle(sum(g) / len(g))), 1),
            'distance': round(float(np.min(dists)), 1),
            'width_deg': round(float(max(g) - min(g)), 1),
            'point_count': len(g),
        })

    clusters.sort(key=lambda c: c['distance'])
    return clusters[:max_clusters]


class ClusterTracker:
    """
    detect_lidar_clusters()는 매 프레임 독립적으로 재계산되어 클러스터가
    프레임 사이에 끊기거나(gap_deg 경계, 반사 노이즈) 개수/순서가 흔들릴 수
    있다. 이 트래커는 프레임 간 클러스터를 각도로 연결해(association) 최소
    연속 프레임(min_hits) 이상 관측된 것만 "안정" 클러스터로 인정하고,
    일시적으로 놓쳐도(max_misses 이하) 트랙을 유지한다.

    RViz에서 raw vs 안정화 결과를 비교 평가하기 위한 실험용 유틸리티 -
    analyze_lidar()의 LLM 파이프라인에는 아직 연결하지 않음 (cluster_visualizer.py 전용).
    """

    def __init__(self, match_angle_tol: float = None, min_hits: int = None,
                 max_misses: int = None):
        self.match_angle_tol = (SETTINGS.CLUSTER_MATCH_ANGLE_TOL
                                 if match_angle_tol is None else match_angle_tol)
        self.min_hits = SETTINGS.CLUSTER_MIN_HITS if min_hits is None else min_hits
        self.max_misses = SETTINGS.CLUSTER_MAX_MISSES if max_misses is None else max_misses
        self.tracks: List[dict] = []  # 각 원소: cluster 필드 + hits/misses

    def update(self, raw_clusters: List[dict]) -> List[dict]:
        """새 프레임의 raw 클러스터를 기존 트랙과 매칭/갱신하고,
        min_hits 이상 연속 관측된 안정 클러스터 리스트를 (가까운 순) 반환."""
        matched = set()
        for c in raw_clusters:
            best_i, best_diff = None, None
            for i, t in enumerate(self.tracks):
                if i in matched:
                    continue
                diff = abs(normalize_angle(c['center_angle'] - t['center_angle']))
                if diff <= self.match_angle_tol and (best_diff is None or diff < best_diff):
                    best_i, best_diff = i, diff

            if best_i is not None:
                t = self.tracks[best_i]
                t.update(c)
                t['hits'] += 1
                t['misses'] = 0
                matched.add(best_i)
            else:
                new_track = dict(c)
                new_track['hits'] = 1
                new_track['misses'] = 0
                self.tracks.append(new_track)
                matched.add(len(self.tracks) - 1)

        for i, t in enumerate(self.tracks):
            if i not in matched:
                t['misses'] += 1

        self.tracks = [t for t in self.tracks if t['misses'] <= self.max_misses]

        stable = [
            {k: v for k, v in t.items() if k not in ('hits', 'misses')}
            for t in self.tracks if t['hits'] >= self.min_hits
        ]
        stable.sort(key=lambda c: c['distance'])
        return stable


def detect_lidar_clusters_3d(points_xyz: np.ndarray, max_range: float = None,
                              voxel_size: float = None, min_points: int = None,
                              max_clusters: int = None, z_min: float = None,
                              z_max: float = None) -> List[dict]:
    """
    3D LiDAR 포인트클라우드(x,y,z, 센서 기준 로컬 좌표) 기반 클러스터링.
    detect_lidar_clusters()(2D LaserScan, 각도 간격 기반)와는 별개 - z를 함께
    걸러 수면 반사/자기구조물을 배제할 수 있고, 각도 하나로는 구분 안 되는
    물체를 x-y 격자(voxel) 인접성으로 더 정확히 묶을 수 있다.

    알고리즘: x-y 평면을 voxel_size 크기 격자로 나눠 포인트를 셀에 배정하고,
    점유된 셀들을 8-연결(인접 셀 공유)로 묶어 connected components를 구한다
    (Euclidean/voxel 클러스터링 - PCL 등에서 흔히 쓰는 방식).

    Args:
        points_xyz: (N, 3) 배열, 센서 로컬 좌표 (x=전방, y=좌현, z=위)
        max_range: 클러스터 탐색 최대 수평 거리 (기본 SETTINGS.CLUSTER3D_MAX_RANGE)
        voxel_size: 격자 셀 크기, m (기본 SETTINGS.CLUSTER3D_VOXEL_SIZE)
        min_points: 노이즈 제외 최소 포인트 수 (기본 SETTINGS.CLUSTER3D_MIN_POINTS)
        max_clusters: 반환할 최대 클러스터 수 (기본 SETTINGS.CLUSTER3D_MAX_COUNT)
        z_min, z_max: 이 범위 밖 점은 무시 (기본 SETTINGS.CLUSTER3D_Z_MIN/MAX)

    Returns:
        가까운 순 정렬된 클러스터 리스트. 각 원소:
        {'center_angle': 도(LiDAR idx 컨벤션과 동일, atan2(y,x)),
         'distance': 중심까지 수평 거리(m), 'width_deg': 각폭 근사,
         'point_count': 포인트 수, 'z': 클러스터 중심 높이(m)}
    """
    if max_range is None:
        max_range = SETTINGS.CLUSTER3D_MAX_RANGE
    if voxel_size is None:
        voxel_size = SETTINGS.CLUSTER3D_VOXEL_SIZE
    if min_points is None:
        min_points = SETTINGS.CLUSTER3D_MIN_POINTS
    if max_clusters is None:
        max_clusters = SETTINGS.CLUSTER3D_MAX_COUNT
    if z_min is None:
        z_min = SETTINGS.CLUSTER3D_Z_MIN
    if z_max is None:
        z_max = SETTINGS.CLUSTER3D_Z_MAX

    pts = np.asarray(points_xyz, dtype=float)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return []

    finite = np.all(np.isfinite(pts), axis=1)
    x, y, z = pts[finite, 0], pts[finite, 1], pts[finite, 2]
    r_xy = np.hypot(x, y)
    valid = (r_xy >= SETTINGS.MIN_VALID_RANGE) & (r_xy < max_range) & (z >= z_min) & (z <= z_max)
    x, y, z = x[valid], y[valid], z[valid]
    if len(x) == 0:
        return []

    cell_x = np.floor(x / voxel_size).astype(int)
    cell_y = np.floor(y / voxel_size).astype(int)

    # 점유 셀 -> 소속 포인트 인덱스
    cell_points: dict = {}
    for i, cell in enumerate(zip(cell_x, cell_y)):
        cell_points.setdefault(cell, []).append(i)

    # 점유 셀들을 8-연결로 묶는 connected components (BFS)
    visited = set()
    clusters = []
    for cell in cell_points:
        if cell in visited:
            continue
        stack = [cell]
        visited.add(cell)
        group_cells = []
        while stack:
            c = stack.pop()
            group_cells.append(c)
            cx, cy = c
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = (cx + dx, cy + dy)
                    if nb in cell_points and nb not in visited:
                        visited.add(nb)
                        stack.append(nb)

        idxs = [i for c in group_cells for i in cell_points[c]]
        if len(idxs) < min_points:
            continue

        gx, gy, gz = x[idxs], y[idxs], z[idxs]
        cx_mean, cy_mean, cz_mean = float(gx.mean()), float(gy.mean()), float(gz.mean())
        angles = np.degrees(np.arctan2(gy, gx))
        width_deg = float(angles.max() - angles.min()) if len(idxs) > 1 else 0.0

        clusters.append({
            'center_angle': round(float(normalize_angle(np.degrees(np.arctan2(cy_mean, cx_mean)))), 1),
            'distance': round(float(np.hypot(cx_mean, cy_mean)), 1),
            'width_deg': round(width_deg, 1),
            'point_count': len(idxs),
            'z': round(cz_mean, 2),
        })

    clusters.sort(key=lambda c: c['distance'])
    return clusters[:max_clusters]


# ============================================================
# 8.5. 클러스터 거부 기록 (rejected clusters) - 세션이 끊겨도 유지되는 압축 상태
# ============================================================
# 대화 컨텍스트를 매 판단마다 짧게 끊어도(bounded-context), "이 방향은 카메라로
# 확인했더니 타깃이 아니었다"는 사실만은 잃지 않도록 한다. 원본 대화 로그 대신
# 전역 좌표 점 하나만 기억하면 충분 (각도는 보트가 움직이면 바뀌지만 위치는 안 바뀜).
# 이 상태(rejected_points 리스트)는 호출자(action_dispatcher)가 들고 있고,
# 여기 함수들은 순수 변환/판정만 수행한다.

def reject_cluster_point(boat: Boat, angle: float, distance: float) -> Tuple[float, float]:
    """각도/거리를 전역 좌표로 변환 (reject_cluster 액션 처리용)."""
    return lidar_point_to_global(boat, angle, distance)


def annotate_cluster_rejection(boat: Boat, clusters: List[dict], rejected_points: List[dict],
                                radius: float = None) -> List[dict]:
    """
    각 클러스터에 이미 '타깃 아님'으로 확인된 지점 근처인지 'rejected' 플래그로 표시.

    Args:
        clusters: detect_lidar_clusters() 결과
        rejected_points: [{'x':.., 'y':..}, ...] 전역 좌표 (호출자가 상태로 보관)
        radius: 같은 지점으로 볼 거리(m) 허용 오차 (기본 SETTINGS.REJECT_CLUSTER_RADIUS_M)
    """
    if radius is None:
        radius = SETTINGS.REJECT_CLUSTER_RADIUS_M
    annotated = []
    for c in clusters:
        gx, gy = lidar_point_to_global(boat, c['center_angle'], c['distance'])
        is_rejected = any(
            (gx - r['x']) ** 2 + (gy - r['y']) ** 2 < radius ** 2
            for r in rejected_points
        )
        annotated.append({**c, 'rejected': is_rejected})
    return annotated


def analyze_lidar(boat: Boat, points_3d: np.ndarray = None) -> dict:
    """
    LiDAR 데이터 요약 (LLM이 상황 판단에 사용).

    Args:
        boat: 보트 상태 (2D boat.scan 기반 섹터 분석/폴백 클러스터링에 사용)
        points_3d: (N,3) 3D 포인트클라우드가 신선하면 전달 (호출자가 staleness
            판단). 주어지면 클러스터는 detect_lidar_clusters_3d()로 계산되고,
            None이면 기존 2D detect_lidar_clusters()로 폴백한다
            (cluster_visualizer.py와 동일한 3D 우선/2D 폴백 방식을
            2026-08-26부터 실제 미션 파이프라인에도 적용).

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

    # 전방 장애물 분포 (LLM이 직접 추론할 수 있게 raw 데이터 제공)
    # -120° ~ +120° 범위를 12개 섹터로 나눠 각 섹터의 최소 거리 제공
    # 양수 = 좌측, 음수 = 우측 (카메라 관점과 일치)
    front_sectors = []
    for sector_idx in range(12):
        angle_start = -120 + sector_idx * 20
        angle_end = angle_start + 20
        angle_center = angle_start + 10

        # scan 배열 인덱스: 0=정면, 90=좌측, 270=우측
        idx_start = int((360 + angle_start) % 360) if angle_start < 0 else int(angle_start)
        idx_end = int((360 + angle_end) % 360) if angle_end < 0 else int(angle_end)

        if idx_start > idx_end:
            indices = list(range(idx_end, idx_start + 1))
        else:
            indices = list(range(idx_start, idx_end + 1))

        sector_scan = scan[indices]
        sector_valid = sector_scan[(sector_scan > 0) & (sector_scan < SETTINGS.LIDAR_MAX_RANGE)]

        if len(sector_valid) > 0:
            min_dist = float(np.min(sector_valid))
            front_sectors.append({
                'sector': f"{'좌' if angle_center > 0 else '우' if angle_center < 0 else '정면'}{abs(angle_center)}°",
                'angle': angle_center,
                'distance': round(min_dist, 1)
            })
        else:
            front_sectors.append({
                'sector': f"{'좌' if angle_center > 0 else '우' if angle_center < 0 else '정면'}{abs(angle_center)}°",
                'angle': angle_center,
                'distance': None
            })

    # 클러스터 정보 (LLM이 대상 물체로 정렬 가능하게 상세 정보)
    # 3D 포인트클라우드가 신선하면 우선 사용, 아니면 2D LaserScan으로 폴백
    if points_3d is not None:
        clusters = detect_lidar_clusters_3d(points_3d)
    else:
        clusters = detect_lidar_clusters(boat)
    # 클러스터에 ID 부여 (LLM이 참조 가능)
    for i, c in enumerate(clusters):
        c['id'] = i

    return {
        'front_clear': bool(sector_clear(350, 370, 10.0) and sector_clear(0, 10, 10.0)),
        'left_clear': bool(sector_clear(60, 120, 8.0)),
        'right_clear': bool(sector_clear(240, 300, 8.0)),
        'closest_obstacle': {
            'angle': int(closest_angle),
            'distance': round(float(closest_dist), 1)
        },
        'front_distribution': front_sectors,
        'obstacle_count': int(np.sum(valid)),
        'clusters': clusters,
        'cluster_source': '3d' if points_3d is not None else '2d',
    }
