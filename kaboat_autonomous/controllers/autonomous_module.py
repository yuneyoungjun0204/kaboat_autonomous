"""
자율주행 경로계획 모듈
SeaNU_KABOAT2024 AutonomousModule.py 포팅 (ROS2)
- Cost 함수 기반 장애물 회피
- 웨이포인트 추종
"""
import numpy as np
from math import ceil, floor, exp
from dataclasses import dataclass
from typing import List, Tuple, Optional
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 설정 import
try:
    from config import settings as SETTINGS
except ImportError:
    import sys
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS


@dataclass
class Boat:
    """보트 상태"""
    position: List[float] = None  # [x, y] UTM 좌표
    psi: float = 0.0              # 헤딩 (도)
    scan: List[float] = None      # LiDAR 스캔 데이터 (360도)

    def __post_init__(self):
        if self.position is None:
            self.position = [0.0, 0.0]
        if self.scan is None:
            self.scan = [0.0] * 360


def normalize_angle(angle: float) -> float:
    """각도를 -180 ~ 180 범위로 정규화"""
    return (angle + 180) % 360 - 180


def smooth_lidar_exponential(scan: List[float], window: int = 3, decay: float = 0.5) -> List[float]:
    """
    LiDAR 데이터 지수 평균 스무딩.

    각 인덱스 i에 대해 주변 인덱스들(i-window ~ i+window)의 가중 평균 계산.
    가까운 인덱스일수록 가중치 높음 (지수 감쇠).

    Args:
        scan: 원본 LiDAR 스캔 데이터 (360개)
        window: 좌우 참조 범위 (기본 3 = i-3 ~ i+3)
        decay: 감쇠율 (0.5 = 한 칸 멀어질 때마다 가중치 절반)

    Returns:
        스무딩된 LiDAR 데이터

    효과:
        - 단일 노이즈 포인트 완화
        - 좁은 틈새를 장애물로 인식 (안전성 향상)
        - 주변 장애물이 현재 방향에 영향을 줌
    """
    n = len(scan)
    if n == 0:
        return scan

    smoothed = [0.0] * n

    for i in range(n):
        weighted_sum = 0.0
        weight_total = 0.0

        for offset in range(-window, window + 1):
            j = (i + offset) % n  # 원형 인덱스 (0-359)

            # 0인 값(무효)은 무시하되, 주변이 모두 0이면 0 유지
            if scan[j] <= 0:
                continue

            # 지수 감쇠 가중치: e^(-|offset| * decay)
            weight = exp(-abs(offset) * decay)
            weighted_sum += scan[j] * weight
            weight_total += weight

        if weight_total > 0:
            smoothed[i] = weighted_sum / weight_total
        else:
            smoothed[i] = scan[i]  # 주변 모두 무효 시 원본 유지

    return smoothed


# 실측(2026-08-25) 검증: HEADING_CMD_OFFSET_DEG=-90을 적용한 상태로 20초간
# 라이브 텔레메트리를 보니 psi_error는 0으로 "수렴"했다고 나오는데, 실제
# boat.psi - goal_heading(atan2(dy,dx))의 차이는 -90도에 고정되고 목표까지
# 거리(goal_dist)는 전혀 줄지 않았음(44.3~45m로 20초간 정체). 즉 보정이
# 반대 방향으로 90도를 더 어긋나게 만들고 있었음이 확인됨.
# 애초에 "90도 근처 고정" 증상의 진짜 원인은 cost_func_distance(0)이 "장애물
# 없음(0)"을 최대 위험으로 잘못 해석해 자기반사 섹터로 psi_error가 고정되던
# 버그였고(위 dist_for_cost 처리로 수정됨), 그 증상을 보고 만든 이 -90 보정은
# 잘못된 패치였다. 근본 원인이 이미 해소됐으므로 오프셋은 0으로 되돌린다.
HEADING_CMD_OFFSET_DEG = 0.0


def cost_func_angle(x: float) -> float:
    """각도에 대한 Cost 함수 (목표 방향에서 벗어날수록 높음)"""
    return 1 - exp(-(x / 60) ** 2)


def cost_func_distance(x: float) -> float:
    """거리에 대한 Cost 함수 (장애물에 가까울수록 높음)"""
    return exp(-(x/30) ** 2)


def calculate_safe_zone(ld: List[float]) -> List[float]:
    """
    LiDAR 데이터를 바탕으로 안전 구역 계산

    Args:
        ld: 360도 LiDAR 거리 데이터

    Returns:
        safe_zone: 각 방향별 안전 거리
    """
    safe_zone = [SETTINGS.AVOID_RANGE] * 360

    for i in range(-180, 181):
        idx = i % 360
        if 0 < ld[idx] < SETTINGS.AVOID_RANGE:
            safe_zone[idx] = 0

    temp = np.array(safe_zone)
    for i in range(-180, 180):
        idx = i % 360
        idx_next = (i + 1) % 360

        if safe_zone[idx] > safe_zone[idx_next]:
            theta = np.arctan2(SETTINGS.BOAT_WIDTH / 2, ld[idx_next]) * 180 / np.pi
            for j in range(floor(i + 1 - theta), i + 1):
                temp[j % 360] = 0

        if safe_zone[idx] < safe_zone[idx_next]:
            theta = np.arctan2(SETTINGS.BOAT_WIDTH / 2, ld[idx]) * 180 / np.pi
            for j in range(i, ceil(i + theta) + 1):
                temp[j % 360] = 0

    return temp.tolist()


def calculate_optimal_psi_d(ld: List[float], safe_ld: List[float], goal_psi: int) -> int:
    """
    Cost 함수를 적용하여 최적 조향 각도 계산

    Args:
        ld: LiDAR 데이터
        safe_ld: 안전 구역 데이터
        goal_psi: 목표 방향 각도

    Returns:
        최적 조향 각도
    """
    theta_list = [[0, 10000]]

    for i in range(-180, 180):
        idx = i % 360
        if safe_ld[idx] > 0:
            # ld[idx] == 0 은 "이 방향엔 측정범위 내 장애물 없음"을 뜻한다
            # (calculate_safe_zone과 동일 컨벤션). cost_func_distance(0)=1은
            # 이를 최대 위험으로 잘못 해석해 자기반사가 있는 좁은 각도대로
            # psi_error가 고정되는 원인이 되므로, 0은 "가장 먼 안전 거리"로
            # 취급한다.
            dist_for_cost = ld[idx] if ld[idx] > 0 else SETTINGS.LIDAR_MAX_RANGE
            cost = (SETTINGS.GAIN_PSI * cost_func_angle(i - goal_psi) +
                    SETTINGS.GAIN_DISTANCE * cost_func_distance(dist_for_cost))
            theta_list.append([i, cost])

    return sorted(theta_list, key=lambda x: x[1])[0][0]


def compute_cost_profile(ld: List[float], safe_ld: List[float], goal_psi: float):
    """
    calculate_optimal_psi_d와 동일한 루프/공식으로 후보 각도별
    angle_cost, dist_cost, total_cost를 모두 기록한다 (시각화 디버그용).

    Returns:
        (angles, angle_costs, dist_costs, total_costs) - 각 리스트는 같은 길이
    """
    angles_deg, angle_costs, dist_costs, total_costs = [], [], [], []
    for i in range(-180, 180):
        idx = i % 360
        if safe_ld[idx] > 0:
            dist_for_cost = ld[idx] if ld[idx] > 0 else SETTINGS.LIDAR_MAX_RANGE
            ac = SETTINGS.GAIN_PSI * cost_func_angle(i - goal_psi)
            dc = SETTINGS.GAIN_DISTANCE * cost_func_distance(dist_for_cost)
            angles_deg.append(i)
            angle_costs.append(ac)
            dist_costs.append(dc)
            total_costs.append(ac + dc)
    return angles_deg, angle_costs, dist_costs, total_costs


def goal_check(boat: Boat, goal_distance: float, goal_psi: float) -> bool:
    """
    목적지까지 경로에 장애물이 있는지 판단

    Args:
        boat: 보트 상태
        goal_distance: 목표까지 거리
        goal_psi: 목표 방향 각도

    Returns:
        True면 장애물 없음, False면 장애물 있음
    """
    l = goal_distance
    theta = ceil(np.degrees(np.arctan2(SETTINGS.BOAT_WIDTH / 2, l)))
    is_able = True

    for i in range(0, 90 - theta):
        angle = int(normalize_angle(int(goal_psi) - 90 + i)) % 360
        r = SETTINGS.BOAT_WIDTH / (2 * np.cos(np.radians(i)))
        if boat.scan[angle] == 0:
            continue
        if r > boat.scan[angle]:
            is_able = False

    for i in range(-theta, theta + 1):
        angle = int(normalize_angle(int(goal_psi) + i)) % 360
        if boat.scan[angle] != 0 and boat.scan[angle] < l:
            is_able = False

    for i in range(0, 90 - theta):
        angle = int(normalize_angle(int(goal_psi) + 90 - i)) % 360
        r = SETTINGS.BOAT_WIDTH / (2 * np.cos(np.radians(i)))
        if boat.scan[angle] == 0:
            continue
        if r > boat.scan[angle]:
            is_able = False

    return is_able


def goal_passed(boat: Boat, goal_x: float, goal_y: float,
                goal_threshold: float = None) -> bool:
    """
    목적지 도착 판단

    Args:
        boat: 보트 상태
        goal_x, goal_y: 목표 좌표 (UTM)
        goal_threshold: 도착 판정 거리

    Returns:
        True면 도착
    """
    if goal_threshold is None:
        goal_threshold = SETTINGS.GOAL_RANGE

    dx = boat.position[0] - goal_x
    dy = boat.position[1] - goal_y
    return (dx ** 2 + dy ** 2) < goal_threshold ** 2


def pathplan(boat: Boat, goal_x: float, goal_y: float) -> Tuple[float, float]:
    """
    경로 계획 메인 함수
    LiDAR 데이터를 바탕으로 최적의 조향각과 추진력 계산

    Args:
        boat: 보트 상태
        goal_x, goal_y: 목표 좌표 (UTM)

    Returns:
        (psi_error, tau_x): 조향 오차(도), 추진력
    """
    if goal_x is None or goal_y is None:
        return (0.0, 0.0)

    # 목표 방향 및 거리 계산
    dx = goal_x - boat.position[0]
    dy = goal_y - boat.position[1]

    # ENU 좌표계: arctan2(dy, dx) = 동쪽(X+)이 0°, 북쪽(Y+)이 90°
    # IMU yaw도 ENU이므로 동일한 좌표계 사용
    goal_heading = np.arctan2(dy, dx) * 180 / np.pi  # 목표 방향 (절대)
    goal_psi = goal_heading - boat.psi  # 상대 각도
    goal_psi = normalize_angle(goal_psi)
    goal_distance = np.sqrt(dx ** 2 + dy ** 2)

    # LiDAR 지수 평균 스무딩 (주변 장애물 영향 반영)
    # window=3: i-3 ~ i+3 범위 참조
    # decay=0.6: 한 칸 멀어질 때마다 가중치 ~55% 감소
    smoothed_scan = smooth_lidar_exponential(boat.scan, window=3, decay=0.6)

    # 디버그: LiDAR 데이터 확인
    lidar_nonzero = sum(1 for x in boat.scan if x > 0)
    front_dist = smoothed_scan[0] if len(smoothed_scan) > 0 else -1

    if len(boat.scan) == 0:
        return (0.0, 0.0)

    # 안전 구역 계산 (스무딩된 데이터 사용)
    safe_ld = calculate_safe_zone(smoothed_scan)
    psi_error = calculate_optimal_psi_d(smoothed_scan, safe_ld, int(goal_psi))

    # goal_check 결과 저장 (스무딩된 스캔으로 체크)
    # 임시로 boat.scan을 스무딩된 것으로 교체해서 체크
    original_scan = boat.scan
    boat.scan = smoothed_scan
    is_clear = goal_check(boat, goal_distance, goal_psi)
    boat.scan = original_scan  # 원본 복원

    # 디버그 출력
    print(f'[pathplan] boat.psi={boat.psi:.1f}° goal_heading={goal_heading:.1f}° goal_psi={goal_psi:.1f}°')
    print(f'[pathplan] lidar_nonzero={lidar_nonzero}/360 front={front_dist:.1f}m is_clear={is_clear}')
    print(f'[pathplan] optimal_psi={psi_error}° (before override)')

    # 추진력 계산 (VRX: 각속도 rad/s) - settings.py의 공용 기준값 사용
    # (backward/hover 등 다른 기동 모듈도 동일 기준을 공유한다)
    max_forward = SETTINGS.MAX_FORWARD_THRUST
    base_thrust = SETTINGS.BASE_CRUISE_THRUST

    if is_clear:
        # 장애물 없음 - 목표 방향으로 직진
        tau_x = base_thrust
        psi_error = goal_psi
        if abs(psi_error) < 2:
            tau_x = min(max_forward * 0.5 + goal_distance * 2.0, max_forward)
    else:
        # 장애물 회피 모드
        tx_dist_min = max_forward * 0.15
        tx_dist_max = max_forward
        dist_danger = 1.5
        dist_safe = 6

        tx_angle_min = max_forward * 0.25
        tx_angle_max = max_forward
        angle_danger = 45

        dist = smoothed_scan[0] if smoothed_scan[0] > 0 else SETTINGS.LIDAR_MAX_RANGE

        # 거리 기반 속도 계산
        if dist <= dist_danger:
            tx_dist = (tx_dist_min / dist_danger) * dist
        elif dist <= dist_safe:
            tx_dist = ((tx_dist_max - tx_dist_min) / (dist_safe - dist_danger)) * dist + tx_dist_min
        else:
            tx_dist = tx_dist_max

        # 각도 기반 속도 계산
        angle = abs(psi_error)
        if angle <= angle_danger:
            tx_angle = ((tx_angle_min - tx_angle_max) / angle_danger) * angle + tx_angle_max
        else:
            tx_angle = (tx_angle_min / (angle_danger - 180)) * (angle - 180)

        if angle > angle_danger:
            tau_x = tx_angle
        else:
            tau_x = min(tx_dist + tx_angle, max_forward)

        tau_x = min(tau_x, max_forward)

    # 임시 보정: 실측 heading 명령 기준점을 90 -> 0으로 이동
    psi_error = normalize_angle(psi_error + HEADING_CMD_OFFSET_DEG)

    return (float(psi_error), float(tau_x))


def rotate(boat: Boat, psi_d: float) -> Tuple[float, float]:
    """
    특정 방향으로 회전 (제자리)

    Args:
        boat: 보트 상태
        psi_d: 목표 헤딩 (도)

    Returns:
        (psi_error, 0): 조향 오차, 추진력 0
    """
    psi_error = normalize_angle(psi_d - boat.psi)
    psi_error = normalize_angle(psi_error + HEADING_CMD_OFFSET_DEG)
    return (float(psi_error), 0.0)
