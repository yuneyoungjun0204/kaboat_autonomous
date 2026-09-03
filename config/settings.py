"""
KABOAT 자율주행 파라미터 설정
SeaNU_KABOAT2024 기반, VRX 시뮬레이터용으로 수정
"""
import json
import math
import os


def latlon_to_utm(lat: float, lon: float) -> tuple:
    """
    WGS84 위도/경도를 UTM 좌표로 변환 (간소화된 구현)
    """
    a = 6378137.0  # WGS84 장축
    f = 1 / 298.257223563
    k0 = 0.9996

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    zone = int((lon + 180) / 6) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)

    e = math.sqrt(2 * f - f ** 2)
    e2 = e ** 2 / (1 - e ** 2)

    n = a / math.sqrt(1 - e ** 2 * math.sin(lat_rad) ** 2)
    t = math.tan(lat_rad) ** 2
    c = e2 * math.cos(lat_rad) ** 2
    A = (lon_rad - lon0) * math.cos(lat_rad)

    M = a * ((1 - e ** 2 / 4 - 3 * e ** 4 / 64) * lat_rad
             - (3 * e ** 2 / 8 + 3 * e ** 4 / 32) * math.sin(2 * lat_rad)
             + (15 * e ** 4 / 256) * math.sin(4 * lat_rad))

    x = k0 * n * (A + (1 - t + c) * A ** 3 / 6 +
                  (5 - 18 * t + t ** 2 + 72 * c - 58 * e2) * A ** 5 / 120) + 500000

    y = k0 * (M + n * math.tan(lat_rad) * (
        A ** 2 / 2 + (5 - t + 9 * c + 4 * c ** 2) * A ** 4 / 24 +
        (61 - 58 * t + t ** 2 + 600 * c - 330 * e2) * A ** 6 / 720))

    if lat < 0:
        y += 10000000

    return (x, y, zone)


# 대회장 기준 GPS 좌표 (VRX Sydney Regatta)
REF_GPS_LAT = -33.7227
REF_GPS_LON = 150.6740
REF_UTM_X, REF_UTM_Y, _ = latlon_to_utm(REF_GPS_LAT, REF_GPS_LON)

# 시뮬레이터 모드
IS_SIMULATOR = True

# 자율주행 파라미터
BOAT_WIDTH = 2.5       # WAM-V 폭 (m)
AVOID_RANGE = 30.0      # 장애물 회피 거리 (m)
GAIN_PSI = 1.0         # 목적지 각도 가중치
GAIN_DISTANCE = 10.0    # 거리 가중치
GOAL_RANGE = 3.0       # 웨이포인트 도착 판정 거리 (m)

# PD 제어 파라미터
# VRX 스러스터는 velocity_control=true (각속도 rad/s 입력)
# 1000N 추력 ≈ 12 rad/s, 2000N ≈ 17 rad/s
# max_thrust_cmd ≈ 2354 rad/s (VRX 설정) - 아래 MAX_THRUST를 2배로 올리면서
# 이 하드웨어 한계를 넘어서게 됐다. 넘는 값은 시뮬레이터에서 자체적으로
# 클리핑되니 안전하지만, 그 이상은 체감 속도가 늘지 않는다는 점 참고.
KP = 100.0              # 비례 계수 (각속도 제어용)
KD = 12.0              # 미분 계수
MAX_THRUST = 2400.0    # 최대 각속도 (rad/s) - VRX 스러스터 최대 (2026-08-26: 2배 증속)

# 회전 감속 구간 - align/dorodori 등 제자리 회전 시, 목표 헤딩에 가까워질수록
# 최대 회전 출력을 낮춰 관성으로 인한 오버슈트를 방지한다.
# 기존에는 KP*error가 max_sat 밑으로 내려가는 지점(약 max_sat/KP ≈ 12°
# 부근)에서만 감속이 시작돼 구간이 너무 좁았음 (2026-08-26 실측: align이
# 목표를 넘겨 반대편까지 틀어버리는 오버슈트 확인). TURN_DECEL_ZONE_DEG부터
# 선형으로 줄여 제동 구간을 넓힌다.
TURN_DECEL_ZONE_DEG = 45.0   # 이 각도(도) 이내부터 회전 출력 선형 감쇠 시작
TURN_MIN_CAP_RATIO = 0.15    # 감쇠 최저치 (max_sat 대비 비율) - 잔여 오차를 계속 좁힐 수 있게 완전히 0으로는 두지 않음

# align 정착(settle) 판정 - tolerance 이내로 순간적으로 스쳐 지나가는 것만으로
# "정렬 완료"로 끝내버리면, 아직 회전 관성이 남아있는 상태에서 제어가 뚝
# 끊겨 목표를 넘어 계속 돌아가버리는 문제가 있었다 (2026-08-26 실측: 116°
# 목표가 141.8°까지 밀려난 뒤에야 정지 - 위치만 보고 끝냈더니 빠르게 회전
# 중에도 tolerance 구간을 스쳐 지나가며 조건을 잠깐 만족시켰음). 그래서
# 위치(tolerance)뿐 아니라 IMU 실측 요(yaw) 각속도(ALIGN_SETTLE_MAX_YAW_RATE_DEG
# 미만)까지 같이 봐야 "진짜로 멈췄다"고 판단한다. 두 조건이 이 틱 수만큼
# 연속 유지돼야("정착") 비로소 정렬 완료로 판정한다 (control_loop가 10Hz이므로
# 5틱 ≈ 0.5초).
ALIGN_SETTLE_TICKS = 5
ALIGN_SETTLE_MAX_YAW_RATE_DEG = 5.0   # 이 각속도(도/초) 미만이어야 "회전 멈춤"으로 인정

# ============================================================
# 추력 설정 (장애물 회피 속도 기준)
# ============================================================
# 속도 티어 (직접 지정) - 2026-08-26: 전부 2배 증속
TURBO_THRUST = 3000.0      # 터보: 최고속 직진
FAST_THRUST = 2000.0       # 빠름: 클리어 직진
NORMAL_THRUST = 1000.0     # 보통: 장애물 회피 = 기준
SLOW_THRUST = 600.0        # 느림: 도킹 접근
CRAWL_THRUST = 400.0       # 초저속: 정밀 접근

# pathplan()의 기준 추력
MAX_FORWARD_THRUST = FAST_THRUST         # 최대 전진 (1000)
BASE_CRUISE_THRUST = NORMAL_THRUST       # 기본 순항 (500) ← 장애물 회피 기준

# 기동(Maneuver) 모듈 - NORMAL_THRUST 기준
BACKWARD_THRUST = NORMAL_THRUST * 0.6    # 후진: 300 (안전하면서 빠르게)
HOVER_MAX_THRUST = SLOW_THRUST           # 호버링 복귀: 300 (위치 벗어나면 복귀, 도착하면 0)
ORBIT_THRUST = NORMAL_THRUST             # 궤도 선회: 500 (장애물 회피와 동일)
WAYPOINT_THRUST = NORMAL_THRUST          # 웨이포인트: 500 (장애물 회피와 동일)

# 정지 출력 (stop, hover 도착 시)
STOP_THRUST = 0.0                        # 정지: 0 (모터 출력 없음)

# 기동 파라미터
HOVER_DEADBAND = 0.5             # 호버링 위치 허용 오차 (m)
DORODORI_HALF_RANGE_DEG = 30.0   # 도리도리 좌우 스윕 범위 (도)
DORODORI_PERIOD_SEC = 6.0        # 도리도리 왕복 주기 (초) - 빠르게
ORBIT_DEFAULT_RADIUS = 8.0       # 궤도 반경 (m)
ORBIT_N_POINTS = 6               # 궤도 웨이포인트 (6개 = 60도 간격)

# 부표 설정
TARGET_BUOY_COLOR = 'green'      # 선회 대상 부표 (green/red/blue)

# 센서 설정
LIDAR_MAX_RANGE = 50.0  # LiDAR 최대 감지 거리 (m)
LIDAR_ANGLES = 360      # LiDAR 각도 분해능
MIN_VALID_RANGE = 1.0   # 이 미만은 LiDAR 마운트 자기반사로 간주해 무시 (m)
LIDAR_SMOOTHING_ENABLED = True   # LiDAR 지수 평균 스무딩
LIDAR_SMOOTHING_DECAY = 0.95     # 스무딩 감쇠율 (높을수록 넓게 평균)

# 클러스터 탐지 (부표 무리/도킹 스테이션 후보 - align 우선 탐색용)
CLUSTER_MAX_RANGE = 25.0   # 클러스터 탐색 최대 거리 (m) - 이 밖은 무시
CLUSTER_GAP_DEG = 6        # 같은 클러스터로 묶을 최대 각도 간격 (도)
CLUSTER_MIN_POINTS = 3     # 노이즈 제외 최소 포인트 수
CLUSTER_MAX_COUNT = 3      # 반환할 최대 클러스터 수 (가까운 순)

# 클러스터 히스테리시스 (ClusterTracker - RViz 평가용, 아직 LLM 파이프라인 미연결)
CLUSTER_MATCH_ANGLE_TOL = 12.0  # 프레임간 같은 클러스터로 볼 각도 오차 허용치 (도)
CLUSTER_MIN_HITS = 2            # 이 프레임 수 이상 연속 관측돼야 "안정" 클러스터로 인정
CLUSTER_MAX_MISSES = 2          # 이 프레임 수 연속 미관측이면 트랙 폐기

# 3D 포인트클라우드 클러스터링 (/wamv/sensors/lidar/points 기준, 평가용)
# 2D LaserScan 클러스터링(CLUSTER_*)과 별개 - x,y 평면에 격자(voxel)를 깔고
# 인접 셀을 연결(connected components)해 물체 후보를 찾는다. z를 함께 걸러
# 수면 반사/자기구조물을 배제할 수 있는 게 2D 대비 핵심 차이점.
CLUSTER3D_MAX_RANGE = 25.0   # 클러스터 탐색 최대 수평 거리 (m)
CLUSTER3D_VOXEL_SIZE = 1.0   # 격자 셀 크기 (m) - 이 셀 크기 이내로 인접하면 같은 물체 후보
CLUSTER3D_MIN_POINTS = 5     # 노이즈 제외 최소 포인트 수 (3D는 점이 훨씬 많아 2D보다 높게)
CLUSTER3D_MAX_COUNT = 3      # 반환할 최대 클러스터 수 (가까운 순)
CLUSTER3D_Z_MIN = -1.1       # 센서 기준 이 아래(z, m)는 수면/자기반사로 간주해 제외
CLUSTER3D_Z_MAX = 5.0        # 센서 기준 이 위는 마스트/구조물 오탐 방지로 제외
# Z_MIN 실측 근거 (2026-08-26, 정박 상태 시뮬레이터): 25m 이내 유효 포인트의
# 99%가 z=-1.5±0.05~0.24m에 몰려 있음(ring 0~5, 수면 반사) - 95th percentile이
# -1.19라 그 위로 여유를 두고 -1.1로 설정. 파도/틸트 있는 실주행에서는 이 값도
# 흔들릴 수 있으니 RViz로 재확인 후 조정할 것 (CLUSTER3D_Z_MAX는 미실측 잠정값).

CLUSTER3D_STALE_SEC = 1.0    # PointCloud2가 이 시간(초) 이상 안 오면 2D LaserScan
                              # 클러스터링(detect_lidar_clusters)으로 폴백 (cluster_visualizer.py)

# 클러스터 거부 기록 (reject_cluster 액션 - 카메라로 "타깃 아님" 확인된 곳을
# 전역 좌표로 기억해 재탐색 방지. LLM 대화가 짧게 끊겨도 action_dispatcher
# 프로세스가 미션 내내 살아있는 동안은 유지됨)
REJECT_CLUSTER_RADIUS_M = 5.0    # 이 거리(m) 이내면 같은 지점으로 간주
REJECT_CLUSTER_MAX_COUNT = 20    # 최대 보관 개수 (초과 시 오래된 것부터 폐기)

# 카메라 시야각 (wamv_camera.xacro horizontal_fov=1.3962634 rad ≈ 80.01° 기준,
# /wamv/sensors/camera/image_raw 전방 카메라 1대 - autonomous.launch.py 참고)
# align 도중 타깃이 실제로 프레임에 들어왔을 때만 카메라를 확인해 헛촬영을
# 피하기 위한 기준값. cluster.center_angle은 보트 기준 상대각이라 카메라
# 중심(보트 정면)과 직접 비교 가능하다.
CAMERA_HALF_FOV_DEG = 40.0       # 카메라 좌우 반시야각 (도)
CAMERA_CHECK_MARGIN_DEG = 10.0   # 프레임 가장자리 여유 (렌즈 왜곡/부분 프레임 회피)

# ROS2 토픽 이름 (VRX)
TOPICS = {
    'gps': '/wamv/sensors/gps/fix',
    'imu': '/wamv/sensors/imu/data',
    'lidar': '/wamv/sensors/lidar/scan',
    'thrust_left': '/wamv/thrusters/left/thrust',
    'thrust_right': '/wamv/thrusters/right/thrust',
}

# ============================================================
# 미션 웨이포인트 (GPS 위경도)
# ============================================================
MISSION_WAYPOINTS_GPS = {
    'start': {
        'lat': -33.72276217109793,
        'lon': 150.67402781112585,
        'desc': '시작점',
        'requires_llm': False,
    },
    # 2026-09-02 실측: 시뮬레이터에서 각 미션 구간 지점에 직접 도달해
    # GPS/IMU(쿼터니언→yaw, ENU 기준 0°=동쪽/90°=북쪽)를 읽어 갱신.
    'gate_start': {
        'lat': -33.72269262,
        'lon': 150.67397336,
        'heading': 85.6,
        'desc': '게이트 통과 시작점',
        'requires_llm': True,   # 카메라로 적/녹 부표 인식 후 gate_pass 필요
    },
    'gate_end': {
        'lat': -33.72189802,
        'lon': 150.67397480,
        'heading': 85.5,
        'desc': '게이트 통과 끝점',
        'requires_llm': False,  # gate_pass 완료 후 자동 도달
    },
    'buoy_orbit': {
        'lat': -33.72155249,
        'lon': 150.67425373,
        'heading': -80.0,
        'desc': '부표선회 시작 지점',
        'requires_llm': True,   # 카메라로 지정 색상 부표 탐색 필요
    },
    'hopping': {
        'lat': -33.72170388,
        'lon': 150.67452439,
        'heading': -87.0,
        'desc': '호핑투어',
        'requires_llm': False,  # 카메라 불필요 (MISSION_NEEDS_CAMERA['hopping_tour']=False)
    },
    'obstacle_end_dock_start': {
        'lat': -33.72259378,
        'lon': 150.67455630,
        'heading': -86.8,
        'desc': '장애물 회피 끝 / 도킹 시작 (호핑투어 끝에서 여기까지 장애물 회피)',
        'requires_llm': True,   # 도킹은 카메라로 도킹 스테이션 인식 필요
    },
}

# 미션 순서 (순차 실행)
MISSION_SEQUENCE = [
    'start',
    'gate_start',
    'gate_end',
    'buoy_orbit',
    'hopping',
    'obstacle_end_dock_start',
]


# 대시보드의 '웨이포인트 설정 모드'(QGroundControl 스타일 클릭 편집)가 읽고 쓰는
# 파일. 존재하면 아래 MISSION_WAYPOINTS_GPS/MISSION_SEQUENCE 대신 이 파일을
# 우선 사용한다 - 하드코딩된 값은 파일이 없거나 깨졌을 때의 폴백으로만 남는다.
MISSION_WAYPOINTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mission_waypoints.json')


def _load_mission_waypoints_from_file():
    """MISSION_WAYPOINTS_FILE을 읽어 (sequence, waypoints_dict)를 반환.
    파일이 없거나 형식이 깨졌으면 None을 반환해 호출부가 하드코딩된
    MISSION_WAYPOINTS_GPS/MISSION_SEQUENCE로 폴백하게 한다."""
    try:
        with open(MISSION_WAYPOINTS_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        sequence = data['sequence']
        waypoints = data['waypoints']
        for name in sequence:
            wp = waypoints[name]
            if wp.get('lat') is None or wp.get('lon') is None:
                return None  # 아직 위경도가 안 찍힌 단계가 있으면 폴백
        return sequence, waypoints
    except (OSError, KeyError, ValueError, json.JSONDecodeError):
        return None


def get_mission_waypoints_local():
    """
    미션 웨이포인트를 로컬 좌표(ENU)로 변환하여 반환
    Returns: [(x, y, heading, name, desc, requires_llm), ...]
    heading은 없으면 None (ENU 기준, 0°=동쪽, 90°=북쪽)
    """
    loaded = _load_mission_waypoints_from_file()
    if loaded is not None:
        sequence, waypoints_gps = loaded
    else:
        sequence, waypoints_gps = MISSION_SEQUENCE, MISSION_WAYPOINTS_GPS

    waypoints = []
    for name in sequence:
        wp = waypoints_gps[name]
        utm_x, utm_y, _ = latlon_to_utm(wp['lat'], wp['lon'])
        local_x = utm_x - REF_UTM_X
        local_y = utm_y - REF_UTM_Y
        waypoints.append((
            local_x, local_y, wp.get('heading_deg', wp.get('heading')),
            name, wp.get('desc', name), wp.get('requires_llm', False)
        ))
    return waypoints


def get_hopping_stops_local():
    """mission_waypoints.json의 hopping_stops(호핑투어 다중 지점)를
    로컬 좌표로 변환해 반환. Returns: [(x, y, heading_deg|None), ...]"""
    try:
        with open(MISSION_WAYPOINTS_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        stops = data.get('hopping_stops', [])
    except (OSError, ValueError, json.JSONDecodeError):
        stops = []

    result = []
    for s in stops:
        lat, lon = s.get('lat'), s.get('lon')
        if lat is None or lon is None:
            continue  # 아직 좌표가 안 찍힌 항목은 건너뜀 (_load_mission_waypoints_from_file과 동일한 방어)
        utm_x, utm_y, _ = latlon_to_utm(lat, lon)
        result.append((utm_x - REF_UTM_X, utm_y - REF_UTM_Y, s.get('heading_deg')))
    return result


def print_mission_waypoints():
    """미션 웨이포인트 출력 (디버그용)"""
    print("=== 미션 웨이포인트 (로컬 좌표) ===")
    for x, y, heading, name, desc, requires_llm in get_mission_waypoints_local():
        tag = "[LLM/비전]" if requires_llm else "[자동]"
        hdg = f"{heading:.1f}°" if heading is not None else "-"
        print(f"  {tag} {name}: ({x:.1f}, {y:.1f}) hdg={hdg} - {desc}")
