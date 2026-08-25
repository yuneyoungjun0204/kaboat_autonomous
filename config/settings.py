"""
KABOAT 자율주행 파라미터 설정
SeaNU_KABOAT2024 기반, VRX 시뮬레이터용으로 수정
"""
import math


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
# max_thrust_cmd ≈ 2354 rad/s (VRX 설정)
KP = 100.0              # 비례 계수 (각속도 제어용)
KD = 12.0              # 미분 계수
MAX_THRUST = 1200.0    # 최대 각속도 (rad/s) - VRX 스러스터 최대

# ============================================================
# 추력 설정 (장애물 회피 속도 기준)
# ============================================================
# 속도 티어 (직접 지정)
FAST_THRUST = 1000.0       # 빠름: 클리어 직진
NORMAL_THRUST = 500.0      # 보통: 장애물 회피 = 기준
SLOW_THRUST = 300.0        # 느림: 도킹 접근

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
        'desc': '시작점'
    },
    'gate_start': {
        'lat': -33.72264316313711,
        'lon': 150.67398440184970,
        'desc': '게이트 통과 시작점'
    },
    'gate_end': {
        'lat': -33.72190811726158,
        'lon': 150.67398512188350,
        'desc': '게이트 통과 끝점'
    },
    'buoy_orbit': {
        'lat': -33.72165457255059,
        'lon': 150.67401729412393,
        'desc': '부표선회 시작 지점'
    },
    'hopping': {
        'lat': -33.72175726537696,
        'lon': 150.67449697363470,
        'desc': '호핑투어'
    },
    'obstacle_end_dock_start': {
        'lat': -33.72256299430913,
        'lon': 150.67453407868342,
        'desc': '장애물 회피 끝 / 도킹 시작 (호핑투어 끝에서 여기까지 장애물 회피)'
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


def get_mission_waypoints_local():
    """
    미션 웨이포인트를 로컬 좌표(ENU)로 변환하여 반환
    Returns: [(x, y, name, desc), ...]
    """
    waypoints = []
    for name in MISSION_SEQUENCE:
        wp = MISSION_WAYPOINTS_GPS[name]
        utm_x, utm_y, _ = latlon_to_utm(wp['lat'], wp['lon'])
        local_x = utm_x - REF_UTM_X
        local_y = utm_y - REF_UTM_Y
        waypoints.append((local_x, local_y, name, wp['desc']))
    return waypoints


def print_mission_waypoints():
    """미션 웨이포인트 출력 (디버그용)"""
    print("=== 미션 웨이포인트 (로컬 좌표) ===")
    for x, y, name, desc in get_mission_waypoints_local():
        print(f"  {name}: ({x:.1f}, {y:.1f}) - {desc}")
