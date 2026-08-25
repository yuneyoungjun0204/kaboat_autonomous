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
AVOID_RANGE = 25.0      # 장애물 회피 거리 (m)
GAIN_PSI = 1.0         # 목적지 각도 가중치
GAIN_DISTANCE = 1.5    # 거리 가중치
GOAL_RANGE = 3.0       # 웨이포인트 도착 판정 거리 (m)

# PD 제어 파라미터
# VRX 스러스터는 velocity_control=true (각속도 rad/s 입력)
# 1000N 추력 ≈ 12 rad/s, 2000N ≈ 17 rad/s
# max_thrust_cmd ≈ 2354 rad/s (VRX 설정)
KP = 100.0              # 비례 계수 (각속도 제어용)
KD = 12.0              # 미분 계수
MAX_THRUST = 500.0     # 최대 각속도 (rad/s) - 기본 전진 500rpm

# pathplan()의 실측 기준 추력 (장애물 회피가 검증된 비율) - 이 두 값이
# 모든 추력 기반 모듈(장애물회피, 후진, 호버링 등)의 공용 기준점이다.
MAX_FORWARD_THRUST = MAX_THRUST * 0.8   # 조향 여유 확보한 최대 전진 추력
BASE_CRUISE_THRUST = MAX_FORWARD_THRUST * 0.85  # 클리어 경로 기본 순항 추력

# 기동(Maneuver) 모듈 기본값 - controllers/maneuvers.py
# 추력은 위 MAX_FORWARD_THRUST 기준 비율로 정의해 장애물회피와 동일한
# 보트 동역학 튜닝을 그대로 물려받는다.
BACKWARD_THRUST = MAX_FORWARD_THRUST * 0.5   # 후진 기본 추력
HOVER_MAX_THRUST = MAX_FORWARD_THRUST * 0.3  # 호버링 복귀 최대 추력 (저속 보정)
HOVER_DEADBAND = 0.5             # 호버링 위치 허용 오차 (m)
DORODORI_HALF_RANGE_DEG = 30.0   # 도리도리 기본 좌우 스윕 범위 (도)
DORODORI_PERIOD_SEC = 8.0        # 도리도리 기본 왕복 주기 (초)
ORBIT_DEFAULT_RADIUS = 8.0       # 궤도(로이터링) 기본 반경 (m)
ORBIT_N_POINTS = 16              # 궤도 웨이포인트 근사 점 개수

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
