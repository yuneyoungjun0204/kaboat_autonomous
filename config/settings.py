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
AVOID_RANGE = 5.0      # 장애물 회피 거리 (m)
GAIN_PSI = 1.0         # 목적지 각도 가중치
GAIN_DISTANCE = 0.5    # 거리 가중치
GOAL_RANGE = 3.0       # 웨이포인트 도착 판정 거리 (m)

# PD 제어 파라미터
KP = 50.0              # 비례 계수 (VRX 스러스터용)
KD = 20.0              # 미분 계수
MAX_THRUST = 250.0     # 최대 스러스터 출력 (N)

# 센서 설정
LIDAR_MAX_RANGE = 50.0  # LiDAR 최대 감지 거리 (m)
LIDAR_ANGLES = 360      # LiDAR 각도 분해능

# ROS2 토픽 이름 (VRX)
TOPICS = {
    'gps': '/wamv/sensors/gps/fix',
    'imu': '/wamv/sensors/imu/data',
    'lidar': '/wamv/sensors/lidar/scan',
    'thrust_left': '/wamv/thrusters/left/thrust',
    'thrust_right': '/wamv/thrusters/right/thrust',
}
