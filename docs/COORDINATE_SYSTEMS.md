# KABOAT 좌표계 가이드

VRX 시뮬레이터 및 KABOAT 자율주행 시스템에서 사용하는 좌표계 정의입니다.

## 1. 좌표계 표준

### ROS REP 103 (공식 표준)

| 항목 | 정의 |
|------|------|
| **Body Frame** | X=forward, Y=left, Z=up |
| **ENU (Geographic)** | X=East, Y=North, Z=Up |
| **Yaw 기준** | **0° = East**, CCW(반시계) 양수 |
| **단위** | 미터(m), 라디안(rad) |

> 참조: [REP 103](https://reps.openrobotics.org/rep-0103/)

### VRX Frame Convention

| 항목 | 정의 |
|------|------|
| **World Frame** | Cartesian XYZ, WGS84 |
| **Local Tangent Plane** | **ENU** (기본값) |
| **Model Frame** | X=forward, Y=port(좌현), Z=up |

> 참조: [VRX Wiki](https://github.com/osrf/vrx/wiki/frame_conventions)

---

## 2. KABOAT 코드 좌표계

### 2.1 위치 좌표 (GPS → Local)

```
GPS (WGS84) → UTM → Local ENU
```

- **X축**: East (동쪽, 미터)
- **Y축**: North (북쪽, 미터)
- **원점**: `REF_GPS_LAT`, `REF_GPS_LON` (settings.py)

### 2.2 각도/헤딩

| 방향 | 각도 |
|------|------|
| **East (동)** | 0° |
| **North (북)** | 90° |
| **West (서)** | 180° / -180° |
| **South (남)** | -90° |

- **양의 방향**: 반시계방향 (CCW)
- **IMU yaw**: ENU 기준 (0° = East)

---

## 3. 좌표 변환 공식

### 3.1 극좌표 → 직교좌표 (ENU)

```python
# ENU 좌표계: cos→X(East), sin→Y(North)
x = distance * cos(angle)
y = distance * sin(angle)
```

### 3.2 직교좌표 → 각도 (ENU)

```python
# ENU 좌표계: arctan2(y, x) = arctan2(North, East)
angle = arctan2(dy, dx)  # 0° = East, 90° = North
```

### 3.3 WGS84 → UTM

```python
from config.settings import latlon_to_utm
utm_x, utm_y, zone = latlon_to_utm(latitude, longitude)
```

### 3.4 GPS → Local ENU

```python
from config.settings import REF_UTM_X, REF_UTM_Y, latlon_to_utm

utm_x, utm_y, _ = latlon_to_utm(lat, lon)
local_x = utm_x - REF_UTM_X  # East
local_y = utm_y - REF_UTM_Y  # North
```

---

## 4. 시각화 좌표계

### 4.1 Global Map (Cartesian)

- **X축**: East (오른쪽)
- **Y축**: North (위쪽)
- **헤딩 0°**: East 방향

```python
# 헤딩 방향 시각화 (ENU)
line_x = pos[0] + length * cos(heading_rad)
line_y = pos[1] + length * sin(heading_rad)
```

### 4.2 Local Map (Polar)

- **0°**: North (위쪽) - `set_theta_zero_location('N')`
- **방향**: 시계방향(CW) 양수 - `set_theta_direction(-1)`
- **용도**: LiDAR 데이터, 명령 각도 표시

**ENU → Polar 변환**: ENU는 CCW 양수, Polar는 CW 양수이므로 부호 반전 필요
```python
# ENU 각도를 polar plot 각도로 변환
polar_angle = -enu_angle
```

---

## 5. 주요 파일

| 파일 | 역할 |
|------|------|
| `config/settings.py` | UTM 변환, 기준점 정의 |
| `controllers/autonomous_module.py` | 경로 계획 (ENU) |
| `controllers/gps_navigator.py` | GPS 네비게이션 |
| `visualize_global.py` | 글로벌 맵 시각화 |
| `visualize_local.py` | 로컬 맵 시각화 |
| `integrated_visualizer.py` | 통합 시각화 |

---

## 6. ENU vs NED 비교

| 항목 | ENU (KABOAT) | NED (항해/항공) |
|------|--------------|-----------------|
| X | East | North |
| Y | North | East |
| Z | Up | Down |
| Yaw 0° | East | North |
| Yaw 양의방향 | CCW | CW |

**주의**: 일부 해양/항공 시스템은 NED를 사용합니다. 외부 시스템 연동 시 변환이 필요할 수 있습니다.

### NED ↔ ENU 변환

```python
# ENU to NED
ned_x = enu_y   # North
ned_y = enu_x   # East
ned_z = -enu_z  # Down
ned_yaw = 90 - enu_yaw  # 시계방향 변환

# NED to ENU
enu_x = ned_y   # East
enu_y = ned_x   # North
enu_z = -ned_z  # Up
enu_yaw = 90 - ned_yaw
```

---

## 7. 검증 체크리스트

- [ ] 보트가 동쪽을 향할 때 헤딩 ≈ 0°
- [ ] 보트가 북쪽을 향할 때 헤딩 ≈ 90°
- [ ] 웨이포인트 방향이 실제 목표와 일치
- [ ] LiDAR 포인트가 실제 장애물 위치와 일치
- [ ] 시각화 X축이 East, Y축이 North
