# KABOAT 자율주행 시스템 빠른 시작 가이드

## 개요

KABOAT 자율주행 시스템은 VRX 시뮬레이터에서 WAM-V 보트를 자율 운항합니다.

## 구성요소

```
┌─────────────────────────────────────────────────────────────┐
│                  Integrated Visualizer                       │
│  ┌─────────────────────┐  ┌─────────────────────┐           │
│  │    Global Map       │  │     Polar Map       │           │
│  │  - 보트 위치/궤적    │  │  - LiDAR 데이터     │           │
│  │  - 클릭→목적지 설정  │  │  - 안전 구역        │           │
│  │  - LiDAR (글로벌)   │  │  - 명령 방향        │           │
│  └─────────────────────┘  └─────────────────────┘           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Mission Runner                            │
│  - GPS/IMU/LiDAR 수신 → 경로 계획 → 명령 발행               │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Motor Controller                           │
│  - PD 제어 → 차동 스러스터 제어                              │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                 VRX Simulator (Gazebo)                       │
└─────────────────────────────────────────────────────────────┘
```

## 실행 방법

### 1. 시뮬레이션 시작 (터미널 1)

```bash
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 launch kaboat_pkg kaboat_sim.launch.py
```

### 2. 자율주행 시스템 시작 (터미널 2)

```bash
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 launch kaboat_autonomous autonomous.launch.py
```

### 3. 통합 시각화 시작 (터미널 3)

```bash
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 run kaboat_autonomous integrated_visualizer
```

## 시각화 사용법

### Global Map (왼쪽)
- **보트 위치**: 빨간 점
- **이동 궤적**: 파란 선
- **헤딩 방향**: 초록 선
- **LiDAR**: 파란 점들
- **클릭**: 해당 위치로 목적지 설정

### Polar Map (오른쪽)
- **LiDAR 데이터**: 파란 점 (극좌표)
- **안전 구역**: 반투명 파란 영역
- **전방**: 초록 선 (0°)
- **명령 방향**: 빨간 선 (psi_error)
- **스러스터 상태**: 하단 텍스트 (L/R)

## 토픽 구조

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/wamv/sensors/gps/fix` | NavSatFix | GPS 위치 |
| `/wamv/sensors/imu/data` | Imu | IMU 데이터 |
| `/wamv/sensors/lidar/scan` | LaserScan | LiDAR 스캔 |
| `/command` | Float32MultiArray | [psi_error, tau_x, max_sat] |
| `/wamv/thrusters/left/thrust` | Float64 | 왼쪽 스러스터 |
| `/wamv/thrusters/right/thrust` | Float64 | 오른쪽 스러스터 |
| `/waypoint_goal` | PointStamped | 시각화 클릭 웨이포인트 |

## 파라미터 조정

`config/settings.py`에서 조정 가능:

```python
# PD 제어
KP = 2.0        # 비례 계수
KD = 0.5        # 미분 계수
MAX_THRUST = 20.0  # 최대 각속도 (rad/s)

# 장애물 회피
BOAT_WIDTH = 2.5   # 보트 폭 (m)
AVOID_RANGE = 5.0  # 회피 거리 (m)
GOAL_RANGE = 3.0   # 도착 판정 거리 (m)
```
