# KABOAT LLM 미션 수행 가이드

## 시스템 구성

```
┌─────────────────────────────────────────────────────────────────────┐
│                    LLM (Claude via ros-mcp)                         │
│                                                                     │
│  입력 (멀티모달):                                                    │
│  ┌──────────────┬──────────────┬─────────────────┐                  │
│  │ 카메라 이미지 │ /sensor_fusion │ 미션 프롬프트   │                  │
│  │ (색상/마커)  │ (GPS/IMU/LiDAR)│ (mission_prompt)│                  │
│  └──────────────┴──────────────┴─────────────────┘                  │
│                                                                     │
│  출력: /llm_action (JSON)                                           │
└─────────────────────────┬───────────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────────┐
│  action_dispatcher → maneuvers.py → /command → motor_controller    │
└─────────────────────────────────────────────────────────────────────┘
```

## 토픽 요약

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/sensor_fusion` | String (JSON) | GPS+IMU+LiDAR 통합 데이터 (5Hz) |
| `/wamv/sensors/camera/image_raw` | Image | 전방 카메라 이미지 |
| `/llm_action` | String (JSON) | LLM 명령 입력 |
| `/action_status` | String (JSON) | 현재 액션 상태 |

## ros-mcp 사용법

### 1. 센서 데이터 구독

```python
# 통합 센서 데이터 (GPS, IMU, LiDAR 요약)
sensor_data = mcp__ros-mcp__subscribe_once(
    topic='/sensor_fusion',
    msg_type='std_msgs/msg/String'
)

# 카메라 이미지 (자동 저장됨)
image = mcp__ros-mcp__subscribe_once(
    topic='/wamv/sensors/camera/image_raw',
    msg_type='sensor_msgs/msg/Image',
    expects_image='true'
)

# 저장된 이미지 다시 보기
mcp__ros-mcp__view_saved_image(image_path='./camera/received_image.jpeg')
```

### 2. 액션 명령

```python
# 게이트 통과
mcp__ros-mcp__publish_once(
    topic='/llm_action',
    msg_type='std_msgs/msg/String',
    msg={'data': '{"action": "gate_pass", "left_idx": 45, "right_idx": 315}'}
)

# 부표 궤도
mcp__ros-mcp__publish_once(
    topic='/llm_action',
    msg_type='std_msgs/msg/String',
    msg={'data': '{"action": "orbit", "lidar_idx": 30, "radius": 8.0, "direction": "cw"}'}
)

# 도리도리 (탐색)
mcp__ros-mcp__publish_once(
    topic='/llm_action',
    msg_type='std_msgs/msg/String',
    msg={'data': '{"action": "dorodori", "center_heading": 0, "duration": 8.0}'}
)
```

## 액션 레퍼런스

### 항법 액션

| 액션 | 설명 | 파라미터 |
|------|------|----------|
| `navigate_avoid` | 장애물 회피 항법 | `goal_x`, `goal_y` |
| `navigate_direct` | 직진 항법 | `goal_x`, `goal_y` |
| `waypoints` | 웨이포인트 순차 방문 | `waypoints: [{x, y}, ...]` |

### 기동 액션

| 액션 | 설명 | 파라미터 |
|------|------|----------|
| `backward` | 후진 | `duration` (초) |
| `dorodori` | 좌우 스캔 | `center_heading`, `duration`, `half_range` |
| `hover` | 위치 유지 | `x`, `y`, `duration` |

### 웨이포인트 생성 액션

| 액션 | 설명 | 파라미터 |
|------|------|----------|
| `gate_pass` | 두 LiDAR 점 중간 | `left_idx`, `right_idx` |
| `orbit` | 부표 궤도 | `lidar_idx`, `radius`, `direction`, `laps` |

### 유틸리티

| 액션 | 설명 |
|------|------|
| `stop` | 정지 |
| `analyze` | LiDAR 분석 결과 반환 |

## 이미지 → LiDAR 인덱스 변환

카메라 FOV ≈ 120° 기준:

```
이미지 X 좌표 (640px 기준)     LiDAR 인덱스
─────────────────────────────────────────
     0 (왼쪽 끝)           →      60° (좌현)
   160 (왼쪽 1/4)          →      30°
   320 (중앙)              →       0° (정면)
   480 (오른쪽 1/4)        →     330° (-30°)
   640 (오른쪽 끝)         →     300° (우현)

공식: lidar_idx = (320 - image_x) / 320 * 60
      (음수면 360 더하기)
```

## 미션 시퀀스

### 미션 1: 게이트 탐색 및 통과

```
1. dorodori로 시야 스캔
2. 이미지에서 적색/녹색 부표 확인
3. 부표 위치에서 LiDAR 인덱스 추정
4. gate_pass 명령
5. 통과 확인
```

### 미션 2: 부표 선회

```
1. dorodori로 지정 색상 부표 탐색
2. 이미지에서 부표 방향 확인
3. orbit 명령 (해당 lidar_idx)
4. 웨이포인트 순차 추종
```

### 미션 3: 호핑투어

```
1. waypoints 액션으로 GPS 좌표 설정
2. 각 웨이포인트 도착 시 hover 3초
3. 다음 웨이포인트로 이동
```

### 미션 4: 장애물 회피 구간

```
1. navigate_avoid로 목적지 설정
2. 자동 LiDAR 기반 경로 계획
3. stuck 시: backward → dorodori → 재시도
```

### 미션 5: 도킹

```
1. dorodori로 도킹 스테이션 탐색
2. 마커 색상/도형 식별
3. gate_pass 또는 직접 좌표로 진입
4. navigate_direct로 저속 접근
5. hover 3초로 정박 완료
```

## /sensor_fusion 데이터 형식

```json
{
  "timestamp": 1234567890.12,
  "gps": {
    "latitude": -33.7227,
    "longitude": 150.6740,
    "local_x": 10.5,
    "local_y": 20.3
  },
  "imu": {
    "heading_deg": 45.2,
    "roll_deg": 1.2,
    "pitch_deg": -0.5,
    "angular_z": 0.05
  },
  "lidar": {
    "front_clear": true,
    "left_clear": true,
    "right_clear": false,
    "gate_detected": true,
    "closest_obstacle": {"angle": 315, "distance": 8.5},
    "obstacle_count": 12
  },
  "command": {
    "psi_error": 15.2,
    "tau_x": 350
  },
  "action": {
    "current": "navigate_avoid",
    "elapsed_sec": 5.2,
    "waypoint_progress": {"total": 3, "current": 1, "remaining": 2}
  }
}
```

## 실행 방법

```bash
# 터미널 1: Gazebo 시뮬레이터 (원격 머신)
ros2 launch vrx_gz competition.launch.py world:=kaboat_course

# 터미널 2: Rosbridge (원격 머신)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 터미널 3: 자율주행 시스템 (로컬 머신)
source ~/vrx_ws/install/setup.bash
ros2 launch kaboat_autonomous autonomous.launch.py

# 터미널 4: Claude Code (ros-mcp)
# LLM이 /sensor_fusion, 카메라 구독하고 /llm_action 발행
```
