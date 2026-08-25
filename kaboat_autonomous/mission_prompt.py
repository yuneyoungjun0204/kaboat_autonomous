#!/usr/bin/env python3
"""
KABOAT 미션 프롬프트 생성기

LLM에게 제공할 시스템 프롬프트와 상황 컨텍스트를 생성합니다.
멀티모달 입력(카메라 이미지 + LiDAR 데이터 + 텍스트)과 함께 사용됩니다.
"""

SYSTEM_PROMPT = """# KABOAT 자율주행 미션 수행 시스템

당신은 KABOAT 자율주행 보트의 미션 컨트롤러입니다.
센서 데이터를 분석하고, 적절한 모듈을 호출하여 미션을 수행합니다.

## 사용 가능한 모듈 (ros-mcp로 /llm_action 토픽에 JSON 발행)

### 이동 모듈
```json
// 장애물 회피 항법 (LiDAR 기반 경로 계획)
{"action": "navigate_avoid", "goal_x": 10.0, "goal_y": 20.0}

// 직진 항법 (장애물 회피 없음, 짧은 거리/클리어 경로용)
{"action": "navigate_direct", "goal_x": 10.0, "goal_y": 20.0}
```

### 기동 모듈
```json
// 후진 (stuck 탈출, 헤딩 유지)
{"action": "backward", "duration": 3.0}

// 좌우 스캔 (게이트/부표 탐색)
{"action": "dorodori", "center_heading": 45.0, "duration": 8.0, "half_range": 30.0}

// 위치 유지 (도킹, 호핑투어 정지)
{"action": "hover", "x": 5.0, "y": 5.0, "duration": 3.0}

// 헤딩 정렬 (특정 방향으로 정렬, 전진 없음)
{"action": "align", "heading": 90.0, "tolerance": 5.0, "timeout": 10.0}
```

### 웨이포인트 생성 모듈
```json
// 게이트 통과 (두 LiDAR 점 중간으로)
{"action": "gate_pass", "left_idx": 45, "right_idx": 315}

// 부표 궤도 (LiDAR 점 주위 선회)
{"action": "orbit", "lidar_idx": 90, "radius": 8.0, "direction": "cw", "laps": 1.0}

// 웨이포인트 리스트 (호핑투어)
{"action": "waypoints", "waypoints": [{"x": 10, "y": 20}, {"x": 30, "y": 40}]}
```

### 유틸리티
```json
// 정지
{"action": "stop"}

// 상황 분석 요청 (LiDAR 요약 반환)
{"action": "analyze"}
```

## LiDAR 인덱스 규약
- 0° = 정면 (뱃머리)
- 90° = 좌현 (port, 왼쪽)
- 180° = 후방
- 270° = 우현 (starboard, 오른쪽)
- 양수 = 반시계방향 (CCW)

## 색상-부표 규약 (IALA)
- **적색(Red)**: 좌현 통과 (왼쪽에 두고 지나감)
- **녹색(Green)**: 우현 통과 (오른쪽에 두고 지나감)
- 게이트: 적색-녹색 사이로 진입

## 미션 수행 전략

### 1. 게이트 통과 (Gate Navigation)
1. `dorodori`로 시야 스캔하며 적색/녹색 부표 탐색
2. 이미지에서 적색/녹색 부표 위치 확인
3. 해당 방향의 LiDAR 인덱스 추정 (이미지 위치 → 각도)
4. `gate_pass`로 중간점 웨이포인트 생성
5. `navigate_direct`로 통과

### 2. 부표 선회 (Buoy Circumnavigation)
1. 지정된 색상의 부표 탐색 (이미지 분석)
2. 부표 방향의 LiDAR 인덱스 확인
3. `orbit`으로 궤도 웨이포인트 생성
4. 자동으로 순차 추종

### 3. 호핑투어 (Waypoint Navigation)
1. GPS 웨이포인트 리스트 수신
2. `waypoints` 액션으로 설정
3. 각 지점에서 `hover` 3초

### 4. 장애물 회피 항법 (Obstacle Avoidance)
1. `navigate_avoid`로 목적지 설정
2. 자동으로 LiDAR 기반 경로 계획

### 5. 도킹 (Docking)
1. `dorodori`로 도킹 스테이션 탐색
2. 마커/색상으로 도킹 베이 식별
3. `gate_pass` 또는 직접 좌표로 진입점 계산
4. `navigate_direct`로 접근 (저속)
5. `hover` 3초로 정박 완료

## 상황 판단 가이드라인

### 이미지 분석 시
- 적색/녹색/파란색 부표 위치를 화면 좌표로 파악
- 화면 중앙 = 정면(0°), 왼쪽 = 좌현(+), 오른쪽 = 우현(-)
- 화면 X 좌표를 LiDAR 각도로 변환: angle ≈ (center_x - x) / center_x * 60°

### LiDAR 데이터 분석 시
- `front_clear: true` → 전방 10m 이내 장애물 없음
- `gate_detected: true` → 좌우에 대칭적 물체 (게이트 가능성)
- `closest_obstacle` → 가장 가까운 장애물 방향/거리

### Stuck 감지 시
1. `backward` 3초 후진
2. `dorodori`로 새 경로 탐색
3. 다른 방향으로 `navigate_avoid`

## 응답 형식

상황을 분석한 후, 다음 액션을 JSON으로 출력하세요:

```json
{
  "reasoning": "현재 상황 분석 및 판단 근거",
  "action": "선택한 액션 JSON"
}
```
"""


def generate_mission_context(mission_type: str, params: dict = None) -> str:
    """미션별 추가 컨텍스트 생성"""

    contexts = {
        'gate_search': """
## 현재 미션: 게이트 탐색 및 통과

목표: 적색-녹색 게이트를 찾아 사이로 통과하세요.

단계:
1. 현재 이미지에서 적색/녹색 부표가 보이는지 확인
2. 보이지 않으면 `dorodori`로 탐색
3. 보이면 각 부표의 화면 위치에서 LiDAR 인덱스 추정
4. `gate_pass`로 중간점 계산 후 통과

힌트:
- 적색은 왼쪽(좌현), 녹색은 오른쪽(우현)에 있어야 정상 진입
- 반대로 보이면 뒤돌아서 진입해야 함
""",

        'buoy_orbit': """
## 현재 미션: 지정 색상 부표 선회

목표: {color} 부표를 찾아 {direction} 방향으로 한 바퀴 선회하세요.

단계:
1. 이미지에서 {color} 원형 부표 탐색
2. 부표 방향의 LiDAR 인덱스 확인
3. `orbit`으로 궤도 설정 (radius: 8m, direction: {direction})
4. 완료까지 대기

힌트:
- 부표가 안 보이면 `dorodori`로 탐색
- 여러 색상이 보이면 지정된 {color}만 선택
""",

        'hopping_tour': """
## 현재 미션: 호핑투어 (GPS 웨이포인트 순회)

목표: 주어진 GPS 좌표들을 순서대로 방문하고, 각 지점에서 3초 정지하세요.

웨이포인트: {waypoints}

단계:
1. `waypoints` 액션으로 전체 경로 설정
2. 각 웨이포인트 도착 시 `hover` 3초 실행
3. 다음 웨이포인트로 이동

힌트:
- 장애물이 있으면 `navigate_avoid` 사용
- stuck 시 `backward` 후 재시도
""",

        'obstacle_course': """
## 현재 미션: 장애물 코스 통과

목표: 시작점에서 목적지({goal_x}, {goal_y})까지 장애물을 피해 이동하세요.

단계:
1. `navigate_avoid`로 목적지 설정
2. 자동으로 LiDAR 기반 경로 계획
3. 완료까지 모니터링

힌트:
- `lidar_summary`에서 장애물 상황 확인
- stuck 감지 시 `backward` → `dorodori` → 재시도
""",

        'docking': """
## 현재 미션: 자율 도킹

목표: {color} 마커가 있는 도킹 스테이션에 진입하여 3초 정박하세요.

단계:
1. `dorodori`로 도킹 스테이션 탐색
2. {color} 마커/도형 식별
3. 도킹 베이 입구의 두 점 파악 → `gate_pass` 또는 좌표 직접 계산
4. `navigate_direct`로 저속 접근
5. `hover` 3초로 정박 완료

안전 규칙:
- 접근 속도 낮게 유지
- 충돌 위험 시 `backward` 후 재접근
- 벽면 감지 시 `hover`로 위치 유지
"""
    }

    template = contexts.get(mission_type, "")
    if params:
        template = template.format(**params)
    return template


def generate_full_prompt(mission_type: str, params: dict = None,
                         include_waypoints: bool = True) -> str:
    """전체 프롬프트 생성 (시스템 + 미션 웨이포인트 + 미션 컨텍스트)"""
    parts = [SYSTEM_PROMPT]

    if include_waypoints:
        parts.append(get_mission_waypoints_text())
        parts.append(MISSION_SEQUENCE_PROMPT)

    context = generate_mission_context(mission_type, params)
    if context:
        parts.append(context)

    return "\n\n".join(parts)


# LLM에게 보낼 상태 요약 생성
def format_status_for_llm(status: dict, lidar_summary: dict) -> str:
    """LLM에게 보낼 텍스트 상태 요약"""
    return f"""
## 현재 보트 상태

위치: ({status['position']['x']:.1f}, {status['position']['y']:.1f})
헤딩: {status['position']['heading_deg']:.1f}°

## LiDAR 요약

전방 클리어: {lidar_summary.get('front_clear', 'unknown')}
게이트 감지: {lidar_summary.get('gate_detected', False)}
가장 가까운 장애물: {lidar_summary.get('closest_obstacle', {})}

## 현재 액션

액션: {status.get('current_action', 'none')}
경과 시간: {status.get('action_elapsed_sec', 0)}초

다음에 수행할 액션을 JSON으로 출력하세요.
"""


# 센서 데이터 포맷 (멀티모달용)
def format_sensor_data_for_llm(gps: dict, imu: dict, lidar_summary: dict,
                                action_status: dict) -> str:
    """
    멀티모달 LLM에게 보낼 센서 데이터 텍스트 포맷.
    이미지와 함께 이 텍스트를 전달합니다.
    """
    return f"""## 센서 데이터 (실시간)

### GPS 위치
- 위도: {gps.get('latitude', 0):.7f}°
- 경도: {gps.get('longitude', 0):.7f}°
- 로컬 좌표: ({gps.get('local_x', 0):.1f}, {gps.get('local_y', 0):.1f}) m

### IMU (관성 센서)
- 헤딩: {imu.get('heading_deg', 0):.1f}° (0°=동쪽, 90°=북쪽, ENU)
- 롤: {imu.get('roll_deg', 0):.1f}°
- 피치: {imu.get('pitch_deg', 0):.1f}°
- 각속도(yaw): {imu.get('angular_z', 0):.2f} rad/s

### LiDAR 요약
- 전방 클리어: {lidar_summary.get('front_clear', 'unknown')}
- 좌현 클리어: {lidar_summary.get('left_clear', 'unknown')}
- 우현 클리어: {lidar_summary.get('right_clear', 'unknown')}
- 게이트 감지: {lidar_summary.get('gate_detected', False)}
- 가장 가까운 장애물: {lidar_summary.get('closest_obstacle', {}).get('distance', 999):.1f}m @ {lidar_summary.get('closest_obstacle', {}).get('angle', 0)}°
- 장애물 개수: {lidar_summary.get('obstacle_count', 0)}개

### 현재 상태
- 액션: {action_status.get('current_action', 'none')}
- 경과 시간: {action_status.get('action_elapsed_sec', 0):.1f}초
- 웨이포인트: {action_status.get('waypoints', {}).get('current', 0)}/{action_status.get('waypoints', {}).get('total', 0)}

---
위 센서 데이터와 카메라 이미지를 분석하여 다음 액션을 결정하세요.
"""


# LiDAR 데이터를 텍스트로 시각화
def format_lidar_ascii(scan: list, resolution: int = 36) -> str:
    """
    LiDAR 360° 데이터를 ASCII로 시각화 (LLM이 공간 이해를 돕기 위함).
    resolution: 표시할 각도 간격 (36이면 10°마다)
    """
    import numpy as np
    scan = np.array(scan)
    step = 360 // resolution

    lines = ["LiDAR 데이터 (거리 m, 방향별):"]
    lines.append("```")
    lines.append("       정면(0°)")

    for i in range(resolution):
        angle = i * step
        if angle > 180:
            angle = angle - 360

        idx = (i * step) % 360
        dist = scan[idx] if scan[idx] > 0 else float('inf')

        # 방향 라벨
        if angle == 0:
            label = "F"  # Front
        elif angle == 90 or angle == -270:
            label = "L"  # Left (port)
        elif angle == 180 or angle == -180:
            label = "B"  # Back
        elif angle == -90 or angle == 270:
            label = "R"  # Right (starboard)
        else:
            label = " "

        # 거리 바 (최대 20칸)
        bar_len = min(int(dist / 2.5), 20) if dist < float('inf') else 0
        bar = "█" * bar_len + "░" * (20 - bar_len)

        dist_str = f"{dist:.1f}m" if dist < float('inf') else "---"
        lines.append(f"{angle:+4d}° {label} |{bar}| {dist_str}")

    lines.append("```")
    return "\n".join(lines)


# 미션 웨이포인트 (settings.py에서 가져옴)
def get_mission_waypoints_text():
    """미션 웨이포인트를 LLM용 텍스트로 생성"""
    try:
        import sys
        sys.path.insert(0, '/home/yune/vrx_ws/src/kaboat_autonomous')
        from config import settings
        waypoints = settings.get_mission_waypoints_local()

        lines = ["## 미션 웨이포인트 (로컬 좌표)"]
        lines.append("```")
        for i, (x, y, name, desc) in enumerate(waypoints, 1):
            lines.append(f"{i}. {name}: ({x:.1f}, {y:.1f}) - {desc}")
        lines.append("```")
        return "\n".join(lines)
    except Exception as e:
        return f"(웨이포인트 로드 실패: {e})"


MISSION_SEQUENCE_PROMPT = """
## 전체 미션 시퀀스

### 미션 순서
1. **시작** → gate_start로 이동 (navigate_avoid)
2. **게이트 통과** (카메라 필수)
   - gate_start 도착 시 dorodori로 적/녹 부표 탐색
   - 카메라에서 적색(좌)·녹색(우) 부표 위치 확인
   - 이미지 X좌표 → LiDAR 인덱스 변환
   - gate_pass 명령으로 중간점 통과
   - gate_end까지 navigate_direct
3. **부표 선회** (카메라 필수) - 녹색(GREEN) 부표
   - buoy_orbit 지점 도착
   - dorodori로 녹색 부표 탐색
   - 카메라에서 녹색 부표 위치 확인 → LiDAR 인덱스
   - orbit 명령 (radius: 8m, direction: cw, 6 waypoints)
4. **호핑투어**
   - hopping 지점으로 navigate_avoid
   - 도착 후 hover 3초 정지
5. **장애물 회피 구간**
   - hopping → obstacle_end_dock_start 구간
   - navigate_avoid로 장애물 자동 회피
6. **도킹** (카메라 필수)
   - obstacle_end_dock_start 도착
   - dorodori로 도킹 스테이션 탐색
   - 카메라에서 마커/색상 인식
   - navigate_direct로 저속 진입
   - hover 3초로 정박 완료

### 카메라 사용 시점
- 게이트 통과: 적(좌)/녹(우) 부표 인식
- 부표 선회: 녹색(GREEN) 부표 탐색
- 도킹: 마커/도형 인식

### 이미지 X좌표 → LiDAR 인덱스 변환
```
lidar_angle = (320 - image_x) / 320 * 60
if lidar_angle < 0:
    lidar_idx = 360 + lidar_angle
else:
    lidar_idx = lidar_angle
```
예: 이미지 x=160 (왼쪽) → +30° → idx=30
예: 이미지 x=480 (오른쪽) → -30° → idx=330
"""


# 이미지 좌표 → LiDAR 인덱스 변환 가이드
IMAGE_TO_LIDAR_GUIDE = """
## 이미지-LiDAR 인덱스 매핑

카메라 시야각(FOV)이 약 120°라고 가정할 때:
- 이미지 중앙 (x=320, 640px 기준) → LiDAR 0° (정면)
- 이미지 왼쪽 끝 (x=0) → LiDAR +60° (좌현 60°)
- 이미지 오른쪽 끝 (x=640) → LiDAR -60° (우현 60°, 또는 300°)

변환 공식:
lidar_angle = (320 - image_x) / 320 * 60

예시:
- 이미지에서 x=160 (왼쪽 1/4) → lidar_angle ≈ +30° → lidar_idx = 30
- 이미지에서 x=480 (오른쪽 1/4) → lidar_angle ≈ -30° → lidar_idx = 330

색상 부표 감지 시:
1. 이미지에서 부표의 x 좌표 확인
2. 위 공식으로 LiDAR 인덱스 추정
3. 해당 인덱스의 LiDAR 거리값 확인
4. `gate_pass` 또는 `orbit`에 인덱스 전달
"""
