#!/usr/bin/env python3
"""
KABOAT 미션 프롬프트 생성기

LLM에게 제공할 시스템 프롬프트와 상황 컨텍스트를 생성합니다.
멀티모달 입력(카메라 이미지 + LiDAR 데이터 + 텍스트)과 함께 사용됩니다.
"""

# 미션별 카메라(색상/마커 인식) 필요 여부 - VISION_GUIDE 포함 여부 결정
MISSION_NEEDS_CAMERA = {
    'gate_search': True,
    'buoy_orbit': True,
    'hopping_tour': False,
    'obstacle_course': False,
    'docking': True,
}

# 항상 포함되는 핵심 프롬프트: 모듈 스키마 + LiDAR 규약 + 응답 형식만.
# 미션별 수행 전략은 generate_mission_context()가 담당하므로 여기서 중복 나열하지 않는다.
CORE_PROMPT = """# KABOAT 자율주행 미션 수행 시스템

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

// 헤딩 정렬 (특정 방향으로 정렬, 전진 없음) - 탐색 시 dorodori보다 우선 사용
{"action": "align", "heading": 90.0, "tolerance": 5.0, "timeout": 10.0}

// 좌우 스캔 (align으로 정렬할 클러스터가 없을 때의 광역 탐색 폴백)
{"action": "dorodori", "center_heading": 45.0, "duration": 8.0, "half_range": 30.0}

// 위치 유지 (도킹, 호핑투어 정지)
{"action": "hover", "x": 5.0, "y": 5.0, "duration": 3.0}
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

## 탐색 우선순위: align 먼저, dorodori는 폴백

`lidar_summary.clusters`에 물체 클러스터 후보가 이미 들어있다 (부표 무리,
도킹 스테이션 등 인접 반사점을 각도로 묶은 결과). 매 스캔마다 자동 계산되므로
별도 액션 호출 없이 바로 확인 가능:
`[{"center_angle": 42.5, "distance": 10.0, "width_deg": 5.0, "point_count": 6}, ...]`
(가까운 순 정렬, 최대 3개)

탐색이 필요할 때:
1. `clusters`가 비어있지 않으면 → 가장 가까운(첫 번째) 클러스터를 목표로
   `align` 실행. `heading = 현재 헤딩(IMU) + cluster.center_angle` (정규화, -180~180)
2. 정렬 완료 후 카메라로 실제 부표/도킹 마커인지 확인
   - 맞으면 해당 미션 절차 진행
   - 아니면 다음 순위 클러스터로 재시도, 그래도 없으면 3번
3. `clusters`가 비어있거나 후보가 모두 아니면 → `dorodori`로 광역 스캔
   (dorodori는 클러스터 없을 때만 쓰는 폴백. clusters가 있는데 바로
   dorodori부터 쓰지 말 것 - 이미 방향을 아는데 광역 스윕은 비효율적)

## LiDAR 데이터 분석 시
- `front_clear: true` → 전방 10m 이내 장애물 없음
- `gate_detected: true` → 좌우에 대칭적 물체 (게이트 가능성)
- `closest_obstacle` → 가장 가까운 장애물 방향/거리
- `clusters` → 위 "탐색 우선순위" 참고

## Stuck 감지 시
1. `backward` 3초 후진
2. `clusters` 확인 후 있으면 `align`, 없으면 `dorodori`로 새 경로 탐색
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

# 카메라(색상/마커 인식)가 필요한 미션에만 포함하는 조각.
# gate_search/buoy_orbit/docking 전용, hopping_tour/obstacle_course에는 불필요.
VISION_GUIDE = """## 색상-부표 규약 (본 코스 실측 기준, IALA와 반대)
- **녹색(Green)**: 좌현 통과 (왼쪽에 두고 지나감)
- **적색(Red)**: 우현 통과 (오른쪽에 두고 지나감)
- 게이트: 녹색-적색 사이로 진입
- (주의: 일반 IALA 규약은 적=좌현/녹=우현이지만, 이 시뮬레이션 코스는 실제 카메라 확인 결과 반대로 배치되어 있음 - 2026-08-26 gate_start 지점에서 확인)

## 이미지 분석 시
- 적색/녹색/파란색 부표 위치를 화면 좌표로 파악
- 화면 중앙 = 정면(0°), 왼쪽 = 좌현(+), 오른쪽 = 우현(-)
- 화면 X 좌표를 LiDAR 각도로 변환: `lidar_angle = (320 - image_x) / 320 * 60`
  - 예: x=160 (왼쪽 1/4) → +30° / x=480 (오른쪽 1/4) → -30° (idx=330)
"""


def generate_mission_context(mission_type: str, params: dict = None) -> str:
    """미션별 추가 컨텍스트 생성"""

    contexts = {
        'gate_search': """
## 현재 미션: 게이트 탐색 및 통과

목표: 적색-녹색 게이트를 찾아 사이로 통과하세요.

단계:
1. 현재 이미지에서 적색/녹색 부표가 보이는지 확인
2. 안 보이면 `clusters` 확인 → 있으면 가장 가까운 클러스터로 `align` (없으면 `dorodori`)
3. 보이면 각 부표의 화면 위치에서 LiDAR 인덱스 추정
4. `gate_pass`로 중간점 계산 후 통과

힌트:
- 녹색은 왼쪽(좌현), 적색은 오른쪽(우현)에 있어야 정상 진입 (이 코스는 IALA 반대 배치)
- 반대로 보이면 뒤돌아서 진입해야 함
""",

        'buoy_orbit': """
## 현재 미션: 지정 색상 부표 선회

목표: {color} 부표를 찾아 {direction} 방향으로 한 바퀴 선회하세요.

단계:
1. 이미지에서 {color} 원형 부표 탐색
2. 안 보이면 `clusters` 확인 → 있으면 가장 가까운 클러스터로 `align` (없으면 `dorodori`)
3. 부표 방향의 LiDAR 인덱스 확인
4. `orbit`으로 궤도 설정 (radius: 8m, direction: {direction})
5. 완료까지 대기

힌트:
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
- stuck 감지 시 `backward` → `align`(clusters 있으면)/`dorodori` → 재시도
""",

        'docking': """
## 현재 미션: 자율 도킹

목표: {color} 마커가 있는 도킹 스테이션에 진입하여 3초 정박하세요.

단계:
1. `clusters` 확인 → 있으면 가장 가까운 클러스터로 `align` (없으면 `dorodori`)로 도킹 스테이션 탐색
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
                         include_waypoints: bool = True,
                         include_sequence: bool = False) -> str:
    """
    미션 실행용 프롬프트 생성 (core + 카메라 필요 시 vision guide + 웨이포인트 + 미션 컨텍스트).

    include_sequence: 전체 6단계 미션 개요(MISSION_SEQUENCE_PROMPT)를 포함할지 여부.
    개별 미션 수행 중엔 불필요한 다른 미션 정보라 기본 False - 미션 감독/전체 계획
    수립처럼 전체 그림이 필요한 호출에서만 True로 켤 것 (토큰 절약).
    """
    parts = [CORE_PROMPT]

    if MISSION_NEEDS_CAMERA.get(mission_type, True):
        parts.append(VISION_GUIDE)

    if include_waypoints:
        parts.append(get_mission_waypoints_text())

    if include_sequence:
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
클러스터: {lidar_summary.get('clusters', [])}

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
- 클러스터(탐색 후보, 가까운 순): {lidar_summary.get('clusters', [])}

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


# 전체 미션 개요 (include_sequence=True일 때만 포함 - 감독/전체 계획용, 기본 미사용)
MISSION_SEQUENCE_PROMPT = """
## 전체 미션 시퀀스

### 미션 순서
1. **시작** → gate_start로 이동 (navigate_avoid)
2. **게이트 통과** (카메라 필수)
   - clusters 있으면 align, 없으면 dorodori로 적/녹 부표 탐색
   - 카메라에서 녹색(좌)·적색(우) 부표 위치 확인 (변환 공식은 VISION_GUIDE 참고)
   - gate_pass 명령으로 중간점 통과
   - gate_end까지 navigate_direct
3. **부표 선회** (카메라 필수) - 녹색(GREEN) 부표
   - buoy_orbit 지점 도착
   - clusters 있으면 align, 없으면 dorodori로 녹색 부표 탐색
   - 카메라에서 녹색 부표 위치 확인 → LiDAR 인덱스
   - orbit 명령 (radius: 8m, direction: cw, 6 waypoints)
4. **호핑투어**
   - hopping 지점으로 navigate_avoid
   - 도착 후 hover 3초 정지
5. **장애물 회피 구간**
   - hopping → obstacle_end_dock_start 구간
   - navigate_avoid로 장애물 자동 회피
6. **도킹** (카메라 필수)
   - clusters 있으면 align, 없으면 dorodori로 도킹 스테이션 탐색
   - 카메라에서 마커/색상 인식
   - navigate_direct로 저속 진입
   - hover 3초로 정박 완료

### 카메라 사용 시점
- 게이트 통과: 적(좌)/녹(우) 부표 인식
- 부표 선회: 녹색(GREEN) 부표 탐색
- 도킹: 마커/도형 인식
"""
