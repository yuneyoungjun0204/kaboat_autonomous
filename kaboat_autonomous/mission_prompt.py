#!/usr/bin/env python3
"""
KABOAT 미션 프롬프트 생성기

LLM에게 제공할 시스템 프롬프트와 상황 컨텍스트를 생성합니다.
멀티모달 입력(카메라 이미지 + LiDAR 데이터 + 텍스트)과 함께 사용됩니다.
"""
import sys

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.insert(0, '/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

# 미션별 카메라(색상/마커 인식) 필요 여부 - VISION_GUIDE 포함 여부 결정
MISSION_NEEDS_CAMERA = {
    'gate_search': True,
    'buoy_orbit': True,
    'hopping_tour': False,
    'obstacle_course': False,
    'docking': True,
}

# 카메라 확인을 시작할 psi_error 임계값 (반시야각 - 여유분). 이 값보다
# |psi_error_deg|가 작아지면 타깃이 실제로 프레임에 들어왔을 가능성이 높다.
_CAMERA_CHECK_TRIGGER_DEG = SETTINGS.CAMERA_HALF_FOV_DEG - SETTINGS.CAMERA_CHECK_MARGIN_DEG


# --- LLM 프롬프트용 헬퍼 함수 ---

def _format_front_distribution(front_dist: list) -> str:
    """전방 장애물 분포를 LLM이 이해하기 쉬운 텍스트로 변환"""
    if not front_dist:
        return "  (데이터 없음)"
    lines = []
    for s in front_dist:
        dist_str = f"{s['distance']:.1f}m" if s.get('distance') is not None else "없음"
        lines.append(f"  {s['sector']}: {dist_str}")
    return "\n".join(lines)


def _format_clusters(clusters: list) -> str:
    """클러스터 정보를 LLM이 추론/참조하기 쉽게 포맷"""
    if not clusters:
        return "  (클러스터 없음)"
    lines = []
    for c in clusters:
        cid = c.get('id', '?')
        angle = c.get('center_angle', 0)
        dist = c.get('distance', 0)
        rejected = c.get('rejected', False)
        status = " [거부됨]" if rejected else ""
        direction = "좌" if angle > 0 else "우" if angle < 0 else "정면"
        lines.append(f"  id={cid}: {direction}{abs(angle):.0f}° @ {dist:.1f}m{status}")
    return "\n".join(lines)

# 항상 포함되는 핵심 프롬프트: 모듈 스키마 + LiDAR 규약 + 응답 형식만.
# 미션별 수행 전략은 generate_mission_context()가 담당하므로 여기서 중복 나열하지 않는다.
# (JSON 예시에 중괄호가 많아 f-string으로 통째로 만들면 깨지므로, 값이 필요한
# 조각만 별도 f-string으로 만들어 아래에서 문자열 연결한다.)
CORE_PROMPT_BODY = """# KABOAT 자율주행 미션 수행 시스템

당신은 KABOAT 자율주행 보트의 미션 컨트롤러입니다.
센서 데이터를 분석하고, 적절한 모듈을 호출하여 미션을 수행합니다.

## 사용 가능한 모듈 (ros-mcp로 /llm_action 토픽에 JSON 발행)

### 이동 모듈
```json
// 장애물 회피 항법 (LiDAR 기반 경로 계획)
{"action": "navigate_avoid", "goal_x": 10.0, "goal_y": 20.0}

// 직진 항법 (장애물 회피 없음, 짧은 거리/클리어 경로용)
{"action": "navigate_direct", "goal_x": 10.0, "goal_y": 20.0}

// 직진 항법 + 헤딩 고정 (hold_heading 지정 시 목표 방향 계산 대신
// 해당 헤딩을 그대로 유지하며 직진 - 게이트 중심선처럼 정렬된 방향을
// 흐트러뜨리지 않고 똑바로 나아가야 할 때 사용. goal_x/goal_y는
// 도달 판정/감속 거리 계산용으로 계속 필요)
{"action": "navigate_direct", "goal_x": 10.0, "goal_y": 20.0, "hold_heading": 45.0}
```

### 기동 모듈
```json
// 후진 (stuck 탈출, 헤딩 유지)
{"action": "backward", "duration": 3.0}

// 헤딩 정렬 (특정 방향으로 정렬, 전진 없음) - 탐색 시 dorodori보다 우선 사용
{"action": "align", "heading": 90.0, "tolerance": 5.0, "timeout": 10.0}

// 클러스터 기준 정렬 (LiDAR 클러스터 id로 해당 물체 방향으로 정렬)
// 카메라로 인식한 부표가 있고 LiDAR 클러스터와 매칭될 때 사용
{"action": "align_to_cluster", "cluster_id": 0, "tolerance": 5.0, "timeout": 10.0}

// 좌우 스캔 (align으로 정렬할 클러스터가 없을 때의 광역 탐색 폴백)
// center_heading: 스윕 기준각(절대 헤딩, 도) - 시작 시점에 한 번만 고정됨
{"action": "dorodori", "center_heading": 45.0, "duration": 8.0, "half_range": 30.0}

// center_heading 대신 현재 미션 구간 목표 방향(진행 방향)을 기준으로 스윕
// - 어느 각도를 기준으로 훑을지 모를 때, 진행 방향 쪽을 우선 탐색하고 싶을 때 사용
{"action": "dorodori", "center_bearing_to_goal": true, "duration": 8.0, "half_range": 30.0}

// 둘 다 생략하면 시작 시점의 현재 헤딩을 기준각으로 사용

// 위치 유지 (도킹, 호핑투어 정지)
{"action": "hover", "x": 5.0, "y": 5.0, "duration": 3.0}
```

### 웨이포인트 생성 모듈
```json
// 게이트 통과 (두 LiDAR 점 중간으로)
{"action": "gate_pass", "left_idx": 45, "right_idx": 315}

// 부표 궤도 (LiDAR 점 주위 선회)
{"action": "orbit", "lidar_idx": 90, "radius": 12.0, "direction": "cw", "laps": 1.0}

// 웨이포인트 리스트 (호핑투어)
{"action": "waypoints", "waypoints": [{"x": 10, "y": 20}, {"x": 30, "y": 40}]}
```

### 유틸리티
```json
// 정지
{"action": "stop"}

// 상황 분석 요청 (LiDAR 요약 반환)
{"action": "analyze"}

// 클러스터가 타깃이 아님을 기록 (전역 좌표로 저장, 재탐색 방지)
{"action": "reject_cluster", "angle": 42.5, "distance": 10.0, "reason": "not_green_buoy"}

// mission_phase_done / mission_auto_pause / mission_auto_resume:
// **이 실행 경로에서는 절대 호출하지 말 것** - 이유는 아래
// "action_status의 mission_phase / mission_phase_idx 필드에 대해" 참고.
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
1. `clusters`에서 `rejected: true`인 항목은 건너뛴다 (이미 카메라로 확인해서
   타깃이 아니라고 판정된 곳 - 세션이 끊겨도 유지되는 기록이므로 재시도 불필요)
2. 남은 클러스터 중 가장 가까운 것을 목표로 `align` 실행.
   `heading = 현재 헤딩(IMU) + cluster.center_angle` (정규화, -180~180)
3. 정렬 완료 후 카메라로 실제 부표/도킹 마커인지 확인
   - 맞으면 해당 미션 절차 진행
   - 아니면 `reject_cluster`로 기록한 뒤 다음 순위 클러스터로 재시도,
     그래도 없으면 4번
4. 쓸 클러스터가 없으면(전부 rejected거나 비어있음) → `dorodori`로 광역 스캔
   (dorodori는 클러스터 없을 때만 쓰는 폴백. clusters가 있는데 바로
   dorodori부터 쓰지 말 것 - 이미 방향을 아는데 광역 스윕은 비효율적)
   - 기준각(center_heading)을 명시하지 않으면 시작 시점의 현재 헤딩이
     기준이 됨. 특정 방향이 유력하면 `center_heading`을 직접 지정하거나,
     진행 방향(현재 미션 구간 목표) 쪽을 우선 훑고 싶으면
     `center_bearing_to_goal: true`를 사용

## 카메라엔 보이는데 클러스터가 안 잡힐 때 (제자리 맴돌기 금지)

`clusters`에 매칭되는 게 없어도 카메라 화면에 타깃이 살짝이라도 걸려 있으면
(부표 일부만 보임, 화면 가장자리 등) **align/dorodori로 그 자리에서 계속
재정렬·재스캔만 반복하지 말 것** - 이건 사거리 밖에 있거나(멀리 있는 부표는
클러스터 최대 탐지 거리보다 멀 수 있음) 물체가 얇아서 안 잡히는 경우가
많고, 제자리에서 맴돌기만 해서는 절대 해결되지 않는다.

방법:
1. 화면 X좌표를 각도로 변환 (`lidar_angle = (640 - image_x) / 640 * 40`,
   1280px 기준 - CAMERA_HALF_FOV_DEG 사용)
2. `heading = 현재 헤딩(IMU) + lidar_angle`로 절대 방향을 구하고, 그 방향으로
   적당히(예: 15~20m) 떨어진 지점의 좌표를 `goal_x = 현재x + 거리*cos(heading),
   goal_y = 현재y + 거리*sin(heading)`로 계산해 `navigate_avoid`로 이동한다
   (LiDAR 장애물을 자동으로 피하면서 거리를 좁힌다)
3. 이동 후 다시 카메라/clusters 확인 → 클러스터가 잡히면 이제 `align_to_cluster`나
   `orbit` 등 정상 절차로 전환, 여전히 안 잡히면 다시 최신 카메라 이미지 기준으로
   방향을 재계산해 반복 (같은 방향으로 무한 반복하지 말 것)

## LiDAR 데이터 분석 시
- `front_clear: true` → 전방 10m 이내 장애물 없음
- `gate_detected: true` → 좌우에 대칭적 물체 (게이트 가능성)
- `closest_obstacle` → 가장 가까운 장애물 방향/거리
- `clusters` → 위 "탐색 우선순위" 참고

## Stuck 감지 시
1. `backward` 3초 후진
2. `clusters` 확인 후 있으면 `align`, 없으면 `dorodori`로 새 경로 탐색
3. 다른 방향으로 `navigate_avoid`

## 폴링 시 decision_needed 확인
`/action_status`, `/sensor_fusion`의 `decision_needed`가 `false`면 진행 중인
액션이 아직 끝나지 않아 control_loop가 LLM 없이 자율 진행 중이라는 뜻이다.
이때는 새 액션을 선택하지 말고, 다음 폴링까지 대기만 하라 (불필요한 판단/
새 action 호출로 진행 중인 기동을 중단시키지 말 것). `decision_needed: true`
(액션 완료, stuck 등)일 때만 본격적으로 상황을 분석해 다음 액션을 결정한다.

## 압축 상태(blackboard) - 대화가 짧게 끊겨도 유지됨
`last_action`/`last_action_result`/`retry_count`는 action_dispatcher 노드가
미션 내내 들고 있는 최소 이력이다 (LLM 대화 자체가 새로 시작돼도 이 값들은
안 사라짐). 매 판단 시 확인할 것:
- `last_action_result`가 실패류(`timeout`, `failed_no_lidar`)면 방금 시도가
  왜 실패했는지 먼저 고려하고 같은 방식을 그대로 반복하지 말 것
- `retry_count[action]`이 2 이상이면 그 액션을 계속 쓰지 말고 다른 접근으로
  전환 (예: align이 계속 timeout → dorodori로 전환, orbit이 계속
  failed_no_lidar → 더 가까이 접근 후 재시도)

## action_status의 mission_phase / mission_phase_idx 필드에 대해
이 필드는 action_dispatcher의 순차 웨이포인트 자동전환 상태기계(전체 미션을
순서대로 도는 용도) 전용이며, 이 실행 경로(대시보드가 미션을 시작/진행시킴)에서는
그 상태기계가 시작되지도, 진행되지도 않는다 - 항상 최초값(첫 웨이포인트 이름)에
고정돼 보인다. **지금 무슨 미션을 하고 있는지, 다음에 뭘 해야 하는지에 대한
신호로 이 필드를 참고하거나 언급하지 말 것** - 예를 들어 `mission_phase`가
"gate_start"로 보인다고 해서 게이트 통과로 전환해야 한다는 뜻이 아니다. 지금
무슨 미션인지는 이 프롬프트 상단의 "현재 미션" 섹션만 근거로 삼을 것.
**그 상태기계를 직접 건드리는 `mission_phase_done`/`mission_auto_pause`/
`mission_auto_resume` 액션도 이 실행 경로에서는 절대 호출하지 말 것** -
시작돼 있지 않은 상태기계를 호출로 깨우면 보트가 지금 미션과 무관하게 다음
웨이포인트로 이탈한다 (2026-09-07: 정확히 이 방식으로 보트가 3분간 엉뚱한
방향으로 항해한 사고 실측 이후 명시).

## 액션 실행 방법 - 반드시 도구를 호출할 것, 텍스트로 출력만 하지 말 것

위 모듈들은 JSON을 응답 텍스트에 출력하는 게 아니라, ros-mcp의 publish_once 도구를
**실제로 호출해서** `/llm_action` 토픽에 발행해야 실행된다:

mcp__ros-mcp__publish_once(topic="/llm_action", msg_type="std_msgs/String", msg={"data": "<액션 JSON을 문자열로>"})

예시: `{"action": "navigate_avoid", "goal_x": 10.0, "goal_y": 20.0}`을 실행하려면
publish_once(topic="/llm_action", msg_type="std_msgs/String", msg={"data": "{\\"action\\": \\"navigate_avoid\\", \\"goal_x\\": 10.0, \\"goal_y\\": 20.0}"})를 호출한다.
`/action_status`, `/lidar_summary` 등 상태 확인은 mcp__ros-mcp__subscribe_once로 조회한다.

## 미션이 실제로 끝날 때까지 도구 호출을 반복할 것

이 실행은 단발 대화가 아니다. 상황 분석·계획을 텍스트로 한 번 설명하고 끝내지 말 것 -
"상태 확인(subscribe_once) → 판단 → 액션 발행(publish_once) → 진행 대기 후 재확인"을
미션이 실제로 완료될 때까지(이 프롬프트의 "현재 미션" 섹션에 명시된 완료 조건을
충족할 때까지 - mission_phase_done 호출과는 무관함, 위 "mission_phase 필드에
대해" 참고) 반복해서 도구를 호출하라. 첫 판단만 내리고 도구 호출 없이 멈추면
보트는 아무것도 하지 않은 채로 프로세스만 종료된다.
"""

# JSON 예시의 중괄호와 섞이지 않도록 별도 f-string으로 만들어 CORE_PROMPT_BODY에
# 삽입한다 (값이 SETTINGS에서 오므로 하드코딩 드리프트 방지).
_CAMERA_TIMING_SECTION = f"""## 카메라 확인 타이밍 (align 중 언제 사진을 찍을지)
카메라는 전방 1대, 반시야각 ≈ {SETTINGS.CAMERA_HALF_FOV_DEG:.0f}°다. 사진을 볼 때는 반드시
아래 압축(JPEG) 토픽을 구독할 것 - 비압축 `/wamv/sensors/camera/image_raw`는
프레임당 ~3.7MB라 subscribe_once가 거의 항상 타임아웃난다(실측 확인됨):

mcp__ros-mcp__subscribe_once(
    topic="/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image/compressed",
    msg_type="sensor_msgs/CompressedImage",
    expects_image="true",
    timeout=10
)

`cluster.center_angle`은 보트 기준 상대각이라 카메라 중심(보트 정면)과 바로
비교 가능 - `|center_angle| < {SETTINGS.CAMERA_HALF_FOV_DEG:.0f}°`면 지금 카메라
시야 안에 있다는 뜻이다.

- align을 막 시작했는데 타깃이 반시야각보다 먼 각도에 있었다면, 그 시점에는
  카메라를 보지 말 것 (아직 프레임 밖일 가능성이 높아 헛촬영이다 - align은
  애초에 "카메라에 안 보이니 보일 만한 각도로 도는" 동작임을 기억할 것)
- align 진행 중에는 `/sensor_fusion`의 `command.psi_error_deg`(실시간 조향
  오차, 새 판단 없이 그냥 확인 가능)를 지켜보다가 `|psi_error_deg| < {_CAMERA_CHECK_TRIGGER_DEG:.0f}°`
  (반시야각에서 여유분을 뺀 값 - 렌즈 가장자리 왜곡/부분 프레임 회피)로
  줄어드는 순간부터 `subscribe_once`로 카메라 확인을 시작한다. 그 전까지는
  새 액션 호출 없이 대기만 한다 (헛촬영/토큰 낭비 방지)
- 회전각이 애초에 작아 시작부터 이미 `|center_angle| < {_CAMERA_CHECK_TRIGGER_DEG:.0f}°`인
  클러스터라면 align 시작과 거의 동시에 확인해도 된다 (남은 정렬 시간과
  이미지 판단을 겹쳐서 처리 - 실제로 대기시간이 줄어드는 유일한 구간)

## 한 프레임에 여러 클러스터 묶어 처리
카메라를 확인할 때 `clusters` 목록에서 목표 클러스터 외에도
`|center_angle| < {SETTINGS.CAMERA_HALF_FOV_DEG:.0f}°`인 다른 후보가 있으면
같은 사진에 함께 보일 가능성이 높다. 그런 후보가 있으면 별도로 align+촬영을
반복하지 말고 한 번의 촬영/분석으로 같이 판별한다 (타깃이 아닌 것으로
확인되면 각각 `reject_cluster`로 기록해 재탐색을 막는다)
"""

CORE_PROMPT = CORE_PROMPT_BODY.replace(
    "## LiDAR 데이터 분석 시", _CAMERA_TIMING_SECTION + "## LiDAR 데이터 분석 시"
)

# 카메라(색상/마커 인식)가 필요한 미션에만 포함하는 조각.
# gate_search/buoy_orbit/docking 전용, hopping_tour/obstacle_course에는 불필요.
VISION_GUIDE = f"""## 색상-부표 규약 (본 코스 실측 기준, IALA와 반대)
- **녹색(Green)**: 좌현 통과 (왼쪽에 두고 지나감)
- **적색(Red)**: 우현 통과 (오른쪽에 두고 지나감)
- 게이트: 녹색-적색 사이로 진입
- (주의: 일반 IALA 규약은 적=좌현/녹=우현이지만, 이 시뮬레이션 코스는 실제 카메라 확인 결과 반대로 배치되어 있음 - 2026-08-26 gate_start 지점에서 확인)

## 이미지 분석 시
- 적색/녹색/파란색 부표 위치를 화면 좌표로 파악
- 화면 중앙 = 정면(0°), 왼쪽 = 좌현(+), 오른쪽 = 우현(-)
- 화면 X 좌표를 LiDAR 각도로 변환 (실제 카메라 해상도 1280x720 기준,
  중앙=640px): `lidar_angle = (640 - image_x) / 640 * {SETTINGS.CAMERA_HALF_FOV_DEG:.0f}`
  - 예: x=320 (왼쪽 1/4) → +{SETTINGS.CAMERA_HALF_FOV_DEG/2:.0f}° / x=960 (오른쪽 1/4) → -{SETTINGS.CAMERA_HALF_FOV_DEG/2:.0f}°

## 카메라-LiDAR 융합 추론 (핵심!)

**센서 개별로도 추론 가능, 함께 보면 더 정확:**

1. **카메라만 있을 때**: 부표 색상/위치로 게이트 구조 파악
   - 녹색(좌)·적색(우) 쌍이 보이면 → 그 사이가 통과 경로
   - 부표 화면 X 좌표 → LiDAR 각도로 변환하여 `gate_pass` 호출

2. **LiDAR만 있을 때**: `front_distribution`에서 패턴 추론
   - 좌우 대칭적으로 가까운 거리의 물체가 있고, 정면은 비어있다면 → 게이트 패턴
   - 예: 좌30°에 8m, 우30°에 9m, 정면은 없음 → 두 부표 사이로 지나갈 수 있음

3. **융합 추론 (권장)**: 카메라 인식 + LiDAR 거리
   - 카메라에서 녹색 부표가 화면 왼쪽에 → `front_distribution` 좌측 섹터에 물체
   - 이 매칭이 확인되면 해당 LiDAR 클러스터 = 녹색 부표
   - `align_to_cluster`로 특정 부표에 정렬 가능

**게이트 통과 판단**:
- 결정론적 `gate_detected`를 쓰지 않음 - 당신이 직접 판단하세요
- `front_distribution`에서 "좌우에 물체, 정중앙 비어있음" 패턴이면 통과 가능
- 카메라에서 색상 확인까지 되면 확신을 갖고 `gate_pass` 실행
- 게이트 미션에서는 애초에 `navigate_avoid`를 쓰지 않으므로(위 "현재 미션"
  섹션 참고) 부표를 장애물로 오인해 피해가는 문제 자체가 발생하지 않는다 -
  `gate_pass`/`navigate_direct`로 중앙을 그대로 통과할 것

**부표 하나만 보일 때 (중요!)**:
- 카메라에 녹색 또는 적색 부표 중 하나만 보이면, 반대편에 다른 부표가 있다고 추론
- 녹색만 보임 → 녹색 부표의 **오른쪽**이 통로 (적색이 더 오른쪽에 있을 것)
- 적색만 보임 → 적색 부표의 **왼쪽**이 통로 (녹색이 더 왼쪽에 있을 것)
- `front_distribution`에서 해당 부표 방향만 장애물 → 반대 방향으로 5~8m 우회해서 직진
- 절대 부표에 부딪히지 말 것: 부표 방향의 클러스터를 피해 반대편 빈 공간으로 진행
"""


def get_waypoint_heading(name):
    """이름으로 미션 웨이포인트에 사전 측량된 헤딩(ENU, 0=동/90=북)을 반환한다.
    docking뿐 아니라 gate/buoy_orbit 사전 정렬(HYBRID_ARCHITECTURE_REFACTORING.md
    §3.3/3.5)도 이 함수를 공유해서 쓴다 - 원래는 obstacle_end_dock_start 전용
    _get_dock_start_heading()이었는데, get_mission_waypoints_local()의 heading
    필드 자체가 처음부터 범용이라 이름만 일반화했다. 웨이포인트를 못 찾거나
    heading_deg가 측량 안 돼 있으면 None."""
    try:
        for _x, _y, heading, wp_name, _desc, _requires_llm in SETTINGS.get_mission_waypoints_local():
            if wp_name == name:
                return heading
    except Exception:
        pass
    return None


def _get_waypoint_local_xy(name):
    """이름으로 미션 웨이포인트의 로컬 (x, y) 좌표를 찾는다. gate_search가
    gate_pass 이후 navigate_direct로 실제 빠져나갈 목표점(gate_end)을 LLM에게
    구체적인 좌표로 알려주기 위함 - 좌표 없이 "적당히 먼 곳으로"만 지시하면
    모델이 gate_pass만 하고 멈추거나 애매한 방향으로 직진해 코스를 벗어나지
    못하는 경우가 있었다. 못 찾으면 (None, None)."""
    try:
        for x, y, _heading, wp_name, _desc, _requires_llm in SETTINGS.get_mission_waypoints_local():
            if wp_name == name:
                return x, y
    except Exception:
        pass
    return None, None


def generate_mission_context(mission_type: str, params: dict = None) -> str:
    """미션별 추가 컨텍스트 생성"""

    contexts = {
        'gate_search': """
## 현재 미션: 게이트 탐색 및 통과

목표: 적색-녹색 게이트를 찾아 사이로 통과하세요.

**게이트는 한 세트가 아닐 수 있다.** 적색-녹색 부표 게이트가 여러 쌍 연속으로
배치돼 있을 수 있고, `gate_pass`로 하나를 통과했다고 미션이 끝난 게 아니다 -
통과할 때마다 다음 게이트가 남아있는지 다시 탐색할 것. 기본 전제는 게이트들이
대략 `gate_start`→`gate_end`(={gate_end_hint}) 방향으로 이어져 있다는 것 -
다음 게이트를 찾을 때 그 방향을 우선 탐색하라. 정지해서 두리번거리기보다
`gate_pass`를 빠르고 안정적으로 반복 실행하는 쪽을 우선한다.

**완료 조건 (중요): `gate_pass`로 중간점을 지나간 것만으로는 미션이 끝난 게
아니다.** 카메라+LiDAR 융합 분석으로 부표들을 실제로 "인식"해서 사이를 통과한
뒤, 더 이상 게이트가 안 보이고 ({gate_end_hint}) 지점까지 실제로 도달해 게이트
구조물 영역을 완전히 벗어나야 완료다 - 마지막 게이트를 통과한 뒤에도 계속 다음
게이트가 있는지 확인할 것. gate_pass만 하고 도구 호출을 멈추지 말 것.

**이 미션에서는 `navigate_avoid`를 쓰지 말 것** - LiDAR 회피 경로 계획은 부표를
장애물로 여겨 피해가려 하므로, 게이트 중앙을 그대로 통과해야 하는 이 미션과
상충한다. `gate_pass`가 이 미션의 주력 이동 수단이고, `align`/`dorodori`/
`hover`/`backward`/`navigate_direct`는 게이트를 찾고 정렬·보정하기 위해
상황에 맞게 섞어 쓰는 보조 액션이다.

단계 (아래를 게이트마다 반복하며 조합):
1. 게이트가 안 보이면: `clusters` 확인 → 있으면 가장 가까운 클러스터로 `align`
   (없으면 `dorodori`, `gate_start`→`gate_end` 방향을 우선 탐색)로 탐색
2. 이미지에서 적색/녹색 부표 위치 확인 → 화면 좌표를 LiDAR 인덱스로 변환
3. 두 부표의 중간점이 계산되면 `gate_pass`로 게이트 진입 (이 미션의 핵심 액션)
4. `gate_pass` 도중 정렬이 틀어지면 `align`로 재정렬 후 재시도, 대기가
   필요하면 `hover`, stuck이면 `backward`로 벗어난 뒤 1번부터 재탐색
5. `gate_pass` 완료 후 카메라/LiDAR로 좌우 부표가 더 이상 정면이 아니라 옆이나
   뒤로 지나갔음을 확인 → 다음 게이트가 있으면 1번부터 반복, 없으면
   `navigate_direct`로 ({gate_end_hint})까지 똑바로 빠져나온다 - 이 지점 도착이
   확인되어야 미션 완료로 간주한다

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
4. `orbit`으로 궤도 설정 (direction: {direction}) - radius는 생략해 기본값(12m)을 쓰거나 12 이상으로 직접 지정할 것
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

시작하자마자 대시보드가 이미 `align` (heading: {dock_heading})으로 도킹 스테이션
방향에 정렬해뒀다 (obstacle_end_dock_start 웨이포인트에 사전 측량된 방향 - 도킹
스테이션 좌표는 몰라도 이 지점에서 바라봐야 할 방향은 이미 알고 있음). 그러니
아래 1단계부터 시작할 것.

단계:
1. `clusters` 확인 → 있으면 가장 가까운 클러스터로 `align` (없으면 `dorodori`, center_heading: {dock_heading})로
   도킹 스테이션 탐색 - 클러스터 방향이 아니라 이 사전 측량 방향을 기준으로 탐색할 것
2. {color} 마커/도형 식별
3. 도킹 베이 입구의 두 점 파악 → `gate_pass` 또는 좌표 직접 계산
4. `navigate_direct`로 저속 접근 - 반드시 `"thrust"` 파라미터를 낮게 지정할 것
   (예: `{{"action":"navigate_direct","goal_x":..,"goal_y":..,"thrust":600}}` -
   생략하면 기본 순항 추력(NORMAL_THRUST=1000)으로 접근해 너무 빠르다).
   더 정밀한 최종 접근이 필요하면 400까지 낮출 수 있다.
5. `hover` 3초로 정박 완료

안전 규칙:
- 접근 속도 낮게 유지 (위 4번 thrust 파라미터로 제어)
- 충돌 위험 시 `backward` 후 재접근
- 벽면 감지 시 `hover`로 위치 유지
"""
    }

    template = contexts.get(mission_type, "")
    fmt_params = dict(params) if params else {}
    if mission_type == 'docking':
        heading = get_waypoint_heading('obstacle_end_dock_start')
        fmt_params.setdefault(
            'dock_heading',
            f"{heading:.1f}" if heading is not None else "현재 헤딩 유지 (웨이포인트 헤딩 없음)"
        )
    elif mission_type == 'gate_search':
        gx, gy = _get_waypoint_local_xy('gate_end')
        fmt_params.setdefault(
            'gate_end_hint',
            f"goal_x={gx:.1f}, goal_y={gy:.1f}" if gx is not None
            else "웨이포인트 미설정 - hold_heading을 지정해 현재 방향으로 20m 이상 직진할 것"
        )
    if fmt_params:
        template = template.format(**fmt_params)
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
    front_dist = _format_front_distribution(lidar_summary.get('front_distribution', []))
    clusters = _format_clusters(lidar_summary.get('clusters', []))
    return f"""
## 현재 보트 상태

위치: ({status['position']['x']:.1f}, {status['position']['y']:.1f})
헤딩: {status['position']['heading_deg']:.1f}°

## LiDAR 요약

전방 클리어: {lidar_summary.get('front_clear', 'unknown')}
가장 가까운 장애물: {lidar_summary.get('closest_obstacle', {})}

### 전방 장애물 분포 (직접 판단하세요)
{front_dist}

### 클러스터 (align_to_cluster로 정렬 가능)
{clusters}

## 현재 액션

액션: {status.get('current_action', 'none')}
경과 시간: {status.get('action_elapsed_sec', 0)}초
이전 액션 결과: {status.get('last_action', 'none')} → {status.get('last_action_result', 'none')}
재시도 횟수: {status.get('retry_count', {})}
미션 단계: {status.get('mission_phase', {})}

다음에 수행할 액션을 JSON으로 출력하세요. 게이트 통과 여부는 front_distribution 패턴으로 직접 추론하세요.
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
- 가장 가까운 장애물: {lidar_summary.get('closest_obstacle', {}).get('distance', 999):.1f}m @ {lidar_summary.get('closest_obstacle', {}).get('angle', 0)}°
- 장애물 개수: {lidar_summary.get('obstacle_count', 0)}개

### 전방 장애물 분포 (LiDAR, 카메라와 대조하여 추론하세요)
{_format_front_distribution(lidar_summary.get('front_distribution', []))}

### 클러스터 (id로 참조 가능, align_to_cluster 액션 지원)
{_format_clusters(lidar_summary.get('clusters', []))}

### 현재 상태
- 액션: {action_status.get('current_action', 'none')}
- 경과 시간: {action_status.get('action_elapsed_sec', 0):.1f}초
- 웨이포인트: {action_status.get('waypoints', {}).get('current', 0)}/{action_status.get('waypoints', {}).get('total', 0)}
- 이전 액션 결과: {action_status.get('last_action', 'none')} → {action_status.get('last_action_result', 'none')}
- 재시도 횟수: {action_status.get('retry_count', {})}
- 미션 단계: {action_status.get('mission_phase', {})}

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
        for i, (x, y, heading, name, desc, requires_llm) in enumerate(waypoints, 1):
            tag = "[LLM/비전]" if requires_llm else "[자동 전환]"
            hdg = f", heading={heading:.1f}°" if heading is not None else ""
            lines.append(f"{i}. {tag} {name}: ({x:.1f}, {y:.1f}{hdg}) - {desc}")
        lines.append("```")
        return "\n".join(lines)
    except (ImportError, KeyError, ValueError) as e:
        return f"(웨이포인트 로드 실패: {e})"


# 전체 미션 개요 (include_sequence=True일 때만 포함 - 감독/전체 계획용, 기본 미사용)
MISSION_SEQUENCE_PROMPT = """
## 전체 미션 시퀀스

이동 전용 구간(start→gate_start, gate_end→buoy_orbit, hopping→
obstacle_end_dock_start)은 action_dispatcher가 자동으로 navigate_avoid를
발행한다 - 아래 목록에서 "(자동)"으로 표시된 단계는 LLM이 별도로 호출할
필요 없다. 비전이 필요한 단계를 마치면 반드시 `mission_phase_done`을
호출해야 다음 자동 구간이 재개된다.

### 미션 순서
1. **시작 → gate_start 이동** (자동)
2. **게이트 통과** (카메라 필수)
   - clusters 있으면 align, 없으면 dorodori로 적/녹 부표 탐색
   - 카메라에서 녹색(좌)·적색(우) 부표 위치 확인 (변환 공식은 VISION_GUIDE 참고)
   - gate_pass 명령으로 중간점 통과 → gate_end까지 navigate_direct
   - 완료 후 `mission_phase_done` 호출
3. **gate_end → buoy_orbit 이동** (자동)
4. **부표 선회** (카메라 필수) - 녹색(GREEN) 부표
   - clusters 있으면 align, 없으면 dorodori로 녹색 부표 탐색
   - 카메라에서 녹색 부표 위치 확인 → LiDAR 인덱스
   - orbit 명령 (radius: 12m, direction: cw, 6 waypoints)
   - 완료 후 `mission_phase_done` 호출
5. **buoy_orbit → hopping → obstacle_end_dock_start 이동** (자동, 장애물 자동 회피 포함)
6. **도킹** (카메라 필수)
   - clusters 있으면 align, 없으면 dorodori로 도킹 스테이션 탐색
   - 카메라에서 마커/색상 인식
   - navigate_direct로 저속 진입 → hover 3초로 정박 완료
   - 완료 후 `mission_phase_done` 호출 (미션 종료)

### 카메라 사용 시점
- 게이트 통과: 적(좌)/녹(우) 부표 인식
- 부표 선회: 녹색(GREEN) 부표 탐색
- 도킹: 마커/도형 인식
"""
