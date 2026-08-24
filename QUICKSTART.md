# KABOAT 자율주행 시스템 실행 가이드

## 개요
이 문서는 KABOAT 시뮬레이터와 자율주행 시스템을 실행하는 방법을 설명합니다.

## 사전 요구사항
- ROS2 Humble
- Gazebo Garden (gz-sim-7)
- vrx_ws 워크스페이스 빌드 완료
- ros-mcp-server 설치

---

## 중요: ros_gz 패키지 호환성

### 문제
`ros-humble-ros-gz-bridge`는 **Ignition Fortress** (ignition-transport11)용으로 빌드되어 있어
**Gazebo Garden** (gz-transport12)과 호환되지 않습니다.

### 해결책: ros-gzgarden 패키지 설치
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-gzgarden-bridge ros-humble-ros-gzgarden-image
```

| 패키지 | Gazebo 버전 | Transport 라이브러리 |
|--------|-------------|---------------------|
| `ros-humble-ros-gz-bridge` | Ignition Fortress | ignition-transport11 |
| `ros-humble-ros-gzgarden-bridge` | **Gazebo Garden** ✓ | gz-transport12 |

### 브릿지 실행 (ros_gzgarden 사용)
```bash
# ros_gz 대신 ros_gzgarden 패키지 사용
ros2 run ros_gzgarden_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
```

---

## 1단계: 환경 설정

### 터미널 열기
```bash
source /opt/ros/humble/setup.bash
source /home/yune/vrx_ws/install/setup.bash
```

### Python 경로 문제 해결 (Linuxbrew 사용 시)
```bash
export PATH="/usr/bin:$PATH"
```

---

## 2단계: 시뮬레이터 실행

### 중요: GZ_IP 설정 (Tailscale 사용 시 필수)
Tailscale VPN 사용 시 Gazebo가 VPN IP에 바인딩되어 ros_gz_bridge와 통신이 안 됩니다.
**시뮬레이터 시작 전에** 아래 환경 변수를 설정하세요:
```bash
# WiFi IP 사용 (권장)
export GZ_IP=10.22.79.185

# 또는 localhost 사용
# export GZ_IP=127.0.0.1
```

### 시뮬레이터 시작
```bash
ros2 launch kaboat_pkg kaboat_sim.launch.py
```

### 시뮬레이터 옵션
```bash
# 헤드리스 모드 (GUI 없음)
ros2 launch kaboat_pkg kaboat_sim.launch.py headless:=True

# 일시정지 상태로 시작
ros2 launch kaboat_pkg kaboat_sim.launch.py paused:=True
```

---

## 3단계: rosbridge 서버 실행 (ros-mcp-server용)

### 새 터미널에서 실행
```bash
export PATH="/usr/bin:$PATH"

# rosapi 시작
nohup /usr/bin/python3 /opt/ros/humble/lib/rosapi/rosapi_node \
    --ros-args -r __node:=rosapi > /tmp/rosapi.log 2>&1 &

# rosbridge 시작
nohup /usr/bin/python3 /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket \
    --ros-args -p port:=9090 > /tmp/rosbridge.log 2>&1 &
```

### 연결 확인
```bash
ss -tlnp | grep 9090
```

---

## 4단계: 보트 릴리즈

시뮬레이터 시작 후 약 8초 뒤 자동 릴리즈됩니다. 수동 릴리즈:
```bash
gz topic -t /vrx/release -m gz.msgs.Empty -p ''
```

---

## 5단계: 시스템 확인

### Gazebo에서 직접 센서 데이터 확인
```bash
# GPS
gz topic -e -t /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat -n 1

# IMU
gz topic -e -t /world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu -n 1

# LiDAR
gz topic -e -t /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan -n 1
```

### 스러스터 테스트 (ROS2)
```bash
# 전진
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "{data: 100}" -r 10 &
ros2 topic pub /wamv/thrusters/right/thrust std_msgs/msg/Float64 "{data: 100}" -r 10 &

# 정지
pkill -f "ros2 topic pub"
```

---

## 6단계: ros-mcp-server 연결

### Claude Code에서 연결
```
connect_to_robot(ip="127.0.0.1", port=9090)
```

### 스러스터 제어 (작동 확인됨)
```
publish_for_durations(
    topic="/wamv/thrusters/left/thrust",
    msg_type="std_msgs/msg/Float64",
    messages=[{"data": 100}],
    durations=[2],
    rate_hz=10
)
```

---

## Gazebo 토픽 참조

### 센서 토픽 (Gazebo)
| 토픽 | 설명 |
|------|------|
| `/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat` | GPS |
| `/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu` | IMU |
| `/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan` | LiDAR |
| `/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image` | 전방 좌측 카메라 |

### 제어 토픽 (ROS2 → Gazebo, 작동 확인됨)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/wamv/thrusters/left/thrust` | `std_msgs/Float64` | 좌측 스러스터 (-250~250 N) |
| `/wamv/thrusters/right/thrust` | `std_msgs/Float64` | 우측 스러스터 (-250~250 N) |

---

## 문제 해결

### rosbridge 연결 실패
```bash
ss -tlnp | grep 9090
pkill -f rosbridge_websocket
pkill -f rosapi_node
# 3단계 다시 실행
```

### 보트가 움직이지 않음
```bash
# 릴리즈 확인
gz topic -t /vrx/release -m gz.msgs.Empty -p ''

# 시뮬레이션 일시정지 해제
gz service -s /world/kaboat_course/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req 'pause: false'
```

### Python 모듈 오류
```bash
export PATH="/usr/bin:$PATH"
```

---

## 5.5단계: 센서 브릿지 실행 (ros-gzgarden 설치 후)

```bash
# 센서 브릿지 런치
ros2 launch kaboat_autonomous sensor_bridge.launch.py

# 또는 개별 실행
GZ_IP=10.22.79.185 ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat' \
  --ros-args -r /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat:=/wamv/sensors/gps/fix
```

### ROS2에서 센서 데이터 확인 (✅ 작동 확인됨)
```bash
ros2 topic echo /wamv/sensors/gps/fix --once
ros2 topic echo /wamv/sensors/imu/data --once
```

---

## 다음 단계

ros_gz_bridge 문제 **해결됨** (ros-humble-ros-gzgarden-bridge 설치 필요):
1. rviz2로 센서 시각화 가능
2. ros-mcp-server로 센서 데이터 읽기 및 스러스터 제어
3. LLM이 고수준 의사결정, 알고리즘이 저수준 제어 담당
