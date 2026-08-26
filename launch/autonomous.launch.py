"""
KABOAT 자율주행 시스템 런치 파일
- sensor_bridge: Gazebo-ROS2 센서 브릿지
- motor_controller: PD 제어기
- mission_runner: 미션 실행기
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

GZ_IP = '10.22.79.185'

def generate_launch_description():
    # 센서 브릿지 (GZ → ROS) - GPS/IMU/LiDAR
    sensor_bridge = ExecuteProcess(
        cmd=['bash', '-c', f'''
            export GZ_IP={GZ_IP}
            ros2 run ros_gz_bridge parameter_bridge \
                /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
                /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
                /world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
                /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
                /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
                --ros-args \
                -r /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat:=/wamv/sensors/gps/fix \
                -r /world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu:=/wamv/sensors/imu/data \
                -r /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan:=/wamv/sensors/lidar/scan \
                -r /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points:=/wamv/sensors/lidar/points
        '''],
        name='sensor_bridge',
        output='screen'
    )

    # 카메라 브릿지 (별도 실행 - QoS 이슈 회피)
    # ros_gz_image 사용으로 이미지 전송 안정화
    camera_bridge = ExecuteProcess(
        cmd=['bash', '-c', f'''
            export GZ_IP={GZ_IP}
            ros2 run ros_gz_image image_bridge \
                /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image \
                --ros-args \
                -r /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image:=/wamv/sensors/camera/image_raw
        '''],
        name='camera_bridge',
        output='screen'
    )

    # 스러스터 브릿지 (ROS → GZ)
    thruster_bridge = ExecuteProcess(
        cmd=['bash', '-c', f'''
            export GZ_IP={GZ_IP}
            ros2 run ros_gz_bridge parameter_bridge \
                /wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double \
                /wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double
        '''],
        name='thruster_bridge',
        output='screen'
    )
    
    # 모터 컨트롤러 (PD 제어)
    motor_controller = Node(
        package='kaboat_autonomous',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )
    
    # 미션 실행기
    mission_runner = Node(
        package='kaboat_autonomous',
        executable='mission_runner',
        name='mission_runner',
        output='screen'
    )

    # 통합 시각화
    integrated_visualizer = Node(
        package='kaboat_autonomous',
        executable='integrated_visualizer',
        name='integrated_visualizer',
        output='screen'
    )

    # LLM 인터페이스 (ros-mcp 연동)
    llm_interface = Node(
        package='kaboat_autonomous',
        executable='llm_interface',
        name='llm_interface',
        output='screen'
    )

    # 액션 디스패처 (LLM 명령 → 모듈 실행)
    action_dispatcher = Node(
        package='kaboat_autonomous',
        executable='action_dispatcher',
        name='action_dispatcher',
        output='screen'
    )

    # 센서 통합 (LLM 멀티모달 입력용)
    sensor_fusion = Node(
        package='kaboat_autonomous',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen'
    )

    # 보트 릴리즈 (플랫폼에서 분리) - 5초 후 실행
    release_boat = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', f'''
                    export GZ_IP={GZ_IP}
                    gz topic -t "/vrx/release" -m gz.msgs.Empty -p ""
                    echo "WAM-V released from platform"
                '''],
                name='release_boat',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        sensor_bridge,
        camera_bridge,
        thruster_bridge,
        motor_controller,
        mission_runner,
        integrated_visualizer,
        llm_interface,
        action_dispatcher,
        sensor_fusion,
        release_boat,
    ])
