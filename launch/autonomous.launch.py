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
    # 센서 브릿지 (GZ → ROS) - GZ_IP 환경변수 포함
    sensor_bridge = ExecuteProcess(
        cmd=['bash', '-c', f'''
            export GZ_IP={GZ_IP}
            ros2 run ros_gz_bridge parameter_bridge \
                /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
                /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
                /world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
                /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
                --ros-args \
                -r /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat:=/wamv/sensors/gps/fix \
                -r /world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu:=/wamv/sensors/imu/data \
                -r /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan:=/wamv/sensors/lidar/scan
        '''],
        name='sensor_bridge',
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
        thruster_bridge,
        motor_controller,
        mission_runner,
        integrated_visualizer,
        release_boat,
    ])
