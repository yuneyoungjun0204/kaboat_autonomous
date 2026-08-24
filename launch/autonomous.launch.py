"""
KABOAT 자율주행 시스템 런치 파일
- sensor_bridge: Gazebo-ROS2 센서 브릿지
- motor_controller: PD 제어기
- mission_runner: 미션 실행기
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # GZ_IP 환경변수 설정 (Tailscale 충돌 방지)
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '10.22.79.185')

    # 센서 브릿지 (GZ → ROS)
    sensor_params = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        '/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ]

    sensor_remappings = [
        ('/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat', '/wamv/sensors/gps/fix'),
        ('/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu', '/wamv/sensors/imu/data'),
        ('/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan', '/wamv/sensors/lidar/scan'),
    ]

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        arguments=sensor_params,
        remappings=sensor_remappings,
        output='screen'
    )

    # 스러스터 브릿지 (ROS → GZ) - 별도 노드
    thruster_params = [
        '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
        '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
    ]

    thruster_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='thruster_bridge',
        arguments=thruster_params,
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

    return LaunchDescription([
        set_gz_ip,
        sensor_bridge,
        thruster_bridge,
        motor_controller,
        mission_runner,
    ])
