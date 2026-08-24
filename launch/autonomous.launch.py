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

    # 센서 브릿지 (Gazebo Garden 용)
    bridge_params = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        '/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
        '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
    ]

    remappings = [
        ('/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat', '/wamv/sensors/gps/fix'),
        ('/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu', '/wamv/sensors/imu/data'),
        ('/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan', '/wamv/sensors/lidar/scan'),
    ]

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_params,
        remappings=remappings,
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
        motor_controller,
        mission_runner,
    ])
