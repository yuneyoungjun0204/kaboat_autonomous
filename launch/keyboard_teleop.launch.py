"""
KABOAT 키보드(WASD) 수동 조종 런치 파일
- thruster_bridge: 스러스터 명령 브릿지 (ROS → GZ)
- release_boat: 플랫폼에서 보트 분리 (5초 후)

주의: keyboard_teleop 노드는 이 런치 파일에 포함하지 않는다.
'ros2 launch'는 자식 프로세스의 stdin을 실제 터미널에 연결해주지 않아
키 입력을 읽을 수 없다. 이 런치를 띄운 뒤 별도 터미널에서
    ros2 run kaboat_autonomous keyboard_teleop
로 직접 실행할 것.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

GZ_IP = '10.22.79.185'


def generate_launch_description():
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

    # 보트 릴리즈 (플랫폼에서 분리) - 5초 후 실행
    release_boat = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', f'''
                    export GZ_IP={GZ_IP}
                    gz topic -t "/vrx/release" -m gz.msgs.Empty -p ""
                    echo "WAM-V released from platform"
                    echo ">>> 다른 터미널에서 실행: ros2 run kaboat_autonomous keyboard_teleop"
                '''],
                name='release_boat',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        thruster_bridge,
        release_boat,
    ])
