"""
모터 컨트롤러 (PD 제어)
SeaNU_KABOAT2024 PWMPublish.py 포팅 (ROS2)
- WAM-V 차동 추진 제어
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS


class MotorController(Node):
    """
    PD 제어기를 사용한 WAM-V 스러스터 제어

    입력: /command (Float32MultiArray) [psi_error, tau_x, max_saturation]
    출력:
        /wamv/thrusters/left/thrust (Float64)
        /wamv/thrusters/right/thrust (Float64)
    """

    def __init__(self):
        super().__init__('motor_controller')

        # Publishers
        self.pub_left = self.create_publisher(
            Float64,
            SETTINGS.TOPICS['thrust_left'],
            10
        )
        self.pub_right = self.create_publisher(
            Float64,
            SETTINGS.TOPICS['thrust_right'],
            10
        )

        # Subscriber
        self.create_subscription(
            Float32MultiArray,
            '/command',
            self.command_callback,
            10
        )

        # PD 제어 상태
        self.last_error = 0.0
        self.last_time = time.time()

        self.get_logger().info('Motor Controller initialized')

    def pd_control(self, error: float) -> float:
        """PD 제어기"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error

        output = SETTINGS.KP * error + SETTINGS.KD * derivative
        return output

    def command_callback(self, msg: Float32MultiArray):
        """
        명령 수신 콜백

        msg.data = [psi_error, tau_x, max_saturation]
        - psi_error: 조향 오차 (도)
        - tau_x: 전진 추력
        - max_saturation: 최대 출력 제한
        """
        if len(msg.data) < 2:
            return

        psi_error = msg.data[0]
        tau_x = msg.data[1]
        max_sat = msg.data[2] if len(msg.data) > 2 else SETTINGS.MAX_THRUST

        # PD 제어로 회전 토크 계산
        # psi_error > 0: 반시계방향(왼쪽) 회전 필요 → 오른쪽 추력 증가
        tau_n = self.pd_control(psi_error)

        # 차동 추진 계산 (WAM-V: 좌/우 스러스터)
        # 반시계방향 회전: right > left
        thrust_left = tau_x - tau_n * 0.5
        thrust_right = tau_x + tau_n * 0.5

        # 정지 상태
        if psi_error == 0 and tau_x == 0:
            thrust_left = 0.0
            thrust_right = 0.0
        else:
            # Saturation
            thrust_left = max(-max_sat, min(max_sat, thrust_left))
            thrust_right = max(-max_sat, min(max_sat, thrust_right))

        # 디버그: 포화 전 값 확인
        raw_left = tau_x - tau_n * 0.5
        raw_right = tau_x + tau_n * 0.5
        if not hasattr(self, '_last_motor_log') or (self.get_clock().now().nanoseconds - self._last_motor_log) > 1e9:
            self._last_motor_log = self.get_clock().now().nanoseconds
            self.get_logger().info(
                f'Motor: psi_err={psi_error:.1f}° tau_x={tau_x:.0f} tau_n={tau_n:.0f} | '
                f'raw=[{raw_left:.0f}, {raw_right:.0f}] → sat=[{thrust_left:.0f}, {thrust_right:.0f}]'
            )

        # Publish
        msg_left = Float64()
        msg_left.data = float(thrust_left)
        msg_right = Float64()
        msg_right.data = float(thrust_right)

        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

    def stop(self):
        """긴급 정지"""
        msg = Float64()
        msg.data = 0.0
        self.pub_left.publish(msg)
        self.pub_right.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
