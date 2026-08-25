"""
키보드(WASD)로 보트를 수동 조종하는 텔레옵 노드
motor_controller(PD 제어)를 거치지 않고 스러스터 토픽에 직접
Float64 값을 발행해 키 입력에 지연 없이 즉시 반응한다.

조작법 (누를 때마다 3단계: -값 / 0 / +값 을 즉시 오간다, 점진적 누적 없음):
    w : 전진 방향으로 한 단계 (정지 -> 전진, 후진 -> 정지)
    s : 후진 방향으로 한 단계 (정지 -> 후진, 전진 -> 정지)
    a : 좌회전 방향으로 한 단계 (제자리 좌회전: left -turn, right +turn)
    d : 우회전 방향으로 한 단계 (제자리 우회전: left +turn, right -turn)
    x / space : 즉시 정지 (전진·회전 모두 0)
    q / Ctrl+C : 종료

    left  = fwd - turn
    right = fwd + turn
"""
import sys
import os
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

FORWARD_THRUST = 2000.0  # 전/후진 한 단계 크기
TURN_THRUST = 1000.0     # 좌/우 회전 한 단계 크기
MAX_RAW_THRUST = 2354.0  # settings.py 주석 기준 VRX 스러스터 한계 (rad/s)

HELP_MSG = """
------- WASD 보트 조종 (즉시 반응, 3단계) -------
  w : 전진 단계 (정지->전진, 후진->정지)
  s : 후진 단계 (정지->후진, 전진->정지)
  a : 좌회전 단계 (제자리 좌회전 방향)
  d : 우회전 단계 (제자리 우회전 방향)
  x / space : 정지
  q : 종료 (Ctrl+C 도 가능)
--------------------------------------------------
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub_left = self.create_publisher(Float64, SETTINGS.TOPICS['thrust_left'], 10)
        self.pub_right = self.create_publisher(Float64, SETTINGS.TOPICS['thrust_right'], 10)

        self.fwd = 0.0
        self.turn = 0.0

        if not sys.stdin.isatty():
            self.get_logger().error(
                "stdin이 실제 터미널(TTY)이 아닙니다. 'ros2 launch'는 자식 프로세스의 "
                "stdin을 터미널에 연결해주지 않아 키 입력을 읽을 수 없습니다.\n"
                "별도 터미널에서 'ros2 run kaboat_autonomous keyboard_teleop'로 직접 실행하세요."
            )
            raise SystemExit(1)

        self.orig_term_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.timer = self.create_timer(0.05, self.loop)  # 20Hz
        print(HELP_MSG)

    def get_key(self) -> str:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            return sys.stdin.read(1)
        return ''

    def loop(self):
        key = self.get_key()

        if key == 'w':
            self.fwd = max(-FORWARD_THRUST, min(FORWARD_THRUST, self.fwd + FORWARD_THRUST))
        elif key == 's':
            self.fwd = max(-FORWARD_THRUST, min(FORWARD_THRUST, self.fwd - FORWARD_THRUST))
        elif key == 'a':
            self.turn = max(-TURN_THRUST, min(TURN_THRUST, self.turn + TURN_THRUST))
        elif key == 'd':
            self.turn = max(-TURN_THRUST, min(TURN_THRUST, self.turn - TURN_THRUST))
        elif key in (' ', 'x'):
            self.fwd = 0.0
            self.turn = 0.0
        elif key == 'q' or key == '\x03':
            self.publish_thrust(0.0, 0.0)
            rclpy.shutdown()
            return

        left = max(-MAX_RAW_THRUST, min(MAX_RAW_THRUST, self.fwd - self.turn))
        right = max(-MAX_RAW_THRUST, min(MAX_RAW_THRUST, self.fwd + self.turn))
        self.publish_thrust(left, right)

        if key:
            print(f'\rleft={left:7.1f}  right={right:7.1f}   ', end='', flush=True)

    def publish_thrust(self, left: float, right: float):
        msg_left = Float64()
        msg_left.data = float(left)
        msg_right = Float64()
        msg_right.data = float(right)
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_thrust(0.0, 0.0)
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
