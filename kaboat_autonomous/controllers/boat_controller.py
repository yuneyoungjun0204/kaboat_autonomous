"""
BoatController: Low-level thruster control for WAM-V
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class BoatController(Node):
    """Controls WAM-V thrusters for differential drive."""

    # Thruster configuration
    MAX_THRUST = 250.0  # Maximum thrust per thruster (N)
    THRUST_SEPARATION = 2.0  # Distance between thrusters (m)

    def __init__(self, node_name='boat_controller'):
        super().__init__(node_name)

        # Publishers for left and right thrusters
        self.left_thrust_pub = self.create_publisher(
            Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(
            Float64, '/wamv/thrusters/right/thrust', 10)

        # Subscribe to cmd_vel for high-level control
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/wamv/cmd_vel', self.cmd_vel_callback, 10)

        # Current thrust values
        self.left_thrust = 0.0
        self.right_thrust = 0.0

        self.get_logger().info('BoatController initialized')

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist to differential thrust commands."""
        linear_x = msg.linear.x  # Forward/backward
        angular_z = msg.angular.z  # Rotation

        # Differential drive mixing
        left, right = self.twist_to_thrust(linear_x, angular_z)
        self.set_thrust(left, right)

    def twist_to_thrust(self, linear: float, angular: float) -> tuple:
        """
        Convert linear/angular velocity to left/right thrust.

        Args:
            linear: Forward velocity (-1 to 1, normalized)
            angular: Angular velocity (-1 to 1, normalized, positive = CCW)

        Returns:
            (left_thrust, right_thrust) in Newtons
        """
        # Scale inputs to thrust range
        linear_thrust = linear * self.MAX_THRUST
        angular_thrust = angular * self.MAX_THRUST * 0.5

        # Differential mixing
        left = linear_thrust - angular_thrust
        right = linear_thrust + angular_thrust

        # Clamp to max thrust
        left = max(-self.MAX_THRUST, min(self.MAX_THRUST, left))
        right = max(-self.MAX_THRUST, min(self.MAX_THRUST, right))

        return left, right

    def set_thrust(self, left: float, right: float):
        """Set thrust values directly."""
        self.left_thrust = left
        self.right_thrust = right

        left_msg = Float64()
        left_msg.data = left
        right_msg = Float64()
        right_msg.data = right

        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)

    def stop(self):
        """Stop all thrusters."""
        self.set_thrust(0.0, 0.0)

    def forward(self, thrust: float = 100.0):
        """Move forward with specified thrust."""
        self.set_thrust(thrust, thrust)

    def backward(self, thrust: float = 100.0):
        """Move backward with specified thrust."""
        self.set_thrust(-thrust, -thrust)

    def turn_left(self, thrust: float = 50.0):
        """Turn left (CCW) in place."""
        self.set_thrust(-thrust, thrust)

    def turn_right(self, thrust: float = 50.0):
        """Turn right (CW) in place."""
        self.set_thrust(thrust, -thrust)


def main(args=None):
    rclpy.init(args=args)
    controller = BoatController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
