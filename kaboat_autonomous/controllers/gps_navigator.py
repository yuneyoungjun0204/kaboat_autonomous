"""
GPSNavigator: GPS waypoint navigation using Pure Pursuit
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

# Earth radius in meters
EARTH_RADIUS = 6371000.0


@dataclass
class Waypoint:
    """GPS waypoint with optional tolerance."""
    latitude: float
    longitude: float
    tolerance: float = 3.0  # meters


@dataclass
class LocalPosition:
    """Local position relative to origin."""
    x: float  # East (meters)
    y: float  # North (meters)


class GPSNavigator(Node):
    """
    GPS-based waypoint navigation controller.
    Uses Pure Pursuit algorithm for path following.
    """

    def __init__(self, node_name='gps_navigator'):
        super().__init__(node_name)

        # Navigation parameters
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('max_thrust', 150.0)
        self.declare_parameter('waypoint_tolerance', 3.0)

        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_thrust = self.get_parameter('max_thrust').value
        self.tolerance = self.get_parameter('waypoint_tolerance').value

        # State
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 0.0  # radians, 0 = East, CCW positive
        self.origin_lat = None
        self.origin_lon = None

        # Waypoints
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_idx = 0

        # Publishers
        self.left_thrust_pub = self.create_publisher(
            Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(
            Float64, '/wamv/thrusters/right/thrust', 10)

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat',
            self.gps_callback,
            10)

        self.imu_sub = self.create_subscription(
            Imu,
            '/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu',
            self.imu_callback,
            10)

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Navigation state
        self.navigation_active = False

        self.get_logger().info('GPSNavigator initialized')

    def gps_callback(self, msg: NavSatFix):
        """Update current GPS position."""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Set origin on first GPS fix
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(
                f'GPS origin set: ({self.origin_lat:.6f}, {self.origin_lon:.6f})')

    def imu_callback(self, msg: Imu):
        """Update current heading from IMU."""
        # Extract yaw from quaternion
        q = msg.orientation
        self.current_heading = self.quaternion_to_yaw(q)

    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        """Convert quaternion to yaw angle (radians)."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gps_to_local(self, lat: float, lon: float) -> LocalPosition:
        """
        Convert GPS coordinates to local ENU coordinates.
        Uses equirectangular approximation (valid for short distances).
        """
        if self.origin_lat is None:
            return LocalPosition(0.0, 0.0)

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        # Equirectangular projection
        x = EARTH_RADIUS * (lon_rad - origin_lon_rad) * math.cos(origin_lat_rad)
        y = EARTH_RADIUS * (lat_rad - origin_lat_rad)

        return LocalPosition(x, y)

    def get_current_position(self) -> Optional[LocalPosition]:
        """Get current position in local coordinates."""
        if self.current_lat is None:
            return None
        return self.gps_to_local(self.current_lat, self.current_lon)

    def distance_to_waypoint(self, wp: Waypoint) -> float:
        """Calculate distance to waypoint in meters."""
        if self.current_lat is None:
            return float('inf')

        current = self.get_current_position()
        target = self.gps_to_local(wp.latitude, wp.longitude)

        dx = target.x - current.x
        dy = target.y - current.y
        return math.sqrt(dx * dx + dy * dy)

    def bearing_to_waypoint(self, wp: Waypoint) -> float:
        """Calculate bearing to waypoint (radians, 0 = East, CCW positive)."""
        if self.current_lat is None:
            return 0.0

        current = self.get_current_position()
        target = self.gps_to_local(wp.latitude, wp.longitude)

        dx = target.x - current.x
        dy = target.y - current.y
        return math.atan2(dy, dx)

    def add_waypoint(self, lat: float, lon: float, tolerance: float = 3.0):
        """Add a waypoint to the navigation queue."""
        wp = Waypoint(lat, lon, tolerance)
        self.waypoints.append(wp)
        self.get_logger().info(f'Waypoint added: ({lat:.6f}, {lon:.6f})')

    def clear_waypoints(self):
        """Clear all waypoints."""
        self.waypoints.clear()
        self.current_waypoint_idx = 0

    def start_navigation(self):
        """Start navigating to waypoints."""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to navigate to')
            return
        self.navigation_active = True
        self.current_waypoint_idx = 0
        self.get_logger().info('Navigation started')

    def stop_navigation(self):
        """Stop navigation and thrusters."""
        self.navigation_active = False
        self.set_thrust(0.0, 0.0)
        self.get_logger().info('Navigation stopped')

    def control_loop(self):
        """Main control loop for navigation."""
        if not self.navigation_active:
            return

        if self.current_lat is None:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            self.stop_navigation()
            return

        wp = self.waypoints[self.current_waypoint_idx]
        distance = self.distance_to_waypoint(wp)

        # Check if waypoint reached
        if distance < wp.tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx + 1} reached!')
            self.current_waypoint_idx += 1
            return

        # Pure Pursuit control
        bearing = self.bearing_to_waypoint(wp)
        heading_error = self.normalize_angle(bearing - self.current_heading)

        # Calculate thrust commands
        left, right = self.pure_pursuit_control(heading_error, distance)
        self.set_thrust(left, right)

    def pure_pursuit_control(self, heading_error: float, distance: float) -> Tuple[float, float]:
        """
        Pure Pursuit steering control.

        Args:
            heading_error: Angle to target (radians, positive = need to turn CCW)
            distance: Distance to target (meters)

        Returns:
            (left_thrust, right_thrust)
        """
        # Proportional control gains
        kp_angular = 100.0  # Angular gain
        kp_linear = 1.0  # Linear gain (reduced when turning)

        # Angular component (steering)
        angular = kp_angular * heading_error
        angular = max(-self.max_thrust, min(self.max_thrust, angular))

        # Linear component (forward thrust)
        # Reduce forward thrust when heading error is large
        linear_scale = math.cos(heading_error) ** 2
        linear = self.max_thrust * linear_scale * kp_linear

        # Reduce thrust when close to waypoint
        if distance < self.lookahead:
            linear *= distance / self.lookahead

        # Differential mixing
        left = linear - angular
        right = linear + angular

        # Clamp
        left = max(-self.max_thrust, min(self.max_thrust, left))
        right = max(-self.max_thrust, min(self.max_thrust, right))

        return left, right

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def set_thrust(self, left: float, right: float):
        """Publish thrust commands."""
        left_msg = Float64()
        left_msg.data = left
        right_msg = Float64()
        right_msg.data = right

        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = GPSNavigator()

    # Example waypoints (can be loaded from config)
    # navigator.add_waypoint(-33.7227, 150.6740)
    # navigator.start_navigation()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.stop_navigation()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
