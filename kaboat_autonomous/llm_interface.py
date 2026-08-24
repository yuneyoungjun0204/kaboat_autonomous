"""
LLM Interface for KABOAT Autonomous System
Provides high-level commands for ros-mcp-server integration
"""
import subprocess
import json
import re
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class BoatState:
    """Current boat state."""
    latitude: float
    longitude: float
    velocity_east: float
    velocity_north: float
    heading: float  # degrees from north


class LLMInterface:
    """
    Interface for LLM to control KABOAT via ros-mcp-server.

    Since ros_gz_bridge has connectivity issues, this class provides
    methods that work with both direct Gazebo access and ROS2 when available.
    """

    # Topic names
    GPS_TOPIC_GZ = "/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat"
    IMU_TOPIC_GZ = "/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu"
    LEFT_THRUST_TOPIC = "/wamv/thrusters/left/thrust"
    RIGHT_THRUST_TOPIC = "/wamv/thrusters/right/thrust"

    def __init__(self):
        self.last_state: Optional[BoatState] = None

    def get_gps_from_gazebo(self) -> Optional[BoatState]:
        """Read GPS directly from Gazebo (workaround for bridge issues)."""
        try:
            result = subprocess.run(
                ['gz', 'topic', '-e', '-t', self.GPS_TOPIC_GZ, '-n', '1'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                return self._parse_navsat(result.stdout)
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            print(f"Error reading GPS: {e}")
        return None

    def _parse_navsat(self, output: str) -> Optional[BoatState]:
        """Parse Gazebo NavSat message output."""
        try:
            lat_match = re.search(r'latitude_deg:\s*([-\d.]+)', output)
            lon_match = re.search(r'longitude_deg:\s*([-\d.]+)', output)
            vel_e_match = re.search(r'velocity_east:\s*([-\d.e+-]+)', output)
            vel_n_match = re.search(r'velocity_north:\s*([-\d.e+-]+)', output)

            if lat_match and lon_match:
                return BoatState(
                    latitude=float(lat_match.group(1)),
                    longitude=float(lon_match.group(1)),
                    velocity_east=float(vel_e_match.group(1)) if vel_e_match else 0.0,
                    velocity_north=float(vel_n_match.group(1)) if vel_n_match else 0.0,
                    heading=0.0  # TODO: get from IMU
                )
        except Exception as e:
            print(f"Error parsing NavSat: {e}")
        return None

    def get_status(self) -> dict:
        """Get current boat status for LLM context."""
        state = self.get_gps_from_gazebo()
        if state:
            self.last_state = state
            return {
                "status": "ok",
                "position": {
                    "latitude": state.latitude,
                    "longitude": state.longitude
                },
                "velocity": {
                    "east_m_s": state.velocity_east,
                    "north_m_s": state.velocity_north
                }
            }
        return {"status": "no_gps_fix"}

    @staticmethod
    def create_thrust_command(left: float, right: float) -> dict:
        """
        Create thrust command for ros-mcp-server.

        Use with mcp.publish_for_durations() or mcp.publish_once()

        Args:
            left: Left thruster value (-250 to 250 Newtons)
            right: Right thruster value (-250 to 250 Newtons)

        Returns:
            Dict with topic, msg_type, and message for ros-mcp-server
        """
        return {
            "left": {
                "topic": "/wamv/thrusters/left/thrust",
                "msg_type": "std_msgs/msg/Float64",
                "msg": {"data": max(-250, min(250, left))}
            },
            "right": {
                "topic": "/wamv/thrusters/right/thrust",
                "msg_type": "std_msgs/msg/Float64",
                "msg": {"data": max(-250, min(250, right))}
            }
        }

    @staticmethod
    def calculate_thrust_for_heading(target_heading: float, current_heading: float,
                                     forward_thrust: float = 100.0) -> Tuple[float, float]:
        """
        Calculate differential thrust to achieve target heading.

        Args:
            target_heading: Desired heading in degrees (0-360, 0=North)
            current_heading: Current heading in degrees
            forward_thrust: Base forward thrust (0-250)

        Returns:
            (left_thrust, right_thrust)
        """
        import math

        # Calculate heading error (-180 to 180)
        error = target_heading - current_heading
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        # Proportional control
        kp = 2.0
        angular_correction = kp * error
        angular_correction = max(-100, min(100, angular_correction))

        # Differential mixing
        left = forward_thrust - angular_correction
        right = forward_thrust + angular_correction

        return (max(-250, min(250, left)), max(-250, min(250, right)))


# Convenience functions for LLM use
def get_boat_status() -> dict:
    """Get current boat status."""
    interface = LLMInterface()
    return interface.get_status()


def move_forward(thrust: float = 100.0, duration: float = 2.0) -> dict:
    """
    Command to move forward.

    Returns ros-mcp-server publish_for_durations parameters.
    """
    return {
        "action": "move_forward",
        "thrust": thrust,
        "duration": duration,
        "ros_mcp_calls": [
            {
                "function": "publish_for_durations",
                "params": {
                    "topic": "/wamv/thrusters/left/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "messages": [{"data": thrust}],
                    "durations": [duration],
                    "rate_hz": 10
                }
            },
            {
                "function": "publish_for_durations",
                "params": {
                    "topic": "/wamv/thrusters/right/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "messages": [{"data": thrust}],
                    "durations": [duration],
                    "rate_hz": 10
                }
            }
        ]
    }


def turn_left(thrust: float = 80.0, duration: float = 1.0) -> dict:
    """Command to turn left (counter-clockwise)."""
    return {
        "action": "turn_left",
        "thrust": thrust,
        "duration": duration,
        "ros_mcp_calls": [
            {
                "function": "publish_for_durations",
                "params": {
                    "topic": "/wamv/thrusters/left/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "messages": [{"data": -thrust}],
                    "durations": [duration],
                    "rate_hz": 10
                }
            },
            {
                "function": "publish_for_durations",
                "params": {
                    "topic": "/wamv/thrusters/right/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "messages": [{"data": thrust}],
                    "durations": [duration],
                    "rate_hz": 10
                }
            }
        ]
    }


def turn_right(thrust: float = 80.0, duration: float = 1.0) -> dict:
    """Command to turn right (clockwise)."""
    return {
        "action": "turn_right",
        "thrust": thrust,
        "duration": duration,
        "ros_mcp_calls": [
            {
                "function": "publish_for_durations",
                "params": {
                    "topic": "/wamv/thrusters/left/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "messages": [{"data": thrust}],
                    "durations": [duration],
                    "rate_hz": 10
                }
            },
            {
                "function": "publish_for_durations",
                "params": {
                    "topic": "/wamv/thrusters/right/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "messages": [{"data": -thrust}],
                    "durations": [duration],
                    "rate_hz": 10
                }
            }
        ]
    }


def stop() -> dict:
    """Command to stop the boat."""
    return {
        "action": "stop",
        "ros_mcp_calls": [
            {
                "function": "publish_once",
                "params": {
                    "topic": "/wamv/thrusters/left/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "msg": {"data": 0.0}
                }
            },
            {
                "function": "publish_once",
                "params": {
                    "topic": "/wamv/thrusters/right/thrust",
                    "msg_type": "std_msgs/msg/Float64",
                    "msg": {"data": 0.0}
                }
            }
        ]
    }
