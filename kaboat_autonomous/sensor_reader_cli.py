#!/usr/bin/env python3
"""
Gazebo CLI-based sensor reader for KABOAT.
Workaround for ros_gz_bridge GZ→ROS compatibility issues.
Reads sensor data directly via `gz topic -e` commands.
"""

import subprocess
import re
import json
from dataclasses import dataclass
from typing import Optional, Tuple
import threading
import time


@dataclass
class GPSData:
    latitude: float
    longitude: float
    altitude: float
    timestamp: float


@dataclass
class IMUData:
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float


class GazeboSensorReader:
    """Reads sensor data directly from Gazebo topics via CLI."""

    # Topic paths
    TOPICS = {
        'gps': '/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat',
        'imu': '/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu',
        'lidar': '/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan',
    }

    def __init__(self, timeout: float = 5.0):
        self.timeout = timeout
        self._cache = {}
        self._cache_lock = threading.Lock()

    def _run_gz_echo(self, topic: str, num_msgs: int = 1) -> Optional[str]:
        """Run gz topic -e and return output."""
        try:
            result = subprocess.run(
                ['gz', 'topic', '-e', '-t', topic, '-n', str(num_msgs)],
                capture_output=True,
                text=True,
                timeout=self.timeout
            )
            return result.stdout
        except subprocess.TimeoutExpired:
            return None
        except Exception as e:
            print(f"Error reading topic {topic}: {e}")
            return None

    def get_gps(self) -> Optional[GPSData]:
        """Read GPS data from Gazebo."""
        output = self._run_gz_echo(self.TOPICS['gps'])
        if not output:
            return None

        lat = re.search(r'latitude_deg:\s*([-\d.]+)', output)
        lon = re.search(r'longitude_deg:\s*([-\d.]+)', output)
        alt = re.search(r'altitude:\s*([-\d.]+)', output)

        if lat and lon:
            return GPSData(
                latitude=float(lat.group(1)),
                longitude=float(lon.group(1)),
                altitude=float(alt.group(1)) if alt else 0.0,
                timestamp=time.time()
            )
        return None

    def get_imu(self) -> Optional[IMUData]:
        """Read IMU data from Gazebo."""
        output = self._run_gz_echo(self.TOPICS['imu'])
        if not output:
            return None

        def extract(pattern: str) -> float:
            match = re.search(pattern, output)
            return float(match.group(1)) if match else 0.0

        return IMUData(
            orientation_x=extract(r'orientation\s*{[^}]*x:\s*([-\d.e]+)'),
            orientation_y=extract(r'orientation\s*{[^}]*y:\s*([-\d.e]+)'),
            orientation_z=extract(r'orientation\s*{[^}]*z:\s*([-\d.e]+)'),
            orientation_w=extract(r'orientation\s*{[^}]*w:\s*([-\d.e]+)'),
            angular_velocity_x=extract(r'angular_velocity\s*{[^}]*x:\s*([-\d.e]+)'),
            angular_velocity_y=extract(r'angular_velocity\s*{[^}]*y:\s*([-\d.e]+)'),
            angular_velocity_z=extract(r'angular_velocity\s*{[^}]*z:\s*([-\d.e]+)'),
            linear_acceleration_x=extract(r'linear_acceleration\s*{[^}]*x:\s*([-\d.e]+)'),
            linear_acceleration_y=extract(r'linear_acceleration\s*{[^}]*y:\s*([-\d.e]+)'),
            linear_acceleration_z=extract(r'linear_acceleration\s*{[^}]*z:\s*([-\d.e]+)'),
        )

    def get_position_heading(self) -> Optional[Tuple[float, float, float]]:
        """Get (lat, lon, heading) for navigation."""
        gps = self.get_gps()
        imu = self.get_imu()

        if not gps:
            return None

        heading = 0.0
        if imu:
            import math
            x, y, z, w = imu.orientation_x, imu.orientation_y, imu.orientation_z, imu.orientation_w
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            heading = math.atan2(siny_cosp, cosy_cosp)

        return (gps.latitude, gps.longitude, heading)


if __name__ == '__main__':
    reader = GazeboSensorReader()

    print("Reading GPS...")
    gps = reader.get_gps()
    if gps:
        print(f"  Lat: {gps.latitude}, Lon: {gps.longitude}, Alt: {gps.altitude}")
    else:
        print("  No GPS data")

    print("\nReading IMU...")
    imu = reader.get_imu()
    if imu:
        print(f"  Orientation: ({imu.orientation_x}, {imu.orientation_y}, {imu.orientation_z}, {imu.orientation_w})")
        print(f"  Angular vel: ({imu.angular_velocity_x}, {imu.angular_velocity_y}, {imu.angular_velocity_z})")
    else:
        print("  No IMU data")

    print("\nReading position + heading...")
    pos = reader.get_position_heading()
    if pos:
        import math
        print(f"  Lat: {pos[0]}, Lon: {pos[1]}, Heading: {math.degrees(pos[2]):.1f}°")
    else:
        print("  No position data")
