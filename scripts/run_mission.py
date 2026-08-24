#!/usr/bin/env python3
"""
KABOAT Mission Runner
Simple test script for waypoint navigation
"""
import rclpy
from kaboat_autonomous.controllers.gps_navigator import GPSNavigator


def main():
    rclpy.init()

    navigator = GPSNavigator()

    # Add test waypoints (adjust based on your simulation)
    # These are example coordinates near the spawn point
    navigator.add_waypoint(-33.7226, 150.6741, tolerance=3.0)
    navigator.add_waypoint(-33.7224, 150.6743, tolerance=3.0)
    navigator.add_waypoint(-33.7227, 150.6740, tolerance=3.0)

    print("Starting navigation to 3 waypoints...")
    navigator.start_navigation()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print("\nNavigation interrupted")
    finally:
        navigator.stop_navigation()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
