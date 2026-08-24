"""
KABOAT Sensor Bridge Launch File
Uses ros_gz_bridge (from ros-humble-ros-gzgarden-bridge package)
for Gazebo Garden compatibility with gz-transport12.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Bridge configuration using gz.msgs (Gazebo Garden compatible)
    bridge_params = [
        # Clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # GPS
        '/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        # IMU
        '/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # LiDAR
        '/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        # Cameras
        '/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_right_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image',
        # Pose
        '/model/wamv/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        # Joint states
        '/world/kaboat_course/model/wamv/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        # Thrusters (ROS to GZ)
        '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
        '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
    ]

    # Remappings for cleaner topic names
    remappings = [
        ('/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat', '/wamv/sensors/gps/fix'),
        ('/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu', '/wamv/sensors/imu/data'),
        ('/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan', '/wamv/sensors/lidar/scan'),
        ('/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points', '/wamv/sensors/lidar/points'),
        ('/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image', '/wamv/sensors/cameras/front_left/image_raw'),
        ('/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_right_camera_sensor/image', '/wamv/sensors/cameras/front_right/image_raw'),
        ('/model/wamv/pose', '/wamv/pose'),
        ('/world/kaboat_course/model/wamv/joint_state', '/wamv/joint_states'),
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_params,
        remappings=remappings,
        output='screen'
    )

    return LaunchDescription([bridge_node])
