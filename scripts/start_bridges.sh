#!/bin/bash
# Start sensor bridges for KABOAT simulation

echo "Starting rosbridge and rosapi..."
export PATH="/usr/bin:/opt/ros/humble/bin:$PATH"

# Start rosapi
nohup /usr/bin/python3 /opt/ros/humble/lib/rosapi/rosapi_node \
    --ros-args -r __node:=rosapi > /tmp/rosapi.log 2>&1 &

# Start rosbridge
nohup /usr/bin/python3 /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket \
    --ros-args -p port:=9090 > /tmp/rosbridge.log 2>&1 &

sleep 2

echo "Starting sensor bridges..."
ros2 run ros_gz_bridge parameter_bridge \
    "/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat" \
    "/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU" \
    "/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
    "/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
    "/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_right_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/middle_right_camera_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double" \
    "/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double" \
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
    &

echo "Bridges started. Releasing WAM-V from platform..."
sleep 3
gz topic -t /vrx/release -m gz.msgs.Empty -p ''

echo "Setup complete!"
echo "rosbridge: ws://localhost:9090"
echo "Topics: ros2 topic list"
