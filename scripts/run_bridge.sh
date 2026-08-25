#!/bin/bash
# GZ_IP 설정 후 parameter_bridge 실행
export GZ_IP=10.22.79.185
exec ros2 run ros_gz_bridge parameter_bridge "$@"
