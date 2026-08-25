#!/bin/bash
# KABOAT 통합 시각화 실행 스크립트

echo "=== KABOAT Integrated Visualizer ==="
echo ""
echo "왼쪽: Global Map (클릭하여 목적지 설정)"
echo "오른쪽: Polar Map (LiDAR, 명령, 안전 구역)"
echo ""

source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash

ros2 run kaboat_autonomous integrated_visualizer
