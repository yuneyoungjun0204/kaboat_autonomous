#!/usr/bin/env python3
"""
LiDAR 클러스터링 RViz 평가용 노드 (실험용, 메인 LLM 파이프라인과 분리)

기본은 detect_lidar_clusters_3d()(3D 포인트클라우드 기반)의 raw 결과와
ClusterTracker로 안정화한 결과를 MarkerArray로 발행해 RViz에서 비교하는
것이지만, PointCloud2가 CLUSTER3D_STALE_SEC 이상 안 들어오면(브리지 누락,
드라이버 문제 등) 2D LaserScan 기반 detect_lidar_clusters()로 자동 폴백한다.
분석용으로만 쓰이며, action_dispatcher/sensor_fusion 등 실제 미션
파이프라인에는 영향을 주지 않는다.

구독:
  /wamv/sensors/lidar/points (PointCloud2) - 3D 원본 (기본)
  /wamv/sensors/lidar/scan (LaserScan) - 2D 폴백용

발행:
  /lidar_clusters_markers (visualization_msgs/MarkerArray)
    - 3D 모드: 노란/초록 구 (raw/안정화)
    - 2D 폴백 모드: 주황/청록 구 (raw/안정화) - 색을 달리해 지금 어느
      소스로 동작 중인지 RViz에서 바로 구분 가능
  /lidar_clusters (std_msgs/String)
    - {'mode': '3d'|'2d_fallback', 'raw': [...], 'stable': [...]} JSON

RViz 설정:
  - Fixed Frame: 이 노드가 시작 시 로그로 출력하는 frame_id로 설정
  - Add -> By topic -> /wamv/sensors/lidar/points (PointCloud2)
  - Add -> By topic -> /lidar_clusters_markers (MarkerArray)

주의: /wamv/sensors/lidar/points가 브리지 안 되어 있으면 2D 폴백으로
     계속 동작은 하지만, 그 사실을 로그(WARN, 최초 1회)로 알려준다.
     원인 파악은 `ros2 topic list`로 먼저 확인할 것.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import json
import time
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from config import settings as SETTINGS
except ImportError:
    sys.path.append('/home/yune/vrx_ws/src/kaboat_autonomous')
    from config import settings as SETTINGS

from kaboat_autonomous.controllers.autonomous_module import Boat
from kaboat_autonomous.controllers import maneuvers


class ClusterVisualizer(Node):
    """raw/안정화 LiDAR 클러스터를 RViz MarkerArray로 발행 (3D 기본, 2D 폴백)"""

    def __init__(self):
        super().__init__('cluster_visualizer')

        self.boat = Boat()
        self.tracker_3d = maneuvers.ClusterTracker()
        self.tracker_2d = maneuvers.ClusterTracker()
        self._logged_3d_frame = False
        self._logged_2d_frame = False
        self._last_3d_time = None  # None = 3D 아직 한 번도 안 들어옴
        self._in_fallback = False

        self.marker_pub = self.create_publisher(MarkerArray, '/lidar_clusters_markers', 10)
        self.json_pub = self.create_publisher(String, '/lidar_clusters', 10)
        self.create_subscription(PointCloud2, '/wamv/sensors/lidar/points', self.points_callback, 10)
        self.create_subscription(LaserScan, '/wamv/sensors/lidar/scan', self.scan_callback, 10)

        self.get_logger().info('=== Cluster Visualizer Started (3D 기본 + 2D 폴백, 실험용) ===')
        self.get_logger().info('구독: /wamv/sensors/lidar/points (3D), /wamv/sensors/lidar/scan (2D 폴백)')

    def _is_3d_stale(self) -> bool:
        return self._last_3d_time is None or (time.time() - self._last_3d_time) > SETTINGS.CLUSTER3D_STALE_SEC

    def points_callback(self, msg: PointCloud2):
        if not self._logged_3d_frame:
            self.get_logger().info(f"pointcloud frame_id='{msg.header.frame_id}' -> RViz Fixed Frame을 이 값으로 설정")
            self._logged_3d_frame = True

        if self._in_fallback:
            self.get_logger().info('PointCloud2 복구됨 - 3D 클러스터링으로 복귀')
            self._in_fallback = False
        self._last_3d_time = time.time()

        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if len(points) == 0:
            return
        xyz = np.column_stack([
            points['x'].astype(np.float64).ravel(),
            points['y'].astype(np.float64).ravel(),
            points['z'].astype(np.float64).ravel(),
        ])

        raw_clusters = maneuvers.detect_lidar_clusters_3d(xyz)
        stable_clusters = self.tracker_3d.update(raw_clusters)

        self.publish_markers(raw_clusters, stable_clusters, msg.header.frame_id, msg.header.stamp, mode='3d')
        self.json_pub.publish(String(data=json.dumps(
            {'mode': '3d', 'raw': raw_clusters, 'stable': stable_clusters})))

    def scan_callback(self, msg: LaserScan):
        if not self._is_3d_stale():
            return  # 3D가 정상 수신 중이면 2D는 무시 (3D 우선)

        if not self._in_fallback:
            self.get_logger().warn(
                f'PointCloud2가 {SETTINGS.CLUSTER3D_STALE_SEC}초 이상 안 들어옴 - '
                '2D LaserScan 클러스터링으로 폴백 (브리지/토픽 확인 필요할 수 있음)')
            self._in_fallback = True

        if not self._logged_2d_frame:
            self.get_logger().info(f"laserscan frame_id='{msg.header.frame_id}' -> RViz Fixed Frame을 이 값으로 설정 (2D 폴백 중)")
            self._logged_2d_frame = True

        # 나머지 2D 노드들과 동일한 전처리
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)
        if len(ranges) != 360:
            indices = np.linspace(0, len(ranges) - 1, 360).astype(int)
            ranges = ranges[indices]
        ranges = np.roll(ranges, 180)
        ranges[(ranges > 0) & (ranges < SETTINGS.MIN_VALID_RANGE)] = 0
        ranges[ranges > SETTINGS.LIDAR_MAX_RANGE] = 0
        self.boat.scan = ranges.tolist()

        raw_clusters = maneuvers.detect_lidar_clusters(self.boat)
        stable_clusters = self.tracker_2d.update(raw_clusters)

        self.publish_markers(raw_clusters, stable_clusters, msg.header.frame_id, msg.header.stamp, mode='2d_fallback')
        self.json_pub.publish(String(data=json.dumps(
            {'mode': '2d_fallback', 'raw': raw_clusters, 'stable': stable_clusters})))

    def publish_markers(self, raw_clusters, stable_clusters, frame_id, stamp, mode):
        arr = MarkerArray()

        clear = Marker()
        clear.header.frame_id = frame_id
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        if mode == '3d':
            raw_color, stable_color = (1.0, 1.0, 0.0, 0.5), (0.0, 1.0, 0.0, 0.9)   # 노랑/초록
        else:
            raw_color, stable_color = (1.0, 0.6, 0.0, 0.5), (0.0, 1.0, 1.0, 0.9)   # 주황/청록 (폴백 표시)

        marker_id = 0
        marker_id = self._add_cluster_markers(
            arr, raw_clusters, frame_id, stamp, marker_id,
            ns=f'{mode}_raw', color=raw_color, radius_scale=0.5)
        marker_id = self._add_cluster_markers(
            arr, stable_clusters, frame_id, stamp, marker_id,
            ns=f'{mode}_stable', color=stable_color, radius_scale=1.0)

        self.marker_pub.publish(arr)

    def _add_cluster_markers(self, arr, clusters, frame_id, stamp, marker_id, ns, color, radius_scale):
        for c in clusters:
            angle_rad = np.radians(c['center_angle'])
            x = c['distance'] * np.cos(angle_rad)
            y = c['distance'] * np.sin(angle_rad)
            z = c.get('z', 0.0)

            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = ns
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=x, y=y, z=z)
            sphere.pose.orientation.w = 1.0
            sz = max(0.5, min(c['width_deg'] / 10.0, 4.0)) * radius_scale
            sphere.scale.x = sz
            sphere.scale.y = sz
            sphere.scale.z = sz
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            sphere.lifetime.sec = 1
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = stamp
            text.ns = ns + '_text'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=x, y=y, z=z + 1.0)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.8
            text.color.r, text.color.g, text.color.b, text.color.a = color
            text.text = f"{c['distance']:.1f}m z={z:.1f} pts={c['point_count']}"
            text.lifetime.sec = 1
            arr.markers.append(text)

        return marker_id


def main(args=None):
    rclpy.init(args=args)
    node = ClusterVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
