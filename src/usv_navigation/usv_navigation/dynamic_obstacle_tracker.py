#!/usr/bin/env python3
"""Detect and predict moving obstacles for the WAM-V local costmap.

This node deliberately keeps the first dynamic-obstacle implementation small
and auditable:

``raw scan -> static-map subtraction -> XY connected-component clustering
          -> nearest-track association -> alpha-beta velocity filter
          -> short-horizon prediction -> body-frame PointCloud2``

The raw scan is already water/self filtered by ``usv_cloud_filter``.  Points
that agree with the persistent ``structure_map_cloud`` are removed before
tracking, so shoreline and fixed buoys are left to the static costmap.  A
newly appearing cluster is retained briefly even before a velocity estimate is
available; this is conservative and prevents a stationary target vessel from
being ignored on its first observations.

The published prediction cloud is in ``wamv/wamv/base_link``.  Nav2 can then
use it as a local observation whose range is measured from the boat rather
than from the map origin.  The topic is published every cycle, including an
empty cloud, so expired tracks disappear deterministically.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Set, Tuple

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


XY = Tuple[float, float]


def _stamp_seconds(msg: PointCloud2, fallback: float) -> float:
    value = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    return value if value > 0.0 else fallback


def _transform_xy(point: XY, transform) -> XY:
    """Apply a geometry_msgs TransformStamped to an XY point."""
    q = transform.transform.rotation
    tx = float(transform.transform.translation.x)
    ty = float(transform.transform.translation.y)
    # Rotation matrix for yaw only is sufficient for a horizontal costmap;
    # retain the full quaternion terms so roll/pitch from boat motion do not
    # silently corrupt the x/y transform for elevated point returns.
    x, y = float(point[0]), float(point[1])
    qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
    xx = 1.0 - 2.0 * (qy * qy + qz * qz)
    xy = 2.0 * (qx * qy - qz * qw)
    yx = 2.0 * (qx * qy + qz * qw)
    yy = 1.0 - 2.0 * (qx * qx + qz * qz)
    return (tx + xx * x + xy * y,
            ty + yx * x + yy * y)


def _inverse_transform_xy(point: XY, transform) -> XY:
    """Apply the inverse of a TransformStamped to an XY point."""
    q = transform.transform.rotation
    qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
    # Inverse rotation is conjugate. Translation must be rotated as well.
    tx = float(point[0]) - float(transform.transform.translation.x)
    ty = float(point[1]) - float(transform.transform.translation.y)
    x = (1.0 - 2.0 * (qy * qy + qz * qz)) * tx + \
        (2.0 * (qx * qy + qz * qw)) * ty
    y = (2.0 * (qx * qy - qz * qw)) * tx + \
        (1.0 - 2.0 * (qx * qx + qz * qz)) * ty
    return x, y


def _grid_clusters(points: Sequence[XY], cell_size: float,
                   min_points: int, max_extent: float,
                   max_points: int) -> List[List[XY]]:
    """Cluster XY points using 8-connected occupied grid cells.

    A grid-connected component is deterministic, has no scipy/sklearn
    dependency, and is adequate for the sparse VRX LiDAR used here.
    Oversized components are discarded because they are almost always a shore
    line or a merged static structure rather than one moving vessel.
    """
    if cell_size <= 0.0:
        return []
    cells: Dict[Tuple[int, int], List[XY]] = {}
    for x, y in points:
        key = (math.floor(x / cell_size), math.floor(y / cell_size))
        cells.setdefault(key, []).append((x, y))

    unvisited: Set[Tuple[int, int]] = set(cells)
    clusters: List[List[XY]] = []
    while unvisited:
        seed = unvisited.pop()
        queue = [seed]
        component_cells = [seed]
        while queue:
            cx, cy = queue.pop()
            for nx in range(cx - 1, cx + 2):
                for ny in range(cy - 1, cy + 2):
                    neighbor = (nx, ny)
                    if neighbor in unvisited:
                        unvisited.remove(neighbor)
                        queue.append(neighbor)
                        component_cells.append(neighbor)
        cluster = [p for cell in component_cells for p in cells[cell]]
        if len(cluster) < min_points or len(cluster) > max_points:
            continue
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        if max(xs) - min(xs) > max_extent or max(ys) - min(ys) > max_extent:
            continue
        clusters.append(cluster)
    return clusters


def _centroid(cluster: Sequence[XY]) -> XY:
    return (sum(p[0] for p in cluster) / len(cluster),
            sum(p[1] for p in cluster) / len(cluster))


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    last_update: float = 0.0
    last_seen: float = 0.0
    hits: int = 1
    missed: int = 0

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


class DynamicObstacleTracker(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_tracker')

        self.declare_parameter('input_topic', '/usv/raw_obstacle_cloud')
        self.declare_parameter('static_map_topic', '/usv/structure_map_cloud')
        self.declare_parameter('output_topic', '/usv/dynamic_obstacle_cloud')
        self.declare_parameter('world_frame', 'camera_init')
        self.declare_parameter('base_frame', 'wamv/wamv/base_link')
        self.declare_parameter('publish_period_s', 0.2)
        self.declare_parameter('static_voxel_size_m', 0.75)
        self.declare_parameter('static_match_radius_m', 1.0)
        self.declare_parameter('cluster_cell_size_m', 0.8)
        self.declare_parameter('cluster_min_points', 3)
        self.declare_parameter('cluster_max_points', 250)
        self.declare_parameter('cluster_max_extent_m', 8.0)
        self.declare_parameter('track_match_distance_m', 4.0)
        self.declare_parameter('track_timeout_s', 1.5)
        self.declare_parameter('min_confirmed_hits', 2)
        self.declare_parameter('new_track_hold_s', 0.8)
        self.declare_parameter('dynamic_speed_threshold_mps', 0.15)
        self.declare_parameter('alpha', 0.65)
        self.declare_parameter('beta', 0.20)
        self.declare_parameter('max_speed_mps', 5.0)
        self.declare_parameter('prediction_horizon_s', 8.0)
        self.declare_parameter('prediction_step_s', 1.0)
        self.declare_parameter('prediction_radius_m', 1.5)
        self.declare_parameter('publish_range_m', 35.0)
        # The cloud filter bootstraps/finalizes the persistent structure map
        # for the same interval. Do not learn the bootstrap scan as dynamics.
        self.declare_parameter('startup_ignore_s', 15.0)
        # ROS 2 declares use_sim_time on the node automatically; declaring it
        # again raises ParameterAlreadyDeclaredException in Jazzy.

        p = self.get_parameter
        self.input_topic = str(p('input_topic').value)
        self.static_map_topic = str(p('static_map_topic').value)
        self.output_topic = str(p('output_topic').value)
        self.world_frame = str(p('world_frame').value)
        self.base_frame = str(p('base_frame').value)
        self.period = max(0.05, float(p('publish_period_s').value))
        self.static_voxel = max(0.05, float(p('static_voxel_size_m').value))
        self.static_match = max(0.1, float(p('static_match_radius_m').value))
        self.cluster_cell = max(0.1, float(p('cluster_cell_size_m').value))
        self.cluster_min = max(1, int(p('cluster_min_points').value))
        self.cluster_max = max(self.cluster_min, int(p('cluster_max_points').value))
        self.cluster_extent = max(0.5, float(p('cluster_max_extent_m').value))
        self.match_distance = max(0.1, float(p('track_match_distance_m').value))
        self.timeout = max(self.period, float(p('track_timeout_s').value))
        self.min_hits = max(1, int(p('min_confirmed_hits').value))
        self.new_hold = max(0.0, float(p('new_track_hold_s').value))
        self.speed_threshold = max(0.0, float(p('dynamic_speed_threshold_mps').value))
        self.alpha = min(1.0, max(0.0, float(p('alpha').value)))
        self.beta = min(1.0, max(0.0, float(p('beta').value)))
        self.max_speed = max(0.1, float(p('max_speed_mps').value))
        self.horizon = max(0.0, float(p('prediction_horizon_s').value))
        self.pred_step = max(0.1, float(p('prediction_step_s').value))
        self.pred_radius = max(0.1, float(p('prediction_radius_m').value))
        self.publish_range = max(1.0, float(p('publish_range_m').value))
        self.startup_ignore_s = max(0.0, float(p('startup_ignore_s').value))
        self._started_at = self.get_clock().now().nanoseconds * 1e-9

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self._sub = self.create_subscription(
            PointCloud2, self.input_topic, self._on_scan, qos)
        self._map_sub = self.create_subscription(
            PointCloud2, self.static_map_topic, self._on_static_map, qos)
        self._cloud_pub = self.create_publisher(
            PointCloud2, self.output_topic, qos)
        self._pose_pub = self.create_publisher(
            PoseArray, '/usv/dynamic_predictions', reliable)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/usv/dynamic_tracks_markers', reliable)
        self._metrics_pub = self.create_publisher(
            Float32MultiArray, '/usv/dynamic_metrics', reliable)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._static_cells: Set[Tuple[int, int]] = set()
        self._tracks: Dict[int, Track] = {}
        self._next_id = 1
        self._last_scan: Optional[PointCloud2] = None
        # Use the node clock for association. Sensor timestamps in the VRX
        # bridge can repeat or arrive out of order; dividing by those tiny
        # deltas produced the 5 m/s speed spikes seen in the first A/B run.
        self._last_measurement_stamp = self._started_at
        self.create_timer(self.period, self._publish)
        self.get_logger().info(
            f'dynamic tracker up: {self.input_topic} -> {self.output_topic}, '
            f'horizon={self.horizon:.1f}s, static_match={self.static_match:.1f}m')

    def _on_static_map(self, msg: PointCloud2):
        cells: Set[Tuple[int, int]] = set()
        try:
            for x, y, _z in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True):
                cells.add((math.floor(float(x) / self.static_voxel),
                           math.floor(float(y) / self.static_voxel)))
        except Exception as exc:
            self.get_logger().warn(
                f'cannot read static map cloud: {exc}',
                throttle_duration_sec=5.0)
            return
        self._static_cells = cells

    def _lookup(self, target: str, source: str):
        if target == source:
            return None
        return self._tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2))

    def _to_world(self, points: Sequence[XY], source: str) -> List[XY]:
        if source == self.world_frame:
            return list(points)
        try:
            transform = self._lookup(self.world_frame, source)
        except TransformException as exc:
            self.get_logger().warn(
                f'skip scan: TF {source}->{self.world_frame} unavailable: {exc}',
                throttle_duration_sec=3.0)
            return []
        return [_transform_xy(pt, transform) for pt in points]

    def _to_base(self, points: Sequence[XY], source: str) -> List[XY]:
        if source == self.base_frame:
            return list(points)
        try:
            transform = self._lookup(self.base_frame, source)
        except TransformException as exc:
            self.get_logger().warn(
                f'cannot publish predictions: TF {source}->{self.base_frame} unavailable: {exc}',
                throttle_duration_sec=3.0)
            return []
        return [_transform_xy(pt, transform) for pt in points]

    def _is_static(self, point: XY) -> bool:
        if not self._static_cells:
            return False
        cell_x = math.floor(point[0] / self.static_voxel)
        cell_y = math.floor(point[1] / self.static_voxel)
        radius_cells = int(math.ceil(self.static_match / self.static_voxel))
        limit_sq = self.static_match * self.static_match
        for ix in range(cell_x - radius_cells, cell_x + radius_cells + 1):
            for iy in range(cell_y - radius_cells, cell_y + radius_cells + 1):
                if (ix, iy) not in self._static_cells:
                    continue
                # Cell centres are conservative: a matching occupied voxel is
                # enough to classify the raw return as mapped structure.
                cx = (ix + 0.5) * self.static_voxel
                cy = (iy + 0.5) * self.static_voxel
                if (point[0] - cx) ** 2 + (point[1] - cy) ** 2 <= limit_sq:
                    return True
        return False

    def _on_scan(self, msg: PointCloud2):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._started_at < self.startup_ignore_s:
            # The structure-map accumulator is still learning fixed shore and
            # buoy returns. Publishing no tracks during this interval avoids
            # turning the bootstrap scan into a false dynamic target.
            self._last_scan = msg
            return
        # The scan timestamp is useful for diagnostics but is not safe for
        # velocity estimation when a bridge republishes a cloud with a stale
        # or repeated header. Associate measurements on a strictly monotonic
        # simulation-clock timeline instead.
        stamp = max(now, self._last_measurement_stamp + 1.0e-3)
        self._last_measurement_stamp = stamp
        raw: List[XY] = []
        try:
            for x, y, _z in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True):
                x, y = float(x), float(y)
                if math.isfinite(x) and math.isfinite(y):
                    raw.append((x, y))
        except Exception as exc:
            self.get_logger().warn(f'cannot read raw cloud: {exc}',
                                   throttle_duration_sec=3.0)
            return
        world = self._to_world(raw, msg.header.frame_id)
        if not world:
            self._last_scan = msg
            return
        residual = [pt for pt in world if not self._is_static(pt)]
        clusters = _grid_clusters(
            residual, self.cluster_cell, self.cluster_min,
            self.cluster_extent, self.cluster_max)
        measurements = [_centroid(cluster) for cluster in clusters]
        self._update_tracks(measurements, stamp)
        self._last_scan = msg

    def _update_tracks(self, measurements: Sequence[XY], stamp: float):
        matched: Set[int] = set()
        matched_measurements: Set[int] = set()
        pairs: List[Tuple[float, int, int, XY]] = []
        for measurement_index, measurement in enumerate(measurements):
            for track_id, track in self._tracks.items():
                dt = max(0.0, min(1.0, stamp - track.last_update))
                predicted = (track.x + track.vx * dt,
                             track.y + track.vy * dt)
                distance = math.hypot(measurement[0] - predicted[0],
                                      measurement[1] - predicted[1])
                if distance <= self.match_distance:
                    pairs.append((distance, track_id, measurement_index, measurement))
        for _distance, track_id, measurement_index, measurement in sorted(pairs):
            if track_id in matched or measurement_index in matched_measurements:
                continue
            track = self._tracks[track_id]
            dt = max(0.05, min(1.0, stamp - track.last_update))
            px = track.x + track.vx * dt
            py = track.y + track.vy * dt
            rx, ry = measurement[0] - px, measurement[1] - py
            track.x, track.y = px + self.alpha * rx, py + self.alpha * ry
            track.vx += self.beta * rx / dt
            track.vy += self.beta * ry / dt
            speed = track.speed
            if speed > self.max_speed:
                scale = self.max_speed / speed
                track.vx *= scale
                track.vy *= scale
            track.last_update = stamp
            track.last_seen = stamp
            track.hits += 1
            track.missed = 0
            matched.add(track_id)
            matched_measurements.add(measurement_index)

        for track_id, track in list(self._tracks.items()):
            if track_id not in matched:
                track.missed += 1
            if stamp - track.last_seen > self.timeout:
                del self._tracks[track_id]

        for measurement_index, measurement in enumerate(measurements):
            if measurement_index in matched_measurements:
                continue
            track_id = self._next_id
            self._next_id += 1
            self._tracks[track_id] = Track(
                track_id, measurement[0], measurement[1],
                last_update=stamp, last_seen=stamp)

    def _active_tracks(self, now: float) -> List[Track]:
        active = []
        for track in self._tracks.values():
            age = max(0.0, now - track.last_seen)
            if age > self.timeout or track.hits < self.min_hits:
                # Keep a just-created track for the configured conservative
                # hold, but do not publish arbitrary one-frame spray forever.
                if not (track.hits == 1 and age <= self.new_hold):
                    continue
            if track.speed >= self.speed_threshold or age <= self.new_hold:
                active.append(track)
        return active

    def _prediction_world(self, track: Track, now: float) -> List[XY]:
        age = max(0.0, now - track.last_update)
        x0 = track.x + track.vx * age
        y0 = track.y + track.vy * age
        points: List[XY] = []
        steps = max(0, int(math.floor(self.horizon / self.pred_step)))
        for i in range(steps + 1):
            t = min(self.horizon, i * self.pred_step)
            x, y = x0 + track.vx * t, y0 + track.vy * t
            if math.hypot(x, y) <= self.publish_range:
                points.append((x, y))
        return points

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        tracks = self._active_tracks(now)
        world_points: List[XY] = []
        for track in tracks:
            prediction = self._prediction_world(track, now)
            for x, y in prediction:
                world_points.append((x, y))
                # Put a small cross around each predicted centre. Nav2's
                # inflation handles the hull clearance; this only prevents a
                # single centre cell from disappearing at coarse resolution.
                world_points.append((x + self.pred_radius, y))
                world_points.append((x - self.pred_radius, y))
                world_points.append((x, y + self.pred_radius))
                world_points.append((x, y - self.pred_radius))
        base_points = self._to_base(world_points, self.world_frame)
        header = PointCloud2().header
        header.frame_id = self.base_frame
        # Nav2's obstacle layer uses the observation stamp for freshness. A
        # zero stamp makes predictions look permanently stale and prevents a
        # clean empty cloud from clearing the dynamic source.
        header.stamp = self.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud_xyz32(
            header, [(x, y, 0.8) for x, y in base_points])
        self._cloud_pub.publish(cloud)

        pose_array = PoseArray()
        pose_array.header = header
        for x, y in world_points:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, 0.8
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        pose_array.header.frame_id = self.world_frame
        self._pose_pub.publish(pose_array)
        self._publish_markers(tracks, now)
        max_speed = max((track.speed for track in tracks), default=0.0)
        self._metrics_pub.publish(Float32MultiArray(data=[
            float(len(tracks)), float(len(base_points)), float(max_speed),
        ]))

    def _publish_markers(self, tracks: Sequence[Track], now: float):
        array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        delete.header.frame_id = self.world_frame
        array.markers.append(delete)
        for track in tracks:
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'dynamic_obstacles'
            marker.id = track.track_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = track.x, track.y
            marker.pose.position.z = 0.8
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 2.0 * self.pred_radius
            marker.scale.z = 1.6
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.1, 0.05
            marker.color.a = 0.85
            marker.lifetime.sec = 1
            marker.lifetime.nanosec = 0
            array.markers.append(marker)
        self._marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
