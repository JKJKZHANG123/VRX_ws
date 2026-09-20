#!/usr/bin/env python3
"""Record Point-LIO/Nav2 perception health to a MATLAB-friendly CSV."""

import argparse
import csv
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


class PipelineRecorder(Node):
    CLOUD_TOPICS = {
        'raw': '/wamv/sensors/lidars/lidar_wamv_sensor/points',
        'filtered': '/wamv/points_filtered',
        'registered': '/cloud_registered',
        'structure': '/usv/structure_cloud',
        'raw_obstacle': '/usv/raw_obstacle_cloud',
        'safety': '/usv/safety_cloud',
    }

    def __init__(self):
        super().__init__('costmap_pipeline_recorder')
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._lock = threading.Lock()
        self.counts = {name: 0 for name in self.CLOUD_TOPICS}
        self.points = {name: 0 for name in self.CLOUD_TOPICS}
        self.zero_stamp = {name: False for name in self.CLOUD_TOPICS}
        self.cloud_stamp_s = {name: math.nan for name in self.CLOUD_TOPICS}
        self.odom_count = 0
        self.clock_count = 0
        self.costmap_count = 0
        self.obstacle_layer_count = 0
        self.clock_s = math.nan
        self.costmap_stamp_s = math.nan
        self.lio = None
        self.truth = None
        self.nonfree_cells = 0
        self.inscribed_cells = 0
        self.lethal_cells = 0
        self.obstacle_layer_nonfree_cells = 0
        self.obstacle_layer_lethal_cells = 0

        for name, topic in self.CLOUD_TOPICS.items():
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, key=name: self._on_cloud(key, msg), sensor_qos)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self._on_odom, 20)
        self.create_subscription(Vector3, '/gz_boat_pose', self._on_truth, sensor_qos)
        self.create_subscription(Clock, '/clock', self._on_clock, sensor_qos)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._on_costmap, map_qos)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/obstacle_layer',
            self._on_obstacle_layer, map_qos)

    @staticmethod
    def _stamp_s(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_cloud(self, name, msg):
        stamp_s = self._stamp_s(msg.header.stamp)
        with self._lock:
            self.counts[name] += 1
            self.points[name] = msg.width * msg.height
            self.zero_stamp[name] = stamp_s == 0.0
            self.cloud_stamp_s[name] = stamp_s

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self.odom_count += 1
            self.lio = (p.x, p.y, p.z)

    def _on_truth(self, msg):
        with self._lock:
            self.truth = (msg.x, msg.y, msg.z)

    def _on_clock(self, msg):
        with self._lock:
            self.clock_count += 1
            self.clock_s = self._stamp_s(msg.clock)

    def _on_costmap(self, msg):
        nonfree = sum(value > 0 for value in msg.data)
        inscribed = sum(value >= 99 for value in msg.data)
        lethal = sum(value >= 100 for value in msg.data)
        with self._lock:
            self.costmap_count += 1
            self.costmap_stamp_s = self._stamp_s(msg.header.stamp)
            self.nonfree_cells = nonfree
            self.inscribed_cells = inscribed
            self.lethal_cells = lethal

    def _on_obstacle_layer(self, msg):
        nonfree = sum(value > 0 for value in msg.data)
        lethal = sum(value >= 100 for value in msg.data)
        with self._lock:
            self.obstacle_layer_count += 1
            self.obstacle_layer_nonfree_cells = nonfree
            self.obstacle_layer_lethal_cells = lethal

    def snapshot(self):
        with self._lock:
            return {
                'counts': dict(self.counts),
                'points': dict(self.points),
                'zero_stamp': dict(self.zero_stamp),
                'cloud_stamp_s': dict(self.cloud_stamp_s),
                'odom_count': self.odom_count,
                'clock_count': self.clock_count,
                'costmap_count': self.costmap_count,
                'obstacle_layer_count': self.obstacle_layer_count,
                'clock_s': self.clock_s,
                'costmap_stamp_s': self.costmap_stamp_s,
                'lio': self.lio,
                'truth': self.truth,
                'nonfree_cells': self.nonfree_cells,
                'inscribed_cells': self.inscribed_cells,
                'lethal_cells': self.lethal_cells,
                'obstacle_layer_nonfree_cells':
                    self.obstacle_layer_nonfree_cells,
                'obstacle_layer_lethal_cells':
                    self.obstacle_layer_lethal_cells,
            }


def age_s(clock_s, stamp_s):
    if not math.isfinite(clock_s) or not math.isfinite(stamp_s) or stamp_s <= 0.0:
        return math.nan
    return clock_s - stamp_s


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.duration <= 0.0:
        raise SystemExit('FAIL: --duration must be positive')

    rclpy.init()
    node = PipelineRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    start = time.monotonic()
    previous_t = start
    previous = node.snapshot()
    next_sample = start + 1.0
    finish = start + args.duration

    try:
        while True:
            target = min(next_sample, finish)
            delay = target - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            now = time.monotonic()
            current = node.snapshot()
            dt = max(now - previous_t, 1e-6)
            rates = {
                name: (current['counts'][name] - previous['counts'][name]) / dt
                for name in node.CLOUD_TOPICS
            }
            odom_hz = (current['odom_count'] - previous['odom_count']) / dt
            clock_hz = (current['clock_count'] - previous['clock_count']) / dt
            costmap_hz = (
                current['costmap_count'] - previous['costmap_count']) / dt
            obstacle_layer_hz = (
                current['obstacle_layer_count'] -
                previous['obstacle_layer_count']) / dt
            lio = current['lio'] or (math.nan,) * 3
            truth = current['truth'] or (math.nan,) * 3
            safety_age = age_s(
                current['clock_s'], current['cloud_stamp_s']['safety'])
            costmap_age = age_s(
                current['clock_s'], current['costmap_stamp_s'])
            rows.append([
                now - start,
                rates['raw'], rates['filtered'], odom_hz, rates['registered'],
                rates['structure'], rates['raw_obstacle'], rates['safety'],
                clock_hz, costmap_hz, obstacle_layer_hz,
                current['points']['raw'], current['points']['filtered'],
                current['points']['registered'], current['points']['structure'],
                current['points']['raw_obstacle'], current['points']['safety'],
                current['nonfree_cells'], current['inscribed_cells'],
                current['lethal_cells'],
                current['obstacle_layer_nonfree_cells'],
                current['obstacle_layer_lethal_cells'],
                *lio, *truth,
                int(current['zero_stamp']['structure']),
                int(current['zero_stamp']['raw_obstacle']),
                int(current['zero_stamp']['safety']),
                current['clock_s'], current['cloud_stamp_s']['safety'],
                current['costmap_stamp_s'], safety_age, costmap_age,
            ])
            print(
                f"t={now-start:4.1f}s clock={clock_hz:5.1f}Hz "
                f"lio={odom_hz:5.1f}Hz reg={rates['registered']:4.1f}Hz "
                f"structure={rates['structure']:4.1f}Hz "
                f"raw_obst={rates['raw_obstacle']:4.1f}Hz "
                f"costmap={costmap_hz:3.1f}Hz "
                f"points={current['points']['structure']}/"
                f"{current['points']['raw_obstacle']} "
                f"cells={current['nonfree_cells']}/"
                f"{current['inscribed_cells']}/"
                f"{current['lethal_cells']} "
                f"raw_layer={current['obstacle_layer_lethal_cells']} "
                f"safety_age={safety_age:.3f}s",
                flush=True)
            previous_t = now
            previous = current
            if target >= finish:
                break
            next_sample += 1.0
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    header = [
        'time_s', 'raw_hz', 'filtered_hz', 'odom_hz', 'registered_hz',
        'structure_hz', 'raw_obstacle_hz', 'safety_hz', 'clock_hz',
        'local_costmap_hz', 'obstacle_layer_hz', 'raw_points',
        'filtered_points',
        'registered_points', 'structure_points', 'raw_obstacle_points',
        'safety_points', 'costmap_nonfree_cells',
        'costmap_inscribed_cells', 'costmap_lethal_cells',
        'obstacle_layer_nonfree_cells', 'obstacle_layer_lethal_cells',
        'lio_x_m', 'lio_y_m', 'lio_z_m', 'truth_x_m', 'truth_y_m',
        'truth_z_m', 'structure_stamp_zero', 'raw_obstacle_stamp_zero',
        'safety_stamp_zero', 'ros_clock_s', 'safety_stamp_s',
        'costmap_stamp_s', 'safety_age_s', 'costmap_age_s',
    ]
    with output.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)

    if not rows:
        raise SystemExit('FAIL: no samples recorded')
    print(f'SUMMARY samples={len(rows)} output={output}')


if __name__ == '__main__':
    main()
