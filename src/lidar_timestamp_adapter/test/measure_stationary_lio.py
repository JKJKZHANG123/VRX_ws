#!/usr/bin/env python3
"""Record stationary Gazebo truth and Point-LIO drift to a MATLAB-friendly CSV."""

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class StationaryRecorder(Node):
    def __init__(self):
        super().__init__('stationary_lio_recorder')
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lio = None
        self.truth = None
        self.raw_points = 0
        self.filtered_points = 0
        self.registered_points = 0
        self.create_subscription(Odometry, '/aft_mapped_to_init', self._on_lio, 20)
        self.create_subscription(Vector3, '/gz_boat_pose', self._on_truth, sensor_qos)
        self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self._on_raw,
            sensor_qos)
        self.create_subscription(
            PointCloud2, '/wamv/points_filtered', self._on_filtered, sensor_qos)
        self.create_subscription(
            PointCloud2, '/cloud_registered', self._on_registered, sensor_qos)

    def _on_lio(self, msg):
        p = msg.pose.pose.position
        self.lio = (p.x, p.y, p.z)

    def _on_truth(self, msg):
        self.truth = (msg.x, msg.y, msg.z)

    def _on_raw(self, msg):
        self.raw_points = msg.width * msg.height

    def _on_filtered(self, msg):
        self.filtered_points = msg.width * msg.height

    def _on_registered(self, msg):
        self.registered_points = msg.width * msg.height


def planar_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    rclpy.init()
    node = StationaryRecorder()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    start_wall = time.monotonic()
    # Schedule from the first synchronized sample, not from process start.
    # If Point-LIO takes a few seconds to initialize, incrementing a fixed
    # start-time deadline can emit many duplicate rows at the same wall time.
    next_sample = None
    lio0 = None
    truth0 = None
    rows = []

    while time.monotonic() - start_wall < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if node.lio is None or node.truth is None:
            continue
        if lio0 is None:
            lio0 = node.lio
            truth0 = node.truth
            next_sample = now
        if now < next_sample:
            continue
        elapsed = now - start_wall
        lio_drift = planar_distance(node.lio, lio0)
        truth_motion = planar_distance(node.truth, truth0)
        rows.append([
            elapsed, *node.truth, *node.lio, lio_drift, truth_motion,
            node.raw_points, node.filtered_points, node.registered_points,
        ])
        print(
            f't={elapsed:5.1f}s truth_motion={truth_motion:6.3f}m '
            f'lio_drift={lio_drift:6.3f}m lio_z={node.lio[2]:7.3f}m '
            f'points={node.filtered_points}',
            flush=True)
        # Start the next period from the actual sample time. This guarantees
        # at most one row per second even after delayed initialization.
        next_sample = now + 1.0

    with output.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'time_s', 'truth_x_m', 'truth_y_m', 'truth_z_m',
            'lio_x_m', 'lio_y_m', 'lio_z_m', 'lio_xy_drift_m',
            'truth_xy_motion_m', 'raw_points', 'filtered_points',
            'registered_points',
        ])
        writer.writerows(rows)

    if rows:
        print(
            f'SUMMARY samples={len(rows)} '
            f'final_lio_xy_drift={rows[-1][7]:.3f}m '
            f'max_lio_xy_drift={max(row[7] for row in rows):.3f}m '
            f'final_truth_xy_motion={rows[-1][8]:.3f}m '
            f'final_lio_z={rows[-1][6]:.3f}m output={output}')
    else:
        print('FAIL: no synchronized LIO/truth samples')
        raise SystemExit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
