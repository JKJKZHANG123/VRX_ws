#!/usr/bin/env python3
"""Gate Nav2 on clean local and persistent obstacle-cloud startup data."""

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class StartupCloudGate(Node):
    def __init__(self, args):
        super().__init__(
            'startup_cloud_gate',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.rows = []
        self.clean_streak = 0
        self.passed = False
        self.monitor_bad = False
        self.monitor_seen = False
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        self.create_subscription(
            PointCloud2, args.topic, lambda msg: self.on_cloud(args.topic, msg), qos)
        if args.monitor_topic and args.monitor_topic != args.topic:
            self.create_subscription(
                PointCloud2, args.monitor_topic,
                lambda msg: self.on_cloud(args.monitor_topic, msg), qos)

    @staticmethod
    def rotate_xyz(x, y, z, q):
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        return (
            (1 - 2 * (qy * qy + qz * qz)) * x
            + 2 * (qx * qy - qz * qw) * y
            + 2 * (qx * qz + qy * qw) * z,
            2 * (qx * qy + qz * qw) * x
            + (1 - 2 * (qx * qx + qz * qz)) * y
            + 2 * (qy * qz - qx * qw) * z,
            2 * (qx * qz - qy * qw) * x
            + 2 * (qy * qz + qx * qw) * y
            + (1 - 2 * (qx * qx + qy * qy)) * z,
        )

    def to_base(self, msg, points):
        if not msg.header.frame_id or msg.header.frame_id == self.args.base_frame:
            return points
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.base_frame, msg.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.2)).transform
        except TransformException as exc:
            self.get_logger().warning(
                f'{msg.header.frame_id}->{self.args.base_frame} unavailable: {exc}')
            return None
        converted = []
        for x, y, z in points:
            rx, ry, rz = self.rotate_xyz(x, y, z, tf.rotation)
            converted.append((rx + tf.translation.x,
                              ry + tf.translation.y,
                              rz + tf.translation.z))
        return converted

    def on_cloud(self, topic, msg):
        raw_points = []
        try:
            for x, y, z in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True):
                values = (float(x), float(y), float(z))
                if all(math.isfinite(value) for value in values):
                    raw_points.append(values)
        except Exception as exc:
            self.get_logger().warning(f'cannot read {topic}: {exc}')
            return

        points = self.to_base(msg, raw_points)
        tf_ok = points is not None
        if points is None:
            points = []
        total = len(points)
        dangerous = 0
        expanded_box = 0
        nearest = math.inf
        sectors = set()
        for x, y, _z in points:
            distance = math.hypot(x, y)
            nearest = min(nearest, distance)
            in_box = (self.args.x_min <= x <= self.args.x_max
                      and abs(y) <= self.args.y_half_width)
            if in_box:
                expanded_box += 1
            if in_box or distance <= self.args.danger_radius:
                dangerous += 1
                angle = (math.atan2(y, x) + math.pi) / (2.0 * math.pi)
                sectors.add(min(self.args.sectors - 1,
                                int(angle * self.args.sectors)))

        required = topic == self.args.topic
        clean = tf_ok and dangerous <= self.args.max_danger_points
        if required:
            self.clean_streak = self.clean_streak + 1 if clean else 0
            self.passed = self.clean_streak >= self.args.clean_frames
        else:
            self.monitor_seen = True
            # A non-empty persistent cloud near the boat is unsafe even when
            # the local raw scan is clean. This was the missing failure mode.
            if not clean and total > 0:
                self.monitor_bad = True

        sim_time = self.get_clock().now().nanoseconds * 1e-9
        self.rows.append([
            time.strftime('%Y-%m-%dT%H:%M:%S%z'), f'{sim_time:.3f}', topic,
            msg.header.frame_id, int(tf_ok), total, dangerous, expanded_box,
            '' if math.isinf(nearest) else f'{nearest:.3f}', len(sectors),
            self.clean_streak if required else '',
            ('PASS' if self.passed else ('clean' if clean else 'blocked'))])
        self.write_csv()
        self.get_logger().info(
            f'{topic}: total={total}, danger={dangerous}, box={expanded_box}, '
            f'nearest={nearest:.2f} m, sectors={len(sectors)}, '
            f'tf={tf_ok}, raw_streak={self.clean_streak}/{self.args.clean_frames}')

    def write_csv(self):
        path = Path(self.args.out_csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow([
                'wall_time', 'sim_time', 'topic', 'frame_id', 'tf_ok',
                'total_points', 'danger_points', 'expanded_box_points',
                'nearest_xy_m', 'danger_sectors', 'raw_clean_streak', 'result'])
            writer.writerows(self.rows)

    def run(self):
        deadline = time.monotonic() + self.args.timeout
        while rclpy.ok() and not self.passed and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.passed and not self.monitor_bad:
            self.get_logger().info(
                'startup cloud cleanliness gate passed '
                f'(monitor_seen={self.monitor_seen})')
            return 0
        self.write_csv()
        reason = 'persistent monitor cloud is unsafe' if self.monitor_bad \
            else 'local raw cloud did not pass'
        self.get_logger().error(f'startup cloud cleanliness gate failed: {reason}')
        return 2


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/usv/raw_obstacle_cloud')
    parser.add_argument('--monitor-topic', default='/usv/structure_map_cloud')
    parser.add_argument('--base-frame', default='wamv/wamv/base_link')
    parser.add_argument('--out-csv', required=True)
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--clean-frames', type=int, default=5)
    parser.add_argument('--max-danger-points', type=int, default=20)
    parser.add_argument('--x-min', type=float, default=-3.2)
    parser.add_argument('--x-max', type=float, default=3.2)
    parser.add_argument('--y-half-width', type=float, default=2.0)
    parser.add_argument('--danger-radius', type=float, default=3.3)
    parser.add_argument('--sectors', type=int, default=16)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.clean_frames < 1 or args.sectors < 1:
        raise SystemExit('clean-frames and sectors must be positive')
    rclpy.init()
    node = StartupCloudGate(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
