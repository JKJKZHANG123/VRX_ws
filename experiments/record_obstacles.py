#!/usr/bin/env python3
"""Record low-rate obstacle cloud samples without affecting Nav2 callbacks."""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ObstacleRecorder(Node):
    def __init__(self, args):
        super().__init__('obstacle_cloud_recorder')
        self.args = args
        out = Path(args.out_prefix)
        out.parent.mkdir(parents=True, exist_ok=True)
        self.file = out.with_name(out.name + '_obstacles.csv').open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t', 'source', 'point_index', 'x', 'y', 'z'])
        self.started = time.monotonic()
        self.x = self.y = self.yaw = math.nan
        self.last = {'raw': -math.inf, 'dynamic': -math.inf}
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_odom, reliable)
        self.create_subscription(PointCloud2, '/usv/raw_obstacle_cloud',
                                 lambda m: self.on_cloud('raw', m), best_effort)
        self.create_subscription(PointCloud2, '/usv/dynamic_obstacle_cloud',
                                 lambda m: self.on_cloud('dynamic', m), best_effort)
        self.get_logger().info('recording low-rate obstacle cloud samples')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.x, self.y = float(p.position.x), float(p.position.y)
        q = p.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_cloud(self, source, msg):
        now = time.monotonic() - self.started
        if now - self.last[source] < self.args.cloud_period:
            return
        self.last[source] = now
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            return
        stride = max(1, int(self.args.stride))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        try:
            for i, (bx, by, bz) in enumerate(point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)):
                if i % stride:
                    continue
                bx, by, bz = float(bx), float(by), float(bz)
                wx = self.x + c * bx - s * by
                wy = self.y + s * bx + c * by
                self.writer.writerow([f'{now:.3f}', source, i,
                                      f'{wx:.4f}', f'{wy:.4f}', f'{bz:.4f}'])
        except Exception as exc:
            self.get_logger().warning(f'cannot record {source} cloud: {exc}')
            return
        self.file.flush()

    def close(self):
        self.file.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-prefix', required=True)
    ap.add_argument('--cloud-period', type=float, default=1.0)
    ap.add_argument('--stride', type=int, default=80)
    args = ap.parse_args()
    rclpy.init()
    node = ObstacleRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
