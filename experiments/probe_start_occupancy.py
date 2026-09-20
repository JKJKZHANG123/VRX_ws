#!/usr/bin/env python3
"""Diagnose a Smac "Start occupied" abort.

Reports, for the boat's current pose: the costmap cost under the hull, and how
many points of the costmap's own observation source (``/usv/raw_obstacle_cloud``,
already in base_link) fall inside the hull envelope.  Self-returns that survive
the cloud filter's self-mask mark the boat's own cell and make every plan fail
at the start, which no goal choice can work around.
"""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Probe(Node):
    def __init__(self):
        super().__init__('start_occupancy_probe')
        self.pose = None
        self.global_map = None
        self.local_map = None
        self.cloud = None
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_global, qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._on_local, qos)
        # obstacle_filter publishes BEST_EFFORT/VOLATILE; a RELIABLE
        # subscription is QoS-incompatible and silently receives nothing.
        cloud_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, '/usv/raw_obstacle_cloud',
                                 self._on_cloud, cloud_qos)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _on_global(self, msg):
        self.global_map = msg

    def _on_local(self, msg):
        self.local_map = msg

    def _on_cloud(self, msg):
        self.cloud = msg

    @staticmethod
    def cost_at(cm, x, y):
        if cm is None:
            return None
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        col = int(math.floor((x - ox) / res))
        row = int(math.floor((y - oy) / res))
        if col < 0 or row < 0 or col >= cm.info.width or row >= cm.info.height:
            return None
        return int(cm.data[row * cm.info.width + col])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--settle-s', type=float, default=15.0)
    # WAM-V collision hull from nav2_params footprint.
    ap.add_argument('--hull-min-x', type=float, default=-2.8)
    ap.add_argument('--hull-max-x', type=float, default=3.1)
    ap.add_argument('--hull-half-y', type=float, default=1.6)
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    end = time.time() + args.settle_s
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.pose and node.global_map and node.local_map and node.cloud:
            break
    if node.pose is None:
        print('FAIL: no odometry')
        return 2

    bx, by, byaw = node.pose
    print(f'boat camera_init x={bx:.2f} y={by:.2f} yaw={byaw:.3f}')

    for label, cm in (('global', node.global_map), ('local', node.local_map)):
        c = node.cost_at(cm, bx, by)
        print(f'{label}_costmap cost_under_hull_centre={c}')

    # Worst cost anywhere under the hull rectangle, in camera_init.
    for label, cm in (('global', node.global_map), ('local', node.local_map)):
        if cm is None:
            continue
        worst = -1
        worst_at = None
        step = cm.info.resolution * 0.5
        x = args.hull_min_x
        while x <= args.hull_max_x:
            y = -args.hull_half_y
            while y <= args.hull_half_y:
                gx = bx + x * math.cos(byaw) - y * math.sin(byaw)
                gy = by + x * math.sin(byaw) + y * math.cos(byaw)
                c = node.cost_at(cm, gx, gy)
                if c is not None and c > worst:
                    worst, worst_at = c, (x, y)
                y += step
            x += step
        print(f'{label}_costmap worst_cost_under_hull={worst} '
              f'at_body_xy={worst_at}')

    if node.cloud is None:
        print('no /usv/raw_obstacle_cloud received')
    else:
        pts = list(point_cloud2.read_points(
            node.cloud, field_names=('x', 'y', 'z'), skip_nans=True))
        inside = [(float(p[0]), float(p[1]), float(p[2])) for p in pts
                  if args.hull_min_x <= float(p[0]) <= args.hull_max_x
                  and abs(float(p[1])) <= args.hull_half_y]
        print(f'raw_obstacle_cloud frame={node.cloud.header.frame_id} '
              f'total={len(pts)} inside_hull={len(inside)}')
        if inside:
            xs = [p[0] for p in inside]
            ys = [p[1] for p in inside]
            zs = [p[2] for p in inside]
            print(f'  SELF-RETURNS PRESENT: x={min(xs):.2f}..{max(xs):.2f} '
                  f'y={min(ys):.2f}..{max(ys):.2f} z={min(zs):.2f}..{max(zs):.2f}')
            for p in inside[:10]:
                print(f'   sample x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
