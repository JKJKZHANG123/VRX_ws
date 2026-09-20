#!/usr/bin/env python3
"""Report which goals the real goal gateway would accept, right now.

This imports ``_costmap_disk_status`` from ``usv_navigation.click_to_goal``
instead of reimplementing it, and subscribes to the same two OccupancyGrid
topics the gateway uses.  A goal is admissible when EITHER map passes, which
is the gateway's actual rule.  Read-only: it never publishes a goal.
"""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from usv_navigation.click_to_goal import _costmap_disk_status


class Probe(Node):
    def __init__(self, args):
        super().__init__('goal_admissibility_probe')
        self.args = args
        self.pose = None
        self.global_map = None
        self.local_map = None
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

    def admissible(self, x, y):
        """Mirror the gateway: accept if either costmap reports the disk free."""
        reasons = []
        for label, cm in (('global', self.global_map), ('local', self.local_map)):
            if cm is None or cm.header.frame_id != self.args.frame:
                continue
            ok, reason = _costmap_disk_status(
                cm, x, y, self.args.radius, self.args.threshold, True)
            if ok:
                return True, f'{label}:free'
            reasons.append(f'{label}:{reason}')
        return False, '; '.join(reasons) or 'no usable costmap'

    def stats(self, cm):
        if cm is None:
            return 'absent'
        data = cm.data
        n = len(data)
        unknown = sum(1 for v in data if v < 0)
        lethal = sum(1 for v in data if v >= 50)
        return (f'{cm.info.width}x{cm.info.height} res={cm.info.resolution} '
                f'unknown={100.0 * unknown / max(n, 1):.1f}% '
                f'cost>=50={100.0 * lethal / max(n, 1):.1f}%')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default='camera_init')
    ap.add_argument('--radius', type=float, default=2.8)
    ap.add_argument('--threshold', type=int, default=50)
    ap.add_argument('--max-range', type=float, default=24.0)
    ap.add_argument('--settle-s', type=float, default=25.0)
    ap.add_argument('--check', action='append', default=[],
                    help='explicit camera_init goal "x,y" to test; repeatable')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args)
    deadline = time.time() + args.settle_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.pose and node.global_map and node.local_map:
            if time.time() > deadline - args.settle_s + 8.0:
                break
    if node.pose is None:
        print('FAIL: no odometry')
        return 2

    bx, by, byaw = node.pose
    print(f'boat camera_init x={bx:.2f} y={by:.2f} yaw={byaw:.3f}')
    print(f'global_costmap {node.stats(node.global_map)}')
    print(f'local_costmap  {node.stats(node.local_map)}')
    if args.check:
        print()
        print(f'{"goal":>16} {"admissible":>11}  why')
        for spec in args.check:
            gx, gy = (float(v) for v in spec.split(','))
            ok, why = node.admissible(gx, gy)
            label = f'({gx:.1f},{gy:.1f})'
            print(f'{label:>16} {str(ok):>11}  {why}')

    print()
    print(f'{"bearing":>7} {"best_r":>7} {"x":>7} {"y":>7}  why')
    for deg in (0, -15, 15, -30, 30, -45, 45):
        rad = byaw + math.radians(deg)
        best = None
        r = 3.0
        while r <= args.max_range:
            gx = bx + r * math.cos(rad)
            gy = by + r * math.sin(rad)
            ok, why = node.admissible(gx, gy)
            if ok:
                best = (r, gx, gy, why)
            r += 0.5
        if best:
            r, gx, gy, why = best
            print(f'{deg:7d} {r:7.1f} {gx:7.2f} {gy:7.2f}  {why}')
        else:
            gx = bx + 8.0 * math.cos(rad)
            gy = by + 8.0 * math.sin(rad)
            _, why = node.admissible(gx, gy)
            print(f'{deg:7d} {"none":>7} {"-":>7} {"-":>7}  @8m {why}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
