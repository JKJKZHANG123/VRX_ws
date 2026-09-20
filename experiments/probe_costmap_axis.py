#!/usr/bin/env python3
"""Read-only probe: boat pose + global costmap cost along candidate goal rays.

The static-obstacle trial keeps failing on geometry rather than control:
a goal inside the obstacle's inflation is rejected by the goal gateway, and a
goal over un-raytraced open water is rejected as unknown.  This probe reports
the actual costmap so a trial goal can be chosen from measurement.
"""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


class Probe(Node):
    def __init__(self, args):
        super().__init__('costmap_axis_probe')
        self.args = args
        self.pose = None
        self.grid = None
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self._on_odom, 10)
        self.create_subscription(Costmap, args.costmap, self._on_grid, latched)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _on_grid(self, msg):
        self.grid = msg

    def cost_at(self, x, y):
        g = self.grid
        md = g.metadata
        res = md.resolution
        ox = md.origin.position.x
        oy = md.origin.position.y
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        if col < 0 or row < 0 or col >= md.size_x or row >= md.size_y:
            return None
        return int(g.data[row * md.size_x + col])

    def worst_in_disc(self, x, y, radius):
        """Max cost within radius, mirroring the goal gateway's check."""
        md = self.grid.metadata
        res = md.resolution
        steps = int(radius / res) + 1
        worst = 0
        unknown = 0
        total = 0
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                dx, dy = i * res, j * res
                if dx * dx + dy * dy > radius * radius:
                    continue
                c = self.cost_at(x + dx, y + dy)
                total += 1
                if c is None:
                    unknown += 1
                    continue
                if c == 255:
                    unknown += 1
                    continue
                worst = max(worst, c)
        return worst, unknown, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--costmap', default='/global_costmap/costmap_raw')
    ap.add_argument('--radius', type=float, default=2.8,
                    help='goal_check_radius_m from goal_guard.yaml')
    ap.add_argument('--max-x', type=float, default=26.0)
    ap.add_argument('--step', type=float, default=1.0)
    ap.add_argument('--sweep', action='store_true',
                    help='scan bearings for the farthest acceptable goal')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args)
    deadline = time.time() + 20.0
    while time.time() < deadline and (node.pose is None or node.grid is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.pose is None:
        print('FAIL: no /aft_mapped_to_init pose')
        return 2
    if node.grid is None:
        print(f'FAIL: no costmap on {args.costmap}')
        return 3

    md = node.grid.metadata
    bx, by, byaw = node.pose
    print(f'boat_pose camera_init x={bx:.2f} y={by:.2f} yaw={byaw:.3f}')
    print(f'costmap origin=({md.origin.position.x:.2f},'
          f'{md.origin.position.y:.2f}) size={md.size_x}x{md.size_y} '
          f'res={md.resolution}')

    if args.sweep:
        print(f'{"bearing":>8} {"range":>6} {"x":>7} {"y":>7} '
              f'{"worst":>5} {"unk%":>6}')
        for deg in range(-60, 61, 15):
            rad = byaw + math.radians(deg)
            best = None
            r = 3.0
            while r <= args.max_x:
                gx = bx + r * math.cos(rad)
                gy = by + r * math.sin(rad)
                worst, unk, tot = node.worst_in_disc(gx, gy, args.radius)
                unk_pct = 100.0 * unk / max(tot, 1)
                if unk_pct == 0.0 and worst < 50:
                    best = (r, gx, gy, worst, unk_pct)
                r += 0.5
            if best:
                r, gx, gy, worst, unk_pct = best
                print(f'{deg:8d} {r:6.1f} {gx:7.2f} {gy:7.2f} '
                      f'{worst:5d} {unk_pct:6.1f}')
            else:
                print(f'{deg:8d} {"none":>6} {"-":>7} {"-":>7} '
                      f'{"-":>5} {"-":>6}')
        node.destroy_node()
        rclpy.shutdown()
        return 0
    print(f'{"x":>7} {"y":>6} {"cost":>5} {"worstR":>7} {"unk%":>6}  verdict')
    x = math.floor(bx) + 1.0
    while x <= args.max_x:
        c = node.cost_at(x, by)
        worst, unk, tot = node.worst_in_disc(x, by, args.radius)
        unk_pct = 100.0 * unk / max(tot, 1)
        if c is None:
            verdict = 'OUTSIDE'
        elif c == 255 or unk_pct > 0:
            verdict = 'REJECT unknown'
        elif worst >= 50:
            verdict = f'REJECT cost{worst}'
        else:
            verdict = 'OK'
        cs = 'None' if c is None else str(c)
        print(f'{x:7.1f} {by:6.2f} {cs:>5} {worst:7d} {unk_pct:6.1f}  {verdict}')
        x += args.step

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
