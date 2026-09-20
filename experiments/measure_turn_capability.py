#!/usr/bin/env python3
"""Measure the WAM-V's real turning capability open-loop.

Nine parameter-tuning attempts failed because every configured value was an
assumption about the hull: minimum_turning_radius 2.5 m, then 8.0 m, both
guesses. This applies a fixed differential thrust directly to the thrusters,
bypassing Nav2 and the control bridge, and records what the hull actually does.

Output: achieved yaw rate and turning radius per differential level, which is
the number minimum_turning_radius should be set from.

Publishes thrust, so it MOVES THE BOAT. It zeroes the thrusters on exit,
including on Ctrl-C.
"""

import argparse
import csv
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


class TurnProbe(Node):
    def __init__(self, args):
        super().__init__('turn_capability_probe')
        self.args = args
        self.pose = None
        self.samples = []
        self.pub_l = self.create_publisher(
            Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_r = self.create_publisher(
            Float64, '/wamv/thrusters/right/thrust', 10)
        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self._on_odom, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose = (t, p.x, p.y, yaw)

    def set_thrust(self, left, right):
        ml, mr = Float64(), Float64()
        ml.data = float(left)
        mr.data = float(right)
        self.pub_l.publish(ml)
        self.pub_r.publish(mr)

    def zero(self):
        for _ in range(10):
            self.set_thrust(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_pose(self, timeout=20.0):
        end = time.time() + timeout
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.pose is not None

    def run_segment(self, surge, diff, duration, label):
        """Hold (surge, diff) for `duration` s and report the achieved motion."""
        left = surge - diff
        right = surge + diff
        start = self.pose
        track = []
        end = time.time() + duration
        while time.time() < end:
            self.set_thrust(left, right)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None:
                track.append(self.pose)
        if len(track) < 5:
            self.get_logger().warn(f'{label}: too few odom samples')
            return None

        t0, x0, y0, yaw0 = track[0]
        t1, x1, y1, yaw1 = track[-1]
        dt = t1 - t0
        if dt <= 0.5:
            return None
        dyaw = yaw1 - yaw0
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        dist = math.hypot(x1 - x0, y1 - y0)
        wz = dyaw / dt
        vx = dist / dt
        radius = abs(vx / wz) if abs(wz) > 1e-4 else float('inf')
        row = dict(label=label, surge_n=surge, diff_n=diff, seconds=round(dt, 2),
                   travel_m=round(dist, 3), heading_deg=round(math.degrees(dyaw), 2),
                   vx_mps=round(vx, 4), wz_rps=round(wz, 5),
                   radius_m=round(radius, 2) if radius != float('inf') else '')
        self.samples.append(row)
        print(f'  {label:26s} vx={vx:5.2f} m/s  wz={wz:+.4f} rad/s  '
              f'heading={math.degrees(dyaw):+7.1f} deg  radius={row["radius_m"]} m')
        return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--surge', type=float, default=57.0,
                    help='N per thruster, ~cruise 0.6 m/s from the drag model')
    ap.add_argument('--segment-s', type=float, default=25.0)
    ap.add_argument('--settle-s', type=float, default=12.0)
    ap.add_argument('--out-csv', default='')
    args = ap.parse_args()

    rclpy.init()
    node = TurnProbe(args)

    def bail(*_):
        node.zero()
        print('\nthrusters zeroed (interrupted)')
        sys.exit(130)

    signal.signal(signal.SIGINT, bail)
    signal.signal(signal.SIGTERM, bail)

    try:
        if not node.wait_pose():
            print('FAIL: no /aft_mapped_to_init odometry')
            return 2
        print(f'surge={args.surge:.1f} N per thruster, '
              f'{args.segment_s:.0f}s per segment')
        print('measuring achieved yaw rate vs differential thrust:')
        # Differentials to sweep. max_yaw_difference_ratio caps the bridge at
        # 0.45*F ~ 25 N, so bracket that with values above and below.
        for diff in (0.0, 8.0, 14.0, 25.0, 40.0):
            node.run_segment(args.surge, diff, args.segment_s, f'diff={diff:.0f} N')
            node.zero()
            # Let the hull stop rotating so segments do not contaminate.
            end = time.time() + args.settle_s
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.zero()
        print('thrusters zeroed')

    if args.out_csv and node.samples:
        out = Path(args.out_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(node.samples[0].keys()))
            w.writeheader()
            w.writerows(node.samples)
        print(f'wrote {out}')

    turning = [s for s in node.samples if s['diff_n'] > 0 and s['radius_m'] != '']
    if turning:
        best = min(turning, key=lambda s: s['radius_m'])
        print(f'\nTIGHTEST measured radius: {best["radius_m"]} m '
              f'at diff={best["diff_n"]:.0f} N')
        print('=> this is what minimum_turning_radius should be based on.')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
