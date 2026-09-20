#!/usr/bin/env python3
"""Measure whether the WAM-V can actually reverse.

This decides whether REEDS_SHEPP is a legitimate planner motion model here.
Reeds-Shepp assumes the vehicle can drive backwards; if the simulated thrusters
ignore negative commands, the planner would emit reverse segments the vessel can
never execute -- a worse failure than the Dubins loop it is meant to replace.

The VRX thruster plugin declares only <max_thrust_cmd> and no min, so config
alone does not answer this. Drive the thrusters negative and watch the hull.

Bypasses Nav2 and the control bridge, so run it with the bridge stopped.
Zeroes the thrusters on exit, including on Ctrl-C.
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


class ReverseProbe(Node):
    def __init__(self):
        super().__init__('reverse_capability_probe')
        self.pose = None
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

    def set_thrust(self, v):
        m = Float64()
        m.data = float(v)
        self.pub_l.publish(m)
        self.pub_r.publish(m)

    def zero(self):
        for _ in range(12):
            self.set_thrust(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_pose(self, timeout=20.0):
        end = time.time() + timeout
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.pose is not None

    def settle(self, seconds):
        self.zero()
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_segment(self, thrust, duration, label):
        start = self.pose
        track = []
        end = time.time() + duration
        while time.time() < end:
            self.set_thrust(thrust)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None:
                track.append(self.pose)
        if len(track) < 5:
            print(f'  {label}: too few odometry samples')
            return None
        t0, x0, y0, yaw0 = track[0]
        t1, x1, y1, yaw1 = track[-1]
        dt = t1 - t0
        if dt <= 0.5:
            print(f'  {label}: no sim-time progress')
            return None
        # Displacement projected on the initial heading: negative = went astern.
        dx, dy = x1 - x0, y1 - y0
        along = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        speed = along / dt
        print(f'  {label:22s} thrust={thrust:+7.1f} N  '
              f'along-track={along:+6.2f} m  speed={speed:+.3f} m/s')
        return speed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--segment-s', type=float, default=20.0)
    ap.add_argument('--settle-s', type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = ReverseProbe()

    def bail(*_):
        node.zero()
        print('\nthrusters zeroed (interrupted)')
        sys.exit(130)

    signal.signal(signal.SIGINT, bail)
    signal.signal(signal.SIGTERM, bail)

    results = {}
    try:
        if not node.wait_pose():
            print('FAIL: no /aft_mapped_to_init odometry')
            return 2
        print('measuring forward vs reverse response '
              f'({args.segment_s:.0f}s per segment):')
        node.settle(3.0)
        results['fwd'] = node.run_segment(57.0, args.segment_s, 'forward +57N')
        node.settle(args.settle_s)
        results['rev_small'] = node.run_segment(-30.0, args.segment_s,
                                               'reverse -30N')
        node.settle(args.settle_s)
        results['rev_big'] = node.run_segment(-57.0, args.segment_s,
                                              'reverse -57N')
    finally:
        node.zero()
        print('thrusters zeroed')

    print()
    fwd = results.get('fwd')
    rs = results.get('rev_small')
    rb = results.get('rev_big')
    if fwd is None or rb is None:
        print('INCONCLUSIVE: missing measurements')
    elif rb < -0.10:
        print(f'REVERSE WORKS: -57 N drove the hull astern at {rb:.3f} m/s')
        print('=> REEDS_SHEPP is physically executable on this vessel.')
        if fwd:
            print(f'   (reverse authority is {abs(rb)/fwd*100:.0f}% of forward)')
    else:
        print(f'REVERSE DOES NOT WORK: -57 N gave {rb:+.3f} m/s '
              f'(-30 N gave {rs if rs is None else round(rs, 3)})')
        print('=> REEDS_SHEPP would plan reverse segments the hull cannot')
        print('   execute. Do NOT switch the motion model; the planner would')
        print('   commit to paths that are unfollowable in a new way.')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
