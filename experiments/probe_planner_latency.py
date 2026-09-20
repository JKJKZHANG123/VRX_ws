#!/usr/bin/env python3
"""Measure ComputePathToPose responsiveness during a live run.

plan19 aborted with "Timed out while waiting for action server to acknowledge
goal request for compute_path_to_pose" -- the planner stopped answering, and the
BT gave up while the boat was already past the obstacle. That was a hypothesis
about search timeouts; this measures it instead.

For each probe it records two separate numbers, because they fail differently:
  * accept_s  -- how long the server took to ACKNOWLEDGE the goal. This is the
                 quantity the BT times out on.
  * result_s  -- how long the full plan took to come back.

Read-only with respect to navigation: it calls the planner action directly and
never publishes a goal or a velocity, so it cannot steer the boat. Run it
alongside a trial to catch the moment the planner goes unresponsive.
"""

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


class PlannerProbe(Node):
    def __init__(self, args):
        super().__init__('planner_latency_probe')
        self.args = args
        self.pose = None
        self.rows = []
        self.client = ActionClient(self, ComputePathToPose,
                                   'compute_path_to_pose')
        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self._on_odom, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def probe(self, index):
        """One planner call, timing acknowledgement and result separately."""
        goal = ComputePathToPose.Goal()
        target = PoseStamped()
        target.header.frame_id = self.args.frame
        target.pose.position.x = self.args.goal_x
        target.pose.position.y = self.args.goal_y
        target.pose.orientation.w = 1.0
        goal.goal = target
        goal.planner_id = self.args.planner_id
        goal.use_start = False

        bx, by = self.pose if self.pose else (float('nan'), float('nan'))
        t0 = time.time()
        send_future = self.client.send_goal_async(goal)
        accept_s = None
        handle = None
        while time.time() - t0 < self.args.timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if send_future.done():
                accept_s = time.time() - t0
                handle = send_future.result()
                break

        if accept_s is None:
            row = dict(probe=index, boat_x=round(bx, 2), boat_y=round(by, 2),
                       accept_s='', result_s='', poses='',
                       outcome='NO_ACK_TIMEOUT')
            self.rows.append(row)
            print(f'  probe {index:3d}  boat=({bx:6.2f},{by:6.2f})  '
                  f'*** NO ACKNOWLEDGEMENT within {self.args.timeout_s:.0f}s '
                  f'-- this is the BT abort condition ***')
            return row

        if handle is None or not handle.accepted:
            row = dict(probe=index, boat_x=round(bx, 2), boat_y=round(by, 2),
                       accept_s=round(accept_s, 3), result_s='', poses='',
                       outcome='REJECTED')
            self.rows.append(row)
            print(f'  probe {index:3d}  boat=({bx:6.2f},{by:6.2f})  '
                  f'accept={accept_s:.3f}s  REJECTED')
            return row

        res_future = handle.get_result_async()
        t1 = time.time()
        result_s = None
        while time.time() - t1 < self.args.timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if res_future.done():
                result_s = time.time() - t1
                break

        if result_s is None:
            row = dict(probe=index, boat_x=round(bx, 2), boat_y=round(by, 2),
                       accept_s=round(accept_s, 3), result_s='', poses='',
                       outcome='NO_RESULT_TIMEOUT')
            print(f'  probe {index:3d}  boat=({bx:6.2f},{by:6.2f})  '
                  f'accept={accept_s:.3f}s  *** no result in '
                  f'{self.args.timeout_s:.0f}s ***')
        else:
            npose = len(res_future.result().result.path.poses)
            outcome = 'OK' if npose > 0 else 'EMPTY_PATH'
            row = dict(probe=index, boat_x=round(bx, 2), boat_y=round(by, 2),
                       accept_s=round(accept_s, 3),
                       result_s=round(result_s, 3), poses=npose,
                       outcome=outcome)
            flag = '' if npose > 0 else '   <-- EMPTY'
            print(f'  probe {index:3d}  boat=({bx:6.2f},{by:6.2f})  '
                  f'accept={accept_s:.3f}s  result={result_s:.3f}s  '
                  f'poses={npose}{flag}')
        self.rows.append(row)
        return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal-x', type=float, default=18.0)
    ap.add_argument('--goal-y', type=float, default=0.0)
    ap.add_argument('--frame', default='camera_init')
    ap.add_argument('--planner-id', default='GridBased')
    ap.add_argument('--period-s', type=float, default=4.0,
                    help='match the BT RateController (0.25 Hz = 4 s)')
    ap.add_argument('--count', type=int, default=40)
    ap.add_argument('--timeout-s', type=float, default=20.0,
                    help='bt_navigator default_server_timeout is 20 s')
    ap.add_argument('--out-csv', default='')
    args = ap.parse_args()

    rclpy.init()
    node = PlannerProbe(args)
    if not node.client.wait_for_server(timeout_sec=15.0):
        print('FAIL: compute_path_to_pose action server not available')
        return 2
    node._spin(2.0)

    print(f'probing compute_path_to_pose every {args.period_s:.1f}s, '
          f'{args.count} times, goal=({args.goal_x},{args.goal_y})')
    for i in range(1, args.count + 1):
        node.probe(i)
        node._spin(args.period_s)

    if args.out_csv and node.rows:
        out = Path(args.out_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(node.rows[0].keys()))
            w.writeheader()
            w.writerows(node.rows)
        print(f'wrote {out}')

    good = [r for r in node.rows if r['outcome'] == 'OK']
    bad = [r for r in node.rows if r['outcome'] != 'OK']
    print()
    print(f'probes: {len(node.rows)}  ok: {len(good)}  problem: {len(bad)}')
    if good:
        acc = [r['accept_s'] for r in good]
        res = [r['result_s'] for r in good]
        print(f'  accept_s  min {min(acc):.3f}  mean {sum(acc)/len(acc):.3f}  '
              f'max {max(acc):.3f}')
        print(f'  result_s  min {min(res):.3f}  mean {sum(res)/len(res):.3f}  '
              f'max {max(res):.3f}')
    for r in bad:
        print(f'  PROBLEM probe {r["probe"]} at ({r["boat_x"]},{r["boat_y"]}): '
              f'{r["outcome"]}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
