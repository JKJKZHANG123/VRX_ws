#!/usr/bin/env python3
"""Send a deterministic multi-goal waypoint sequence through the Gazebo bridge.

The script intentionally exercises the same world-click conversion used by the
Gazebo GUI: camera_init waypoints are converted to world coordinates, published
as click points, then ``navigate`` is sent. It writes an event CSV so the
trajectory CSV and action lifecycle can be joined during review.
"""
import csv
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String


def rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


class MultiGoalCommand(Node):
    def __init__(self, out_events, waypoints, spawn_yaw=1.0):
        super().__init__('m5_multigoal_command')
        self.spawn_yaw = float(spawn_yaw)
        self.waypoints = waypoints
        self.boat_world = None
        self.boat_cam = None
        self.status = 'IDLE code=0'
        self.status_time = None
        self.navigate_time = None
        self.events_path = Path(out_events)
        self.events_path.parent.mkdir(parents=True, exist_ok=True)
        self.events = []
        self.pub_point = self.create_publisher(Vector3, '/gz_click/point', 10)
        self.pub_cmd = self.create_publisher(String, '/gz_waypoint_cmd', 10)
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Vector3, '/gz_boat_pose', self.on_world, q)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_cam, 10)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, '/usv/goal_status', self.on_status,
                                 status_qos)

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def record(self, event, detail):
        row = (time.strftime('%Y-%m-%dT%H:%M:%S%z'), f'{self.sim_time():.3f}',
               event, detail)
        self.events.append(row)
        with self.events_path.open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['wall_time', 'sim_time', 'event', 'detail'])
            w.writerows(self.events)
        self.get_logger().info(f'{event}: {detail}')

    def on_world(self, msg):
        self.boat_world = (msg.x, msg.y)

    def on_cam(self, msg):
        self.boat_cam = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_status(self, msg):
        self.status = msg.data
        self.status_time = self.sim_time()
        self.record('status', msg.data)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def camera_to_world(self, x, y):
        rx, ry = rot(self.boat_cam[0], self.boat_cam[1], self.spawn_yaw)
        tx = self.boat_world[0] - rx
        ty = self.boat_world[1] - ry
        return rot(x, y, self.spawn_yaw)[0] + tx, rot(x, y, self.spawn_yaw)[1] + ty

    def publish_cmd(self, text):
        self.pub_cmd.publish(String(data=text))
        self.record('command', text)
        self.spin_for(0.8)

    def run(self, timeout_s):
        self.record('waypoints_camera_init', '; '.join(
            f'({x:.2f},{y:.2f})' for x, y in self.waypoints))
        deadline = time.monotonic() + 25.0
        while (self.boat_world is None or self.boat_cam is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.boat_world is None or self.boat_cam is None:
            self.record('error', 'boat pose not available within 25 s')
            return 2
        self.record('transform_ready',
                    f'boat_world={self.boat_world}; boat_camera={self.boat_cam}')
        self.publish_cmd('clear')
        for index, (x, y) in enumerate(self.waypoints, 1):
            wx, wy = self.camera_to_world(x, y)
            self.pub_point.publish(Vector3(x=wx, y=wy, z=0.0))
            self.record('marker', f'{index}: camera=({x:.3f},{y:.3f}) world=({wx:.3f},{wy:.3f})')
            self.spin_for(0.7)
        self.navigate_time = self.sim_time()
        self.publish_cmd('navigate')
        self.record('navigate_sent', f'{len(self.waypoints)} waypoints')
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.status_time is not None and self.status_time >= self.navigate_time and
                    any(word in self.status for word in ('SUCCEEDED', 'CANCELED', 'ABORTED', 'FAILED', 'REJECTED'))):
                self.record('terminal', self.status)
                return 0 if 'SUCCEEDED' in self.status else 1
        self.record('timeout', f'no terminal status after {timeout_s:.1f} s')
        self.publish_cmd('cancel')
        cancel_deadline = time.monotonic() + 8.0
        while time.monotonic() < cancel_deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if any(word in self.status for word in ('CANCELED', 'ABORTED', 'FAILED')):
                self.record('cancel_terminal', self.status)
                break
        return 3


def main():
    if len(sys.argv) < 2:
        print('usage: m5_multigoal_command.py EVENTS_CSV [timeout_s]', file=sys.stderr)
        return 2
    events = sys.argv[1]
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    # Safe, ordered route in camera_init; final point is the terminal target.
    waypoints = [(1.5, 0.0), (3.0, 0.5), (4.0, 0.0)]
    rclpy.init()
    node = MultiGoalCommand(events, waypoints)
    try:
        return node.run(timeout)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
