#!/usr/bin/env python3
"""Spawn and move a deterministic Gazebo obstacle in camera_init coordinates."""
import argparse
import csv
import math
import re
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


class Scenario(Node):
    def __init__(self, args):
        super().__init__(
            'dynamic_obstacle_scenario',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.boat_world = None
        self.boat_camera = None
        self.frame_translation = None
        self.rows = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Vector3, '/gz_boat_pose', self._world_cb, qos)
        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self._camera_cb, 10)

    def _world_cb(self, msg):
        self.boat_world = (msg.x, msg.y)

    def _camera_cb(self, msg):
        p = msg.pose.pose.position
        self.boat_camera = (p.x, p.y)

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def record(self, event, cx, cy, wx, wy, detail=''):
        self.rows.append((time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                          f'{self.sim_time():.3f}', event,
                          f'{cx:.3f}', f'{cy:.3f}',
                          f'{wx:.3f}', f'{wy:.3f}', detail))
        path = Path(self.args.out_events)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wall_time', 'sim_time', 'event',
                             'camera_x', 'camera_y', 'world_x', 'world_y',
                             'detail'])
            writer.writerows(self.rows)
        self.get_logger().info(f'{event}: camera=({cx:.2f},{cy:.2f}) '
                               f'world=({wx:.2f},{wy:.2f}) {detail}')

    def freeze_frame_transform(self):
        """Capture the fixed camera_init-to-Gazebo transform once per trial.

        Recomputing the translation while the boat moves makes a commanded
        camera_init obstacle path follow odometry error and vessel motion.
        """
        bx, by = rotate(self.boat_camera[0], self.boat_camera[1],
                        self.args.frame_yaw)
        self.frame_translation = (self.boat_world[0] - bx,
                                  self.boat_world[1] - by)

    def camera_to_world(self, x, y):
        if self.frame_translation is None:
            raise RuntimeError('camera-to-world transform has not been frozen')
        rx, ry = rotate(x, y, self.args.frame_yaw)
        return (rx + self.frame_translation[0],
                ry + self.frame_translation[1])

    def gz_service(self, service, reqtype, request, timeout=2000):
        cmd = ['gz', 'service', '-s', service, '--reqtype', reqtype,
               '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout),
               '--req', request]
        result = subprocess.run(cmd, text=True, capture_output=True,
                                timeout=max(5.0, timeout / 1000 + 2.0))
        output = (result.stdout + result.stderr).strip()
        if result.returncode != 0 or 'data: true' not in output.lower():
            raise RuntimeError(f'Gazebo service failed ({result.returncode}): {output}')
        return output

    def set_pose(self, cx, cy):
        wx, wy = self.camera_to_world(cx, cy)
        request = (f'name: "{self.args.model_name}", '
                   f'position: {{x: {wx:.9f}, y: {wy:.9f}, z: {self.args.z:.6f}}}, '
                   'orientation: {w: 1.0}')
        self.gz_service(f'/world/{self.args.world}/set_pose',
                        'gz.msgs.Pose', request, 1000)
        return wx, wy

    def spawn(self):
        cx, cy = self.args.start_x, self.args.start_y
        wx, wy = self.camera_to_world(cx, cy)
        sdf = str(Path(self.args.sdf).resolve())
        request = (f'name: "{self.args.model_name}", '
                   f'sdf_filename: "{sdf}", allow_renaming: false, '
                   f'pose: {{position: {{x: {wx:.9f}, y: {wy:.9f}, z: {self.args.z:.6f}}}, '
                   'orientation: {w: 1.0}}')
        self.gz_service(f'/world/{self.args.world}/create',
                        'gz.msgs.EntityFactory', request, 5000)
        self.record('spawned', cx, cy, wx, wy,
                    f'speed={self.args.speed:.3f} m/s; update_hz={self.args.update_hz:.1f}')

    def query_boat_world_from_gz(self):
        """Read the WAM-V world pose straight from Gazebo.

        /gz_boat_pose comes from the Gazebo GUI plugin, which does not always
        load (its QML has been observed failing with "plugin is not defined").
        When that happens the topic exists with zero publishers and a trial
        aborts before the obstacle is spawned.  `gz model -p` needs no GUI, so
        it is the more dependable source for a fixed pre-goal measurement.
        """
        try:
            result = subprocess.run(
                ['gz', 'model', '-m', 'wamv', '-p'],
                capture_output=True, text=True, timeout=25)
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warn(f'gz model query failed: {exc}')
            return None
        # Pose block: a "[ XYZ (m) ]" header line, then "[x y z]".
        lines = (result.stdout + result.stderr).splitlines()
        for idx, line in enumerate(lines):
            if 'XYZ (m)' not in line:
                continue
            for follow in lines[idx + 1:idx + 3]:
                nums = re.findall(r'[-+]?\d+\.\d+', follow)
                if len(nums) >= 3:
                    return float(nums[0]), float(nums[1])
        self.get_logger().warn('could not parse gz model pose output')
        return None

    def run(self):
        deadline = time.monotonic() + 30.0
        while ((self.boat_world is None or self.boat_camera is None) and
               time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.boat_world is None:
            # Fall back to Gazebo itself rather than failing the whole trial.
            self.boat_world = self.query_boat_world_from_gz()
            if self.boat_world is not None:
                self.get_logger().warn(
                    'no /gz_boat_pose publisher; using gz model query '
                    f'boat_world={self.boat_world}')
        if self.boat_world is None or self.boat_camera is None:
            raise RuntimeError('boat world/camera poses unavailable within 30 s')
        clock_deadline = time.monotonic() + 10.0
        while self.sim_time() <= 0.0 and time.monotonic() < clock_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.sim_time() <= 0.0:
            raise RuntimeError('simulation clock unavailable within 10 s')
        self.freeze_frame_transform()
        tx, ty = self.frame_translation
        wx, wy = self.camera_to_world(0.0, 0.0)
        self.record(
            'transform_frozen', 0.0, 0.0, wx, wy,
            f'tx={tx:.6f}; ty={ty:.6f}; yaw={self.args.frame_yaw:.6f}; '
            f'boat_world={self.boat_world}; boat_camera={self.boat_camera}')
        self.spawn()
        if self.args.static:
            # Static experiments only need one authoritative obstacle-center
            # sample.  Returning here avoids treating a zero-length route as
            # a dynamic motion interval and lets the caller send the goal
            # after the obstacle is known to exist.
            cx, cy = self.args.start_x, self.args.start_y
            wx, wy = self.camera_to_world(cx, cy)
            self.record('static_ready', cx, cy, wx, wy,
                        'static obstacle ready; no motion loop')
            return 0
        dx = self.args.end_x - self.args.start_x
        dy = self.args.end_y - self.args.start_y
        distance = math.hypot(dx, dy)
        duration = distance / self.args.speed if distance > 1e-6 else self.args.hold_s
        start_t = self.sim_time()
        next_update = start_t
        next_record = start_t
        self.record('motion_start', self.args.start_x, self.args.start_y,
                    *self.camera_to_world(self.args.start_x, self.args.start_y),
                    f'duration={duration:.3f} s')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = self.sim_time()
            if now < next_update:
                continue
            # A zero-length trajectory is a stationary obstacle.  It still
            # needs to be held and sampled for hold_s seconds so the event CSV
            # documents the full static interval rather than one instant.
            fraction = min(1.0, max(0.0, (now - start_t) / duration))
            cx = self.args.start_x + fraction * dx
            cy = self.args.start_y + fraction * dy
            wx, wy = self.set_pose(cx, cy)
            if now >= next_record or fraction >= 1.0:
                self.record('pose_sample', cx, cy, wx, wy,
                            f'fraction={fraction:.3f}')
                next_record += 1.0
            next_update += 1.0 / self.args.update_hz
            if fraction >= 1.0:
                self.record('motion_complete', cx, cy, wx, wy,
                            f'elapsed={now-start_t:.3f} s')
                return 0


def parse_args():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='sydney_regatta')
    parser.add_argument('--model-name', default='dynamic_obstacle_ab')
    parser.add_argument('--sdf', default=str(root / 'models/dynamic_obstacle/model.sdf'))
    parser.add_argument('--out-events', required=True)
    parser.add_argument('--start-x', type=float, default=4.0)
    parser.add_argument('--start-y', type=float, default=-3.0)
    parser.add_argument('--end-x', type=float, default=4.0)
    parser.add_argument('--end-y', type=float, default=3.0)
    parser.add_argument('--speed', type=float, default=0.4)
    parser.add_argument('--update-hz', type=float, default=5.0)
    parser.add_argument('--frame-yaw', type=float, default=1.0)
    parser.add_argument('--hold-s', type=float, default=0.0,
                        help='hold time for a zero-length trajectory')
    parser.add_argument('--static', action='store_true',
                        help='spawn once, record static_ready, and exit')
    parser.add_argument('--z', type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.speed <= 0.0 or args.update_hz <= 0.0:
        raise SystemExit('speed and update-hz must be positive')
    if args.hold_s < 0.0:
        raise SystemExit('hold-s must be non-negative')
    rclpy.init()
    node = Scenario(args)
    try:
        return node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
