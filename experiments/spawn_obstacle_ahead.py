#!/usr/bin/env python3
"""Spawn an obstacle directly ahead of a moving boat, mid-transit.

The static trials spawned the obstacle before the goal was sent, so Nav2's very
first plan already routed around it -- nothing was ever reacted to. This injects
the obstacle only once the vessel is genuinely under way, at a fixed distance
ahead of its live pose, so the recorded data contains the detection-to-avoidance
transient.

The trigger is the boat's own motion (speed + distance travelled), not a fixed
sleep, so the spawn geometry is repeatable across runs even if startup timing
drifts.

Reads the camera_init -> Gazebo world transform the same way
dynamic_obstacle_scenario.py does: freeze it once from a simultaneous pair of
poses, then convert. The boat's Gazebo pose comes from `gz model -p` rather than
the GUI-plugin topic, which does not always load.
"""

import argparse
import csv
import math
import re
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


class AheadSpawner(Node):
    def __init__(self, args):
        super().__init__(
            'spawn_obstacle_ahead',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.pose = None          # (x, y, yaw) in camera_init
        self.speed = 0.0
        self.rows = []
        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self._on_odom, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def record(self, event, cx, cy, wx, wy, detail=''):
        self.rows.append((time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                          f'{self.sim_time():.3f}', event,
                          f'{cx:.3f}', f'{cy:.3f}',
                          f'{wx:.3f}', f'{wy:.3f}', detail))
        path = Path(self.args.out_events)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['wall_time', 'sim_time', 'event',
                        'camera_x', 'camera_y', 'world_x', 'world_y', 'detail'])
            w.writerows(self.rows)
        self.get_logger().info(
            f'{event}: camera=({cx:.2f},{cy:.2f}) world=({wx:.2f},{wy:.2f}) {detail}')

    def boat_world(self):
        """WAM-V pose in Gazebo world coords, straight from the server.

        Returns (x, y, yaw). `gz model -p` prints the XYZ line followed by an
        RPY line, so the yaw is the third number of the second block.

        This is ground truth and is the ONLY thing the spawn point may be built
        from. An earlier version placed the obstacle by converting a camera_init
        target through a transform frozen at trial start; by the time the boat
        had run 30 m the LIO frame had drifted far enough that "8 m ahead"
        landed ON the hull -- Gazebo truth showed 1.99 m centre separation and
        the box embedded beside the boat while LIO still claimed 6.72 m.
        """
        try:
            res = subprocess.run(['gz', 'model', '-m', 'wamv', '-p'],
                                 capture_output=True, text=True, timeout=25)
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warn(f'gz model query failed: {exc}')
            return None
        lines = (res.stdout + res.stderr).splitlines()
        for i, line in enumerate(lines):
            if 'XYZ (m)' not in line:
                continue
            block = []
            for follow in lines[i + 1:i + 4]:
                nums = re.findall(r'[-+]?\d+\.\d+(?:e[-+]?\d+)?', follow)
                if len(nums) >= 3:
                    block.append([float(v) for v in nums[:3]])
                if len(block) == 2:
                    break
            if len(block) == 2:
                # block[0] = XYZ, block[1] = RPY -> yaw is RPY[2]
                return block[0][0], block[0][1], block[1][2]
            if len(block) == 1:
                return block[0][0], block[0][1], None
        return None

    def gz_service(self, service, reqtype, request, timeout_ms):
        cmd = ['gz', 'service', '-s', service,
               '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
               '--timeout', str(timeout_ms), '--req', request]
        res = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=max(5.0, timeout_ms / 1000 + 2.0))
        out = (res.stdout + res.stderr).strip()
        if res.returncode != 0 or 'data: true' not in out.lower():
            raise RuntimeError(f'gz service failed ({res.returncode}): {out}')
        return out

    def spawn(self, cx, cy, wx, wy):
        sdf = str(Path(self.args.sdf).resolve())
        req = (f'name: "{self.args.model_name}", '
               f'sdf_filename: "{sdf}", allow_renaming: false, '
               f'pose: {{position: {{x: {wx:.9f}, y: {wy:.9f}, '
               f'z: {self.args.z:.6f}}}, orientation: {{w: 1.0}}}}')
        self.gz_service(f'/world/{self.args.world}/create',
                        'gz.msgs.EntityFactory', req, 5000)

    def run(self):
        # Wait for odometry (used only for the speed/travel TRIGGER) and clock.
        deadline = time.monotonic() + 30.0
        while self.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.pose is None:
            raise RuntimeError('no /aft_mapped_to_init odometry')
        while self.sim_time() <= 0.0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        bw = self.boat_world()
        if bw is None or bw[2] is None:
            raise RuntimeError('cannot read WAM-V world pose+yaw from Gazebo')
        bx, by, _ = self.pose
        self.record('start', bx, by, bw[0], bw[1],
                    f'boat_world=({bw[0]:.3f},{bw[1]:.3f}) yaw={bw[2]:.3f}; '
                    f'lio_camera=({bx:.3f},{by:.3f}); '
                    'spawn point will be computed from Gazebo truth, not LIO')

        start = (bx, by)
        self.record('waiting_for_motion', bx, by, bw[0], bw[1],
                    f'need speed>={self.args.min_speed:.2f} m/s and '
                    f'travel>={self.args.min_travel:.1f} m')

        # Trigger on the boat's own motion so spawn timing is repeatable, but
        # compute WHERE to spawn from Gazebo ground truth.
        t_end = time.monotonic() + self.args.timeout_s
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is None:
                continue
            cx, cy, _ = self.pose
            travelled = math.hypot(cx - start[0], cy - start[1])
            if self.speed < self.args.min_speed or travelled < self.args.min_travel:
                continue

            # Re-read ground truth AT the trigger instant; do not reuse the
            # startup pose or any frozen transform.
            live = self.boat_world()
            if live is None or live[2] is None:
                self.get_logger().warn('gz pose read failed at trigger; retrying')
                continue
            wbx, wby, wyaw = live
            wx = wbx + self.args.ahead * math.cos(wyaw) \
                - self.args.lateral * math.sin(wyaw)
            wy = wby + self.args.ahead * math.sin(wyaw) \
                + self.args.lateral * math.cos(wyaw)
            # Verify the intended geometry before committing the spawn, so a bad
            # pose read cannot drop a box onto the hull again.
            gap = math.hypot(wx - wbx, wy - wby)
            if gap < self.args.ahead * 0.9:
                raise RuntimeError(
                    f'computed spawn only {gap:.2f} m from the boat, expected '
                    f'{self.args.ahead:.1f} m -- refusing to spawn')

            self.record('trigger_met', cx, cy, wbx, wby,
                        f'speed={self.speed:.2f} m/s; travelled={travelled:.2f} m; '
                        f'world_yaw={wyaw:.3f} rad')
            self.spawn(0.0, 0.0, wx, wy)
            self.record('spawned', wx - wbx, wy - wby, wx, wy,
                        f'GAZEBO-TRUTH spawn {self.args.ahead:.1f} m ahead of '
                        f'boat_world=({wbx:.2f},{wby:.2f}) yaw={wyaw:.3f}; '
                        f'lateral={self.args.lateral:.1f} m; '
                        f'verified_gap={gap:.2f} m; boat_speed={self.speed:.2f} m/s '
                        '(camera_x/camera_y columns here are a world-frame offset, '
                        'not camera_init coordinates)')
            return 0
        raise RuntimeError(
            f'boat never reached speed {self.args.min_speed} m/s and travel '
            f'{self.args.min_travel} m within {self.args.timeout_s}s')


def main():
    root = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='sydney_regatta')
    ap.add_argument('--model-name', default='sudden_obstacle')
    ap.add_argument('--sdf', default=str(root / 'models/dynamic_obstacle/model.sdf'))
    ap.add_argument('--out-events', required=True)
    ap.add_argument('--ahead-m', dest='ahead', type=float, default=8.0)
    ap.add_argument('--lateral-m', dest='lateral', type=float, default=0.0)
    ap.add_argument('--min-speed', type=float, default=0.25)
    ap.add_argument('--min-travel-m', dest='min_travel', type=float, default=4.0)
    ap.add_argument('--timeout-s', type=float, default=120.0)
    ap.add_argument('--frame-yaw', type=float, default=1.0)
    ap.add_argument('--z', type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = AheadSpawner(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
