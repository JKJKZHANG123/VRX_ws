#!/usr/bin/env python3
"""Record comparable Point-LIO and external robot_localization EKF data.

The recorder is deliberately passive: it never publishes commands or goals.
Each trial writes one MATLAB-friendly CSV with relative trajectories, truth,
raw sensor availability, point counts, and finite-value diagnostics.
"""
import argparse
import csv
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from std_msgs.msg import Bool


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotate_xy(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return x * c - y * s, x * s + y * c


def finite(*values):
    return all(math.isfinite(float(v)) for v in values)


class Recorder(Node):
    def __init__(self):
        super().__init__('ekf_ab_recorder')
        self.lio = None
        self.ekf = None
        self.truth = None
        self.gps = None
        self.imu = None
        self.cmd = None
        self.cmd_nonzero_seen = False
        self.safety_stop = False
        self.filtered_points = 0
        self.registered_points = 0
        self.safety_points = 0
        self.raw_obstacle_points = 0
        self.nan_count = 0
        self._last_msg_stamp = {}
        self._stamp_backwards = 0

        # Point-LIO may publish at kHz rates. KEEP_LAST(1) prevents a recorder
        # backlog from comparing an old LIO sample with a new Gazebo truth pose.
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 lambda m: self._odom('lio', m), latest)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: self._odom('ekf', m), latest)
        self.create_subscription(Vector3, '/gz_boat_pose', self._truth, latest)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix',
                                 self._gps, latest)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data',
                                 self._imu, latest)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._cmd, reliable)
        self.create_subscription(Bool, '/usv/safety_stop',
                                 lambda m: setattr(self, 'safety_stop', bool(m.data)),
                                 reliable)
        for topic, attr in (
            ('/wamv/points_filtered', 'filtered_points'),
            ('/cloud_registered', 'registered_points'),
            ('/usv/safety_cloud', 'safety_points'),
            ('/usv/raw_obstacle_cloud', 'raw_obstacle_points'),
        ):
            self.create_subscription(PointCloud2, topic,
                                     lambda m, a=attr: self._cloud(a, m), latest)

    def _check_stamp(self, key, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        old = self._last_msg_stamp.get(key)
        if old is not None and stamp < old:
            self._stamp_backwards += 1
        self._last_msg_stamp[key] = stamp

    def _odom(self, key, msg):
        self._check_stamp(key, msg)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tw = msg.twist.twist
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                tw.linear.x, tw.linear.y, tw.angular.z)
        if not finite(*vals):
            self.nan_count += 1
            return
        self.__dict__[key] = {
            'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw_from_q(q),
            'vx': tw.linear.x, 'vy': tw.linear.y, 'wz': tw.angular.z,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

    def _truth(self, msg):
        if finite(msg.x, msg.y, msg.z):
            self.truth = {'x': msg.x, 'y': msg.y, 'z': msg.z}

    def _gps(self, msg):
        if finite(msg.latitude, msg.longitude, msg.altitude):
            self.gps = {'lat': msg.latitude, 'lon': msg.longitude,
                        'alt': msg.altitude}

    def _imu(self, msg):
        v = msg.angular_velocity
        self.imu = {'gx': v.x, 'gy': v.y, 'gz': v.z}

    def _cmd(self, msg):
        self.cmd = {'vx': msg.linear.x, 'wz': msg.angular.z}
        if abs(msg.linear.x) >= 0.02 or abs(msg.angular.z) >= 0.02:
            self.cmd_nonzero_seen = True

    def _cloud(self, attr, msg):
        setattr(self, attr, int(msg.width * msg.height))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', required=True)
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--mode', choices=('static', 'running'), required=True)
    ap.add_argument('--warmup', type=float, default=5.0,
                    help='seconds after required streams/command before metric origin')
    ap.add_argument('--spawn-yaw', type=float, default=1.0,
                    help='VRX spawn yaw: world = Rz(yaw)*camera_init + translation')
    args = ap.parse_args()

    rclpy.init()
    node = Recorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    wall_start = time.monotonic()
    origin = None
    rows = []
    metric_start = None

    def rel(cur, key):
        if cur is None or origin is None or key not in origin:
            return (math.nan, math.nan, math.nan)
        o = origin[key]
        return (cur['x'] - o[0], cur['y'] - o[1], cur.get('z', 0.0) - o[2])

    try:
        while time.monotonic() - wall_start < args.duration:
            time.sleep(0.2)
            elapsed = time.monotonic() - wall_start
            required = node.lio is not None and node.truth is not None
            if args.mode == 'running' and not node.cmd_nonzero_seen:
                required = False
            if metric_start is None and required:
                metric_start = elapsed + args.warmup
            if elapsed < (metric_start or float('inf')):
                continue
            if origin is None:
                origin = {
                    'lio': (node.lio['x'], node.lio['y'], node.lio['z']),
                    'truth': (node.truth['x'], node.truth['y'], node.truth['z']),
                }
                if node.ekf is not None:
                    origin['ekf'] = (node.ekf['x'], node.ekf['y'], node.ekf['z'])

            lx, ly, lz = rel(node.lio, 'lio')
            ex, ey, ez = rel(node.ekf, 'ekf')
            twx, twy, tz = rel(node.truth, 'truth')
            tx, ty = rotate_xy(twx, twy, -args.spawn_yaw)
            ecx, ecy = rotate_xy(ex, ey, -args.spawn_yaw) if finite(ex, ey) else (math.nan, math.nan)
            lio_drift = math.hypot(lx, ly) if finite(lx, ly) else math.nan
            ekf_drift = math.hypot(ex, ey) if finite(ex, ey) else math.nan
            truth_motion = math.hypot(tx, ty) if finite(tx, ty) else math.nan
            lio_err = math.hypot(lx - tx, ly - ty) if finite(lx, ly, tx, ty) else math.nan
            # EKF world_frame=odom is ENU-aligned; compare its raw relative XY
            # against Gazebo world truth. The camera-frame EKF columns are only
            # for overlaying all trajectories in one MATLAB axes.
            ekf_err = math.hypot(ex - twx, ey - twy) if finite(ex, ey, twx, twy) else math.nan
            row = {
                'elapsed_s': elapsed,
                'mode': args.mode,
                'spawn_yaw_rad': args.spawn_yaw,
                'lio_x_rel_m': lx, 'lio_y_rel_m': ly, 'lio_z_rel_m': lz,
                'ekf_x_rel_m': ex, 'ekf_y_rel_m': ey, 'ekf_z_rel_m': ez,
                'ekf_camera_x_rel_m': ecx, 'ekf_camera_y_rel_m': ecy,
                'truth_x_rel_m': tx, 'truth_y_rel_m': ty, 'truth_z_rel_m': tz,
                'truth_world_x_rel_m': twx, 'truth_world_y_rel_m': twy,
                'lio_xy_drift_m': lio_drift, 'ekf_xy_drift_m': ekf_drift,
                'truth_xy_motion_m': truth_motion,
                'lio_truth_xy_error_m': lio_err,
                'ekf_truth_xy_error_m': ekf_err,
                'lio_yaw_rad': node.lio['yaw'],
                'ekf_yaw_rad': node.ekf['yaw'] if node.ekf else math.nan,
                'lio_vx_mps': node.lio['vx'], 'lio_vy_mps': node.lio['vy'],
                'lio_wz_radps': node.lio['wz'],
                'ekf_vx_mps': node.ekf['vx'] if node.ekf else math.nan,
                'ekf_vy_mps': node.ekf['vy'] if node.ekf else math.nan,
                'ekf_wz_radps': node.ekf['wz'] if node.ekf else math.nan,
                'cmd_vx_mps': node.cmd['vx'] if node.cmd else 0.0,
                'cmd_wz_radps': node.cmd['wz'] if node.cmd else 0.0,
                'gps_lat': node.gps['lat'] if node.gps else math.nan,
                'gps_lon': node.gps['lon'] if node.gps else math.nan,
                'imu_gz_radps': node.imu['gz'] if node.imu else math.nan,
                'filtered_points': node.filtered_points,
                'registered_points': node.registered_points,
                'safety_cloud_points': node.safety_points,
                'raw_obstacle_points': node.raw_obstacle_points,
                'safety_stop': int(node.safety_stop),
                'nan_count': node.nan_count,
                'stamp_backwards': node._stamp_backwards,
            }
            rows.append(row)
            print(f"[{args.mode}] t={elapsed:5.1f}s truth={truth_motion:6.2f}m "
                  f"LIO={lio_drift:6.2f}m EKF={ekf_drift:6.2f}m "
                  f"err(L/E)={lio_err:5.2f}/{ekf_err:5.2f}", flush=True)
    finally:
        fields = list(rows[0].keys()) if rows else [
            'elapsed_s', 'mode', 'spawn_yaw_rad', 'lio_x_rel_m', 'lio_y_rel_m',
            'lio_z_rel_m', 'ekf_x_rel_m', 'ekf_y_rel_m', 'ekf_z_rel_m',
            'ekf_camera_x_rel_m', 'ekf_camera_y_rel_m', 'truth_x_rel_m',
            'truth_y_rel_m', 'truth_z_rel_m', 'truth_world_x_rel_m',
            'truth_world_y_rel_m', 'lio_xy_drift_m', 'ekf_xy_drift_m',
            'truth_xy_motion_m', 'lio_truth_xy_error_m', 'ekf_truth_xy_error_m']
        with out.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print(f'WROTE {len(rows)} rows -> {out}')


if __name__ == '__main__':
    main()
