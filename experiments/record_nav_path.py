#!/usr/bin/env python3
"""Record Nav2 path snapshots, odometry trajectory, and obstacle points.

The normal nav_data_logger stores only the latest path length and clearance.
This recorder keeps the actual geometry of every ``/plan`` message so a run
with an obstacle can be plotted as: planned path(s) + measured trajectory +
observed obstacle returns.
"""

import argparse
import csv
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener


class NavPathRecorder(Node):
    def __init__(self, args):
        super().__init__('nav_path_recorder')
        self.args = args
        out = Path(args.out_prefix)
        out.parent.mkdir(parents=True, exist_ok=True)
        self.traj_file = out.with_name(out.name + '_trajectory.csv')
        self.plan_file = out.with_name(out.name + '_plans.csv')
        self.obstacle_file = out.with_name(out.name + '_obstacles.csv')
        self.record_clouds = bool(args.record_clouds)
        self.event_file = out.with_name(out.name + '_events.csv')
        self.traj = self.traj_file.open('w', newline='')
        self.plans = self.plan_file.open('w', newline='')
        self.obstacles = (self.obstacle_file.open('w', newline='')
                          if self.record_clouds else None)
        self.events = self.event_file.open('w', newline='')
        self.tw = csv.writer(self.traj)
        self.pw = csv.writer(self.plans)
        self.ow = csv.writer(self.obstacles) if self.obstacles else None
        self.ew = csv.writer(self.events)
        self.tw.writerow([
            't', 'pose_x', 'pose_y', 'pose_yaw', 'goal_x', 'goal_y',
            'goal_active', 'goal_status', 'plan_id', 'plan_nearest_error_m',
            'raw_obstacle_points', 'dynamic_obstacle_points', 'dynamic_tracks',
            'dynamic_predicted_points', 'dynamic_max_speed_mps', 'safety_stop',
            'cmd_vx', 'cmd_wz', 'left_thrust_n', 'right_thrust_n',
        ])
        self.pw.writerow([
            'plan_id', 't', 'source', 'frame_id', 'point_index',
            'x', 'y', 'yaw', 'raw_x', 'raw_y', 'raw_yaw'])
        if self.ow:
            self.ow.writerow(['t', 'source', 'point_index', 'x', 'y', 'z'])
        self.ew.writerow(['wall_time', 't', 'event', 'detail'])

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_odom, reliable)
        # Record planner and controller-facing path variants.  Keeping the
        # source column makes it possible to distinguish the raw planner
        # output from the smoothed path used by the controller.
        # Only /plan and /unsmoothed_plan are genuine planner output. RPP
        # republishes /received_global_plan at its 10 Hz control rate -- it is the
        # SAME path echoed back, not a replan. Counting those as plan snapshots
        # inflated the figure ~35x (plan19: 335 reported vs 9 real replans) and
        # made a healthy 0.25 Hz replan cadence look like runaway replanning.
        self.planner_sources = ('/plan', '/unsmoothed_plan')
        for topic in ('/plan', '/unsmoothed_plan', '/received_global_plan', '/plan_smoothed'):
            self.create_subscription(
                NavPath, topic, lambda m, topic=topic: self.on_plan(topic, m), reliable)
        if self.record_clouds:
            self.create_subscription(PointCloud2, '/usv/raw_obstacle_cloud',
                                     lambda m: self.on_cloud('raw', m), qos)
            self.create_subscription(PointCloud2, '/usv/dynamic_obstacle_cloud',
                                     lambda m: self.on_cloud('dynamic', m), qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, reliable)
        status_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, '/usv/goal_status', self.on_status, status_qos)
        self.create_subscription(Bool, '/usv/safety_stop', self.on_safety, reliable)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.on_cmd_vel, reliable)
        self.create_subscription(Float64, '/wamv/thrusters/left/thrust',
                                 lambda m: setattr(self, 'left_thrust', float(m.data)), reliable)
        self.create_subscription(Float64, '/wamv/thrusters/right/thrust',
                                 lambda m: setattr(self, 'right_thrust', float(m.data)), reliable)
        self.create_subscription(Float32MultiArray, '/usv/dynamic_metrics',
                                 self.on_dynamic_metrics, reliable)

        self.world_frame = args.world_frame
        self.base_frame = args.base_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x = self.y = self.yaw = math.nan
        self.goal_x = self.goal_y = math.nan
        self.goal_active = False
        self.status = 'IDLE code=0'
        self.safety_stop = False
        self.cmd_vx = math.nan
        self.cmd_wz = math.nan
        self.left_thrust = math.nan
        self.right_thrust = math.nan
        self.plan_id = 0
        self.latest_plan = []
        self.raw_count = 0
        self.dynamic_count = 0
        self.dynamic_tracks = 0.0
        self.dynamic_predicted_points = 0.0
        self.dynamic_max_speed = 0.0
        self.last_cloud_record = {'raw': -math.inf, 'dynamic': -math.inf}
        self.started = self.now_s()
        self.create_timer(max(0.05, args.period), self.tick)
        self.record_event('recorder_started', f'prefix={out}')
        self.get_logger().info('recording trajectory, /plan snapshots and obstacle clouds')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rel_t(self):
        return self.now_s() - self.started

    def record_event(self, event, detail=''):
        self.ew.writerow([time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                          f'{self.rel_t():.3f}', event, detail])
        self.events.flush()

    @staticmethod
    def yaw_from_quat(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.x, self.y = p.position.x, p.position.y
        self.yaw = self.yaw_from_quat(p.orientation)

    def on_goal(self, msg):
        self.goal_x, self.goal_y = msg.pose.position.x, msg.pose.position.y
        self.goal_active = True
        self.record_event('goal_received',
                          f'frame={msg.header.frame_id}; x={self.goal_x:.3f}; y={self.goal_y:.3f}')

    def on_status(self, msg):
        self.status = msg.data
        terminal = self.status.upper().startswith(
            ('SUCCEEDED', 'CANCELED', 'CANCELLED', 'ABORTED', 'FAILED', 'REJECTED'))
        if terminal:
            self.goal_active = False
        self.record_event('goal_status', self.status)

    def on_safety(self, msg):
        self.safety_stop = bool(msg.data)

    def on_dynamic_metrics(self, msg):
        if len(msg.data) >= 3:
            self.dynamic_tracks = float(msg.data[0])
            self.dynamic_predicted_points = float(msg.data[1])
            self.dynamic_max_speed = float(msg.data[2])

    def on_cmd_vel(self, msg):
        self.cmd_vx = float(msg.linear.x)
        self.cmd_wz = float(msg.angular.z)

    @staticmethod
    def transform_xy(x, y, transform):
        q = transform.transform.rotation
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        xx = 1.0 - 2.0 * (qy * qy + qz * qz)
        xy = 2.0 * (qx * qy - qz * qw)
        yx = 2.0 * (qx * qy + qz * qw)
        yy = 1.0 - 2.0 * (qx * qx + qz * qz)
        return (float(transform.transform.translation.x) + xx * x + xy * y,
                float(transform.transform.translation.y) + yx * x + yy * y)

    def plan_transform(self, frame, stamp):
        if frame == self.world_frame:
            return None
        try:
            # Planner paths in this workspace are normally stamped with zero;
            # Time() asks TF for the newest transform and is preferable to
            # silently plotting a body-frame path as if it were world-frame.
            return self.tf_buffer.lookup_transform(
                self.world_frame, frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warning(
                f'cannot transform plan {frame}->{self.world_frame}: {exc}')
            return False

    def on_plan(self, source, msg):
        # Only advance the snapshot counter for real planner output; controller
        # echoes keep the current id so a replan can be counted by plan_id alone.
        if source in self.planner_sources:
            self.plan_id += 1
        t = self.rel_t()
        frame = msg.header.frame_id or 'unknown'
        transform = self.plan_transform(frame, msg.header.stamp)
        current = []
        for i, stamped in enumerate(msg.poses):
            pose = stamped.pose
            raw_x, raw_y = float(pose.position.x), float(pose.position.y)
            raw_yaw = self.yaw_from_quat(pose.orientation)
            if transform is False:
                continue
            if transform is None:
                x, y, yaw = raw_x, raw_y, raw_yaw
            else:
                x, y = self.transform_xy(raw_x, raw_y, transform)
                yaw = raw_yaw + self.yaw_from_quat(transform.transform.rotation)
            current.append((x, y))
            self.pw.writerow([self.plan_id, f'{t:.3f}', source, frame, i,
                              f'{x:.4f}', f'{y:.4f}', f'{yaw:.5f}',
                              f'{raw_x:.4f}', f'{raw_y:.4f}', f'{raw_yaw:.5f}'])
        if current:
            self.latest_plan = current
        self.plans.flush()
        self.record_event('plan_received',
                          f'id={self.plan_id}; source={source}; points={len(current)}; frame={frame}; world_frame={self.world_frame}')

    def transform_to_world(self, x, y, z):
        if not (math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.yaw)):
            return None
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y, z

    def on_cloud(self, source, msg):
        # Point clouds arrive much faster and denser than the path/odometry
        # topics. Throttle first, then use the vectorized ROS helper instead
        # of iterating over every point in Python. This keeps the diagnostic
        # recorder from starving Nav2 callbacks.
        now = self.rel_t()
        if now - self.last_cloud_record[source] < self.args.cloud_period:
            return
        self.last_cloud_record[source] = now
        try:
            points = point_cloud2.read_points_numpy(
                msg, field_names=['x', 'y', 'z'], skip_nans=True)
            total = int(points.shape[0])
            if total == 0:
                selected = points
            else:
                sample_count = min(total, max(1, int(self.args.max_cloud_points)))
                indices = (range(total) if sample_count == total else
                           [int(i * total / sample_count) for i in range(sample_count)])
                selected = points[list(indices)]
            for i, point in zip((int(i) for i in (range(total) if total <= self.args.max_cloud_points else
                                                   [int(j * total / self.args.max_cloud_points)
                                                    for j in range(self.args.max_cloud_points)])), selected):
                x, y, z = (float(v) for v in point[:3])
                world = self.transform_to_world(x, y, z)
                if world is None:
                    continue
                self.ow.writerow([f'{now:.3f}', source, i,
                                  f'{world[0]:.4f}', f'{world[1]:.4f}', f'{world[2]:.4f}'])
        except Exception as exc:
            self.get_logger().warning(f'cannot record {source} cloud: {exc}')
            return
        if source == 'raw':
            self.raw_count = total
        else:
            self.dynamic_count = total
        if self.obstacles:
            self.obstacles.flush()

    def nearest_plan_error(self):
        if not self.latest_plan or not math.isfinite(self.x):
            return math.nan
        return min(math.hypot(self.x - px, self.y - py)
                   for px, py in self.latest_plan)

    def tick(self):
        if not math.isfinite(self.x):
            return
        self.tw.writerow([
            f'{self.rel_t():.3f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.yaw:.5f}',
            f'{self.goal_x:.4f}' if math.isfinite(self.goal_x) else '',
            f'{self.goal_y:.4f}' if math.isfinite(self.goal_y) else '',
            int(self.goal_active), self.status, self.plan_id,
            f'{self.nearest_plan_error():.4f}' if math.isfinite(self.nearest_plan_error()) else '',
            self.raw_count, self.dynamic_count,
            f'{self.dynamic_tracks:.0f}', f'{self.dynamic_predicted_points:.0f}',
            f'{self.dynamic_max_speed:.3f}', int(self.safety_stop),
            f'{self.cmd_vx:.4f}' if math.isfinite(self.cmd_vx) else '',
            f'{self.cmd_wz:.4f}' if math.isfinite(self.cmd_wz) else '',
            f'{self.left_thrust:.3f}' if math.isfinite(self.left_thrust) else '',
            f'{self.right_thrust:.3f}' if math.isfinite(self.right_thrust) else '',
        ])
        self.traj.flush()

    def close(self):
        self.record_event('recorder_stopped')
        for f in (self.traj, self.plans, self.obstacles, self.events):
            if f:
                f.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out-prefix', required=True,
                        help='Output prefix, e.g. report3/data/static_obstacle_20260830')
    parser.add_argument('--period', type=float, default=0.2)
    parser.add_argument('--record-clouds', action='store_true',
                        help='also subscribe to point clouds; use the separate obstacle recorder for live runs')
    parser.add_argument('--cloud-stride', type=int, default=5,
                        help='legacy option; max-cloud-points controls output size')
    parser.add_argument('--max-cloud-points', type=int, default=300,
                        help='maximum sampled points written per sampled cloud')
    parser.add_argument('--cloud-period', type=float, default=1.0,
                        help='minimum seconds between recorded clouds per source')
    parser.add_argument('--world-frame', default='camera_init',
                        help='frame used for stored trajectory and plan geometry')
    parser.add_argument('--base-frame', default='wamv/wamv/base_link',
                        help='boat frame used by obstacle clouds')
    args = parser.parse_args()
    rclpy.init()
    node = NavPathRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exc:
        # SIGINT may invalidate the rclpy context before spin() returns.
        # Preserve the CSV evidence and avoid turning an intentional stop into
        # a noisy traceback in the experiment log.
        if rclpy.ok():
            node.get_logger().error(f'recorder stopped unexpectedly: {exc}')
        else:
            print(f'recorder stopped during ROS shutdown: {exc}', file=sys.stderr)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
