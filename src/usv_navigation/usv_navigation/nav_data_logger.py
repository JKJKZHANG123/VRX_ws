#!/usr/bin/env python3
"""
Nav data logger (for MATLAB before/after comparison).

Records, at a fixed cadence, everything needed to assess trajectory quality:
  t, pose_x, pose_y, pose_yaw, goal_x, goal_y, goal_active,
  des_vx, des_wz, act_vx, act_wz, crosstrack_err, along_err,
  heading_err, mode, obstacle_pts, raw_obstacle_pts, safety_cloud_pts,
  lio_input_pts, registered_pts, dynamic_track_count, dynamic_point_count,
  dynamic_max_speed, goal_accepted, goal_status, goal_result_code

  - pose/goal from /aft_mapped_to_init + /goal_pose (camera_init frame)
  - des_vx/des_wz from /cmd_vel_smoothed (Nav2 command)
  - act_vx/act_wz from /aft_mapped_to_init twist (actually achieved)
  - crosstrack_err = perpendicular distance from the boat to the straight
    start->goal line (how far off-course it drifts)
  - mode from /nav_mode (PLAN / DIRECT)
  - cloud point counts from the LIO input/output and both obstacle filters,
    allowing the water-rejection change to be compared directly in MATLAB

Output: a CSV (path given by out_csv param, or data/nav_log.csv by default).
"""
import csv
import math
import os
import re

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from visualization_msgs.msg import MarkerArray
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray, Float64, String


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def _ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class NavDataLogger(Node):
    def __init__(self):
        super().__init__('nav_data_logger')

        self.declare_parameter('out_csv', '')
        self.declare_parameter('period_s', 0.2)        # 5 Hz
        self.declare_parameter('goal_tolerance', 2.0)  # arrived threshold (m)
        self._out = self.get_parameter('out_csv').value
        self._period = self.get_parameter('period_s').value
        self._tol = self.get_parameter('goal_tolerance').value

        if not self._out:
            d = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             '..', '..', '..', 'data')
            os.makedirs(d, exist_ok=True)
            self._out = os.path.join(d, 'nav_log.csv')
        os.makedirs(os.path.dirname(os.path.abspath(self._out)), exist_ok=True)

        self._f = open(self._out, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow([
            't', 'pose_x', 'pose_y', 'pose_yaw',
            'goal_x', 'goal_y', 'goal_active',
            'des_vx', 'des_wz', 'act_vx', 'act_wz',
            'crosstrack_err', 'along_err', 'heading_err',
            'mode', 'obstacle_pts', 'left_thrust', 'right_thrust',
            'safety_stop', 'path_min_clearance', 'path_length',
            'raw_obstacle_pts', 'safety_cloud_pts',
            'lio_input_pts', 'registered_pts',
            'dynamic_track_count', 'dynamic_point_count', 'dynamic_max_speed',
            'goal_accepted', 'goal_status', 'goal_result_code',
        ])
        self.get_logger().info(f'nav_data_logger -> {self._out}')

        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
        qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(Odometry, '/aft_mapped_to_init', self._on_odom, q)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, q)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._on_cmd, q)
        self.create_subscription(String, '/nav_mode', self._on_mode, q)
        self.create_subscription(PointCloud2, '/usv/structure_cloud',
                                 self._on_cloud, qb)
        self.create_subscription(PointCloud2, '/usv/raw_obstacle_cloud',
                                 self._on_raw_cloud, qb)
        self.create_subscription(PointCloud2, '/usv/safety_cloud',
                                 self._on_safety_cloud, qb)
        self.create_subscription(PointCloud2, '/wamv/points_filtered',
                                 self._on_lio_input, qb)
        self.create_subscription(PointCloud2, '/cloud_registered',
                                 self._on_registered, qb)
        self.create_subscription(Float32MultiArray, '/usv/dynamic_metrics',
                                 self._on_dynamic_metrics, q)
        self.create_subscription(Float64, '/wamv/thrusters/left/thrust',
                                 self._on_left_thrust, q)
        self.create_subscription(Float64, '/wamv/thrusters/right/thrust',
                                 self._on_right_thrust, q)
        self.create_subscription(Bool, '/usv/safety_stop',
                                 self._on_safety_stop, q)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Bool, '/usv/goal_accepted',
                                 self._on_goal_accepted, status_qos)
        self.create_subscription(String, '/usv/goal_status',
                                 self._on_goal_status, status_qos)
        # Nav2 action status is the authoritative fallback when the gateway
        # status event is missed by DDS.  The gateway text remains preferred.
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self._on_action_status, q)
        self.create_subscription(GoalStatusArray,
                                 '/navigate_through_poses/_action/status',
                                 self._on_through_action_status, q)
        self.create_subscription(MarkerArray, '/nav_waypoints_markers',
                                 self._on_waypoint_markers, q)
        self.create_subscription(NavPath, '/plan', self._on_path, q)

        self._t0 = self.get_clock().now().nanoseconds
        self._start = None   # (x, y) at first goal
        self._ox = self._oy = self._oyaw = 0.0
        self._avx = self._avz = 0.0
        self._gx = self._gy = 0.0
        self._goal_active = False
        self._dvx = self._dwz = 0.0
        self._mode = 'PLAN'
        self._obst = 0
        self._raw_obst = 0
        self._safety_cloud_pts = 0
        self._lio_input_pts = 0
        self._registered_pts = 0
        self._dynamic_tracks = 0.0
        self._dynamic_points = 0.0
        self._dynamic_max_speed = 0.0
        self._left_thrust = self._right_thrust = 0.0
        self._safety_stop = False
        self._goal_accepted = False
        self._goal_status = 'IDLE code=0'
        self._goal_result_code = 0
        self._action_status_seen = False
        self._multi_goal_active = False
        self._path_xy = []
        self._cloud_xy = []
        self._path_min_clearance = float('nan')
        self._path_length = 0.0
        self.create_timer(self._period, self._tick)

    def _on_odom(self, m: Odometry):
        self._ox = m.pose.pose.position.x
        self._oy = m.pose.pose.position.y
        self._oyaw = _yaw(m.pose.pose.orientation)
        self._avx = m.twist.twist.linear.x
        self._avz = m.twist.twist.angular.z

    def _on_goal(self, m: PoseStamped):
        self._multi_goal_active = False
        self._gx = m.pose.position.x
        self._gy = m.pose.position.y
        self._goal_active = True
        self._start = (self._ox, self._oy)
        self._goal_status = 'GOAL_SENT code=1'
        self._goal_result_code = 1
        self._action_status_seen = False
        self._multi_goal_active = False

    def _on_cmd(self, m: Twist):
        self._dvx = m.linear.x
        self._dwz = m.angular.z

    def _on_mode(self, m: String):
        self._mode = m.data

    def _on_left_thrust(self, m: Float64):
        self._left_thrust = m.data

    def _on_right_thrust(self, m: Float64):
        self._right_thrust = m.data

    def _on_safety_stop(self, m: Bool):
        self._safety_stop = bool(m.data)

    def _on_goal_accepted(self, m: Bool):
        self._goal_accepted = bool(m.data)

    def _on_goal_status(self, m: String):
        self._goal_status = m.data
        match = re.search(r'\bcode=(-?\d+)', m.data)
        if match:
            self._goal_result_code = int(match.group(1))
        terminal = ('SUCCEEDED' in m.data or 'CANCELED' in m.data or
                    'ABORTED' in m.data or 'REJECTED' in m.data or
                    'FAILED' in m.data)
        if terminal:
            self._goal_active = False

    def _on_action_status(self, msg: GoalStatusArray):
        if not self._goal_active or not msg.status_list:
            return
        # The action server publishes the newest status at the end of the
        # array.  This fallback deliberately does not overwrite a descriptive
        # terminal status already supplied by click_to_goal.
        status = int(msg.status_list[-1].status)
        labels = {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                  4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        label = labels.get(status, 'UNKNOWN')
        if 'code=' not in self._goal_status or self._goal_status.startswith('GOAL_SENT'):
            self._goal_status = f'{label} code={status}'
            self._goal_result_code = status
        self._action_status_seen = True
        if status in (4, 5, 6):
            self._goal_active = False

    def _on_waypoint_markers(self, msg: MarkerArray):
        # The Gazebo waypoint bridge keeps markers latched on screen.  The
        # final marker is the terminal target for multi-goal metrics.
        points = [m for m in msg.markers if m.action == 0]
        if points:
            last = points[-1]
            self._gx = last.pose.position.x
            self._gy = last.pose.position.y

    def _on_through_action_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        status = int(msg.status_list[-1].status)
        labels = {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                  4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        label = labels.get(status, 'UNKNOWN')
        if status in (1, 2, 3) and not self._multi_goal_active:
            self._multi_goal_active = True
            self._goal_active = True
            self._start = (self._ox, self._oy)
        if not self._multi_goal_active and status in (4, 5, 6):
            return
        self._goal_status = f'{label} code={status}'
        self._goal_result_code = status
        self._action_status_seen = True
        if status in (4, 5, 6):
            self._goal_active = False
            self._multi_goal_active = False

    def _on_path(self, m: NavPath):
        self._path_xy = [(p.pose.position.x, p.pose.position.y)
                         for p in m.poses]
        self._path_length = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(self._path_xy, self._path_xy[1:]))
        self._update_path_clearance()

    def _on_cloud(self, m: PointCloud2):
        n = 0
        if m.point_step > 0:
            n = len(m.data) // m.point_step
        self._obst = n
        # structure_cloud is in the boat frame. Convert it to camera_init with
        # the current LIO pose so MATLAB can compare the planned line against
        # observed shoreline/structure points. Limit samples to bound CPU.
        c, s = math.cos(self._oyaw), math.sin(self._oyaw)
        pts = []
        try:
            for i, (x, y) in enumerate(point_cloud2.read_points(
                    m, field_names=('x', 'y'), skip_nans=True)):
                if i % 3 != 0:
                    continue
                pts.append((self._ox + c * x - s * y,
                            self._oy + s * x + c * y))
                if len(pts) >= 1000:
                    break
        except Exception:
            pts = []
        self._cloud_xy = pts
        self._update_path_clearance()

    @staticmethod
    def _point_count(m: PointCloud2):
        return len(m.data) // m.point_step if m.point_step > 0 else 0

    def _on_raw_cloud(self, m: PointCloud2):
        self._raw_obst = self._point_count(m)

    def _on_safety_cloud(self, m: PointCloud2):
        self._safety_cloud_pts = self._point_count(m)

    def _on_lio_input(self, m: PointCloud2):
        self._lio_input_pts = self._point_count(m)

    def _on_registered(self, m: PointCloud2):
        self._registered_pts = self._point_count(m)

    def _on_dynamic_metrics(self, m: Float32MultiArray):
        if len(m.data) >= 3:
            self._dynamic_tracks = float(m.data[0])
            self._dynamic_points = float(m.data[1])
            self._dynamic_max_speed = float(m.data[2])

    def _update_path_clearance(self):
        if not self._path_xy or not self._cloud_xy:
            self._path_min_clearance = float('nan')
            return
        # Sample the path to keep this diagnostic inexpensive at 5 Hz.
        path = self._path_xy[::max(1, len(self._path_xy) // 200)]
        self._path_min_clearance = min(
            math.hypot(px - ox, py - oy)
            for px, py in path for ox, oy in self._cloud_xy)

    def _tick(self):
        t = (self.get_clock().now().nanoseconds - self._t0) * 1e-9
        ct = at = 0.0
        if self._goal_active and self._start is not None:
            sx, sy = self._start
            dx, dy = self._gx - sx, self._gy - sy
            L = math.hypot(dx, dy)
            if L > 1e-6:
                ux, uy = dx / L, dy / L
                px, py = self._ox - sx, self._oy - sy
                at = px * ux + py * uy           # along-track distance
                ct = abs(px * uy - py * ux)      # crosstrack distance
        heading_err = 0.0
        if self._goal_active:
            want = math.atan2(self._gy - self._oy, self._gx - self._ox)
            heading_err = _ang_diff(want, self._oyaw)
        self._w.writerow([
            f'{t:.3f}', f'{self._ox:.3f}', f'{self._oy:.3f}', f'{self._oyaw:.4f}',
            f'{self._gx:.3f}', f'{self._gy:.3f}', int(self._goal_active),
            f'{self._dvx:.3f}', f'{self._dwz:.3f}',
            f'{self._avx:.3f}', f'{self._avz:.3f}',
            f'{ct:.3f}', f'{at:.3f}', f'{heading_err:.4f}',
            self._mode, self._obst,
            f'{self._left_thrust:.3f}', f'{self._right_thrust:.3f}',
            int(self._safety_stop),
            (f'{self._path_min_clearance:.3f}'
             if math.isfinite(self._path_min_clearance) else 'nan'),
            f'{self._path_length:.3f}',
            self._raw_obst, self._safety_cloud_pts,
            self._lio_input_pts, self._registered_pts,
            f'{self._dynamic_tracks:.0f}', f'{self._dynamic_points:.0f}',
            f'{self._dynamic_max_speed:.3f}',
            int(self._goal_accepted), self._goal_status, self._goal_result_code,
        ])
        self._f.flush()


def main(args=None):
    rclpy.init(args=args)
    node = NavDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._f.close()
        node.get_logger().info('nav_data_logger closed')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
