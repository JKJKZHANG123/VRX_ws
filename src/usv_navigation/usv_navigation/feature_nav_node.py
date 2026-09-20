#!/usr/bin/env python3
"""
Feature-aware adaptive navigation for the WAM-V.

The node keeps ``/goal_pose`` as the user-facing goal topic, but submits the
actual goal to Nav2's NavigateToPose action.  This is important: Nav2 does not
consume an arbitrary PoseStamped topic as a navigation command.

Modes:
  PLAN   Nav2 owns the command and plans/follows the costmap path.
  DIRECT  On a fresh, feature-poor cloud, drive toward the saved goal with a
          heading-corrected command.  A feature spike returns to PLAN.

A stale/missing cloud never enables DIRECT; the safe fallback is PLAN.
"""
import copy
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, String


class FeatureNavNode(Node):
    def __init__(self):
        super().__init__('feature_nav_node')

        self.declare_parameter('min_feature_points', 30)
        self.declare_parameter('hysteresis_ratio', 0.7)
        self.declare_parameter('direct_linear_speed', 1.5)
        self.declare_parameter('direct_angular_gain', 1.0)
        self.declare_parameter('update_period_s', 0.5)
        self.declare_parameter('cloud_timeout_s', 5.0)
        self.declare_parameter('enable_direct_mode', False)

        self.min_pts = float(self.get_parameter('min_feature_points').value)
        self.hyst = float(self.get_parameter('hysteresis_ratio').value)
        self.v_go = float(self.get_parameter('direct_linear_speed').value)
        self.k_ang = float(self.get_parameter('direct_angular_gain').value)
        self.period = float(self.get_parameter('update_period_s').value)
        self.cloud_timeout = float(self.get_parameter('cloud_timeout_s').value)
        self.enable_direct_mode = bool(
            self.get_parameter('enable_direct_mode').value)

        self._mode = 'PLAN'
        self._last_feature_pts = 0.0
        self._last_feature_time = 0
        self._goal = None
        self._goal_seq = 0
        self._goal_dirty = False
        self._nav_goal_handle = None
        self._nav_goal_pending = False
        self._ox = self._oy = self._yaw = 0.0

        q = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qb = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, q)
        self.create_subscription(Empty, '/cancel_navigation',
                                 self._on_cancel, q)
        self.create_subscription(Odometry, '/aft_mapped_to_init',
                                 self._on_odom, q)
        self.create_subscription(PointCloud2, '/usv/structure_cloud',
                                 self._on_cloud, qb)

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel_smoothed', q)
        self._pub_mode = self.create_publisher(String, '/nav_mode', q)
        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f'feature_nav_node up: threshold={self.min_pts:.0f} pts, '
            f'hyst={self.hyst:.2f}, direct vel={self.v_go:.2f} m/s, '
            f'direct_mode={self.enable_direct_mode}')

    def _on_goal(self, msg: PoseStamped):
        # Keep the exact user goal.  Never replace it with the boat pose when
        # cancelling Nav2; doing so loses the destination during DIRECT mode.
        self._goal = copy.deepcopy(msg)
        self._goal_seq += 1
        self._goal_dirty = True
        self.get_logger().info(
            f'goal set: ({msg.pose.position.x:.1f}, '
            f'{msg.pose.position.y:.1f}) frame={msg.header.frame_id}')

        if self._mode != 'PLAN':
            self._mode = 'PLAN'
            self._stop_cmd()
            self._pub_mode.publish(String(data='PLAN'))
        self._cancel_nav2()
        self._send_nav_goal_if_ready()

    def _on_cancel(self, _msg: Empty):
        self._goal = None
        self._goal_dirty = False
        self._goal_seq += 1
        self._cancel_nav2()
        self._stop_cmd()
        if self._mode != 'PLAN':
            self._mode = 'PLAN'
            self._pub_mode.publish(String(data='PLAN'))
        self.get_logger().info('navigation goal cancelled')

    def _on_odom(self, msg: Odometry):
        self._ox = msg.pose.pose.position.x
        self._oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )

    def _on_cloud(self, msg: PointCloud2):
        self._last_feature_pts = (
            len(msg.data) // msg.point_step if msg.point_step > 0 else 0)
        self._last_feature_time = self.get_clock().now().nanoseconds

    def _tick(self):
        if self._goal is None:
            return

        # Safety default: this node is an observer only. In particular it must
        # never cancel Nav2 or publish an open-loop command merely because a
        # registered cloud is temporarily sparse/stale.
        if not self.enable_direct_mode:
            return

        if self._mode == 'PLAN':
            self._send_nav_goal_if_ready()

        now_ns = self.get_clock().now().nanoseconds
        age_s = (now_ns - self._last_feature_time) * 1e-9
        fresh = self._last_feature_time != 0 and age_s < self.cloud_timeout
        # Missing/stale perception is a safety condition: remain in PLAN and
        # let Nav2 use its current costmap rather than driving open-loop.
        if not fresh:
            if self._mode == 'DIRECT':
                self._switch_to_plan(0.0)
            return

        threshold = (self.min_pts if self._mode == 'PLAN'
                     else self.min_pts * self.hyst)
        pts = self._last_feature_pts
        if self._mode == 'PLAN' and pts < threshold:
            self._switch_to_direct(pts)
        elif self._mode == 'DIRECT' and pts >= threshold:
            self._switch_to_plan(pts)
        elif self._mode == 'DIRECT':
            self._drive_straight()

    def _send_nav_goal_if_ready(self):
        if (self._goal is None or self._mode != 'PLAN' or
                not self._goal_dirty or self._nav_goal_pending):
            return
        if not self._nav_client.server_is_ready():
            self.get_logger().debug(
                'NavigateToPose action server not ready',
                throttle_duration_sec=2.0)
            return

        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(self._goal)
        seq = self._goal_seq
        self._nav_goal_pending = True
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_nav_goal_response(f, seq))

    def _on_nav_goal_response(self, future, seq):
        self._nav_goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:  # action transport failure
            self.get_logger().error(f'NavigateToPose send failed: {exc}')
            self._goal_dirty = True
            return

        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected NavigateToPose goal')
            self._goal_dirty = True
            return

        # A goal can become obsolete while the async request is in flight.
        if seq != self._goal_seq or self._goal is None or self._mode != 'PLAN':
            handle.cancel_goal_async()
            return

        self._nav_goal_handle = handle
        self._goal_dirty = False
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_nav_result(f, seq, handle))
        self.get_logger().info('NavigateToPose goal accepted by Nav2')

    def _on_nav_result(self, _future, seq, handle):
        if seq == self._goal_seq and self._nav_goal_handle is handle:
            self._nav_goal_handle = None
            self.get_logger().info('Nav2 NavigateToPose action finished')

    def _switch_to_direct(self, pts):
        self._mode = 'DIRECT'
        self.get_logger().info(
            f'[DIRECT] fresh feature points {pts:.0f} < {self.min_pts:.0f}')
        self._cancel_nav2()
        self._pub_mode.publish(String(data='DIRECT'))
        self._drive_straight()

    def _switch_to_plan(self, pts):
        self._mode = 'PLAN'
        self.get_logger().info(
            f'[PLAN] feature points {pts:.0f} -> Nav2 planning')
        self._stop_cmd()
        self._goal_dirty = self._goal is not None
        self._pub_mode.publish(String(data='PLAN'))
        self._send_nav_goal_if_ready()

    def _drive_straight(self):
        if self._goal is None:
            return
        gx = self._goal.pose.position.x
        gy = self._goal.pose.position.y
        dx, dy = gx - self._ox, gy - self._oy
        dist = math.hypot(dx, dy)
        if dist < 0.5:
            self._stop_cmd()
            return
        want_yaw = math.atan2(dy, dx)
        err = math.atan2(
            math.sin(want_yaw - self._yaw), math.cos(want_yaw - self._yaw))
        v = self.v_go
        if dist < 8.0:
            v *= max(0.2, dist / 8.0)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(self.k_ang * err)
        self._pub_cmd.publish(msg)

    def _cancel_nav2(self):
        handle = self._nav_goal_handle
        self._nav_goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()

    def _stop_cmd(self):
        self._pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FeatureNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
