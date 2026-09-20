#!/usr/bin/env python3
"""Validated RViz/Gazebo goal bridge for Nav2.

The original node only published ``/goal_pose``.  Nav2's default navigator does
not treat that topic as a navigation command, so a goal could be displayed in
RViz while no NavigateToPose action was actually running.  This node now acts
as the single goal gateway:

* RViz ``/clicked_point`` and external ``/goal_pose`` messages are checked;
* a goal must be inside the configured operating boundary and a known,
  sufficiently free costmap cell;
* the accepted pose is sent to Nav2's ``NavigateToPose`` action;
* ``/cancel_navigation`` cancels the active action without creating a new goal.

The check is deliberately conservative.  A missing or unknown costmap is not
considered safe, which prevents a click beyond the mapped/observed area from
sending the boat toward an unseen shoreline.  Dynamic obstacle prediction will
be added as a separate local-costmap layer later; this gateway remains useful
for both static and dynamic modes.
"""

import copy
import math
from typing import Optional, Tuple

from geometry_msgs.msg import PointStamped, PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _is_identity_orientation(pose: PoseStamped, eps: float = 1e-6) -> bool:
    q = pose.pose.orientation
    return (abs(q.x) < eps and abs(q.y) < eps and abs(q.z) < eps and
            abs(abs(q.w) - 1.0) < eps)


def _map_bounds(msg: OccupancyGrid) -> Tuple[float, float, float, float]:
    """Return axis-aligned x/y bounds for the usual unrotated costmap."""
    res = float(msg.info.resolution)
    ox = float(msg.info.origin.position.x)
    oy = float(msg.info.origin.position.y)
    return ox, oy, ox + msg.info.width * res, oy + msg.info.height * res


def _costmap_disk_status(
    msg: OccupancyGrid,
    x: float,
    y: float,
    radius: float,
    occupied_threshold: int,
    reject_unknown: bool,
) -> Tuple[bool, str]:
    """Check a circular footprint against an OccupancyGrid.

    The Nav2 inflation layer has already expanded obstacles, but checking a
    disk rather than one cell prevents a goal from being accepted with the
    WAM-V hull partly outside the free area.  Costmap origins in this project
    have no yaw, so an axis-aligned index conversion is correct.
    """
    if msg.info.resolution <= 0.0 or msg.info.width == 0 or msg.info.height == 0:
        return False, 'costmap has invalid geometry'

    res = float(msg.info.resolution)
    ox, oy, max_x, max_y = _map_bounds(msg)
    if (x - radius < ox or y - radius < oy or
            x + radius >= max_x or y + radius >= max_y):
        return False, 'goal footprint is outside the costmap'

    step = max(res * 0.5, 0.05)
    nx = max(1, int(math.ceil(radius / step)))
    ny = nx
    for ix in range(-nx, nx + 1):
        for iy in range(-ny, ny + 1):
            px = x + ix * step
            py = y + iy * step
            if ix * ix + iy * iy > (radius / step) ** 2:
                continue
            col = int(math.floor((px - ox) / res))
            row = int(math.floor((py - oy) / res))
            if col < 0 or row < 0 or col >= msg.info.width or row >= msg.info.height:
                return False, 'goal footprint is outside the costmap'
            value = int(msg.data[row * msg.info.width + col])
            if value < 0:
                if reject_unknown:
                    return False, 'goal footprint contains unknown costmap cells'
                continue
            if value >= occupied_threshold:
                return False, f'goal footprint cost {value} >= {occupied_threshold}'
    return True, 'free'


class ClickToGoal(Node):
    """Safety gateway from clicked/external poses to Nav2 NavigateToPose."""

    def __init__(self):
        super().__init__('click_to_goal')

        self.declare_parameter('global_frame', 'camera_init')
        self.declare_parameter('max_goal_distance_m', 45.0)
        self.declare_parameter('goal_check_radius_m', 2.8)
        self.declare_parameter('occupied_cost_threshold', 50)
        self.declare_parameter('reject_unknown', True)
        self.declare_parameter('require_costmap', True)
        self.declare_parameter('geofence_enabled', False)
        self.declare_parameter('geofence_min_x_m', -100.0)
        self.declare_parameter('geofence_max_x_m', 100.0)
        self.declare_parameter('geofence_min_y_m', -100.0)
        self.declare_parameter('geofence_max_y_m', 100.0)

        self._global_frame = str(self.get_parameter('global_frame').value)
        self._max_goal_distance = float(
            self.get_parameter('max_goal_distance_m').value)
        self._goal_radius = float(self.get_parameter('goal_check_radius_m').value)
        self._occupied_threshold = int(
            self.get_parameter('occupied_cost_threshold').value)
        self._reject_unknown = bool(self.get_parameter('reject_unknown').value)
        self._require_costmap = bool(self.get_parameter('require_costmap').value)
        self._geofence_enabled = bool(self.get_parameter('geofence_enabled').value)
        self._fence = (
            float(self.get_parameter('geofence_min_x_m').value),
            float(self.get_parameter('geofence_max_x_m').value),
            float(self.get_parameter('geofence_min_y_m').value),
            float(self.get_parameter('geofence_max_y_m').value),
        )

        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Status is state, not an event.  Transient-local QoS lets a logger or
        # MATLAB recorder that starts after the goal still receive the latest
        # accepted/terminal result.
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._status_pub = self.create_publisher(
            String, '/usv/goal_status', status_qos)
        self._accepted_pub = self.create_publisher(
            Bool, '/usv/goal_accepted', status_qos)
        self.create_subscription(PointStamped, '/clicked_point', self._on_click, 10)
        # This subscription makes command-line and Gazebo single-marker goals
        # use exactly the same validation/action path as RViz clicks.
        self.create_subscription(PoseStamped, '/goal_pose', self._on_external_goal, 10)
        self.create_subscription(Empty, '/cancel_navigation', self._on_cancel, 10)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self._on_odom, 10)

        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._on_global_costmap,
            costmap_qos)
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._on_local_costmap,
            costmap_qos)

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._active_goal_handle = None
        self._status_sequence = 0
        self._pending_goal: Optional[PoseStamped] = None
        self._last_global_costmap: Optional[OccupancyGrid] = None
        # RViz clicks are re-published on /goal_pose for logging and external
        # consumers. Remember that one local publication so this node does not
        # consume its own message and send the same Nav2 action twice.
        self._self_published_goal: Optional[Tuple[float, float]] = None
        self._last_local_costmap: Optional[OccupancyGrid] = None
        self._pose_xy: Optional[Tuple[float, float]] = None
        self._pose_yaw = 0.0
        self.create_timer(0.2, self._try_send_pending)
        self._publish_status(False, 'IDLE code=0')

        self.get_logger().info(
            'click_to_goal up: validated goals -> NavigateToPose, '
            f'frame={self._global_frame}, radius={self._goal_radius:.1f}m, '
            f'reject_unknown={self._reject_unknown}')

    def _on_odom(self, msg: Odometry):
        self._pose_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._pose_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_global_costmap(self, msg: OccupancyGrid):
        self._last_global_costmap = msg

    def _on_local_costmap(self, msg: OccupancyGrid):
        self._last_local_costmap = msg

    def _on_click(self, msg: PointStamped):
        goal = PoseStamped()
        goal.header = msg.header
        goal.header.frame_id = msg.header.frame_id or self._global_frame
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = 0.0
        # A point click has no meaningful final yaw.  Hold the current heading
        # instead of silently forcing yaw=0, which was a source of needless
        # turn commands near the shoreline.
        goal.pose.orientation.z = math.sin(self._pose_yaw / 2.0)
        goal.pose.orientation.w = math.cos(self._pose_yaw / 2.0)
        if self._validate_goal(goal):
            goal.header.frame_id = self._global_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            self._self_published_goal = (
                goal.pose.position.x, goal.pose.position.y)
            self._goal_pub.publish(goal)  # RViz display/logging
            self._queue_goal(goal)
            self.get_logger().info(
                f'click -> accepted goal ({goal.pose.position.x:.2f}, '
                f'{goal.pose.position.y:.2f})')

    def _on_external_goal(self, msg: PoseStamped):
        if self._self_published_goal is not None:
            sx, sy = self._self_published_goal
            if (math.hypot(msg.pose.position.x - sx,
                           msg.pose.position.y - sy) < 1e-6):
                self._self_published_goal = None
                return
        goal = copy.deepcopy(msg)
        if _is_identity_orientation(goal):
            goal.pose.orientation.z = math.sin(self._pose_yaw / 2.0)
            goal.pose.orientation.w = math.cos(self._pose_yaw / 2.0)
        if self._validate_goal(goal):
            goal.header.frame_id = self._global_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            self._queue_goal(goal)
            self.get_logger().info(
                f'external goal accepted ({goal.pose.position.x:.2f}, '
                f'{goal.pose.position.y:.2f})')

    def _validate_goal(self, goal: PoseStamped) -> bool:
        if goal.header.frame_id != self._global_frame:
            return self._reject(
                f'frame {goal.header.frame_id!r} != {self._global_frame!r}')
        x, y = goal.pose.position.x, goal.pose.position.y
        if not math.isfinite(x) or not math.isfinite(y):
            return self._reject('goal contains non-finite coordinates')
        if self._pose_xy is not None and self._max_goal_distance > 0.0:
            distance = math.hypot(x - self._pose_xy[0], y - self._pose_xy[1])
            if distance > self._max_goal_distance:
                return self._reject(
                    f'goal distance {distance:.1f}m exceeds '
                    f'{self._max_goal_distance:.1f}m')
        if self._geofence_enabled:
            min_x, max_x, min_y, max_y = self._fence
            if not (min_x <= x <= max_x and min_y <= y <= max_y):
                return self._reject('goal is outside the configured geofence')

        maps = [self._last_global_costmap, self._last_local_costmap]
        usable_map = False
        reasons = []
        for costmap in maps:
            if costmap is None or costmap.header.frame_id != self._global_frame:
                continue
            usable_map = True
            ok, reason = _costmap_disk_status(
                costmap, x, y, self._goal_radius, self._occupied_threshold,
                self._reject_unknown)
            if ok:
                return True
            reasons.append(reason)
        if self._require_costmap and not usable_map:
            return self._reject('no camera_init costmap is available yet')
        if self._require_costmap:
            return self._reject('; '.join(dict.fromkeys(reasons)))
        return True

    def _queue_goal(self, goal: PoseStamped):
        self._pending_goal = copy.deepcopy(goal)
        self._publish_status(False, 'QUEUED code=1; waiting for Nav2 action server')
        self._cancel_active_goal()
        self._try_send_pending()

    def _try_send_pending(self):
        if self._pending_goal is None or not self._nav_client.server_is_ready():
            return
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(self._pending_goal)
        self._pending_goal = None
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info('validated goal sent to NavigateToPose')

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._publish_status(False, f'FAILED code=6; action send failed: {exc}')
            self.get_logger().error(f'NavigateToPose send failed: {exc}')
            return
        if not handle.accepted:
            self._publish_status(False, 'REJECTED code=0; Nav2 rejected goal')
            self.get_logger().warn('Nav2 rejected validated goal')
            return
        self._active_goal_handle = handle
        self._publish_status(True, 'ACCEPTED code=1; Nav2 goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            status = int(future.result().status)
            labels = {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
            }
            label = labels.get(status, 'FAILED')
            self._publish_status(True, f'{label} code={status}')
            self.get_logger().info(
                f'NavigateToPose finished: {label} (status={status})')
        except Exception as exc:
            self._publish_status(True, f'FAILED code=6; result read failed: {exc}')
            self.get_logger().warn(f'could not read Nav2 result: {exc}')
        self._active_goal_handle = None

    def _on_cancel(self, _msg: Empty):
        self._pending_goal = None
        self._cancel_active_goal()
        self._publish_status(True, 'CANCELED code=5; navigation cancelled')
        self.get_logger().info('navigation goal cancelled')

    def _cancel_active_goal(self):
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

    def _reject(self, reason: str) -> bool:
        self._publish_status(False, f'rejected: {reason}')
        self.get_logger().warn(f'goal rejected: {reason}')
        return False

    def _publish_status(self, accepted: bool, text: str):
        # Include a monotonic sequence from this gateway.  The status topic has
        # two publishers (single-goal and Gazebo multi-goal), so a terminal
        # string alone cannot tell a test whether it belongs to the new goal.
        self._status_sequence += 1
        text = f'{text}; status_seq={self._status_sequence}'
        self._accepted_pub.publish(Bool(data=bool(accepted)))
        self._status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = ClickToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_active_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
