#!/usr/bin/env python3
"""
Gazebo marker/waypoint -> Nav2 navigation.

  in : /gz_click/point      (Vector3, world-frame marker point from plugin)
       /gz_boat_pose        (Vector3, WAM-V world pose)
       /aft_mapped_to_init  (Point-LIO odometry: boat pose in camera_init)
       /gz_waypoint_cmd     (String: "clear" / "navigate")
  out: /goal_pose           (single-marker navigation)
       /navigate_through_poses action (multi-marker navigation)

Coordinate math: the plugin reports world (ENU) points, but Nav2 now plans in
the Point-LIO world frame `camera_init`. The two differ by a fixed yaw (the
boat's spawn heading: camera_init's x-axis = spawn-forward) plus a translation
(the LIO map origin = the boat's first pose). For any boat pose p the rigid
relation is exact:
    p_world = Rz(spawn_yaw) * p_camera_init + t
    t       = boat_world - Rz(spawn_yaw) * boat_camera_init      (constant)
so a click converts to camera_init by
    click_camera = Rz(-spawn_yaw) * (click_world - t)

spawn_yaw must match the WAM-V spawn yaw in the world (vrx_gz competition.launch
spawns at yaw=1.0) AND match what Point-LIO's camera_init alignment produces
(its gravity alignment keeps camera_init yaw = the IMU's absolute ENU yaw at
startup, which is the spawn yaw). 1.0 is the VRX sim constant.

User workflow (Gazebo):
  1. Toggle "Mark Point: ON", click scene N times -> N persistent markers.
  2. "Navigate" -> single marker: /goal_pose; N markers: NavigateThroughPoses.
  3. "Clear Markers" -> clear list.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from rclpy.action import ActionClient
from geometry_msgs.msg import Vector3, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateThroughPoses


def _rot(vx, vy, th):
    """Rotate (vx, vy) CCW by th (rad)."""
    c, s = math.cos(th), math.sin(th)
    return (vx * c - vy * s, vx * s + vy * c)


class GzClickToGoalNode(Node):
    def __init__(self):
        super().__init__('gz_click_to_goal_node')

        self._spawn_yaw = self.declare_parameter('spawn_yaw', 1.0).value
        self._boat_world = None   # (x, y) in world/ENU
        self._boat_cam = None     # (x, y) in camera_init
        self._t = None            # camera_init origin in world coords (x, y)
        self._waypoints = []      # list of (x, y) in camera_init
        self._through_handle = None
        # Persistent display list: red markers stay on screen until "clear".
        self._marker_points = []

        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self._sub_point = self.create_subscription(
            Vector3, '/gz_click/point', self._on_point, q)
        self._sub_cmd = self.create_subscription(
            String, '/gz_waypoint_cmd', self._on_cmd, q)
        self._sub_boat = self.create_subscription(
            Vector3, '/gz_boat_pose', self._on_boat_world, qb)
        self._sub_odom = self.create_subscription(
            Odometry, '/aft_mapped_to_init', self._on_cam, q)

        self._pub_goal = self.create_publisher(PoseStamped, '/goal_pose', q)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/nav_waypoints_markers', q)
        self._nav_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses')
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._status_pub = self.create_publisher(
            String, '/usv/goal_status', status_qos)
        self._accepted_pub = self.create_publisher(
            Bool, '/usv/goal_accepted', status_qos)

        self.get_logger().info(
            'gz_click_to_goal_node up: Gazebo markers -> Nav2 '
            f'(spawn_yaw={self._spawn_yaw:.3f})')

    # ── subscribers ──────────────────────────────────────────
    def _on_boat_world(self, msg: Vector3):
        self._boat_world = (msg.x, msg.y)
        self._reset_transform()

    def _on_cam(self, msg: Odometry):
        self._boat_cam = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._reset_transform()

    def _reset_transform(self):
        # t = boat_world - Rz(+s)*boat_cam  is constant for any boat pose
        # (rigid transform); recompute whenever a fresh sample arrives.
        if self._boat_world is None or self._boat_cam is None:
            return
        rx, ry = _rot(*self._boat_cam, self._spawn_yaw)
        self._t = (self._boat_world[0] - rx, self._boat_world[1] - ry)

    def _world_to_camera(self, x, y):
        if self._t is None:
            return None
        # click_camera = Rz(-s) * (click_world - t)
        return _rot(x - self._t[0], y - self._t[1], -self._spawn_yaw)

    def _on_point(self, msg: Vector3):
        # Every marker click appends a waypoint.
        pt = self._world_to_camera(msg.x, msg.y)
        if pt is None:
            self.get_logger().warn('marker point ignored: no boat pose yet')
            return
        self._waypoints.append(pt)
        self._marker_points.append(pt)
        self._publish_markers()
        self.get_logger().info(
            f'marker #{len(self._marker_points)}: ({pt[0]:.2f}, {pt[1]:.2f})')

    def _on_cmd(self, msg: String):
        cmd = msg.data
        if cmd == 'clear':
            self._waypoints.clear()
            self._marker_points.clear()
            self._publish_markers()   # sends DELETE_ALL, removes red markers
            self.get_logger().info('waypoints cleared')
        elif cmd == 'navigate':
            self._navigate()
        elif cmd == 'cancel':
            self._cancel_through()
        else:
            self.get_logger().warn(f'unknown cmd: {cmd}')

    # ── navigation ───────────────────────────────────────────
    def _cancel_through(self):
        if self._through_handle is None:
            self._publish_status(False, 'REJECTED code=0; no active multi-goal')
            self.get_logger().warn('no active multi-goal to cancel')
            return
        self._publish_status(True, 'CANCELING code=3; cancel requested')
        future = self._through_handle.cancel_goal_async()
        future.add_done_callback(self._on_through_cancel_response)

    def _on_through_cancel_response(self, future):
        try:
            response = future.result()
            count = len(response.goals_canceling)
            self.get_logger().info(
                f'NavigateThroughPoses cancel acknowledged for {count} goal(s)')
            if count == 0:
                self._publish_status(True, 'FAILED code=6; cancel not accepted')
        except Exception as exc:
            self._publish_status(True, f'FAILED code=6; cancel failed: {exc}')
            self.get_logger().warn(f'NavigateThroughPoses cancel failed: {exc}')

    def _navigate(self):
        n = len(self._waypoints)
        if n == 0:
            self._publish_status(False, 'REJECTED code=0; no markers to navigate')
            self.get_logger().warn('no markers to navigate')
            return
        if n == 1:
            self._publish_single_goal()
        else:
            self._navigate_through()
        self._waypoints.clear()   # consumed

    def _publish_markers(self):
        """Persistent red spheres at every clicked waypoint (camera_init frame)."""
        arr = MarkerArray()
        if not self._marker_points:
            d = Marker()
            d.header.frame_id = 'camera_init'
            d.ns = 'nav_waypoints'
            d.action = Marker.DELETEALL
            arr.markers.append(d)
        else:
            for i, (x, y) in enumerate(self._marker_points):
                m = Marker()
                m.header.frame_id = 'camera_init'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'nav_waypoints'
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.5      # hover slightly above water
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 1.0
                m.color.r = 1.0              # red
                m.color.g = 0.2
                m.color.b = 0.2
                m.color.a = 1.0
                m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # persistent
                arr.markers.append(m)
        self._pub_markers.publish(arr)

    def _publish_single_goal(self):
        (x, y) = self._waypoints[0]
        g = PoseStamped()
        g.header.frame_id = 'camera_init'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.position.z = 0.0
        g.pose.orientation.w = 1.0
        self._pub_goal.publish(g)
        self.get_logger().info(
            f'single marker -> goal ({x:.2f}, {y:.2f})')

    def _navigate_through(self):
        self._publish_status(False, 'QUEUED code=1; waiting for NavigateThroughPoses')
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self._publish_status(False, 'FAILED code=6; NavigateThroughPoses server not up')
            self.get_logger().warn('navigate_through_poses server not up')
            return
        goal = NavigateThroughPoses.Goal()
        for (x, y) in self._waypoints:
            p = PoseStamped()
            p.header.frame_id = 'camera_init'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            goal.poses.append(p)
        self.get_logger().info(
            f'sending {len(goal.poses)} markers to Nav2')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_through_goal_response)

    def _on_through_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._publish_status(False, f'FAILED code=6; action send failed: {exc}')
            self.get_logger().error(f'NavigateThroughPoses send failed: {exc}')
            return
        if not handle.accepted:
            self._publish_status(False, 'REJECTED code=0; Nav2 rejected multi-goal')
            self.get_logger().warn('Nav2 rejected multi-goal')
            return
        self._through_handle = handle
        self._publish_status(True, 'ACCEPTED code=1; Nav2 multi-goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_through_goal_result)

    def _on_through_goal_result(self, future):
        try:
            status = int(future.result().status)
            self._through_handle = None
            labels = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            label = labels.get(status, 'FAILED')
            self._publish_status(True, f'{label} code={status}')
            self.get_logger().info(
                f'NavigateThroughPoses finished: {label} (status={status})')
        except Exception as exc:
            self._through_handle = None
            self._publish_status(True, f'FAILED code=6; result read failed: {exc}')
            self.get_logger().warn(f'could not read multi-goal result: {exc}')

    def _publish_status(self, accepted: bool, text: str):
        self._accepted_pub.publish(Bool(data=bool(accepted)))
        self._status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = GzClickToGoalNode()
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
