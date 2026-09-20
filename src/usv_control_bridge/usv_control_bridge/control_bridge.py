#!/usr/bin/env python3
"""
M5: WAM-V Control Bridge (feed-forward)
=======================================
Maps Nav2's desired velocity to WAM-V differential thrust.

Data flow:
  /cmd_vel_smoothed  (Twist: desired vx + wz from Nav2)
        |
  surge feed-forward:  F  = k_v_lin*vx + k_v_quad*vx*|vx|   (drag model)
  yaw   feed-forward:  dF = k_yaw * wz * yaw_sign
        | differential allocation (H-config)
  left  = F - dF
  right = F + dF
        |
  /wamv/thrusters/{left,right}/thrust  (std_msgs/Float64)

Why feed-forward (NO velocity PID)?
-----------------------------------
The EKF /odometry/filtered TWIST is unreliable: the velocity state diverges
to +/-60 m/s while the boat physically moves at <1 m/s (LIO body-frame
velocity fights GPS-position-implied velocity when the yaw estimate drifts).
Feeding that into a velocity PID drove the thrusters full-scale forward /
reverse every cycle -> violent twitching.

Nav2's RPP is ALREADY a closed-loop controller: it observes robot pose vs
the path and outputs a velocity command. This bridge only converts that
command into the thrust that produces it at steady state (drag-model
feed-forward). RPP closes the outer loop through position, so no inner
velocity feedback is needed -- and we avoid the garbage EKF twist entirely.

Feed-forward drag coefficients come from the WAM-V URDF:
  x_u  (linear drag)    = 51.3
  x_uu (quadratic drag) = 72.4
  (v=1 -> 124 N, v=2 -> 392 N, v=3 -> 805 N)

Yaw convention:
  With the WAM-V aft-thruster layout (left thruster at +y, right at -y, and
  +thrust = forward along +x), a positive yaw-rate command (CCW, +wz) requires
  MORE thrust on the RIGHT thruster to yaw CCW. The differential allocation
  below (left = F - dF, right = F + dF, with dF = k_yaw*wz*yaw_sign) is the
  physical convention. If the boat is observed circling the WRONG way, the
  upstream RPP/odometry sign is the bug; `yaw_sign` is the one safe inverter
  here (flip to -1.0). It is exposed as a parameter for exactly that reason.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class ControlBridge(Node):
    def __init__(self):
        super().__init__('control_bridge')

        # Surge feed-forward drag model: F = k_v_lin*v + k_v_quad*v*|v|
        self.declare_parameter('k_v_lin', 51.3)    # linear drag (URDF x_u)
        self.declare_parameter('k_v_quad', 72.4)   # quadratic drag (URDF x_uu)
        # Yaw feed-forward: dF = k_yaw * wz
        self.declare_parameter('k_yaw', 200.0)
        # Yaw feed-forward SIGN. With the WAM-V aft-thruster layout (left at +y,
        # right at -y; +thrust = forward along +x), a positive yaw-rate command
        # (CCW) needs MORE right thrust, i.e. dF = +k_yaw*wz on the right and - on
        # the left. If the boat circles the wrong way this is the prime suspect --
        # flip to -1.0 to invert.
        self.declare_parameter('yaw_sign', 1.0)

        # Safety
        self.declare_parameter('max_thrust_n', 1000.0)
        self.declare_parameter('max_slew_n_per_step', 250.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('min_thrust_deadband', 2.0)
        # Independent forward-corridor stop. Nav2 remains the planner, but
        # this last-resort gate prevents stale/late costmap updates from
        # driving the boat into a shoreline. The input cloud is in base_link.
        self.declare_parameter('safety_cloud_topic', '/usv/safety_cloud')
        # Two-stage direct-LiDAR gate.  The warning corridor may permit a
        # deliberately slow, clearly commanded turn.  The emergency envelope
        # covers the hull plus margin and ALWAYS commands zero thrust.
        self.declare_parameter('safety_emergency_min_x_m', -2.8)
        self.declare_parameter('safety_emergency_max_x_m', 3.5)
        self.declare_parameter('safety_emergency_half_width_m', 1.8)
        self.declare_parameter('safety_stop_distance_m', 6.0)
        self.declare_parameter('safety_corridor_start_x_m', 3.5)
        self.declare_parameter('safety_corridor_half_width_m', 2.4)
        self.declare_parameter('safety_min_points', 3)
        self.declare_parameter('safety_cloud_timeout_s', 2.0)
        # When an obstacle is detected ahead, do not freeze a valid Nav2
        # avoidance command.  Allow a low-speed, same-sign differential
        # thrust escape so the hull can turn onto the replanned path.
        self.declare_parameter('safety_escape_linear_velocity_mps', 0.18)
        self.declare_parameter('safety_escape_min_yaw_rate_rps', 0.08)
        # Speed-invariant turn test for the warning corridor. See the escape
        # branch in _control_loop for why an absolute yaw-rate floor alone is
        # the wrong unit here. 0.0 disables the curvature test.
        self.declare_parameter('safety_escape_min_curvature_rad_per_m', 0.10)
        # Swept-path escape test: judge the commanded arc against the observed
        # returns instead of asking whether the command is a turn.
        self.declare_parameter('safety_swept_horizon_s', 6.0)
        self.declare_parameter('safety_swept_half_width_m', 1.1)
        self.declare_parameter('safety_swept_margin_m', 0.6)
        # Speed the warning corridor caps surge to (m/s). This corridor no
        # longer STOPS the boat -- it only slows it while keeping enough yaw
        # authority to steer, letting Nav2's planned detour execute. Impact
        # protection is the emergency envelope, not this. Keep >= the Nav2
        # regulated_linear_scaling_min_speed so a capped command can still turn.
        self.declare_parameter('safety_warning_speed_mps', 0.5)
        self.declare_parameter('max_yaw_difference_ratio', 0.45)
        # Yaw authority floor for the escape branch.
        #
        # max_dF is normally a fraction of the surge thrust F, which is correct
        # while cruising: the differential must not overpower forward thrust and
        # spin the hull in place.  But in the warning corridor vx is capped to
        # safety_escape_linear_velocity_mps, and F collapses with it
        # (0.18 m/s -> ~11.6 N), so max_dF fell to ~5.2 N and the boat could no
        # longer turn.  It then crept straight at the obstacle until its bow
        # entered the inscribed inflation and Smac aborted with "Start occupied".
        #
        # Sizing the escape yaw limit from a reference speed instead decouples
        # "go slow" from "cannot steer".  0.0 restores the old coupled behavior.
        self.declare_parameter('yaw_authority_ref_vx_mps', 0.6)
        self.declare_parameter('goal_status_topic', '/usv/goal_status')

        self._kvl = self.get_parameter('k_v_lin').value
        self._kvq = self.get_parameter('k_v_quad').value
        self._kyaw = self.get_parameter('k_yaw').value
        self._yaw_sign = self.get_parameter('yaw_sign').value
        self._max_t = self.get_parameter('max_thrust_n').value
        self._slew = self.get_parameter('max_slew_n_per_step').value
        self._tout = self.get_parameter('cmd_timeout_s').value
        self._db = self.get_parameter('min_thrust_deadband').value
        self._safety_topic = str(self.get_parameter('safety_cloud_topic').value)
        self._safety_emergency_min_x = float(
            self.get_parameter('safety_emergency_min_x_m').value)
        self._safety_emergency_max_x = float(
            self.get_parameter('safety_emergency_max_x_m').value)
        self._safety_emergency_half_width = float(
            self.get_parameter('safety_emergency_half_width_m').value)
        self._safety_dist = float(self.get_parameter('safety_stop_distance_m').value)
        self._safety_start_x = float(
            self.get_parameter('safety_corridor_start_x_m').value)
        self._safety_half_width = float(
            self.get_parameter('safety_corridor_half_width_m').value)
        self._safety_min_points = int(self.get_parameter('safety_min_points').value)
        self._safety_timeout = float(
            self.get_parameter('safety_cloud_timeout_s').value)
        self._safety_escape_v = float(
            self.get_parameter('safety_escape_linear_velocity_mps').value)
        self._safety_escape_w = float(
            self.get_parameter('safety_escape_min_yaw_rate_rps').value)
        self._safety_escape_curv = float(
            self.get_parameter('safety_escape_min_curvature_rad_per_m').value)
        self._swept_horizon = float(
            self.get_parameter('safety_swept_horizon_s').value)
        self._swept_half_width = float(
            self.get_parameter('safety_swept_half_width_m').value)
        self._swept_margin = float(
            self.get_parameter('safety_swept_margin_m').value)
        self._safety_warn_speed = float(
            self.get_parameter('safety_warning_speed_mps').value)
        self._max_yaw_ratio = float(
            self.get_parameter('max_yaw_difference_ratio').value)
        self._yaw_ref_vx = float(
            self.get_parameter('yaw_authority_ref_vx_mps').value)
        # Reference surge thrust used only as a floor on the escape yaw limit.
        # Linear drag term only, deliberately: including the quadratic term
        # would put max_dF (~25 N) far above the capped surge thrust (~12 N) and
        # drive the retreating thruster hard negative, reintroducing the in-place
        # counter-rotation that the allocation below exists to prevent.  With the
        # linear term the differential stays close to F, so the retreating
        # thruster only grazes slightly negative at the limit.
        self._yaw_ref_thrust = self._kvl * self._yaw_ref_vx
        self._goal_status_topic = str(
            self.get_parameter('goal_status_topic').value)

        self._des_vx = 0.0
        self._des_wz = 0.0
        self._left_prev = 0.0
        self._right_prev = 0.0
        self._last_cmd_t = None
        self._safety_cloud_t = None
        self._safety_ready = False
        self._safety_stop = False
        self._safety_emergency = False
        # Forward returns from the latest safety cloud, base_link, for the
        # swept-path escape test. Empty means "nothing to judge against".
        self._forward_points = []
        # A planner/controller abort can leave the last smoothed command alive
        # for a short interval.  Treat the goal gateway's terminal state as a
        # hard command interlock instead of waiting for cmd_vel timeout.
        self._goal_terminal = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._pub_left = self.create_publisher(
            Float64, '/wamv/thrusters/left/thrust', qos)
        self._pub_right = self.create_publisher(
            Float64, '/wamv/thrusters/right/thrust', qos)
        self._pub_safety = self.create_publisher(
            Bool, '/usv/safety_stop', qos)
        self._pub_emergency = self.create_publisher(
            Bool, '/usv/collision_emergency_stop', qos)
        # Consume the smoothed Nav2 command directly.
        # Chain: controller -> /cmd_vel -> velocity_smoother -> /cmd_vel_smoothed -> here.
        self.create_subscription(
            Twist, '/cmd_vel_smoothed', self._on_cmd_vel, qos)
        self.create_subscription(
            PointCloud2, self._safety_topic, self._on_obstacle_cloud,
            rclpy.qos.qos_profile_sensor_data)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            String, self._goal_status_topic, self._on_goal_status, status_qos)

        self.create_timer(0.05, self._control_loop)   # 20 Hz
        self.get_logger().info(
            f'control_bridge (feed-forward) ready  '
            f'k_v_lin={self._kvl} k_v_quad={self._kvq} k_yaw={self._kyaw} '
            f'yaw_sign={self._yaw_sign}  max={self._max_t:.0f}N, '
            f'safety_topic={self._safety_topic}, '
            f'goal_status_topic={self._goal_status_topic}, '
            f'emergency_box=x[{self._safety_emergency_min_x:.1f},'
            f'{self._safety_emergency_max_x:.1f}]m +/-y{self._safety_emergency_half_width:.1f}, '
            f'warning_corridor=x[{self._safety_start_x:.1f},'
            f'{self._safety_start_x + self._safety_dist:.1f}]m +/-y{self._safety_half_width:.1f}, '
            f'yaw_ratio={self._max_yaw_ratio:.2f}, '
            f'escape_v={self._safety_escape_v:.2f}m/s, '
            f'escape_w={self._safety_escape_w:.2f}rad/s')

    def _on_cmd_vel(self, msg: Twist):
        self._des_vx = msg.linear.x
        self._des_wz = msg.angular.z
        self._last_cmd_t = self.get_clock().now()

    def _on_goal_status(self, msg: String):
        status = (msg.data or '').strip().upper()
        # IDLE/QUEUED/ACCEPTED/EXECUTING are non-terminal.  REJECTED and all
        # explicit terminal action results must stop propulsion immediately.
        self._goal_terminal = (
            status.startswith(('SUCCEEDED', 'CANCELED', 'CANCELLED',
                               'ABORTED', 'FAILED', 'REJECTED')))
        if self._goal_terminal:
            self._zero_thrust(f'goal terminal: {msg.data}')

    def _on_obstacle_cloud(self, msg: PointCloud2):
        # The filter publishes the direct raw-LiDAR safety cloud in base_link.
        # Only use it for the emergency gate when that invariant is true;
        # otherwise a world-frame cloud could incorrectly stop the boat.
        self._safety_cloud_t = self.get_clock().now()
        if msg.header.frame_id not in ('wamv/wamv/base_link', 'base_link'):
            self._safety_ready = False
            self._set_emergency(True, 'unexpected safety-cloud frame')
            self._set_safety(True, 'unexpected safety-cloud frame')
            return
        self._safety_ready = True
        warning_hits = 0
        emergency_hits = 0
        # Keep the forward returns themselves, not just a hit count. The escape
        # decision needs to know WHERE the obstacle is so it can test whether the
        # commanded arc actually approaches it (see _control_loop).
        forward_points = []
        try:
            for x, y, z in point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True):
                if abs(z) > 5.0:
                    continue
                if (x > -1.0 and x <= self._safety_start_x + self._safety_dist
                        + 4.0 and abs(y) <= self._safety_half_width + 4.0):
                    forward_points.append((float(x), float(y)))
                in_emergency = (
                    self._safety_emergency_min_x <= x <=
                    self._safety_emergency_max_x and
                    abs(y) <= self._safety_emergency_half_width)
                in_warning = (
                    self._safety_start_x <= x <=
                    self._safety_start_x + self._safety_dist and
                    abs(y) <= self._safety_half_width)
                if in_emergency:
                    emergency_hits += 1
                elif in_warning:
                    warning_hits += 1
                # NOTE: no early break. The old loop stopped as soon as both hit
                # counts were satisfied, which is fine for a boolean gate but
                # would truncate forward_points and make the swept-path test
                # below judge the arc against a partial obstacle set.
        except Exception as exc:
            self.get_logger().warn(
                f'safety cloud parse failed: {exc}',
                throttle_duration_sec=5.0)
            self._safety_ready = False
            self._set_emergency(True, 'safety cloud parse failed')
            self._set_safety(True, 'safety cloud parse failed')
            return

        emergency = emergency_hits >= self._safety_min_points
        warning = warning_hits >= self._safety_min_points
        self._set_emergency(
            emergency,
            f'obstacle inside hull envelope ({emergency_hits} points)'
            if emergency else '')
        self._set_safety(
            warning,
            f'obstacle in warning corridor ({warning_hits} points)'
            if warning else '')
        self._forward_points = forward_points

    def _swept_path_clearance(self, vx: float, wz: float) -> float:
        """Min clearance between the commanded arc and forward returns (m).

        Integrates the (vx, wz) command into the short-horizon path the hull
        would actually trace, then measures the closest approach of any forward
        obstacle point to that path, minus the hull half width.  inf means
        nothing relevant is in front.
        """
        points = self._forward_points
        if not points or vx <= 1e-3:
            return float('inf')
        steps = 16
        dt = self._swept_horizon / steps
        x = y = th = 0.0
        path = [(0.0, 0.0)]
        for _ in range(steps):
            x += vx * math.cos(th) * dt
            y += vx * math.sin(th) * dt
            th += wz * dt
            path.append((x, y))
        best = float('inf')
        for px, py in points:
            for (ax, ay), (bx, by) in zip(path, path[1:]):
                dx, dy = bx - ax, by - ay
                seg = dx * dx + dy * dy
                if seg <= 0.0:
                    continue
                t = ((px - ax) * dx + (py - ay) * dy) / seg
                t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
                if d < best:
                    best = d
            if best - self._swept_half_width <= 0.0:
                break
        return best - self._swept_half_width

    def _set_emergency(self, value: bool, reason: str = ''):
        value = bool(value)
        if value != self._safety_emergency:
            self._safety_emergency = value
            if value:
                self.get_logger().error(
                    f'COLLISION EMERGENCY STOP: {reason or "unsafe hull envelope"}')
            else:
                self.get_logger().info('collision emergency stop cleared')
        msg = Bool()
        msg.data = value
        self._pub_emergency.publish(msg)

    def _set_safety(self, value: bool, reason: str = ''):
        value = bool(value)
        if value != self._safety_stop:
            self._safety_stop = value
            if value:
                self.get_logger().warn(
                    f'FORWARD SAFETY STOP: {reason or "unsafe perception state"}')
            else:
                self.get_logger().info('forward safety stop cleared')
        msg = Bool()
        msg.data = value
        self._pub_safety.publish(msg)

    def _control_loop(self):
        now = self.get_clock().now()

        # Last-resort collision gate. It is deliberately independent of Nav2:
        # a stale costmap or a bad route must result in zero thrust, not a
        # full-speed impact. A stale cloud is not treated as clear.
        if self._safety_cloud_t is None:
            self._set_emergency(True, 'waiting for direct LiDAR safety cloud')
            self._set_safety(True, 'waiting for direct LiDAR safety cloud')
            self._zero_thrust('waiting for obstacle cloud')
            return
        cloud_age = (now - self._safety_cloud_t).nanoseconds * 1e-9
        if cloud_age > self._safety_timeout or not self._safety_ready:
            self._set_emergency(True, 'direct LiDAR cloud stale/unavailable')
            self._set_safety(True, 'direct LiDAR cloud stale/unavailable')
            self._zero_thrust('obstacle cloud stale/unavailable')
            return
        # Watchdog: zero thrust if cmd_vel stale (or never received).
        # A live avoidance command is allowed to turn out of the emergency
        # corridor; stopping both thrusters here used to deadlock the boat
        # while Nav2 was already publishing a valid detour.
        if self._goal_terminal:
            self._zero_thrust('goal terminal')
            return
        if self._last_cmd_t is None:
            return
        age = (now - self._last_cmd_t).nanoseconds * 1e-9
        if age > self._tout:
            self._zero_thrust('watchdog: cmd_vel stale')
            return

        vx = self._des_vx
        wz = self._des_wz
        # Emergency envelope is a non-negotiable stop.  Do not let an escape
        # command continue to push the hull after the obstacle has entered the
        # physical boat envelope.
        if self._safety_emergency:
            self._zero_thrust('obstacle inside hull safety envelope')
            return

        escaping = False
        if self._safety_stop:
            # An obstacle sits in the warning corridor (3.5-6.5 m ahead). This
            # is a caution distance, NOT a collision distance: the hull-geometry
            # emergency envelope handled just above (x[-2.8, 3.5]) owns actual
            # impact protection and remains a hard stop. The warning corridor's
            # only job is to slow the boat while it still has room to steer, so
            # that Nav2's already-planned detour can be executed.
            #
            # Earlier revisions ZEROED thrust here unless some test proved the
            # command was actively avoiding -- first a yaw-rate floor, then a
            # curvature floor, then a swept-arc clearance check. Every one failed
            # the same way, because RPP approaches an obstacle on a nearly
            # straight command and only yaws once it reaches the detour arc. So
            # any "are you avoiding yet?" test reads the straight approach leg as
            # unsafe and cuts thrust, and the boat stalls ~5 m short with
            # "Failed to make progress" (report3 static_plan_20260901
            # plan06/plan10/plan13/plan15; the swept check logged clearance
            # 0.19-0.58 m < 0.6 while the PLANNED path cleared the obstacle by
            # >2 m). The planner's costmap + footprint collision check already
            # guarantees the path clears the obstacle; overriding it from here
            # only deadlocks navigation.
            #
            # Therefore: do not stop in the warning corridor. Cap surge to keep
            # yaw authority (a slower hull turns wider) and let Nav2 steer. The
            # clearance is still computed and logged for diagnostics only.
            clearance = self._swept_path_clearance(vx, wz)
            if vx > 0.0:
                vx = min(vx, self._safety_warn_speed)
                escaping = True
                self.get_logger().info(
                    f'warning corridor: cap vx={vx:.2f} m/s, Nav2 retains '
                    f'steering (swept clearance={clearance:.2f} m, diagnostic)',
                    throttle_duration_sec=2.0)

        # Surge feed-forward (drag model): thrust to hold desired vx
        F = self._kvl * vx + self._kvq * vx * abs(vx)

        # Yaw feed-forward: differential thrust from desired wz.
        # left = F - dF, right = F + dF  => +wz -> more RIGHT thrust -> CCW turn.
        dF = self._kyaw * wz * self._yaw_sign
        # Never let the differential command overpower forward thrust.
        # Counter-rotating thrusters turn the WAM-V in place; that caused the
        # observed circles and made the planner's collision stop ineffective.
        # RPP already supplies a forward command while tracking a Dubins path,
        # so bounded same-sign thrust is the safe allocation here.
        # While escaping, size the differential limit from the reference speed
        # rather than the capped surge thrust, so a slow pass still steers.
        # Cruise behavior is unchanged: outside the escape branch this is
        # exactly abs(F) * max_yaw_difference_ratio as before.
        yaw_budget = abs(F)
        if escaping:
            yaw_budget = max(yaw_budget, self._yaw_ref_thrust)
        max_dF = max(0.0, yaw_budget * self._max_yaw_ratio)
        dF = max(-max_dF, min(max_dF, dF))

        left_raw = F - dF
        right_raw = F + dF

        left = self._slew_limit(left_raw, self._left_prev)
        right = self._slew_limit(right_raw, self._right_prev)

        if abs(left) < self._db:
            left = 0.0
        if abs(right) < self._db:
            right = 0.0

        self._publish(left, right)
        self._left_prev = left
        self._right_prev = right

        self.get_logger().debug(
            f'des[vx={vx:+.2f} wz={self._des_wz:+.2f}] '
            f'F={F:.0f} dF={dF:.0f} L={left:.0f} R={right:.0f}',
            throttle_duration_sec=1.0)

    def _slew_limit(self, desired: float, previous: float) -> float:
        delta = desired - previous
        delta = max(-self._slew, min(self._slew, delta))
        return max(-self._max_t, min(self._max_t, previous + delta))

    def _zero_thrust(self, reason: str = ''):
        if self._left_prev != 0.0 or self._right_prev != 0.0:
            self.get_logger().info(f'zeroing thrust: {reason}')
        self._publish(0.0, 0.0)
        self._left_prev = 0.0
        self._right_prev = 0.0

    def _publish(self, left: float, right: float):
        lm, rm = Float64(), Float64()
        lm.data, rm.data = float(left), float(right)
        self._pub_left.publish(lm)
        self._pub_right.publish(rm)


def main(args=None):
    rclpy.init(args=args)
    node = ControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('shutting down - zeroing thrusters')
        node._zero_thrust('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
