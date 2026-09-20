#!/usr/bin/env python3
"""Covariance injector for the M2 EKF fusion chain.

Root cause of the /odometry/filtered -> NaN blow-up (diagnosed 2026-08-10):
robot_localization fuses ABSOLUTE pose measurements from two sources that both
publish an ALL-ZERO pose covariance:

  * /aft_mapped_to_init  -- Point-LIO only fills covariance when odom_only=true;
                            VRX runs mapping mode (odom_only=false) for
                            /cloud_registered, so the matrix stays zero.
  * /wamv/.../gps/fix     -- the gz NavSatFix has position_covariance = 0,
                            type = 0 (UNKNOWN); navsat_transform passes the
                            zero straight through to /odometry/gps.

Zero covariance == infinite certainty -> the EKF update step inverts a singular
innovation covariance -> NaN state -> navsat bakes the NaN/huge value into its
UTM datum -> positive feedback. Fixing either source alone is not enough; both
absolute anchors must carry a finite covariance.

This node is a pure relay (zero intrusion into Point-LIO or the sim):
  in : /aft_mapped_to_init          -> out: /aft_mapped_to_init/cov
  in : /wamv/sensors/gps/gps/fix    -> out: /wamv/sensors/gps/gps/fix_cov
Only the covariance fields are touched; all data is passed through untouched.
The EKF (odom0) and navsat_transform (gps/fix) are remapped onto the /cov
outputs in localization.launch.py.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def _rot_z(q, offset):
    """Return (x, y, z, w) of quaternion q rotated by `offset` rad about z.

    camera_init differs from odom/ENU by a pure z-rotation (the spawn yaw), so
    pre-multiplying by Rz(offset) is exact here; roll/pitch are preserved.
    """
    cr, sr = math.cos(offset / 2.0), math.sin(offset / 2.0)
    return (
        cr * q.x - sr * q.y,
        cr * q.y + sr * q.x,
        cr * q.z + sr * q.w,
        cr * q.w - sr * q.z,
    )


class CovarianceInjector(Node):
    def __init__(self):
        super().__init__('covariance_injector')

        # --- tunable variances (declared as params for field integ.) ---
        # LIO pose: only YAW is fused by the EKF (x/y/z/roll/pitch are false in
        # odom0_config; two_d_mode drops z/roll/pitch). The yaw is ENU-rotated
        # by spawn_yaw_offset below. Covariances here must stay finite to keep
        # the EKF update matrix regular.
        self.declare_parameter('lio_pos_var', 0.01)     # x,y  (m^2)
        self.declare_parameter('lio_z_var', 0.05)       # z    (m^2)
        self.declare_parameter('lio_rot_var', 0.02)     # roll,pitch,yaw (rad^2)
        # LIO twist: this is what the EKF actually fuses (odom0 = vx,vy,vyaw).
        # Point-LIO's twist is clean (~+/-3 m/s), unlike its 2.7 kHz pose stream
        # whose nanosecond dt jitter blew up differential-mode fusion. Trust it
        # moderately ("high-weight velocity" per the plan) but not infinitely.
        self.declare_parameter('lio_vel_var', 0.05)     # vx,vy (m/s)^2
        self.declare_parameter('lio_vz_var', 0.10)      # vz    (m/s)^2
        self.declare_parameter('lio_vyaw_var', 0.02)    # vyaw  (rad/s)^2
        # Throttle LIO republish rate. Point-LIO emits ~2.7 kHz; forwarding all
        # of it makes the 30 Hz EKF chew ~90 queued msgs/cycle and occasionally
        # miss its update rate ("Failed to meet update rate"). 50 Hz is ample for
        # velocity fusion on a slow boat and subsumes the dt<=0 monotonic guard.
        self.declare_parameter('lio_min_period_s', 0.02)   # 50 Hz
        # GPS absolute XY: sim GPS is RTK-clean. This is the ONLY absolute
        # position anchor (LIO contributes velocity only), so it must pull hard
        # enough to stop LIO velocity noise from integrating into metres of
        # stationary drift. 0.25 m^2 (0.5 m std) held EKF stationary error well
        # under the 1.5 m M2 gate. History: 1.0 too loose (~3.3 m stationary
        # wander); 0.25 fixed static but left ~13% motion overshoot (EKF ran
        # ahead of GPS while integrating LIO velocity); 0.1 anchors position
        # harder to pull that overshoot back.
        self.declare_parameter('gps_pos_var', 0.1)      # E,N  (m^2)
        self.declare_parameter('gps_alt_var', 3.0)      # up   (m^2)

        # Frame reconciliation: the WAM-V spawns at yaw = 1.0 rad in the ENU
        # world (competition.launch.py). Point-LIO's camera_init (== its odom
        # header frame) inherits that spawn-relative heading, while the EKF is
        # anchored to ENU GPS (navsat.yaml yaw_offset handled GPS; the fused LIO
        # yaw never was). We rotate the relayed LIO pose by +spawn_yaw_offset so
        # the EKF's fused heading is ENU-consistent with its GPS position --
        # otherwise the controller steers toward a goal bearing computed in ENU
        # while the boat's actual travel direction is off by ~57 deg -> circling.
        # Must stay in sync with the static TF odom->camera_init (rotation -yaw,
        # localization.launch.py) and navsat.yaml (yaw_offset -> 0 once EKF yaw
        # is already ENU). Twist (vx/vy/vyaw) is body-frame and is NOT rotated.
        self.declare_parameter('spawn_yaw_offset', 1.0)    # rad

        self.lio_pos = self.get_parameter('lio_pos_var').value
        self.lio_z = self.get_parameter('lio_z_var').value
        self.lio_rot = self.get_parameter('lio_rot_var').value
        self.lio_vel = self.get_parameter('lio_vel_var').value
        self.lio_vz = self.get_parameter('lio_vz_var').value
        self.lio_vyaw = self.get_parameter('lio_vyaw_var').value
        self.lio_min_period_ns = int(
            self.get_parameter('lio_min_period_s').value * 1e9)
        self.gps_pos = self.get_parameter('gps_pos_var').value
        self.gps_alt = self.get_parameter('gps_alt_var').value
        self.spawn_yaw = self.get_parameter('spawn_yaw_offset').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.pub_odom = self.create_publisher(
            Odometry, '/aft_mapped_to_init/cov', sensor_qos)
        self.pub_gps = self.create_publisher(
            NavSatFix, '/wamv/sensors/gps/gps/fix_cov', sensor_qos)

        self.create_subscription(
            Odometry, '/aft_mapped_to_init', self.on_odom, sensor_qos)
        self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/gps/fix', self.on_gps, sensor_qos)

        self._warned_odom = False
        self._warned_gps = False
        self._last_odom_stamp = None   # ns, for monotonic-guard
        self._dropped_dt0 = 0
        self._dropped_nan = 0
        self.get_logger().info(
            'covariance_injector up: '
            '/aft_mapped_to_init->/aft_mapped_to_init/cov, '
            '/gps/fix->/gps/fix_cov')

    def on_odom(self, msg: Odometry):
        # --- guard 1: drop non-finite frames ---------------------------------
        # A single NaN/Inf pose permanently poisons the downstream EKF (it never
        # recovers), so filter defensively even though Point-LIO is usually clean.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in vals):
            self._dropped_nan += 1
            if self._dropped_nan in (1, 100):
                self.get_logger().warn(
                    f'dropped non-finite LIO odom frame '
                    f'(total {self._dropped_nan})')
            return

        # --- guard 2: throttle to lio_min_period (also drops dt <= 0) --------
        # ROOT CAUSE of the EKF NaN blow-up (diagnosed 2026-08-10): Point-LIO
        # publishes at ~2.7 kHz and emits ~2% of frames <0.01 ms after the last
        # (min 1 ns, ~1% exactly dt == 0). robot_localization computed a velocity
        # from those, and a near-zero dt amplified mm jitter into ~1e6 m/s ->
        # position ran to millions of metres / NaN. Enforcing a minimum inter-
        # frame period both kills the near-zero-dt spikes AND caps the rate the
        # 30 Hz EKF must service (was 869 Hz -> occasional "Failed to meet update
        # rate"). Any frame closer than lio_min_period to the last kept one --
        # including dt <= 0 -- is dropped.
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_odom_stamp is not None \
                and stamp - self._last_odom_stamp < self.lio_min_period_ns:
            self._dropped_dt0 += 1
            if self._dropped_dt0 in (1, 1000, 10000):
                self.get_logger().info(
                    f'throttled LIO odom frame (< min period), '
                    f'total {self._dropped_dt0}')
            return
        self._last_odom_stamp = stamp

        cov = list(msg.pose.covariance)
        # inject only when Point-LIO left the pose block zero (mapping mode)
        if all(v == 0.0 for v in cov):
            cov = [0.0] * 36
            cov[0] = self.lio_pos    # x
            cov[7] = self.lio_pos    # y
            cov[14] = self.lio_z     # z
            cov[21] = self.lio_rot   # roll
            cov[28] = self.lio_rot   # pitch
            cov[35] = self.lio_rot   # yaw
            msg.pose.covariance = cov
            if not self._warned_odom:
                self.get_logger().info(
                    'LIO pose covariance was zero -> injecting diagonal')
                self._warned_odom = True

        # twist covariance: Point-LIO also leaves this zero in mapping mode, and
        # the EKF fuses vx,vy,vyaw from here -> must be finite or the update is
        # singular. Inject a diagonal whenever it arrives zero.
        tcov = list(msg.twist.covariance)
        if all(v == 0.0 for v in tcov):
            tcov = [0.0] * 36
            tcov[0] = self.lio_vel    # vx
            tcov[7] = self.lio_vel    # vy
            tcov[14] = self.lio_vz    # vz
            tcov[21] = self.lio_vyaw  # vroll (unused, keep finite)
            tcov[28] = self.lio_vyaw  # vpitch (unused, keep finite)
            tcov[35] = self.lio_vyaw  # vyaw
            msg.twist.covariance = tcov

        # Reconcile heading frame: rotate the relayed pose so its yaw is in the
        # ENU odom frame (spawn-relative -> absolute). Twist untouched (body frame).
        if self.spawn_yaw != 0.0:
            x, y, z, w = _rot_z(msg.pose.pose.orientation, self.spawn_yaw)
            msg.pose.pose.orientation.x = x
            msg.pose.pose.orientation.y = y
            msg.pose.pose.orientation.z = z
            msg.pose.pose.orientation.w = w
        self.pub_odom.publish(msg)

    def on_gps(self, msg: NavSatFix):
        pc = msg.position_covariance
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN \
                or all(v == 0.0 for v in pc):
            msg.position_covariance = [
                self.gps_pos, 0.0, 0.0,
                0.0, self.gps_pos, 0.0,
                0.0, 0.0, self.gps_alt,
            ]
            msg.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            if not self._warned_gps:
                self.get_logger().info(
                    'GPS covariance was unknown/zero -> injecting diagonal')
                self._warned_gps = True
        self.pub_gps.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CovarianceInjector()
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
