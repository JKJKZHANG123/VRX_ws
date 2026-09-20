#!/usr/bin/env python3
"""Recover the Gazebo-to-ROS simulation clock bridge if it stops publishing."""

import signal
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock


class ClockBridgeWatchdog(Node):
    def __init__(self):
        super().__init__('clock_bridge_watchdog')
        self.declare_parameter('timeout_sec', 3.0)
        self.declare_parameter('startup_grace_sec', 10.0)
        self.declare_parameter('check_period_sec', 1.0)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.startup_grace_sec = float(
            self.get_parameter('startup_grace_sec').value)
        check_period = float(self.get_parameter('check_period_sec').value)

        self.started_at = time.monotonic()
        self.last_clock_at = None
        self.recovery = None
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Clock, '/clock', self._on_clock, clock_qos)
        self.create_timer(check_period, self._check_clock)
        self.get_logger().info(
            f'watching /clock (timeout={self.timeout_sec:.1f}s, '
            f'grace={self.startup_grace_sec:.1f}s)')

    def _on_clock(self, _msg):
        self.last_clock_at = time.monotonic()

    def _bridge_command(self):
        prefix = Path(get_package_prefix('ros_gz_bridge'))
        executable = prefix / 'lib' / 'ros_gz_bridge' / 'parameter_bridge'
        return [
            str(executable),
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args', '-r', '__node:=clock_recovery_bridge',
        ]

    def _check_clock(self):
        now = time.monotonic()
        if self.recovery is not None and self.recovery.poll() is not None:
            self.get_logger().error(
                f'clock recovery bridge exited with code {self.recovery.returncode}')
            self.recovery = None

        if self.last_clock_at is None:
            stale_for = now - self.started_at
            stale = stale_for >= self.startup_grace_sec
        else:
            stale_for = now - self.last_clock_at
            stale = stale_for >= self.timeout_sec

        if stale and self.recovery is None:
            self.get_logger().error(
                f'/clock has been silent for {stale_for:.1f}s; '
                'starting a dedicated recovery bridge')
            # Keep the child in this launch service process group. A group kill
            # from usv.sh then terminates both watchdog and recovery bridge.
            self.recovery = subprocess.Popen(self._bridge_command())

    def close(self):
        if self.recovery is None or self.recovery.poll() is not None:
            return
        self.recovery.send_signal(signal.SIGINT)
        try:
            self.recovery.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.recovery.kill()
            self.recovery.wait(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ClockBridgeWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
