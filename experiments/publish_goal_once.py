#!/usr/bin/env python3
"""Reliably publish one goal while avoiding duplicate Nav2 submissions."""

import argparse
import math
import re
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class GoalPublisher(Node):
    def __init__(self, topic: str):
        super().__init__('dynamic_trial_goal_publisher')
        goal_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(PoseStamped, topic, goal_qos)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.status = 'IDLE code=0'
        self.status_generation = 0
        self.status_sequence = None
        self.create_subscription(String, '/usv/goal_status', self._on_status,
                                 status_qos)

    def _on_status(self, msg: String):
        self.status = msg.data
        self.status_generation += 1
        match = re.search(r'\bstatus_seq=(\d+)', msg.data)
        if match:
            self.status_sequence = int(match.group(1))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/goal_pose')
    parser.add_argument('--frame-id', default='camera_init')
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout-s', type=float, default=10.0)
    parser.add_argument('--period-s', type=float, default=0.5)
    parser.add_argument('--hold-s', type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = GoalPublisher(args.topic)
    try:
        deadline = time.monotonic() + args.timeout_s
        while node.publisher.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                node.get_logger().error('no goal subscriber became ready')
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)

        # First receive the current transient-local status before taking the
        # baseline.  Without this gate, the latched IDLE message can arrive
        # just after the baseline and be mistaken for the status of this goal.
        initial_deadline = min(deadline, time.monotonic() + 5.0)
        while node.status_generation == 0 and time.monotonic() < initial_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.status_generation == 0:
            node.get_logger().error('no initial /usv/goal_status message became ready')
            return 3

        # Drain any duplicate transient-local delivery, then capture a strict
        # baseline.  A new status must be published after this point and, when
        # available, carry a larger gateway sequence number.
        drain_end = time.monotonic() + 1.0
        while time.monotonic() < drain_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        baseline_generation = node.status_generation
        baseline_sequence = node.status_sequence
        node.get_logger().info(
            f'baseline status={node.status!r}; generation={baseline_generation}; '
            f'status_seq={baseline_sequence}')

        goal = PoseStamped()
        goal.header.frame_id = args.frame_id
        goal.pose.position.x = args.x
        goal.pose.position.y = args.y
        goal.pose.position.z = args.z
        goal.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.pose.orientation.w = math.cos(args.yaw / 2.0)
        node.publisher.publish(goal)
        node.get_logger().info(
            f'published goal ({args.x:.3f}, {args.y:.3f}), yaw={args.yaw:.3f}')
        last_generation = baseline_generation
        last_sequence = baseline_sequence
        saw_fresh_status = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            is_new_publication = node.status_generation > last_generation
            sequence_is_new = (
                node.status_sequence is None or last_sequence is None or
                node.status_sequence > last_sequence)
            if not (is_new_publication and sequence_is_new):
                continue

            last_generation = node.status_generation
            last_sequence = node.status_sequence
            saw_fresh_status = True
            status = node.status
            node.get_logger().info(f'goal gateway changed status to: {status}')
            print(f'STATUS:{status}', flush=True)
            upper = status.upper()
            if re.search(r'\b(ACCEPTED|EXECUTING)\b', upper):
                node.get_logger().info(
                    'Nav2 accepted the goal; target publication verified')
                hold_end = time.monotonic() + args.hold_s
                while time.monotonic() < hold_end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                return 0
            if re.search(r'\b(REJECTED|FAILED|ABORTED|CANCELED|CANCELLED|SUCCEEDED)\b', upper):
                node.get_logger().error(
                    f'goal reached terminal status before acceptance: {status}')
                return 4

        if saw_fresh_status:
            node.get_logger().error(
                'goal gateway never reported ACCEPTED/EXECUTING after the fresh status')
        else:
            node.get_logger().error('goal gateway did not publish a fresh status')
        return 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
