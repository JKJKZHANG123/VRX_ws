"""Measure Point-LIO processing lag: odometry header stamp vs /clock.
If lag grows over time, the pipeline can't keep up with the input rate."""
import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

rclpy.init()
node = Node('diag_lag')
s = {}

def ocb(m):
    s['odom_t'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

def ccb(m):
    s['clock'] = m.clock.sec + m.clock.nanosec * 1e-9

node.create_subscription(Odometry, '/aft_mapped_to_init', ocb, 10)
node.create_subscription(Clock, '/clock', ccb, 10)

t0 = time.time()
while time.time() - t0 < 25:
    end = time.time() + 5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if 'odom_t' in s and 'clock' in s:
        lag = s['clock'] - s['odom_t']
        print(f"t={time.time()-t0:4.1f}s  sim_clock={s['clock']:8.1f}  odom_stamp={s['odom_t']:8.1f}  LAG={lag:6.2f}s")
    else:
        print("waiting for topics...")
