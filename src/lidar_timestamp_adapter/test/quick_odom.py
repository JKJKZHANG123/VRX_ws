"""Ten-second odometry liveness check: message count + latest pose."""
import rclpy, time
from rclpy.node import Node
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('quick_odom')
s = {'n': 0}

def cb(m):
    p = m.pose.pose.position
    s['n'] += 1
    s['pose'] = (round(p.x, 2), round(p.y, 2), round(p.z, 2))

node.create_subscription(Odometry, '/aft_mapped_to_init', cb, 10)
end = time.time() + 10
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
print(f"odom msgs in 10s: {s['n']}, latest pose: {s.get('pose')}")
