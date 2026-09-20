"""Dump the live TF tree: which frames exist and who publishes what."""
import rclpy, time
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

rclpy.init()
node = Node('tf_dump')
dyn, sta = {}, {}

def mk(store):
    def cb(m):
        for t in m.transforms:
            store[(t.header.frame_id, t.child_frame_id)] = (
                round(t.transform.translation.x, 3),
                round(t.transform.translation.y, 3),
                round(t.transform.translation.z, 3))
    return cb

node.create_subscription(TFMessage, '/tf', mk(dyn), 50)
node.create_subscription(TFMessage, '/tf_static', mk(sta), 50)
end = time.time() + 8
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)

print("=== /tf (dynamic) ===")
for (p, c), tr in sorted(dyn.items()):
    print(f"  {p} -> {c}  {tr}")
print("=== /tf_static ===")
for (p, c), tr in sorted(sta.items()):
    print(f"  {p} -> {c}  {tr}")
