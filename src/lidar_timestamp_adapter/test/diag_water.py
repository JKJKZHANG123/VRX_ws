"""How many finite returns are water-surface hits?
Lidar is level-mounted ~1.8m above deck; in sensor frame the water plane
sits around z ~= -1.5..-2.5 (waves). Histogram z to see the split."""
import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('diag_water')
done = []

def cb(msg):
    t = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    dt = np.dtype({'names':[f.name for f in msg.fields],
                   'formats':[t[f.datatype] for f in msg.fields],
                   'offsets':[f.offset for f in msg.fields],
                   'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dt)
    fin = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
    z = pts['z'][fin]
    r = np.hypot(pts['x'][fin], pts['y'][fin])
    print(f"finite: {fin.sum()}")
    for lo, hi in [(-5,-2.5),(-2.5,-2.0),(-2.0,-1.5),(-1.5,-1.0),(-1.0,-0.5),(-0.5,0.5),(0.5,2),(2,10),(10,100)]:
        m = (z >= lo) & (z < hi)
        if m.sum():
            print(f"  z [{lo:5.1f},{hi:5.1f}): {m.sum():6d} pts  ({100*m.sum()/fin.sum():4.1f}%)  r: {r[m].min():.1f}..{r[m].max():.1f}m")
    # candidate water cut: z < -1.2
    wm = z < -1.2
    print(f"z < -1.2 (water candidate): {wm.sum()} ({100*wm.sum()/fin.sum():.1f}%)")
    done.append(1)

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', cb, qos)
end = time.time() + 20
while not done and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.5)
if not done:
    print("no cloud")
