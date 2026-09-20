"""Verify cloud_filter output: time field present, in [0, scan_period],
correlated with azimuth; no non-finite points; other fields passed through."""
import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('output_check')
done = []

def cb(msg):
    t = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    names = [f.name for f in msg.fields]
    dt = np.dtype({'names': names,
                   'formats': [t[f.datatype] for f in msg.fields],
                   'offsets': [f.offset for f in msg.fields],
                   'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dt)
    print(f"fields: {names}")
    print(f"points: {len(pts)}, point_step={msg.point_step}, is_dense={msg.is_dense}")
    fin = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
    print(f"non-finite remaining: {(~fin).sum()}")
    if 'time' in names:
        tm = pts['time']
        print(f"time: min={tm.min():.4f} max={tm.max():.4f} (expect 0..0.1)")
        az = np.unwrap(np.radians(np.degrees(np.arctan2(pts['y'], pts['x']))))
        r0 = pts['ring'] == pts['ring'][0]
        if r0.sum() > 100:
            corr = np.corrcoef(tm[r0], az[r0])[0, 1]
            print(f"corr(time, azimuth) within ring {pts['ring'][0]}: {corr:.3f}")
    else:
        print("ERROR: no time field!")
    print(f"ring range: {pts['ring'].min()}..{pts['ring'].max()}")
    done.append(1)

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(PointCloud2, '/wamv/points_filtered', cb, qos)
end = time.time() + 20
while not done and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.5)
if not done:
    print("NO FILTERED CLOUD (is cloud_filter running?)")
