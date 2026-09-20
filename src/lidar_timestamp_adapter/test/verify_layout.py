"""Verify the M1 assumption behind cloud_filter_node's time synthesis:
the raw VRX cloud is organized 16(rings) x 1875(azimuth cols), row-major,
so column index tracks azimuth. Checks ring-vs-row and azimuth-vs-column."""
import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('layout_check')
done = []

def cb(msg):
    t = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    dt = np.dtype({'names':[f.name for f in msg.fields],
                   'formats':[t[f.datatype] for f in msg.fields],
                   'offsets':[f.offset for f in msg.fields],
                   'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dt)
    print(f"height={msg.height} width={msg.width} total={len(pts)}")
    n = len(pts)
    W = msg.width
    rows = np.arange(n) // W
    ring = pts['ring'].astype(int)
    match_row = (ring == rows).mean()
    cols = np.arange(n) % W
    finite = np.isfinite(pts['x']) & np.isfinite(pts['y'])
    print(f"ring==row_idx match: {100*match_row:.1f}%")
    print(f"ring range: {ring.min()}..{ring.max()}")
    az = np.degrees(np.arctan2(pts['y'], pts['x']))
    best = None
    for r in range(max(msg.height, 1)):
        m = (rows == r) & finite
        if best is None or m.sum() > best[1]:
            best = (r, m.sum(), m)
    r, cnt, m = best
    a = az[m]; c = cols[m]
    da = np.diff(a)
    da = (da + 180) % 360 - 180
    mono = (np.abs(da) < 5).mean()
    print(f"row {r}: {cnt} finite pts, azimuth span {a.min():.0f}..{a.max():.0f} deg, "
          f"consecutive-step<5deg fraction: {100*mono:.1f}%")
    if cnt > 100:
        corr = np.corrcoef(c, np.unwrap(np.radians(a)))[0, 1]
        print(f"corr(col_index, unwrapped azimuth) on row {r}: {corr:.3f}")
    done.append(1)

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', cb, qos)
end = time.time() + 25
while not done and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.5)
if not done:
    print("NO CLOUD RECEIVED")
