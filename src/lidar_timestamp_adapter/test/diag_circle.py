"""Diagnose circling path: is the boat physically rotating (IMU/GPS ground
truth) or is LIO odometry drifting in a circle?"""
import rclpy, time, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

rclpy.init()
node = Node('diag_circle')
s = {}

def ocb(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    s['lio'] = (p.x, p.y, p.z, math.degrees(yaw))

def gcb(m):
    s['gps'] = (m.latitude, m.longitude)

def icb(m):
    s['gyro_z'] = m.angular_velocity.z

node.create_subscription(Odometry, '/aft_mapped_to_init', ocb, 10)
node.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', gcb, 10)
node.create_subscription(Imu, '/wamv/sensors/imu/imu/data', icb, 10)

def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

def gps_m(a, b):
    R = 6371000.0
    dlat = math.radians(b[0]-a[0]); dlon = math.radians(b[1]-a[1])
    return (dlon*math.cos(math.radians(a[0]))*R, dlat*R)

spin(4)
if 'lio' not in s:
    print("no LIO"); raise SystemExit
lio0, gps0 = s['lio'], s['gps']
gyros = []
samples = []
T = 30
t0 = time.time()
while time.time() - t0 < T:
    spin(1)
    gyros.append(s.get('gyro_z', 0.0))
    samples.append((time.time()-t0, s['lio'], gps_m(gps0, s['gps'])))

lio1 = s['lio']
d_lio = math.hypot(lio1[0]-lio0[0], lio1[1]-lio0[1])
gx, gy = gps_m(gps0, s['gps'])
d_gps = math.hypot(gx, gy)
dyaw = lio1[3] - lio0[3]
mean_gyro = sum(gyros)/len(gyros)
max_gyro = max(abs(g) for g in gyros)
print(f"over {T}s:")
print(f"  LIO XY moved:   {d_lio:.2f} m, LIO yaw changed: {dyaw:+.1f} deg")
print(f"  GPS XY moved:   {d_gps:.2f} m  (ground truth)")
print(f"  IMU gyro_z: mean {math.degrees(mean_gyro):+.2f} deg/s, max |{math.degrees(max_gyro):.2f}| deg/s (ground truth rotation)")
print()
if d_gps > 2 or abs(math.degrees(mean_gyro)) > 0.5:
    print("VERDICT: the BOAT IS PHYSICALLY MOVING/ROTATING (sim ground truth confirms)")
else:
    if d_lio > 2 or abs(dyaw) > 5:
        print("VERDICT: LIO DRIFT - boat is still but odometry moves")
    else:
        print("VERDICT: both still - path circle must be historical trajectory")
print()
print("trace (t, lio_xy, gps_xy):")
for t, l, g in samples[::5]:
    print(f"  t={t:4.1f}s lio=({l[0]:7.2f},{l[1]:7.2f}) yaw={l[3]:6.1f}  gps=({g[0]:6.2f},{g[1]:6.2f})")
