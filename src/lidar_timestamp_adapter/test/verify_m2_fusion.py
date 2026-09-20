"""M2 acceptance: compare raw Point-LIO drift vs fused EKF output.
Stationary phase: fused output must stay bounded while raw LIO may wander.
Uses GPS as ground truth for translation."""
import rclpy, time, math, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0

rclpy.init()
node = Node('m2_check')
s = {}

def mk(key):
    def cb(m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        s[key] = (p.x, p.y, p.z, yaw)
    return cb

def gcb(m):
    s['gps'] = (m.latitude, m.longitude)

node.create_subscription(Odometry, '/aft_mapped_to_init', mk('lio'), 10)
node.create_subscription(Odometry, '/odometry/filtered', mk('ekf'), 10)
node.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', gcb, 10)

def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

def gps_m(a, b):
    R = 6371000.0
    dlat = math.radians(b[0]-a[0]); dlon = math.radians(b[1]-a[1])
    return math.hypot(dlon*math.cos(math.radians(a[0]))*R, dlat*R)

spin(5)
missing = [k for k in ('lio', 'ekf', 'gps') if k not in s]
if missing:
    print(f"MISSING topics: {missing}"); raise SystemExit(1)

lio0, ekf0, gps0 = s['lio'], s['ekf'], s['gps']
print(f"start: lio=({lio0[0]:.2f},{lio0[1]:.2f}) ekf=({ekf0[0]:.2f},{ekf0[1]:.2f})")
t0 = time.time()
max_lio = max_ekf = 0.0
while time.time() - t0 < DURATION:
    spin(10)
    lio, ekf, gps = s['lio'], s['ekf'], s['gps']
    d_lio = math.hypot(lio[0]-lio0[0], lio[1]-lio0[1])
    d_ekf = math.hypot(ekf[0]-ekf0[0], ekf[1]-ekf0[1])
    d_gps = gps_m(gps0, gps)
    max_lio = max(max_lio, abs(d_lio - d_gps))
    max_ekf = max(max_ekf, abs(d_ekf - d_gps))
    t = time.time() - t0
    print(f"t={t:5.0f}s  LIO_err={d_lio-d_gps:+7.2f}m  EKF_err={d_ekf-d_gps:+6.2f}m  (gps moved {d_gps:.2f}m)  ekf_z={ekf[2]:.2f}")

print()
print(f"max |LIO error| = {max_lio:.2f} m")
print(f"max |EKF error| = {max_ekf:.2f} m")
print("M2 STATIONARY", "PASS" if max_ekf < 1.5 else "FAIL")
