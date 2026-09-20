"""M1 end-to-end check: stationary stability, then drive and compare
LIO XY displacement against GPS ground displacement."""
import rclpy, time, math
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

rclpy.init()
node = Node('m1_motion_check')
lt = node.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
rt = node.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
state = {'n': 0}

def ocb(m):
    p = m.pose.pose.position
    state['lio'] = (p.x, p.y, p.z)
    state['n'] += 1

def gcb(m):
    state['gps'] = (m.latitude, m.longitude)

node.create_subscription(Odometry, '/aft_mapped_to_init', ocb, 10)
node.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', gcb, 10)

def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

def gps_m(a, b):
    R = 6371000.0
    dlat = math.radians(b[0] - a[0])
    dlon = math.radians(b[1] - a[1])
    return math.hypot(dlon * math.cos(math.radians(a[0])) * R, dlat * R)

def drive(l, r, sec):
    ml, mr = Float64(), Float64()
    ml.data, mr.data = float(l), float(r)
    end = time.time() + sec
    while time.time() < end:
        lt.publish(ml); rt.publish(mr)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.08)

spin(5)
if 'lio' not in state:
    print("FAIL: no LIO odometry")
    raise SystemExit(1)

# 1) stationary 20s
p0 = state['lio']; n0 = state['n']
spin(20)
p1 = state['lio']
drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
rate = (state['n'] - n0) / 20.0
print(f"[stationary 20s] XY drift={drift:.3f}m, odom rate={rate:.0f}Hz")

# 2) forward 30s, compare with GPS
lio0, gps0 = state['lio'], state['gps']
drive(350, 350, 30)
drive(0, 0, 2)
spin(6)
lio1, gps1 = state['lio'], state['gps']
d_lio = math.hypot(lio1[0] - lio0[0], lio1[1] - lio0[1])
d_gps = gps_m(gps0, gps1)
err = abs(d_lio - d_gps)
print(f"[forward 30s] LIO={d_lio:.2f}m GPS={d_gps:.2f}m err={err:.2f}m ({100*err/max(d_gps,0.01):.1f}%)")
print(f"[forward 30s] Z drift={lio1[2]-lio0[2]:.2f}m")

# 3) turn then forward, still alive?
drive(300, -300, 12)
drive(300, 300, 10)
drive(0, 0, 2)
n1 = state['n']
spin(5)
alive = state['n'] > n1
print(f"[turn+fwd] odometry alive: {alive}, final pose: "
      f"({state['lio'][0]:.1f}, {state['lio'][1]:.1f}, {state['lio'][2]:.1f})")
print("M1 CHECK", "PASS" if alive and err / max(d_gps, 0.01) < 0.10 else "REVIEW")
