"""Inspect the navsat pipeline inputs/outputs: IMU yaw, GPS odom, EKF odom."""
import rclpy, time, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

rclpy.init()
node = Node('diag_navsat')
s = {}

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

node.create_subscription(Imu, '/wamv/sensors/imu/imu/data',
    lambda m: s.__setitem__('imu_yaw', yaw_of(m.orientation)), 10)
node.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix',
    lambda m: s.__setitem__('gps', (m.latitude, m.longitude)), 10)
node.create_subscription(Odometry, '/odometry/gps',
    lambda m: s.__setitem__('gpsodom', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
node.create_subscription(Odometry, '/odometry/filtered',
    lambda m: s.__setitem__('ekf', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
node.create_subscription(Odometry, '/aft_mapped_to_init',
    lambda m: s.__setitem__('lio', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)

for i in range(6):
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f"[{i*3:2d}s] imu_yaw={math.degrees(s.get('imu_yaw', float('nan'))):7.2f}deg  "
          f"gps_odom={tuple(round(v,1) for v in s.get('gpsodom', (0,0)))}  "
          f"ekf={tuple(round(v,1) for v in s.get('ekf', (0,0)))}  "
          f"lio={tuple(round(v,1) for v in s.get('lio', (0,0)))}")
