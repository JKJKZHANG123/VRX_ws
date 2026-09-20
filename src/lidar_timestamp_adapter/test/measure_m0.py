#!/usr/bin/env python3
"""M0 verification: measure sensor rates + point cloud format over a fixed window."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix


class M0Meter(Node):
    def __init__(self):
        super().__init__('m0_meter')
        self.t0 = time.time()
        self.n_pc = self.n_imu = self.n_gps = 0
        self.pc_fields = None
        self.pc_size = None
        self.create_subscription(
            PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.pc_cb, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, qos_profile_sensor_data)

    def pc_cb(self, msg):
        self.n_pc += 1
        if self.pc_fields is None:
            self.pc_fields = [f.name for f in msg.fields]
            self.pc_size = (msg.height, msg.width, msg.height * msg.width)

    def imu_cb(self, msg):
        self.n_imu += 1

    def gps_cb(self, msg):
        self.n_gps += 1


def main():
    rclpy.init()
    node = M0Meter()
    window = 12.0
    end = time.time() + window
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    dt = time.time() - node.t0
    print(f"window: {dt:.1f}s")
    print(f"LiDAR points: {node.n_pc/dt:.2f} Hz  (n={node.n_pc})")
    print(f"IMU:          {node.n_imu/dt:.2f} Hz  (n={node.n_imu})")
    print(f"GPS:          {node.n_gps/dt:.2f} Hz  (n={node.n_gps})")
    print(f"PC2 fields:   {node.pc_fields}")
    print(f"PC2 size:     {node.pc_size}  (height, width, total)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
