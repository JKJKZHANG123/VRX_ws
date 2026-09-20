#!/usr/bin/env python3
"""Adapt the VRX gpu_ray point cloud for Point-LIO's velodyne handler.

Two defects of the raw Gazebo cloud make Point-LIO unusable without this node:

1. No-return rays carry inf coordinates (60-80% of points over open water,
   despite is_dense=true). Point-LIO has no finite-check and the ikd-Tree
   chokes on them.
2. There is no per-point 'time' field. PCL's fromROSMsg leaves the unmapped
   struct member as UNINITIALIZED MEMORY, so Point-LIO's given_offset_time
   detection reads garbage: runs randomly either work (garbage <= 0, azimuth
   fallback kicks in) or stall forever / crash in cos_sinc_sqrt (garbage > 0
   pushes lidar_end_time centuries ahead).

The input cloud is organized (height=16 rings, width=1875 azimuth steps,
row-major), so the exact firing time of column j is j/width * scan_period.
We append that as a float32 'time' field in seconds (timestamp_unit: 0),
then drop non-finite points and flatten.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class CloudAdapter(Node):
    def __init__(self):
        super().__init__('lidar_timestamp_adapter')
        self.declare_parameter('input_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/points')
        self.declare_parameter('output_topic', '/wamv/points_filtered')
        self.declare_parameter('scan_period', 0.1)  # 1 / lidar update_rate
        # The VRX LiDAR is approximately 1.8 m above the water. Reject the
        # unstable water sheet while retaining low shoreline and buoy returns.
        self.declare_parameter('reject_below_sensor_z', True)
        self.declare_parameter('sensor_z_min', -1.5)
        self.scan_period = float(self.get_parameter('scan_period').value)
        self.reject_below_sensor_z = bool(
            self.get_parameter('reject_below_sensor_z').value)
        self.sensor_z_min = float(self.get_parameter('sensor_z_min').value)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self.callback, qos)
        self.in_dtype = None
        self.out_dtype = None
        self.out_fields = None
        self.logged = False

    TYPE_MAP = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

    def build_dtypes(self, msg):
        names = [f.name for f in msg.fields]
        formats = [self.TYPE_MAP[f.datatype] for f in msg.fields]
        offsets = [f.offset for f in msg.fields]
        self.in_dtype = np.dtype({'names': names, 'formats': formats,
                                  'offsets': offsets, 'itemsize': msg.point_step})
        # output layout: input fields + trailing float32 'time'
        self.out_dtype = np.dtype({'names': names + ['time'],
                                   'formats': formats + ['f4'],
                                   'offsets': offsets + [msg.point_step],
                                   'itemsize': msg.point_step + 4})
        self.out_fields = list(msg.fields) + [
            PointField(name='time', offset=msg.point_step,
                       datatype=PointField.FLOAT32, count=1)]

    def callback(self, msg):
        if self.in_dtype is None:
            self.build_dtypes(msg)
        pts = np.frombuffer(msg.data, dtype=self.in_dtype)
        n = len(pts)
        out = np.zeros(n, dtype=self.out_dtype)
        for name in self.in_dtype.names:
            out[name] = pts[name]
        # organized cloud: row-major, column index = azimuth step
        width = msg.width if msg.width > 1 else n
        cols = np.arange(n) % width
        out['time'] = (cols / float(width) * self.scan_period).astype(np.float32)

        finite = (np.isfinite(out['x']) & np.isfinite(out['y'])
                  & np.isfinite(out['z']))
        # Water in the LiDAR frame lies near z=-1.8 m. It is dynamic and nearly
        # planar, so feeding it to scan matching makes stationary Point-LIO
        # appear to translate and sink. Keep the lower bound at -1.5 m: direct VRX samples put the marker-buoy
        # returns around -1.33 m and round-buoy returns around -1.1 m in the
        # LiDAR frame, while the water sheet is around -1.8 m. A -1.2 m cutoff
        # removes the marker buoys before Point-LIO can map them.
        if self.reject_below_sensor_z:
            selected = finite & (out['z'] >= self.sensor_z_min)
        else:
            selected = finite
        kept = out[selected]
        if not self.logged:
            non_finite_count = int((~finite).sum())
            below_sensor_z_count = int(
                (finite & (out['z'] < self.sensor_z_min)).sum())
            z_filter_status = (
                f'enabled at {self.sensor_z_min:.2f}m; '
                f'{below_sensor_z_count} low points dropped'
                if self.reject_below_sensor_z else
                f'disabled; {below_sensor_z_count} finite points below '
                f'{self.sensor_z_min:.2f}m retained')
            self.get_logger().info(
                f'adapting: {n} -> {len(kept)} pts; '
                f'dropped {non_finite_count} non-finite; '
                f'sensor z filter {z_filter_status}; '
                f'time field appended (0..{self.scan_period}s over {width} cols)')
            self.logged = True

        m = PointCloud2()
        m.header = msg.header
        m.height = 1
        m.width = len(kept)
        m.fields = self.out_fields
        m.is_bigendian = msg.is_bigendian
        m.point_step = self.out_dtype.itemsize
        m.row_step = m.point_step * len(kept)
        m.is_dense = True
        m.data = kept.tobytes()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = CloudAdapter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
