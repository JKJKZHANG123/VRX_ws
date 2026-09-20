#!/usr/bin/env python3
"""
水面点云过滤 (方向3)

目的: 从雷达点云里剔除水面反射点和水花离群噪声, 保留真实障碍物
      (浮标/岸线/他船), 输出干净点云供后续 costmap / 避障使用.

处理流程 (纯几何 + 可选 RGB 语义, 全部离线):
  1. 非有限点剔除 (Gazebo 无回波射线输出 inf)
  2. 高度带裁剪: 在重力对齐系里保留水面以上 [z_min, z_max] 的点
     - 若能拿到里程计姿态则用它做重力对齐; 否则退化为传感器系近似
       (船在波浪中大致水平, two_d_mode 前提一致)
  3. 半径离群滤波: 去掉孤立的水花/浪尖噪点 (scipy cKDTree)

输入:  /wamv/sensors/lidars/lidar_wamv_sensor/points  (原始, 含 inf)
输出:  /usv/obstacle_cloud                             (过滤后, 供 costmap)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from scipy.spatial import cKDTree


class WaterFilter(Node):
    def __init__(self):
        super().__init__('water_filter')

        self.declare_parameter('cloud_topic',
                               '/wamv/sensors/lidars/lidar_wamv_sensor/points')
        self.declare_parameter('output_topic', '/usv/obstacle_cloud')
        # 高度带 (传感器系, 雷达在水面上方约 1.8m; 水面反射点 z≈-1.8):
        #   z_min 略高于水面, 砍掉水面反射; z_max 砍掉高空无关点
        self.declare_parameter('z_min', -1.5)   # 保留水面以上
        self.declare_parameter('z_max', 8.0)    # 高空裁剪
        self.declare_parameter('range_min', 2.0)    # 船体自遮挡
        self.declare_parameter('range_max', 100.0)
        # 半径离群滤波: 半径 r 内邻居少于 k 个 → 判为离群 (水花)
        self.declare_parameter('outlier_radius', 0.6)
        self.declare_parameter('outlier_min_neighbors', 4)
        self.declare_parameter('enable_outlier', True)
        self.declare_parameter('voxel_size', 0.15)   # 降采样, <=0 关闭

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.out_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.cb, 5)
        self.pub = self.create_publisher(PointCloud2, self.out_topic, 5)

        self.get_logger().info('WaterFilter 已启动')
        self.get_logger().info(f'  输入: {self.cloud_topic}')
        self.get_logger().info(f'  输出: {self.out_topic}')

    def cb(self, msg: PointCloud2):
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if pts.shape[0] == 0:
            return
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float32)

        n0 = xyz.shape[0]

        # 1. 非有限点剔除
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        if xyz.shape[0] == 0:
            return

        # 2. 距离裁剪 (船体自遮挡 + 远处)
        rng = np.linalg.norm(xyz, axis=1)
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        rmin = self.get_parameter('range_min').value
        rmax = self.get_parameter('range_max').value
        keep = (rng >= rmin) & (rng <= rmax) & \
               (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
        xyz = xyz[keep]
        if xyz.shape[0] == 0:
            self._publish(xyz, msg)
            return
        n_after_band = xyz.shape[0]

        # 3. 体素降采样 (减轻离群滤波和 costmap 负担)
        vs = self.get_parameter('voxel_size').value
        if vs and vs > 0:
            keys = np.floor(xyz / vs).astype(np.int64)
            _, idx = np.unique(keys, axis=0, return_index=True)
            xyz = xyz[np.sort(idx)]

        # 4. 半径离群滤波 (去水花/浪尖孤立点)
        if self.get_parameter('enable_outlier').value and xyz.shape[0] > 10:
            r = self.get_parameter('outlier_radius').value
            kmin = self.get_parameter('outlier_min_neighbors').value
            tree = cKDTree(xyz)
            # 查每个点半径 r 内的邻居数 (含自身), 少于 kmin+1 判离群
            counts = tree.query_ball_point(xyz, r, return_length=True)
            inlier = counts >= (kmin + 1)
            xyz = xyz[inlier]

        n_out = xyz.shape[0]
        self._publish(xyz, msg)

        self.get_logger().info(
            f'过滤: {n0} → 高度带{n_after_band} → 输出{n_out} '
            f'({n0 - n_out} 剔除)',
            throttle_duration_sec=5.0)

    def _publish(self, xyz: np.ndarray, src: PointCloud2):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header = src.header
        out = pc2.create_cloud(header, fields, xyz.astype(np.float32))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WaterFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
