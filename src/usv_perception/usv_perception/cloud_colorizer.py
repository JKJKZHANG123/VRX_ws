#!/usr/bin/env python3
"""
方向2: LiDAR-相机彩色点云融合 (VRX 版)

将相机 RGB 投影到 LiDAR 点云上, 输出彩色点云.
相比真机版 (lidar_camera_fusion), VRX 的关键简化:
  - VRX 发布 optical/ 系相机话题, 自带标准光学系 TF (x右y下z前),
    外参直接从 tf2 取, 无需手写 LiDAR->相机 90° 基准旋转.
  - 仿真相机无畸变 (d 全 0), 直接用针孔模型投影.

输入:  LiDAR 点云 + 相机 optical image_raw + optical camera_info + tf
输出:  /usv/colored_cloud   (逐帧, 雷达系, 带 rgb 字段)
       /usv/colored_map     (累积, 全局系 camera_init, 体素降采样)  [可选]

彩色点云既是可视化增强, 也是方向1浮标 3D 定位、方向3语义过滤的公共投影基础.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

from tf2_ros import Buffer, TransformListener


class CloudColorizer(Node):
    def __init__(self):
        super().__init__('cloud_colorizer')

        self.declare_parameter('cloud_topic', '/wamv/sensors/lidars/lidar_wamv_sensor/points')
        self.declare_parameter('image_topic',
                               '/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/wamv/sensors/cameras/front_left_camera_sensor/optical/camera_info')
        self.declare_parameter('colored_cloud_topic', '/usv/colored_cloud')
        self.declare_parameter('min_depth', 0.2)        # 相机前方最小深度 (米)
        self.declare_parameter('max_time_diff', 0.15)   # 点云/图像最大时间差 (秒)

        # 累积全局彩色地图
        self.declare_parameter('accumulate', True)
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('global_frame', 'camera_init')
        self.declare_parameter('colored_map_topic', '/usv/colored_map')
        self.declare_parameter('voxel_size', 0.2)       # 体素降采样 (米)
        self.declare_parameter('map_publish_period', 1.0)
        self.declare_parameter('max_map_points', 3000000)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.caminfo_topic = self.get_parameter('camera_info_topic').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_dt = self.get_parameter('max_time_diff').value

        self.accumulate = self.get_parameter('accumulate').value
        self.global_frame = self.get_parameter('global_frame').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_map_points = self.get_parameter('max_map_points').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.latest_image = None    # (rgb_np, stamp)
        self.latest_cloud = None    # (msg, stamp)
        self.K = None               # (fx, fy, cx, cy, w, h)
        self.cam_frame = None       # optical frame id (from camera_info)
        self.latest_odom_T = None
        self.voxel_map = {}
        self.map_dirty = False

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, 5)
        self.create_subscription(Image, self.image_topic, self.image_cb, 5)
        self.caminfo_sub = self.create_subscription(
            CameraInfo, self.caminfo_topic, self.caminfo_cb, 5)

        self.colored_pub = self.create_publisher(
            PointCloud2, self.get_parameter('colored_cloud_topic').value, 5)

        if self.accumulate:
            self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                     self.odom_cb, 5)
            map_qos = QoSProfile(depth=1)
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            map_qos.history = HistoryPolicy.KEEP_LAST
            self.map_pub = self.create_publisher(
                PointCloud2, self.get_parameter('colored_map_topic').value, map_qos)
            self.create_timer(self.get_parameter('map_publish_period').value,
                              self.publish_map)

        self.create_timer(0.1, self.process)

        self.get_logger().info('CloudColorizer (VRX) 已启动')
        self.get_logger().info(f'  点云: {self.cloud_topic}')
        self.get_logger().info(f'  图像: {self.image_topic}')
        self.get_logger().info(f'  外参: tf2 (optical 系, 无需手工基准旋转)')
        self.get_logger().info(f'  累积地图: {"开" if self.accumulate else "关"}')

    def caminfo_cb(self, msg: CameraInfo):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)
        self.cam_frame = msg.header.frame_id
        self.get_logger().info(
            f'相机内参: {msg.width}x{msg.height} fx={msg.k[0]:.1f} '
            f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} frame={self.cam_frame}')
        self.destroy_subscription(self.caminfo_sub)

    def image_cb(self, msg: Image):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_image = (rgb, msg.header.stamp)
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}', throttle_duration_sec=5.0)

    def cloud_cb(self, msg: PointCloud2):
        self.latest_cloud = (msg, msg.header.stamp)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom_T = self._tf_to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    @staticmethod
    def _tf_to_matrix(x, y, z, qx, qy, qz, qw):
        # 四元数 -> 旋转矩阵 (无 scipy 依赖, 手写更稳)
        n = qx*qx + qy*qy + qz*qz + qw*qw
        if n < 1e-12:
            return np.eye(4)
        s = 2.0 / n
        R = np.array([
            [1 - s*(qy*qy+qz*qz), s*(qx*qy-qz*qw),     s*(qx*qz+qy*qw)],
            [s*(qx*qy+qz*qw),     1 - s*(qx*qx+qz*qz), s*(qy*qz-qx*qw)],
            [s*(qx*qz-qy*qw),     s*(qy*qz+qx*qw),     1 - s*(qx*qx+qy*qy)],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def get_extrinsic(self, lidar_frame):
        """tf2 取 lidar -> camera_optical 的 4x4 变换"""
        try:
            tfs = self.tf_buffer.lookup_transform(
                self.cam_frame, lidar_frame, rclpy.time.Time())
            t = tfs.transform.translation
            q = tfs.transform.rotation
            return self._tf_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        except Exception:
            return None

    def project_and_color(self, cloud_msg, rgb_img, T):
        pts = pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return None, None
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float32)
        # 有些字段是 inf (VRX 无回波点), skip_nans 不管 inf, 手动滤
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        if xyz.shape[0] == 0:
            return None, None

        N = xyz.shape[0]
        homo = np.hstack([xyz, np.ones((N, 1), dtype=np.float32)])
        cam = (T @ homo.T).T[:, :3]

        fx, fy, cx, cy, w_img, h_img = self.K
        front = cam[:, 2] > self.min_depth
        cam = cam[front]
        xyz = xyz[front]
        if xyz.shape[0] == 0:
            return None, None

        Z = cam[:, 2]
        u = (fx * cam[:, 0] / Z + cx).astype(np.int32)
        v = (fy * cam[:, 1] / Z + cy).astype(np.int32)
        inside = (u >= 0) & (u < w_img) & (v >= 0) & (v < h_img)
        u, v = u[inside], v[inside]
        xyz = xyz[inside]
        if xyz.shape[0] == 0:
            return None, None

        colors = rgb_img[v, u, :]  # rgb8 -> 已是 RGB 顺序
        return xyz, colors

    def process(self):
        if self.latest_cloud is None or self.latest_image is None or self.K is None:
            return
        cloud_msg, cstamp = self.latest_cloud
        rgb_img, istamp = self.latest_image
        ct = cstamp.sec + cstamp.nanosec * 1e-9
        it = istamp.sec + istamp.nanosec * 1e-9
        if abs(ct - it) > self.max_dt:
            return

        T = self.get_extrinsic(cloud_msg.header.frame_id)
        if T is None:
            return

        xyz, rgb = self.project_and_color(cloud_msg, rgb_img, T)
        if xyz is None:
            return

        msg = self._build_cloud(xyz, rgb, cloud_msg.header, cloud_msg.header.frame_id)
        self.colored_pub.publish(msg)

        if self.accumulate and self.latest_odom_T is not None:
            self._accumulate(xyz, rgb)

        self.get_logger().info(
            f'着色点数: {xyz.shape[0]}'
            + (f' | 地图体素: {len(self.voxel_map)}' if self.accumulate else ''),
            throttle_duration_sec=5.0)

    def _accumulate(self, xyz, rgb):
        N = xyz.shape[0]
        homo = np.hstack([xyz, np.ones((N, 1), dtype=np.float32)])
        g = (self.latest_odom_T @ homo.T).T[:, :3]
        vs = self.voxel_size
        if vs and vs > 0:
            keys = np.floor(g / vs).astype(np.int64)
            for i in range(N):
                k = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
                self.voxel_map[k] = (g[i, 0], g[i, 1], g[i, 2],
                                     rgb[i, 0], rgb[i, 1], rgb[i, 2])
        if len(self.voxel_map) > self.max_map_points:
            self.get_logger().warn('彩色地图达点数上限', throttle_duration_sec=10.0)
        self.map_dirty = True

    def publish_map(self):
        if not self.map_dirty or len(self.voxel_map) == 0:
            return
        if self.map_pub.get_subscription_count() == 0:
            return
        vals = np.array(list(self.voxel_map.values()), dtype=np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        msg = self._build_cloud(vals[:, 0:3], vals[:, 3:6], header, self.global_frame)
        self.map_pub.publish(msg)
        self.map_dirty = False

    @staticmethod
    def _build_cloud(xyz, rgb, header, frame_id):
        r = rgb[:, 0].astype(np.uint32)
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 2].astype(np.uint32)
        rgb_uint = (r << 16) | (g << 8) | b
        rgb_float = rgb_uint.view(np.float32)
        data = np.zeros((xyz.shape[0], 4), dtype=np.float32)
        data[:, 0:3] = xyz
        data[:, 3] = rgb_float
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out_header = header
        out_header.frame_id = frame_id
        return pc2.create_cloud(out_header, fields, data)


def main(args=None):
    rclpy.init(args=args)
    node = CloudColorizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
