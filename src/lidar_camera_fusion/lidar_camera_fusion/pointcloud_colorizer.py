#!/usr/bin/env python3
"""
LiDAR Point Cloud Colorizer
将 RGB 图像颜色投影到 LiDAR 点云上, 输出彩色点云

核心流程:
1. 订阅 /unilidar/cloud (LiDAR 点云) 和 /camera/color/image_raw (RGB)
2. 通过 tf2 获取 LiDAR→Camera 外参
3. 将每个 LiDAR 点投影到图像平面, 采样颜色
4. 发布 RGB 彩色点云 (PointCloud2 with RGB field)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from cv_bridge import CvBridge
import numpy as np
import struct
import cv2
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class PointCloudColorizer(Node):
    def __init__(self):
        super().__init__('pointcloud_colorizer')

        # ---- 参数 ----
        self.declare_parameter('lidar_frame', 'unilidar_lidar')
        self.declare_parameter('camera_frame', 'camera_color_frame')
        self.declare_parameter('cloud_topic', '/unilidar/cloud')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('colored_cloud_topic', '/colored_cloud')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('min_depth', 0.1)       # 最小投影深度 (米), 滤除太近的点
        self.declare_parameter('max_range', 100.0)      # LiDAR 点最大距离 (米)
        self.declare_parameter('use_tf_extrinsic', True) # 从 tf 获取外参; False 则用参数手动指定

        # ---- 外参微调量 (在内置 LiDAR->相机光学基准旋转之上叠加) ----
        # 带 range 描述, rqt_reconfigure 会显示为滑块, 可实时拖动标定
        from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

        def _range_desc(desc, lo, hi, step):
            return ParameterDescriptor(
                description=desc,
                floating_point_range=[FloatingPointRange(
                    from_value=lo, to_value=hi, step=step)])

        # 平移微调: ±0.5 米 (杆臂), 步进 1mm
        self.declare_parameter('extrinsic_x', 0.0, _range_desc('平移X 微调(m)', -0.5, 0.5, 0.001))
        self.declare_parameter('extrinsic_y', 0.0, _range_desc('平移Y 微调(m)', -0.5, 0.5, 0.001))
        self.declare_parameter('extrinsic_z', 0.0, _range_desc('平移Z 微调(m)', -0.5, 0.5, 0.001))
        # 旋转微调: ±30 度 (安装误差), 步进 0.1 度
        self.declare_parameter('extrinsic_roll',  0.0, _range_desc('旋转Roll 微调(deg)',  -30.0, 30.0, 0.1))
        self.declare_parameter('extrinsic_pitch', 0.0, _range_desc('旋转Pitch 微调(deg)', -30.0, 30.0, 0.1))
        self.declare_parameter('extrinsic_yaw',   0.0, _range_desc('旋转Yaw 微调(deg)',   -30.0, 30.0, 0.1))

        # ---- 累积彩色地图 (方案 B) ----
        self.declare_parameter('accumulate', True)             # 是否累积全局彩色地图
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')  # Point-LIO 里程计
        self.declare_parameter('global_frame', 'camera_init')  # 全局坐标系
        self.declare_parameter('colored_map_topic', '/colored_map')
        self.declare_parameter('voxel_size', 0.05)             # 体素降采样尺寸 (米), <=0 关闭
        self.declare_parameter('map_publish_period', 1.0)      # 地图发布周期 (秒)
        self.declare_parameter('max_map_points', 5000000)      # 地图点数上限, 防止内存爆炸

        lidar_frame = self.get_parameter('lidar_frame').value
        camera_frame = self.get_parameter('camera_frame').value
        cloud_topic = self.get_parameter('cloud_topic').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        colored_topic = self.get_parameter('colored_cloud_topic').value
        qsize = self.get_parameter('queue_size').value

        # ---- tf2 缓冲区 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- cv_bridge ----
        self.bridge = CvBridge()

        # ---- 缓存最新数据 ----
        self.latest_image = None      # (rgb_np, stamp)
        self.latest_cloud = None      # (ros_msg, stamp)
        self.camera_model = None      # image_geometry.PinholeCameraModel
        self.T_lidar_cam = None       # 4x4 外参矩阵

        # ---- 累积地图状态 ----
        self.accumulate = self.get_parameter('accumulate').value
        self.global_frame = self.get_parameter('global_frame').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_map_points = self.get_parameter('max_map_points').value
        self.latest_odom_T = None     # 4x4: camera_init <- unilidar_lidar (最新里程计位姿)
        # 用字典做体素哈希: key=(vx,vy,vz) -> (x,y,z,r,g,b), 天然去重降采样
        self.voxel_map = {}
        self.map_dirty = False        # 是否有新点待发布

        # ---- 订阅 ----
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, qsize)
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qsize)
        self.caminfo_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.caminfo_callback, qsize)

        # 累积模式需要里程计
        if self.accumulate:
            from nav_msgs.msg import Odometry
            odom_topic = self.get_parameter('odom_topic').value
            self.odom_sub = self.create_subscription(
                Odometry, odom_topic, self.odom_callback, qsize)

        # ---- 发布 ----
        self.colored_pub = self.create_publisher(
            PointCloud2, colored_topic, qsize)
        if self.accumulate:
            map_topic = self.get_parameter('colored_map_topic').value
            # 地图用 TRANSIENT_LOCAL, 新订阅者(如RViz)接入即可收到最新地图
            from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
            map_qos = QoSProfile(depth=1)
            map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            map_qos.history = HistoryPolicy.KEEP_LAST
            self.map_pub = self.create_publisher(
                PointCloud2, map_topic, map_qos)

        # ---- 定时处理 (10 Hz) ----
        self.timer = self.create_timer(0.1, self.process_timer)
        if self.accumulate:
            map_period = self.get_parameter('map_publish_period').value
            self.map_timer = self.create_timer(map_period, self.publish_map)

        # ---- 标定微调: 外参参数变化时, 清空已累积地图, 让它用新外参重建 ----
        # (否则调滑块时, 旧的错位颜色会一直残留在全局地图里, 看不清对齐效果)
        self._extrinsic_param_names = {
            'extrinsic_x', 'extrinsic_y', 'extrinsic_z',
            'extrinsic_roll', 'extrinsic_pitch', 'extrinsic_yaw',
        }
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info('PointCloudColorizer 已启动')
        self.get_logger().info(f'  监听 LiDAR: {cloud_topic}')
        self.get_logger().info(f'  监听 RGB:   {image_topic}')
        self.get_logger().info(f'  发布彩色点云: {colored_topic}')
        self.get_logger().info('  外参获取: tf2' if self.get_parameter('use_tf_extrinsic').value
                               else '  外参获取: 手动参数')
        if self.accumulate:
            self.get_logger().info(
                f'  ✓ 累积模式开启 → 发布 {self.get_parameter("colored_map_topic").value} '
                f'(全局系: {self.global_frame}, 体素: {self.voxel_size}m)')
        else:
            self.get_logger().info('  累积模式: 关闭 (仅逐帧)')

    def _on_param_change(self, params):
        """外参微调参数变化 → 清空累积地图, 用新外参重建"""
        from rcl_interfaces.msg import SetParametersResult
        changed_extrinsic = any(
            p.name in self._extrinsic_param_names for p in params)
        if changed_extrinsic and self.accumulate:
            self.voxel_map.clear()
            self.map_dirty = True
            self.get_logger().info('外参已调整 → 清空并重建彩色地图')
        return SetParametersResult(successful=True)

    def caminfo_callback(self, msg: CameraInfo):
        """接收相机内参"""
        from image_geometry import PinholeCameraModel
        model = PinholeCameraModel()
        model.fromCameraInfo(msg)
        self.camera_model = model
        # 只 log 一次
        if model is not None:
            self.get_logger().info(
                f'相机内参: {model.width}x{model.height} '
                f'fx={model.fx():.1f} fy={model.fy():.1f} '
                f'cx={model.cx():.1f} cy={model.cy():.1f}')
            # 销毁订阅 (只需要一次内参)
            self.destroy_subscription(self.caminfo_sub)

    def image_callback(self, msg: Image):
        """缓存最新 RGB 图像"""
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = (rgb, msg.header.stamp)
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')

    def cloud_callback(self, msg: PointCloud2):
        """缓存最新点云"""
        self.latest_cloud = (msg, msg.header.stamp)

    def odom_callback(self, msg):
        """缓存最新里程计位姿, 构造 camera_init <- unilidar_lidar 变换矩阵"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom_T = self._tf_to_matrix(
            p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def get_extrinsic_from_tf(self, lidar_frame: str, camera_frame: str):
        """从 tf2 获取 LiDAR → Camera 变换矩阵 (4x4)"""
        try:
            tfs: TransformStamped = self.tf_buffer.lookup_transform(
                camera_frame, lidar_frame, rclpy.time.Time())
            t = tfs.transform.translation
            q = tfs.transform.rotation
            T = self._tf_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            return T
        except Exception:
            return None

    @staticmethod
    def _base_lidar_to_optical():
        """
        LiDAR 系 (x=前,y=左,z=上) → 相机光学系 (x=右,y=下,z=前) 的基准旋转.
        这是两种坐标约定之间固定的 ~90° 变换, 是着色能大致对齐的前提.

        映射关系:
          相机_x(右)  = -LiDAR_y (左的反方向)
          相机_y(下)  = -LiDAR_z (上的反方向)
          相机_z(前)  =  LiDAR_x (前)
        """
        R = np.array([
            [0.0, -1.0,  0.0],   # cam_x =  -y_lidar
            [0.0,  0.0, -1.0],   # cam_y =  -z_lidar
            [1.0,  0.0,  0.0],   # cam_z =   x_lidar
        ])
        return R

    def get_extrinsic_from_params(self):
        """
        构造 4x4 外参矩阵 T_lidar_cam (把 LiDAR 点变换到相机光学系).

        T = 微调旋转 @ 基准旋转,  平移为杆臂微调.
        微调量 (roll/pitch/yaw/x/y/z) 全 0 时即为理想安装的基准对齐.
        """
        from scipy.spatial.transform import Rotation
        x = self.get_parameter('extrinsic_x').value
        y = self.get_parameter('extrinsic_y').value
        z = self.get_parameter('extrinsic_z').value
        roll = self.get_parameter('extrinsic_roll').value
        pitch = self.get_parameter('extrinsic_pitch').value
        yaw = self.get_parameter('extrinsic_yaw').value

        R_base = self._base_lidar_to_optical()
        # roll/pitch/yaw 参数单位为度, 更直观
        R_delta = Rotation.from_euler('xyz', [roll, pitch, yaw],
                                      degrees=True).as_matrix()
        R = R_delta @ R_base

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def _tf_to_matrix(x, y, z, qx, qy, qz, qw):
        """tf 平移 + 四元数 → 4x4 变换矩阵"""
        from scipy.spatial.transform import Rotation
        R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def project_and_color(self, cloud_msg: PointCloud2, rgb_img: np.ndarray,
                           T_lidar_cam: np.ndarray):
        """
        将点云投影到图像平面, 给每个点赋予 RGB 颜色.
        返回: (xyz_nx3, rgb_nx3) — 只包含投影成功且颜色有效的点
        """
        # ---- 解析点云为 numpy [N, 3] ----
        from sensor_msgs_py import point_cloud2 as pc2
        # Jazzy 的 read_points 返回结构化 ndarray
        pts = pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'),
                              skip_nans=True)
        if pts.shape[0] == 0:
            return None, None
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float32)  # [N, 3]

        # ---- 坐标变换: LiDAR → 相机 ----
        N = xyz.shape[0]
        xyz_homo = np.hstack([xyz, np.ones((N, 1), dtype=np.float32)])  # [N, 4]
        xyz_cam = (T_lidar_cam @ xyz_homo.T).T[:, :3]  # [N, 3] 在相机坐标系

        # ---- 滤除相机后方的点 ----
        valid = xyz_cam[:, 2] > self.get_parameter('min_depth').value
        xyz_cam = xyz_cam[valid]
        xyz = xyz[valid]
        if xyz_cam.shape[0] == 0:
            return None, None

        # ---- 投影到像素平面 ----
        if self.camera_model is None:
            self.get_logger().warn('尚未收到 camera_info, 无法投影', throttle_duration_sec=5.0)
            return None, None

        fx, fy = self.camera_model.fx(), self.camera_model.fy()
        cx, cy = self.camera_model.cx(), self.camera_model.cy()
        w_img, h_img = self.camera_model.width, self.camera_model.height

        Z = xyz_cam[:, 2]
        u = (fx * xyz_cam[:, 0] / Z + cx).astype(np.int32)
        v = (fy * xyz_cam[:, 1] / Z + cy).astype(np.int32)

        # ---- 滤除图像外的点 ----
        in_img = (u >= 0) & (u < w_img) & (v >= 0) & (v < h_img)
        u, v, Z = u[in_img], v[in_img], Z[in_img]
        xyz = xyz[in_img]
        if xyz.shape[0] == 0:
            return None, None

        # ---- 采样颜色 ----
        colors = rgb_img[v, u, :]  # [M, 3] BGR
        # 滤除无效颜色 (全0的点可能是投影到无纹理区域)
        valid_color = colors.sum(axis=1) > 0
        xyz = xyz[valid_color]
        colors = colors[valid_color]
        if xyz.shape[0] == 0:
            return None, None

        # BGR → RGB (PointCloud2 标准)
        rgb = colors[:, ::-1]
        return xyz, rgb

    @staticmethod
    def build_colored_cloud_msg(xyz: np.ndarray, rgb: np.ndarray,
                                 header, frame_id: str) -> PointCloud2:
        """构造带 RGB 字段的 PointCloud2 消息"""
        from sensor_msgs_py import point_cloud2 as pc2

        # 将 RGB [0,255] 打包成一个 float32 (标准 PCL rgb 字段格式)
        r = rgb[:, 0].astype(np.uint32)
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 2].astype(np.uint32)
        rgb_uint = (r << 16) | (g << 8) | b
        # PCL 约定 rgb 字段为 FLOAT32, 用位模式重解释 uint32
        rgb_float = rgb_uint.view(np.float32) if rgb_uint.dtype == np.uint32 \
            else rgb_uint.astype(np.uint32).view(np.float32)

        # 组装 [N, 4] float32 点数组: x, y, z, rgb
        cloud_data = np.zeros((xyz.shape[0], 4), dtype=np.float32)
        cloud_data[:, 0] = xyz[:, 0]
        cloud_data[:, 1] = xyz[:, 1]
        cloud_data[:, 2] = xyz[:, 2]
        cloud_data[:, 3] = rgb_float

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        out_header = header
        out_header.frame_id = frame_id
        msg = pc2.create_cloud(out_header, fields, cloud_data)
        return msg

    def process_timer(self):
        """定时处理: 同步最新点云和图像"""
        if self.latest_cloud is None or self.latest_image is None:
            return

        cloud_msg, cloud_stamp = self.latest_cloud
        image_rgb, img_stamp = self.latest_image

        # 时间差过大则跳过 (>0.5s)
        # header.stamp 是 builtin_interfaces/Time, 需手动换算成秒再相减
        cloud_t = cloud_stamp.sec + cloud_stamp.nanosec * 1e-9
        img_t = img_stamp.sec + img_stamp.nanosec * 1e-9
        time_diff = abs(cloud_t - img_t)
        if time_diff > 0.5:
            return

        # ---- 获取外参 ----
        if self.get_parameter('use_tf_extrinsic').value:
            lidar_f = self.get_parameter('lidar_frame').value
            cam_f = self.get_parameter('camera_frame').value
            self.T_lidar_cam = self.get_extrinsic_from_tf(lidar_f, cam_f)
            if self.T_lidar_cam is None:
                return  # tf 还没准备好, 静默等待
        else:
            # 每帧重新读取参数, 支持 rqt_reconfigure 实时标定
            self.T_lidar_cam = self.get_extrinsic_from_params()

        # ---- 投影和着色 ----
        xyz, rgb = self.project_and_color(
            cloud_msg, image_rgb, self.T_lidar_cam)

        if xyz is None:
            return

        # ---- 发布彩色点云 (逐帧, 雷达坐标系) ----
        colored_msg = self.build_colored_cloud_msg(
            xyz, rgb, cloud_msg.header, cloud_msg.header.frame_id)
        self.colored_pub.publish(colored_msg)

        # ---- 累积到全局彩色地图 (方案 B) ----
        if self.accumulate and self.latest_odom_T is not None:
            self.accumulate_points(xyz, rgb)

        # 统计信息 (低频)
        self.get_logger().info(
            f'着色点数: {xyz.shape[0]}'
            + (f' | 地图体素数: {len(self.voxel_map)}' if self.accumulate else ''),
            throttle_duration_sec=5.0)

    def accumulate_points(self, xyz: np.ndarray, rgb: np.ndarray):
        """将当前帧彩色点变换到全局系, 体素哈希去重后累加进地图"""
        # 变换到全局系: p_global = T_odom @ p_lidar
        N = xyz.shape[0]
        homo = np.hstack([xyz, np.ones((N, 1), dtype=np.float32)])
        xyz_g = (self.latest_odom_T @ homo.T).T[:, :3]

        vs = self.voxel_size
        if vs and vs > 0:
            # 体素哈希: 同一体素只保留一个点 (后来的覆盖)
            keys = np.floor(xyz_g / vs).astype(np.int64)
            for i in range(N):
                k = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
                self.voxel_map[k] = (xyz_g[i, 0], xyz_g[i, 1], xyz_g[i, 2],
                                     rgb[i, 0], rgb[i, 1], rgb[i, 2])
        else:
            # 不降采样: 用递增序号作 key
            base = len(self.voxel_map)
            for i in range(N):
                self.voxel_map[base + i] = (
                    xyz_g[i, 0], xyz_g[i, 1], xyz_g[i, 2],
                    rgb[i, 0], rgb[i, 1], rgb[i, 2])

        # 点数上限保护
        if len(self.voxel_map) > self.max_map_points:
            self.get_logger().warn(
                f'地图点数达上限 {self.max_map_points}, 停止累加新点',
                throttle_duration_sec=10.0)

        self.map_dirty = True

    def publish_map(self):
        """定时发布累积的全局彩色地图"""
        if not self.map_dirty or len(self.voxel_map) == 0:
            return
        if self.map_pub.get_subscription_count() == 0:
            return  # 没人订阅就不打包, 省 CPU

        # 从体素字典取出所有点
        vals = np.array(list(self.voxel_map.values()), dtype=np.float32)
        xyz = vals[:, 0:3]
        rgb = vals[:, 3:6]

        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        msg = self.build_colored_cloud_msg(xyz, rgb, header, self.global_frame)
        self.map_pub.publish(msg)
        self.map_dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudColorizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
