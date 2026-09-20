#!/usr/bin/env python3
"""
方向1: 浮标语义识别 + 3D 定位 (buoy semantic detection with LiDAR fusion)

流程:
  1. RGB 图像 HSV 颜色分割 -> 候选颜色块 (红/绿/黑/白/橙 + 2D 框)
  2. 把 LiDAR 点云投影到图像平面 (复用相机内参 K + tf2 外参)
  3. LiDAR 融合门控: 只保留"框内有雷达点支撑、距离在量程内、高度在水面带内"
     的颜色块 -> 滤掉远岸树线/天空等纯 2D 颜色误检
  4. 用框内雷达点算出浮标的 3D 位置 (LiDAR 系)

输出:
  - /usv/buoy/image        标注调试图 (框 + 类别 + 距离)
  - /usv/buoy/detections   JSON (类别 + 像素框 + 3D 位置 + 距离)
  - /usv/buoy/markers      visualization_msgs/MarkerArray (RViz 3D 球 + 文字)

纯 OpenCV + numpy + tf2, 无深度学习, 完全离线.
海事规则中红绿浮标标示航道边界, 是后续语义导航 (COLREGs) 的输入.
"""
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from tf2_ros import Buffer, TransformListener
from sensor_msgs_py import point_cloud2 as pc2


# HSV 颜色区间 (OpenCV: H 0-179, S 0-255, V 0-255)
COLOR_RANGES = {
    'red':   [((0, 120, 70),   (10, 255, 255)),
              ((170, 120, 70), (179, 255, 255))],
    'green': [((40, 80, 40),   (85, 255, 255))],
    'orange':[((11, 120, 90),  (25, 255, 255))],
    'white': [((0, 0, 200),    (179, 40, 255))],
    'black': [((0, 0, 0),      (179, 255, 55))],
}

DRAW_BGR = {
    'red':   (0, 0, 255),
    'green': (0, 255, 0),
    'orange':(0, 165, 255),
    'white': (255, 255, 255),
    'black': (60, 60, 60),
}

# RViz marker 颜色 (RGBA 0-1)
MARK_RGBA = {
    'red':   (1.0, 0.1, 0.1, 1.0),
    'green': (0.1, 1.0, 0.1, 1.0),
    'orange':(1.0, 0.6, 0.0, 1.0),
    'white': (1.0, 1.0, 1.0, 1.0),
    'black': (0.2, 0.2, 0.2, 1.0),
}


class BuoyDetector(Node):
    def __init__(self):
        super().__init__('buoy_detector')

        self.declare_parameter('image_topic',
                               '/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/wamv/sensors/cameras/front_left_camera_sensor/optical/camera_info')
        self.declare_parameter('cloud_topic',
                               '/wamv/sensors/lidars/lidar_wamv_sensor/points')
        self.declare_parameter('min_area', 60)          # 最小连通域面积 (像素)
        self.declare_parameter('max_area', 200000)      # 最大面积
        self.declare_parameter('min_fill_ratio', 0.30)  # 轮廓填充率
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('classes', ['red', 'green', 'white', 'black', 'orange'])

        # --- LiDAR 融合门控参数 ---
        self.declare_parameter('fusion_enable', True)   # 关闭则退回纯 2D (会误检岸线)
        self.declare_parameter('max_range', 80.0)       # 浮标最大距离 (m); 远岸树线超此被滤
        self.declare_parameter('min_range', 1.0)        # 最小距离, 滤船体自反射
        self.declare_parameter('z_min', -2.5)           # 高度带下限 (LiDAR 系, m): 水面以上
        self.declare_parameter('z_max', 2.0)            # 高度带上限: 滤高处树冠
        self.declare_parameter('min_support', 2)        # 框内所需最少雷达点数
        self.declare_parameter('bbox_dilate', 4)        # 框膨胀像素 (投影/标定容差)

        self.image_topic = self.get_parameter('image_topic').value
        caminfo_topic = self.get_parameter('camera_info_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.min_fill = self.get_parameter('min_fill_ratio').value
        self.pub_debug = self.get_parameter('publish_debug_image').value
        self.classes = list(self.get_parameter('classes').value)
        self.fusion = self.get_parameter('fusion_enable').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.min_support = self.get_parameter('min_support').value
        self.bbox_dilate = self.get_parameter('bbox_dilate').value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.K = None            # 3x3 相机内参
        self.cam_frame = None    # 相机光学系 frame_id
        self.img_w = self.img_h = None
        self.latest_cloud = None # (xyz Nx3 in lidar frame, lidar_frame_id)

        self.sub_img = self.create_subscription(
            Image, self.image_topic, self.image_cb, 5)
        self.sub_info = self.create_subscription(
            CameraInfo, caminfo_topic, self.caminfo_cb, 5)
        self.sub_cloud = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_cb, 5)

        self.det_pub = self.create_publisher(String, '/usv/buoy/detections', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/usv/buoy/markers', 5)
        if self.pub_debug:
            self.img_pub = self.create_publisher(Image, '/usv/buoy/image', 5)

        self.get_logger().info('BuoyDetector 已启动 (LiDAR 融合门控)')
        self.get_logger().info(f'  图像: {self.image_topic}')
        self.get_logger().info(f'  点云: {cloud_topic}')
        self.get_logger().info(f'  类别: {self.classes}')
        self.get_logger().info(
            f'  融合门控: {"开" if self.fusion else "关(纯2D)"} '
            f'| 量程[{self.min_range},{self.max_range}]m '
            f'| 高度带[{self.z_min},{self.z_max}]m | 最少支撑点{self.min_support}')

    def caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.cam_frame = msg.header.frame_id
        self.img_w, self.img_h = msg.width, msg.height
        self.get_logger().info(
            f'相机内参: {msg.width}x{msg.height} fx={self.K[0,0]:.1f} '
            f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f} frame={self.cam_frame}')
        self.destroy_subscription(self.sub_info)

    def cloud_cb(self, msg: PointCloud2):
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)
        self.latest_cloud = (xyz, msg.header.frame_id)

    def _lookup_T(self, target_frame, source_frame):
        """target <- source 的 4x4 变换 (把 source 系点转到 target 系)"""
        from scipy.spatial.transform import Rotation
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def project_cloud(self):
        """
        把 LiDAR 点投影到图像. 返回:
          uv   : Nx2 int 像素坐标
          depth: N   相机系深度 (m)
          xyz_l: Nx3 对应的 LiDAR 系原坐标 (用于算 3D 位置)
        仅含投影到相机前方且落在图像内的点.
        """
        if self.latest_cloud is None or self.K is None or self.cam_frame is None:
            return None
        xyz_l, lidar_frame = self.latest_cloud
        T = self._lookup_T(self.cam_frame, lidar_frame)  # cam <- lidar
        if T is None:
            return None

        # 高度带 + 量程门控 (在 LiDAR 系, z 上, 水平距离)
        rng = np.linalg.norm(xyz_l[:, :2], axis=1)
        band = ((xyz_l[:, 2] >= self.z_min) & (xyz_l[:, 2] <= self.z_max)
                & (rng >= self.min_range) & (rng <= self.max_range))
        xyz_l = xyz_l[band]
        if xyz_l.shape[0] == 0:
            return None

        N = xyz_l.shape[0]
        homo = np.hstack([xyz_l, np.ones((N, 1))])
        xyz_c = (T @ homo.T).T[:, :3]            # 相机光学系
        front = xyz_c[:, 2] > 0.1                # 相机前方
        xyz_c, xyz_l = xyz_c[front], xyz_l[front]
        if xyz_c.shape[0] == 0:
            return None

        uvw = (self.K @ xyz_c.T).T               # 投影
        uv = uvw[:, :2] / uvw[:, 2:3]
        depth = xyz_c[:, 2]
        u = uv[:, 0].astype(np.int32)
        v = uv[:, 1].astype(np.int32)
        inside = (u >= 0) & (u < self.img_w) & (v >= 0) & (v < self.img_h)
        if not np.any(inside):
            return None
        return (np.stack([u[inside], v[inside]], axis=-1),
                depth[inside], xyz_l[inside])

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}', throttle_duration_sec=5.0)
            return

        proj = self.project_cloud() if self.fusion else None
        if self.fusion and proj is None:
            # 融合开启但点云/外参未就绪: 本帧不出检测, 避免纯 2D 误检
            self.get_logger().warn('等待点云/TF/内参 就绪...', throttle_duration_sec=5.0)
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        annotated = bgr.copy() if self.pub_debug else None
        detections = []

        for cls in self.classes:
            ranges = COLOR_RANGES.get(cls)
            if ranges is None:
                continue
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_area or area > self.max_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                fill = area / float(w * h) if w * h > 0 else 0.0
                if fill < self.min_fill:
                    continue

                pos3d = None
                dist = None
                if self.fusion:
                    # LiDAR 融合门控: 框内(膨胀后)是否有雷达点支撑
                    uv, depth, xyz_l = proj
                    d = self.bbox_dilate
                    inbox = ((uv[:, 0] >= x - d) & (uv[:, 0] < x + w + d)
                             & (uv[:, 1] >= y - d) & (uv[:, 1] < y + h + d))
                    n_sup = int(np.count_nonzero(inbox))
                    if n_sup < self.min_support:
                        continue  # 无雷达支撑 -> 判为背景(树线/天空), 丢弃
                    pts_l = xyz_l[inbox]
                    pos3d = np.median(pts_l, axis=0)  # LiDAR 系 3D 位置
                    dist = float(np.linalg.norm(pos3d[:2]))

                cx, cy = x + w // 2, y + h // 2
                det = {
                    'class': cls,
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'center': [int(cx), int(cy)],
                    'area': int(area),
                }
                if pos3d is not None:
                    det['pos_lidar'] = [float(pos3d[0]), float(pos3d[1]), float(pos3d[2])]
                    det['range'] = round(dist, 2)
                detections.append(det)

                if annotated is not None:
                    color = DRAW_BGR[cls]
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
                    label = cls if dist is None else f'{cls} {dist:.1f}m'
                    cv2.putText(annotated, label, (x, max(y - 5, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        out = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'buoys': detections,
        }
        self.det_pub.publish(String(data=json.dumps(out)))
        self._publish_markers(detections)

        if annotated is not None:
            dbg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            dbg.header = msg.header
            self.img_pub.publish(dbg)

        if detections:
            summary = ', '.join(
                f"{d['class']}"
                + (f"@{d['range']}m" if 'range' in d else f"@{tuple(d['center'])}")
                for d in detections[:8])
            self.get_logger().info(f'检测到 {len(detections)} 个浮标: {summary}',
                                   throttle_duration_sec=2.0)

    def _publish_markers(self, detections):
        """3D 浮标位置发布为 RViz MarkerArray (LiDAR 系)"""
        if self.latest_cloud is None:
            return
        lidar_frame = self.latest_cloud[1]
        arr = MarkerArray()
        # 先发一个 DELETEALL 清旧
        clr = Marker()
        clr.header.frame_id = lidar_frame
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)
        mid = 0
        for d in detections:
            if 'pos_lidar' not in d:
                continue
            r, g, b, a = MARK_RGBA[d['class']]
            m = Marker()
            m.header.frame_id = lidar_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'buoy'
            m.id = mid; mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = d['pos_lidar']
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.5
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
            arr.markers.append(m)
        if mid > 0:
            self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
