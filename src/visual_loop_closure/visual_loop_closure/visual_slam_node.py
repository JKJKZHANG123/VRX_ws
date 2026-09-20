#!/usr/bin/env python3
"""
Visual SLAM Node — 视觉回环检测 + 位姿图优化

订阅:
  /camera/color/image_raw  — RGB 图像
  /Odometry                — Point-LIO 里程计 (或 /aft_mapped_to_init)

发布:
  /optimized_path          — 优化后的轨迹 (nav_msgs/Path)
  /loop_markers            — 回环约束可视化
  /loop_status             — 回环检测状态
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

# 项目内模块
from visual_loop_closure.keyframe_database import (
    KeyframeDatabase, extract_orb_features)
from visual_loop_closure.pose_graph import PoseGraph


class VisualSLAMNode(Node):
    def __init__(self):
        super().__init__('visual_slam_node')

        # ---- 参数 ----
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('max_keyframes', 500)
        self.declare_parameter('min_keyframe_dist', 0.3)          # 米
        self.declare_parameter('min_keyframe_angle_deg', 15.0)    # 度
        self.declare_parameter('loop_min_matches', 30)
        self.declare_parameter('loop_match_ratio', 0.75)
        self.declare_parameter('loop_min_gap', 20)                # 最少间隔帧数
        self.declare_parameter('loop_search_radius', 80.0)        # 米
        self.declare_parameter('pose_graph_fixed_first', True)
        self.declare_parameter('optimize_period', 5.0)            # 秒
        self.declare_parameter('orb_max_features', 1000)
        self.declare_parameter('enable_rgb_coloring', False)      # 是否同时处理着色的云

        # ---- 状态 ----
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_stamp = None
        self.latest_odom_pose = None      # [x,y,z,qx,qy,qz,qw] in camera_init
        self.latest_odom_msg = None       # 原始 Odometry 消息
        self.current_kf_id = 0

        # 里程计路径 (用于位姿图)
        self.odom_trajectory: list[tuple] = []   # (pose_7, timestamp)

        # 关键帧数据库
        self.database = KeyframeDatabase(
            max_keyframes=self.get_parameter('max_keyframes').value,
            min_keyframe_distance=self.get_parameter('min_keyframe_dist').value,
            min_keyframe_angle_deg=self.get_parameter('min_keyframe_angle_deg').value,
        )

        # 位姿图
        self.pose_graph = PoseGraph()
        self.pose_graph.fixed_first = self.get_parameter('pose_graph_fixed_first').value

        # 回环检测结果
        self.loop_events: list[dict] = []

        # ---- 订阅 ----
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )

        # ---- 发布 ----
        self.path_pub = self.create_publisher(Path, '/optimized_path', 10)
        self.raw_path_pub = self.create_publisher(Path, '/raw_odom_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/loop_markers', 10)
        self.status_pub = self.create_publisher(String, '/loop_status', 10)

        # ---- 定时器 ----
        self.process_timer = self.create_timer(0.2, self.process_image)
        self.optimize_timer = self.create_timer(
            self.get_parameter('optimize_period').value, self.optimize_pose_graph)
        self.publish_timer = self.create_timer(1.0, self.publish_paths)
        self.diag_timer = self.create_timer(10.0, self.print_diagnostics)

        # 上次优化的节点数
        self.last_optimized_nodes = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('Visual SLAM Node 已启动')
        self.get_logger().info(f'  图像话题: {self.get_parameter("image_topic").value}')
        self.get_logger().info(f'  里程计话题: {self.get_parameter("odom_topic").value}')
        self.get_logger().info(f'  最大关键帧数: {self.get_parameter("max_keyframes").value}')
        self.get_logger().info(f'  回环最少匹配数: {self.get_parameter("loop_min_matches").value}')
        self.get_logger().info(f'  优化周期: {self.get_parameter("optimize_period").value}s')
        self.get_logger().info('=' * 60)

    # ===================== 回调 =====================

    def image_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = img
            self.latest_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'图像解码失败: {e}', throttle_duration_sec=5.0)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom_pose = np.array([
            p.x, p.y, p.z, q.x, q.y, q.z, q.w
        ])
        self.latest_odom_msg = msg

        # 存储到轨迹
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_trajectory.append((self.latest_odom_pose.copy(), t_sec))
        # 限制轨迹长度
        if len(self.odom_trajectory) > 10000:
            self.odom_trajectory = self.odom_trajectory[-10000:]

    # ===================== 核心逻辑 =====================

    def process_image(self):
        """提取特征 + 关键帧管理 + 回环检测"""
        if self.latest_image is None or self.latest_odom_pose is None:
            return

        # 提取 ORB 特征
        kp, desc = extract_orb_features(
            self.latest_image,
            max_features=self.get_parameter('orb_max_features').value,
        )
        if desc is None or desc.shape[0] < 10:
            return

        # 判断是否添加关键帧
        if not self.database.should_add_keyframe(self.latest_odom_pose):
            return

        t_sec = (self.latest_image_stamp.sec +
                 self.latest_image_stamp.nanosec * 1e-9)

        # 添加关键帧
        kf_id = self.database.add_keyframe(
            self.latest_odom_pose, kp, desc, t_sec)
        self.current_kf_id = kf_id

        # 添加到 PoseGraph
        self.pose_graph.add_node(kf_id, self.latest_odom_pose)

        # 添加里程计边 (与前一个关键帧)
        if kf_id > 0:
            prev_pose = self.database.get_pose(kf_id - 1)
            if prev_pose is not None:
                rel_odom = relative_pose(prev_pose, self.latest_odom_pose)
                self.pose_graph.add_odom_edge(kf_id - 1, kf_id, rel_odom)

        # 回环检测 (每 5 个关键帧检查一次, 节省计算)
        if kf_id % 5 == 0 and kf_id > self.get_parameter('loop_min_gap').value:
            self.detect_loop(desc)

    def detect_loop(self, query_desc: np.ndarray):
        """执行回环检测"""
        matched_kf, num_inliers = self.database.detect_loop(
            query_desc,
            self.latest_odom_pose,
            min_matches=self.get_parameter('loop_min_matches').value,
            match_ratio=self.get_parameter('loop_match_ratio').value,
            min_loop_gap=self.get_parameter('loop_min_gap').value,
            search_radius=self.get_parameter('loop_search_radius').value,
        )

        if matched_kf is not None:
            matched_pose = self.database.get_pose(matched_kf)
            if matched_pose is None:
                return

            # 视觉回环约束: "当前帧与历史关键帧处于同一位置"
            # 因此期望的相对位姿为单位阵 (identity)
            # 权重由匹配质量决定
            rel_identity = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            weight = min(num_inliers / 30.0, 5.0)
            self.pose_graph.add_loop_edge(
                matched_kf, self.current_kf_id, rel_identity, weight)

            # 记录回环事件
            event = {
                'from_kf': matched_kf,
                'to_kf': self.current_kf_id,
                'position': self.latest_odom_pose[:3].tolist(),
                'inliers': num_inliers,
                'weight': weight,
            }
            self.loop_events.append(event)

            self.get_logger().info(
                f'🔁 检测到回环! KF {matched_kf} ↔ KF {self.current_kf_id} '
                f'(内点数: {num_inliers}, 权重: {weight:.1f})')

            # 立即通知
            msg = String()
            msg.data = (f'loop_detected: {matched_kf} -> {self.current_kf_id} '
                        f'inliers={num_inliers}')
            self.status_pub.publish(msg)

    def optimize_pose_graph(self):
        """运行位姿图优化"""
        if self.pose_graph.node_count() < 3:
            return
        if len(self.pose_graph.loop_edges) == 0:
            return

        node_count = self.pose_graph.node_count()
        self.get_logger().info(
            f'🔧 开始位姿图优化... 节点数: {node_count}, '
            f'里程计边: {len(self.pose_graph.odom_edges)}, '
            f'回环边: {len(self.pose_graph.loop_edges)}')

        t0 = time.time()
        self.pose_graph.optimize(max_iter=10, verbose=True)
        elapsed = time.time() - t0

        self.last_optimized_nodes = node_count
        self.get_logger().info(
            f'✅ 优化完成 (耗时 {elapsed:.3f}s, {node_count} 个节点)')

    # ===================== 发布 =====================

    def publish_paths(self):
        """发布优化后的轨迹和原始里程计轨迹"""
        # 原始里程计轨迹
        if len(self.odom_trajectory) > 1:
            raw_path = Path()
            raw_path.header.frame_id = 'camera_init'
            raw_path.header.stamp = self.get_clock().now().to_msg()
            for pose, _ in self.odom_trajectory[::5]:  # 降采样
                ps = PoseStamped()
                ps.header = raw_path.header
                ps.pose.position.x = pose[0]
                ps.pose.position.y = pose[1]
                ps.pose.position.z = pose[2]
                ps.pose.orientation.x = pose[3]
                ps.pose.orientation.y = pose[4]
                ps.pose.orientation.z = pose[5]
                ps.pose.orientation.w = pose[6]
                raw_path.poses.append(ps)
            self.raw_path_pub.publish(raw_path)

        # 优化后轨迹
        if self.pose_graph.node_count() > 1:
            opt_path = Path()
            opt_path.header.frame_id = 'camera_init'
            opt_path.header.stamp = self.get_clock().now().to_msg()
            for i in range(self.pose_graph.node_count()):
                pose = self.pose_graph.get_pose(i)
                ps = PoseStamped()
                ps.header = opt_path.header
                ps.pose.position.x = pose[0]
                ps.pose.position.y = pose[1]
                ps.pose.position.z = pose[2]
                ps.pose.orientation.x = pose[3]
                ps.pose.orientation.y = pose[4]
                ps.pose.orientation.z = pose[5]
                ps.pose.orientation.w = pose[6]
                opt_path.poses.append(ps)
            self.path_pub.publish(opt_path)

    def print_diagnostics(self):
        """定期打印诊断信息"""
        n_nodes = self.pose_graph.node_count()
        n_loops = len(self.pose_graph.loop_edges)
        n_kfs = len(self.database.keyframes)
        self.get_logger().info(
            f'📊 状态: 位姿图节点={n_nodes}, 回环约束={n_loops}, '
            f'关键帧={n_kfs}, 检测到回环事件={len(self.loop_events)}')


def main(args=None):
    rclpy.init(args=args)
    node = VisualSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
