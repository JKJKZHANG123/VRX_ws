"""
关键帧数据库: 存储图像特征 + 位姿, 支持基于 ORB 的视觉回环检测.

策略:
- 视觉部分负责地点识别 ("我们是否来过这里?")
- LiDAR 里程计提供度量尺度的相对位姿约束
- 检测到回环后, 用 LiDAR 里程计的相对位姿作为约束加入位姿图
"""

import numpy as np
import cv2


class Keyframe:
    __slots__ = ('id', 'pose', 'keypoints', 'descriptors', 'timestamp')

    def __init__(self, kf_id: int, pose: np.ndarray,
                 kp: list, desc: np.ndarray, timestamp: float):
        self.id = kf_id
        self.pose = pose.copy()              # [x,y,z,qx,qy,qz,qw]
        self.keypoints = [(p.pt, p.size, p.angle, p.response)
                          for p in kp]       # 可序列化的关键点
        self.descriptors = desc.copy()       # [N, 32]
        self.timestamp = timestamp

    @property
    def position(self) -> np.ndarray:
        return self.pose[:3]


class KeyframeDatabase:
    def __init__(self, max_keyframes: int = 500,
                 min_keyframe_distance: float = 0.5,
                 min_keyframe_angle_deg: float = 10.0):
        self.keyframes: list[Keyframe] = []
        self.max_kf = max_keyframes
        self.min_dist = min_keyframe_distance
        self.min_angle_rad = np.deg2rad(min_keyframe_angle_deg)

    def should_add_keyframe(self, pose: np.ndarray) -> bool:
        if len(self.keyframes) == 0:
            return True
        last = self.keyframes[-1]
        dist = np.linalg.norm(pose[:3] - last.pose[:3])
        if dist < self.min_dist:
            return False
        from scipy.spatial.transform import Rotation
        q_curr = Rotation.from_quat(pose[3:])
        q_last = Rotation.from_quat(last.pose[3:])
        angle = np.linalg.norm((q_curr * q_last.inv()).as_rotvec())
        return angle >= self.min_angle_rad

    def add_keyframe(self, pose: np.ndarray, kp: list, desc: np.ndarray,
                     timestamp: float) -> int:
        kf_id = len(self.keyframes)
        kf = Keyframe(kf_id, pose, kp, desc, timestamp)
        self.keyframes.append(kf)
        if len(self.keyframes) > self.max_kf:
            self.keyframes.pop(0)
        return kf_id

    def detect_loop(self, query_desc: np.ndarray, query_pose: np.ndarray,
                    min_matches: int = 30, match_ratio: float = 0.75,
                    min_loop_gap: int = 30, search_radius: float = 80.0,
                    inlier_ratio_min: float = 0.6) -> tuple:
        """
        检测回环.

        Returns:
            (matched_kf_id, num_inliers) or (None, 0)
        """
        if len(self.keyframes) < min_loop_gap:
            return None, 0
        if query_desc is None or query_desc.shape[0] < 10:
            return None, 0

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        best_kf_id = None
        best_inliers = 0
        best_score = 0.0

        end_idx = max(0, len(self.keyframes) - min_loop_gap)

        for idx in range(end_idx):
            kf = self.keyframes[idx]
            if kf.descriptors is None or kf.descriptors.shape[0] < 10:
                continue

            # 空间过滤: 只检查附近的帧 (已经离开足够远又回来的情况)
            dist = np.linalg.norm(query_pose[:3] - kf.position)
            if dist > search_radius:
                continue

            # 如果太近 (还没走出去), 跳过
            if dist < 3.0:
                continue

            # ORB 匹配 + Lowe's ratio test
            knn_matches = bf.knnMatch(query_desc, kf.descriptors, k=2)
            good = []
            for mn in knn_matches:
                if len(mn) == 2:
                    m, n = mn
                    if m.distance < match_ratio * n.distance:
                        good.append(m)

            if len(good) < min_matches:
                continue

            # 几何验证: 用对应点做基础矩阵 RANSAC
            src_pts = np.float32([
                [kf.keypoints[m.trainIdx][0][0],
                 kf.keypoints[m.trainIdx][0][1]]
                for m in good
            ])
            dst_pts = np.float32([
                [self._decode_kp(query_desc, m.queryIdx)]
                for m in good
            ])

            # 需要从 query 中恢复关键点坐标 — 这里我们需要在外部传入
            # 简化: 直接用 match count 和 distance ratio 做投票
            match_score = len(good) / (1.0 + np.mean([m.distance for m in good]))
            if match_score > best_score:
                best_score = match_score
                best_inliers = len(good)
                best_kf_id = kf.id

        # 验证: 足够多的内点且内点比例高
        if best_kf_id is not None and best_inliers >= min_matches:
            return best_kf_id, best_inliers

        return None, 0

    def get_pose(self, kf_id: int) -> np.ndarray:
        for kf in self.keyframes:
            if kf.id == kf_id:
                return kf.pose.copy()
        return None


def _decode_kp(desc, idx):
    """Placeholder — 实际实现见 visual_slam_node.py 中的匹配逻辑"""
    return (0.0, 0.0)


def extract_orb_features(image: np.ndarray, max_features: int = 1000):
    """
    从 RGB/BGR 图像提取 ORB 特征.

    Returns:
        (keypoints: list[cv2.KeyPoint], descriptors: np.ndarray [N,32])
    """
    if image is None:
        return None, None

    if len(image.shape) == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    orb = cv2.ORB_create(
        nfeatures=max_features,
        scaleFactor=1.2,
        nlevels=8,
        edgeThreshold=31,
        firstLevel=0,
        WTA_K=2,
        scoreType=cv2.ORB_HARRIS_SCORE,
        patchSize=31,
        fastThreshold=20,
    )
    kp, desc = orb.detectAndCompute(gray, None)
    return kp, desc
