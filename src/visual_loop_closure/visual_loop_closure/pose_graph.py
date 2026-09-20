"""
SE(3) 位姿图优化器 (Pose Graph Optimizer)

在 Lie algebra se(3) 上用 Gauss-Newton 求解, 右乘扰动.
不依赖 g2o/GTSAM, 仅使用 scipy.sparse + numpy.

参考:
  - Grisetti et al., "A Tutorial on Graph-Based SLAM"
  - g2o 源码 (SE3 edge Jacobian)
"""

import numpy as np
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
from scipy.spatial.transform import Rotation


def skew_so3(v):
    """3-vector -> 3x3 skew-symmetric matrix"""
    return np.array([
        [0,      -v[2],   v[1]],
        [v[2],    0,     -v[0]],
        [-v[1],   v[0],   0   ]
    ])


def adjoint_se3(T):
    """
    SE(3) 伴随矩阵 (6x6).
    T: 4x4 变换矩阵.
    返回: 6x6 Ad_T 矩阵.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    Ad = np.eye(6)
    Ad[:3, :3] = R
    Ad[:3, 3:] = skew_so3(t) @ R
    Ad[3:, 3:] = R
    return Ad


def exp_se3(xi):
    """
    se(3) 指数映射.
    xi: 6-vector [v; ω]  (v=平移分量, ω=旋转分量).
    返回: 4x4 SE(3) 矩阵.
    """
    v = xi[:3]
    omega = xi[3:]
    theta = np.linalg.norm(omega)

    R = np.eye(3)
    V = np.eye(3)  # left Jacobian of SO(3)

    if theta > 1e-12:
        omega_hat = skew_so3(omega)
        omega_hat_sq = omega_hat @ omega_hat
        # Rodrigues
        R = (np.eye(3) +
             (np.sin(theta) / theta) * omega_hat +
             ((1 - np.cos(theta)) / (theta * theta)) * omega_hat_sq)
        # Left Jacobian V(ω)
        V = (np.eye(3) +
             ((1 - np.cos(theta)) / (theta * theta)) * omega_hat +
             ((theta - np.sin(theta)) / (theta * theta * theta)) * omega_hat_sq)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = V @ v
    return T


def log_se3(T):
    """
    SE(3) 对数映射.
    T: 4x4 SE(3) 矩阵.
    返回: 6-vector [v; ω] in se(3).
    """
    R = T[:3, :3]
    t = T[:3, 3]

    # 旋转 log
    trace = np.clip((np.trace(R) - 1) / 2, -1, 1)
    theta = np.arccos(trace)

    if theta < 1e-12:
        omega = np.zeros(3)
        V_inv = np.eye(3)
    else:
        omega_vec = np.array([R[2, 1] - R[1, 2],
                               R[0, 2] - R[2, 0],
                               R[1, 0] - R[0, 1]])
        omega = (theta / (2 * np.sin(theta))) * omega_vec
        omega_hat = skew_so3(omega)
        # Inverse of left Jacobian V(ω)
        V_inv = (np.eye(3) -
                 0.5 * omega_hat +
                 ((1.0 / (theta * theta)) -
                  (1 + np.cos(theta)) / (2 * theta * np.sin(theta))) *
                 omega_hat @ omega_hat)

    v = V_inv @ t
    return np.concatenate([v, omega])


# ---- Pose helpers ----

def pose_to_mat(pose):
    """pose [x,y,z,qx,qy,qz,qw] -> 4x4 matrix"""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    T[:3, 3] = pose[:3]
    return T


def mat_to_pose(T):
    """4x4 matrix -> [x,y,z,qx,qy,qz,qw]"""
    t = T[:3, 3]
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    return np.array([t[0], t[1], t[2], q[0], q[1], q[2], q[3]])


def relative_pose(pose_from, pose_to):
    """T_from_to = T_from^{-1} @ T_to"""
    T_f = pose_to_mat(pose_from)
    T_t = pose_to_mat(pose_to)
    return mat_to_pose(np.linalg.inv(T_f) @ T_t)


def pose_error_vec(pose_i, pose_j, meas_rel):
    """
    边误差: e = log(meas_rel^{-1} * pose_i^{-1} * pose_j)
    返回 6-vector in se(3).
    """
    T_i = pose_to_mat(pose_i)
    T_j = pose_to_mat(pose_j)
    T_meas = pose_to_mat(meas_rel)
    T_err = np.linalg.inv(T_meas) @ np.linalg.inv(T_i) @ T_j
    return log_se3(T_err)


# ---- Pose Graph ----

class PoseGraph:
    def __init__(self):
        self.nodes: list[np.ndarray] = []       # list of [x,y,z,qx,qy,qz,qw]
        self.odom_edges: list[tuple] = []       # (idx_i, idx_j, rel_pose_7)
        self.loop_edges: list[tuple] = []       # (idx_i, idx_j, rel_pose_7, weight)

    def add_node(self, node_id: int, pose: np.ndarray):
        self.nodes.append(pose.copy())

    def add_odom_edge(self, idx_i: int, idx_j: int, rel_pose: np.ndarray):
        self.odom_edges.append((idx_i, idx_j, rel_pose.copy()))

    def add_loop_edge(self, idx_i: int, idx_j: int, rel_pose: np.ndarray,
                       weight: float = 1.0):
        self.loop_edges.append((idx_i, idx_j, rel_pose.copy(), weight))

    def node_count(self) -> int:
        return len(self.nodes)

    def get_pose(self, idx: int) -> np.ndarray:
        return self.nodes[idx].copy()

    def optimize(self, max_iter: int = 10, verbose: bool = False,
                 fix_first: bool = True) -> np.ndarray:
        """
        Gauss-Newton on SE(3) manifold with right perturbation.
        返回优化后的 (n, 7) 位姿数组.
        """
        n = len(self.nodes)
        if n < 2:
            return np.array([])

        # 汇聚所有边 (odom + loop)
        all_edges = [(a, b, r, 1.0) for (a, b, r) in self.odom_edges]
        all_edges += [(a, b, r, w) for (a, b, r, w) in self.loop_edges]
        m = len(all_edges)

        if m == 0:
            return np.array([])

        poses = [p.copy() for p in self.nodes]

        # 添加阻尼 (Levenberg-Marquardt)
        lam = 1e-3

        for it in range(max_iter):
            H = lil_matrix((6 * n, 6 * n), dtype=np.float64)
            b = np.zeros(6 * n, dtype=np.float64)
            total_err = 0.0

            for idx_i, idx_j, meas_rel, weight in all_edges:
                # 当前误差
                err = pose_error_vec(poses[idx_i], poses[idx_j], meas_rel)
                e_sq = np.dot(err, err)
                total_err += weight * e_sq

                # 预测的相对变换 X = T_i^{-1} * T_j
                T_i = pose_to_mat(poses[idx_i])
                T_j = pose_to_mat(poses[idx_j])
                X = np.linalg.inv(T_i) @ T_j   # 4x4

                # 右 Jacobian 的逆 (近似为单位阵)
                # 对于小误差, Jr^{-1} ≈ I + 1/2 * ad_e
                theta = np.linalg.norm(err[3:])
                if theta > 1e-10:
                    omega_hat = skew_so3(err[3:])
                    v_hat = skew_so3(err[:3])
                    ad_e = np.zeros((6, 6))
                    ad_e[:3, :3] = omega_hat
                    ad_e[:3, 3:] = v_hat
                    ad_e[3:, 3:] = omega_hat
                    Jr_inv = np.eye(6) + 0.5 * ad_e
                else:
                    Jr_inv = np.eye(6)

                # Adjoint of X^{-1}
                X_inv = np.linalg.inv(X)
                Ad_X_inv = adjoint_se3(X_inv)  # 6x6

                # Jacobians (right perturbation convention)
                J_i = -Jr_inv @ Ad_X_inv   # 6x6
                J_j = Jr_inv               # 6x6

                # Weight = w * I
                W = weight * np.eye(6)

                H_ii = J_i.T @ W @ J_i
                H_ij = J_i.T @ W @ J_j
                H_ji = J_j.T @ W @ J_i
                H_jj = J_j.T @ W @ J_j

                b_i = -J_i.T @ W @ err
                b_j = -J_j.T @ W @ err

                # 累加到全局系统
                i0, j0 = 6 * idx_i, 6 * idx_j
                H[i0:i0+6, i0:i0+6] += H_ii
                H[i0:i0+6, j0:j0+6] += H_ij
                H[j0:j0+6, i0:i0+6] += H_ji
                H[j0:j0+6, j0:j0+6] += H_jj
                b[i0:i0+6] += b_i
                b[j0:j0+6] += b_j

            # 固定第一个节点
            if fix_first:
                H[0:6, 0:6] += 1e8 * np.eye(6)

            # 阻尼
            H_diag = H.diagonal()
            for k in range(6 * n):
                H[k, k] += lam * max(H_diag[k], 1.0)

            # 求解
            try:
                dx = spsolve(H.tocsr(), b)
            except Exception as e:
                if verbose:
                    print(f'  [PoseGraph] 求解失败: {e}')
                break

            # 更新位姿 (右乘扰动)
            max_dx = 0.0
            for i in range(n):
                delta = dx[6*i:6*i+6]
                T_delta = exp_se3(delta)
                T_i_new = pose_to_mat(poses[i]) @ T_delta
                poses[i] = mat_to_pose(T_i_new)
                max_dx = max(max_dx, np.max(np.abs(delta)))

            if verbose:
                print(f'  [PoseGraph] iter {it}: err={total_err:.4f} '
                      f'max_dx={max_dx:.6f} lam={lam:.1e}')

            if total_err < 1e-6:
                break

            # 自适应地调整阻尼
            if it > 0 and total_err >= prev_err:
                lam *= 2.0
            else:
                lam *= 0.5
            prev_err = total_err

            if max_dx < 1e-6:
                break

        self.nodes = poses
        return np.array(poses)
