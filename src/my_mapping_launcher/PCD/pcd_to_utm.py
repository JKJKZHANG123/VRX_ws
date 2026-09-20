import os
import numpy as np
import open3d as o3d
import laspy
from scipy.spatial.transform import Rotation as R

def generate_utm_las():
    input_pcd_path = "input/scans.pcd"
    output_dir = "output"
    output_las_path = os.path.join(output_dir, "utm_map.las")

    # 你的绝对地理密码
    trans = np.array([394337.671, 3417006.334, 0.000], dtype=np.float64)
    quat = [0.000, 0.000, -0.005, 1.000]  

    print(f"--> [1/4] 正在加载原始 PCD 点云: {input_pcd_path}")
    if not os.path.exists(input_pcd_path):
        print("错误：找不到输入文件！")
        return

    pcd = o3d.io.read_point_cloud(input_pcd_path)
    points = np.asarray(pcd.points, dtype=np.float64)
    print(f"    成功读取 {len(points)} 个点。")

    print("--> [2/4] 执行高精度 UTM 矩阵变换...")
    r_matrix = R.from_quat(quat).as_matrix()
    points_utm = np.dot(points, r_matrix.T) + trans

    print("--> [3/4] 正在配置 LAS 工业级文件头 (Header)...")
    # 创建 LAS 文件头，指定格式版本
    header = laspy.LasHeader(point_format=3, version="1.2")
    
    # 核心黑科技 1：设置缩放因子 (精度保留到毫米级 0.001米)
    header.scales = np.array([0.001, 0.001, 0.001])
    
    # 核心黑科技 2：自动计算并设置全局偏移量 (Offset)
    # 这样底层只会存储非常小的数据，彻底释放内存！
    header.offsets = np.floor(np.min(points_utm, axis=0))

    # 创建 LAS 数据对象
    las = laspy.LasData(header)
    
    # 写入坐标数据
    las.x = points_utm[:, 0]
    las.y = points_utm[:, 1]
    las.z = points_utm[:, 2]

    print(f"--> [4/4] 正在极速写入硬盘...")
    os.makedirs(output_dir, exist_ok=True)
    las.write(output_las_path)
    
    print(f"--> 🎉 测绘级 LAS 文件生成完毕！请查看: {output_las_path}")

if __name__ == "__main__":
    generate_utm_las()