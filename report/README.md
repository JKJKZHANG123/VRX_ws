# VRX 无人船 Point-LIO 与障碍物过滤改进报告

## 1. 报告目的

本目录保存原始基线、改进版本以及 MATLAB 对比脚本。主对比对象是静止船测试：

- `data/raw_lio_stationary_before_drift_fix_20260827.csv`：项目已经记录的“改动前/漂移修复前”基线。它不是 Git 初始提交的原始数据，而是当前数据目录中最接近原始未改进状态的可复现实验记录。
- `data/improved_lio_stationary_final_optimized_20260827.csv`：当前报告采用的最终参考版本。
- `data/improved_lio_stationary_after_zupt_candidate_20260827.csv`：已有记录中较新的 ZUPT 候选版本，单项指标优于参考版本，但尚未用完整启动链路复测确认。
- `data/improved_lio_stationary_after_drift_fix_20260827.csv`：中间版本，仅用于查看调参过程。

这些文件来自先后进行的 A/B 测试，不是同一时刻同时采集；因此应比较统计指标和相对轨迹，不应直接比较绝对世界坐标。

## 2. 数据含义

静止测试 CSV 的主要字段如下：

- `time_s`：记录时间；`truth_*`：Gazebo 真值位置。
- `lio_*`：Point-LIO 输出位置。
- `lio_xy_drift_m`：相对测试起点的 LIO 平面漂移。
- `truth_xy_motion_m`：仿真中船体真值的实际平面运动。
- `raw_points`、`filtered_points`、`registered_points`：各点云阶段点数。

`data/lio_metrics_summary.csv` 是由上述 CSV 计算出的统计汇总；其中 `rmse_relative_xy_error_m` 是分别将每次测试的起点平移到零点后，LIO 相对轨迹与 Gazebo 真值相对轨迹的 RMSE。

`data/pipeline_metrics_summary.csv` 同时保存了旧的水面过滤快照和改进链路的运行统计。两部分测试条件不同，不能把它们解释为严格的水面残留率。

## 3. MATLAB 使用方法

在 MATLAB 当前目录切换到 `report/matlab`，运行：

```matlab
plot_lio_before_after
```

脚本会生成：

1. 归一化 XY 航迹：真值与 LIO 轨迹对比；
2. LIO 平面漂移曲线；
3. 输入点数与配准点数曲线；
4. 关键统计量柱状图；
5. 改进链路点数、频率和代价地图栅格统计；
6. SafetyCloud 点数与 Nav2/TF 启动状态统计。

## 4. 当前结论

`lio_metrics_summary.csv` 中的 `final_optimized` 是当前报告采用的参考版本：相较基线，最大平面漂移约从 0.382 m 降到 0.296 m，末端漂移从 0.131 m 降到 0.032 m。

但它不是所有单项指标的绝对最优。新增的 `lio_candidate_comparison_20260827.csv` 显示，已有 `zupt_candidate` 记录的末端漂移 0.022 m、最大漂移 0.283 m、相对轨迹 RMSE 0.159 m，均优于 `final_optimized` 的 0.032 m、0.296 m、0.165 m；`final_optimized` 仅在 P95 漂移上略优（0.229 m 对 0.238 m）。由于这些测试的启动状态、随机种子和采样时段不完全一致，ZUPT 候选目前只能称为“数据上更优的候选”，还不能直接替换当前参考版本，必须在同一 VRX 启动链路下复测。
## 5. 本次 SafetyCloud/Nav2 核验数据

`data/safetycloud_nav2_runtime_validation_20260827.csv` 是从本次运行的 `.usv_logs/nav2.log` 与 `.usv_logs/cloud_filter.log` 统计得到的启动/运行验证数据，不是导航性能数据。建议在 MATLAB 中展示：

- Transform 等待次数、TF skip 次数：柱状图；
- SafetyCloud 每帧点数范围和中位数：箱线图或误差条；
- `nav2_managed_nodes_active` 与 `cloud_error_count`：状态表或通过/失败标记。

注意：日志中的 5 次 Transform 等待发生在启动约前 2 秒，之后 Nav2 已进入 `Managed nodes are active`；不要把这几条启动信息统计成持续故障。当前 `./usv.sh status` 已确认本轮记录的进程均已停止，报告数据仍可复现查看。

## 6. EKF 静止/运行 A-B 实验（2026-08-28）

本轮只做两类实验，每类包含无 EKF 基线和被动 EKF 版本；每次试验约 55 s，运行实验发送目标点 `(8, 0, 0)`。被动 EKF 发布 `/odometry/filtered`，设置 `publish_tf=false`，不接管 Nav2 的 `camera_init -> base_link` 定位链。

数据文件：

- `data/ekf_ab_static_before.csv` / `ekf_ab_static_after.csv`：静止无 EKF / 有 EKF。
- `data/ekf_ab_running_before.csv` / `ekf_ab_running_after.csv`：运行无 EKF / 有 EKF。
- `data/ekf_ab_metrics_summary.csv`：由 `experiments/compute_ekf_ab_metrics.py` 生成的 RMSE、P95、最大误差、漂移、路径长度、点云和安全状态汇总。

CSV 同时保存 Gazebo 真值、Point-LIO、EKF、GPS/IMU、命令速度、SafetyCloud 点数、`safety_stop`、NaN 和时间戳倒退计数。运行轨迹统一提供 `ekf_camera_*`，便于在同一 `camera_init` 相对坐标系中画图。

MATLAB 使用：

```matlab
cd report/matlab
plot_ekf_ab_comparison
```

关键结果：

| 实验 | 定位输出 | XY 真值误差 RMSE | P95 | 最大值 |
|---|---|---:|---:|---:|
| 静止，无 EKF | Point-LIO | 0.0650 m | 0.1170 m | 0.2330 m |
| 静止，有 EKF（同次采集） | Point-LIO | 0.0498 m | 0.0955 m | 0.1633 m |
| 静止，有 EKF（同次采集） | EKF | **0.0122 m** | **0.0273 m** | **0.0514 m** |
| 运行，无 EKF | Point-LIO | 1.7641 m | 2.0606 m | 2.2961 m |
| 运行，有 EKF（同次采集） | Point-LIO | 3.0254 m | 4.0214 m | 4.0677 m |
| 运行，有 EKF（同次采集） | EKF | **0.1443 m** | **0.2388 m** | **0.2678 m** |

在同一次有 EKF 试验内，EKF 相对 Point-LIO 的 RMSE 降幅约为静止 75.5%、运行 95.2%。但无 EKF 与有 EKF 是独立冷启动：两次运行真值航程分别约 8.68 m 和 12.58 m，不能把它们当作完全相同轨迹的严格配对试验。算法效果应优先比较 `ekf_ab_*_after.csv` 中同一时间采集的 LIO、EKF 与真值列。

四份文件的 `nan_count` 和 `stamp_backwards` 均为 0。无 EKF 基线文件里的 `ekf_*` 列使用 `NaN` 表示“该试验未启动 EKF”，这是预期占位，不是传感器或定位输出出现非有限值。
