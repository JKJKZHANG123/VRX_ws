# 数据文件说明

| 文件 | 用途 |
|---|---|
| `raw_lio_stationary_before_drift_fix_20260827.csv` | 项目现有的漂移修复前静止测试基线；不是 Git 初始数据 |
| `improved_lio_stationary_final_optimized_20260827.csv` | 当前最终优化静止测试 |
| `improved_lio_stationary_after_drift_fix_20260827.csv` | 中间改进版本 |
| `improved_lio_stationary_after_zupt_candidate_20260827.csv` | 已有 ZUPT 候选静止测试，需同链路复测 |
| `lio_candidate_comparison_20260827.csv` | 多次静止测试候选的统一指标比较 |
| `lio_metrics_summary.csv` | 从三组静止 CSV 计算的漂移、误差、点数统计 |
| `cloud_filter_validation_20260827.csv` | 原始水面/结构云过滤快照 |
| `improved_costmap_pipeline_20260827.csv` | 改进链路的频率、点数和 costmap 运行数据 |
| `pipeline_metrics_summary.csv` | 上面两类数据的汇总，已注明不可直接等同比较的指标 |
| `historical_nav_log.csv` | 历史导航运行日志，字段由 `nav_data_logger` 定义 |
| `safetycloud_nav2_runtime_validation_20260827.csv` | 从本次 Nav2/SafetyCloud 日志统计 TF 启动竞态、点云输出和 lifecycle 激活状态 |


## 展示建议

- 轨迹图：用每组数据的 `lio_x_m/lio_y_m` 和 `truth_x_m/truth_y_m` 各自减去首样本，画 XY 曲线。
- 漂移图：横轴 `time_s`，纵轴 `lio_xy_drift_m`，重点看最大值、95 分位和末值。
- 点云图：画 `filtered_points`、`registered_points` 随时间变化；不要只看点数，还要确认 `registered_points>0`。
- 过滤图：旧快照中的 `water_band_points` 与 `raw_obstacle_water_band_points` 只能作为基线证据；改进数据应使用实时记录补充水面残留计数后再计算严格过滤率。

## EKF A/B 数据

| 文件 | 用途 |
|---|---|
| `ekf_ab_static_before.csv` | 独立冷启动静止基线，无外部 EKF |
| `ekf_ab_static_after.csv` | 独立冷启动静止试验，同时记录 LIO 与被动 EKF |
| `ekf_ab_running_before.csv` | 独立冷启动运行基线，无外部 EKF，目标 `(8,0,0)` |
| `ekf_ab_running_after.csv` | 独立冷启动运行试验，同时记录 LIO 与被动 EKF，目标 `(8,0,0)` |
| `ekf_ab_metrics_summary.csv` | 四个试验的误差、漂移、路径长度和运行诊断汇总 |

关键字段：`truth_x_rel_m/truth_y_rel_m` 是旋转到 `camera_init` 的真值相对轨迹；`lio_*` 是 Point-LIO；`ekf_x_rel_m/ekf_y_rel_m` 是 EKF 的世界系相对轨迹；`ekf_camera_*` 仅用于与 LIO/真值叠加绘图；`*_truth_xy_error_m` 是相对起点后的平面真值误差；`*_xy_drift_m` 是相对起点漂移。`safety_stop`、点云计数、`nan_count` 和 `stamp_backwards` 用于判断实验是否被安全链或数据异常污染。

汇总中的 `final/max/p95/rmse` 分别表示末样本、最大值、95 百分位和均方根；路径长度是相邻有效 XY 样本间距离之和。

无 EKF 的 `*_before.csv` 中，全部 `ekf_*` 字段为 `NaN`，含义是“该列不适用”。这与 `nan_count=0` 不矛盾：`nan_count` 只统计实际收到的 ROS 定位消息中是否出现非有限数值。
