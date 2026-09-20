# EKF 与 Point-LIO 对比实验数据

导出日期：2026-08-28

本文件夹保存无人船 VRX 仿真中的两类 EKF 对比实验：静止实验和运行实验。每类实验均包含无 EKF 基线和有 EKF（被动 EKF）数据。

## 数据文件

- `data/ekf_ab_static_before.csv`：静止、无 EKF，Point-LIO 基线。
- `data/ekf_ab_static_after.csv`：静止、启动被动 EKF，同时记录 Point-LIO、EKF 和 Gazebo 真值。
- `data/ekf_ab_running_before.csv`：运行、无 EKF，目标点为 `(8, 0, 0)`。
- `data/ekf_ab_running_after.csv`：运行、启动被动 EKF，目标点为 `(8, 0, 0)`。
- `data/ekf_ab_metrics_summary.csv`：四组数据的 RMSE、P95、最大误差、漂移、路径长度和安全状态汇总。

`truth_*` 是 Gazebo 仿真真值，`lio_*` 是 Point-LIO 输出，`ekf_*` 是 EKF 融合输出。无 EKF 文件中的 `ekf_*` 为 NaN，表示该实验没有启动 EKF，不是实际消息错误。

## 主要结果

| 实验 | 输出 | XY 误差 RMSE | P95 | 最大误差 |
|---|---|---:|---:|---:|
| 静止，无 EKF | Point-LIO | 0.0650 m | 0.1170 m | 0.2330 m |
| 静止，有 EKF | Point-LIO | 0.0498 m | 0.0955 m | 0.1633 m |
| 静止，有 EKF | EKF | **0.0122 m** | **0.0273 m** | **0.0514 m** |
| 运行，无 EKF | Point-LIO | 1.7641 m | 2.0606 m | 2.2961 m |
| 运行，有 EKF | Point-LIO | 3.0254 m | 4.0214 m | 4.0677 m |
| 运行，有 EKF | EKF | **0.1443 m** | **0.2388 m** | **0.2678 m** |

无 EKF 与有 EKF 的运行实验是独立冷启动，实际航程不同；严格对比应优先查看 `ekf_ab_running_after.csv` 中同一次实验的 Point-LIO 与 EKF 列。

## MATLAB 绘图

在 MATLAB 中运行：

```matlab
cd report2/matlab
plot_ekf_ab_comparison
```

脚本会绘制静止误差与漂移、运行 XY 航迹、运行误差、安全停止状态及 RMSE/P95 柱状图。脚本会自动从 `report2/data/` 读取 CSV。

## 指标复算

在仓库根目录执行：

```bash
python3 report2/scripts/compute_ekf_ab_metrics.py \
  --output /tmp/ekf_ab_metrics_check.csv \
  report2/data/ekf_ab_static_before.csv \
  report2/data/ekf_ab_static_after.csv \
  report2/data/ekf_ab_running_before.csv \
  report2/data/ekf_ab_running_after.csv
```

当前实验中 `safety_stop=0`、实际 ROS 数据 `nan_count=0`、时间戳倒退次数为 `0`。EKF 目前是被动评估输出，未接管 Nav2 的 TF 和控制链。

## 本次 EKF 配置快照

`config/` 保存实验使用的融合配置快照：

- `ekf.yaml`：`robot_localization` 平面 EKF 参数；
- `navsat.yaml`：GPS 到局部里程计坐标的转换参数；
- `covariance_injector.py`：为 Point-LIO/GPS 消息补充协方差并统一航向坐标的节点快照。

这些文件用于说明本次数据的生成条件；项目实际运行仍读取 `src/usv_localization/` 下的配置。
