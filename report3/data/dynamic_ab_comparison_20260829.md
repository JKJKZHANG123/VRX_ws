# Dynamic Obstacle A/B Validation — 2026-08-29

固定条件：`sydney_regatta`，目标 `camera_init (10.0, 0.0, 0.0)`；自动生成的障碍物从 `(6.5, -6.0)` 以配置速度 `0.60 m/s` 沿 Y 轴运动到 `(6.5, 6.0)`，更新频率 `5 Hz`。两次实验均使用同一障碍物模型、目标和记录流程。

## 结果

| 指标 | tracker disabled | tracker enabled |
|---|---:|---:|
| 记录样本 / 时长 | 471 / 94.401 s | 952 / 190.401 s |
| 最终状态 | SUCCEEDED code=4 | SUCCEEDED code=4 |
| 最终位置 / 目标误差 | (9.743, 0.351) / 0.435 m | (9.893, 0.484) / 0.496 m |
| 最小目标误差 | 0.298 m | 0.221 m |
| 最大横向误差 / P95 | 1.158 / 0.989 m | 0.790 / 0.624 m |
| 最小路径净空 | 1.400 m | 1.503 m |
| 最大路径长度 | 13.717 m | 13.911 m |
| 峰值左右推力 | 63.874 / 57.145 N | 64.494 / 63.653 N |
| 非零左右推力样本 | 239 / 238 | 325 / 325 |
| 终止左右推力 | 0.000 / 0.000 N | 0.000 / 0.000 N |
| SafetyCloud 停车样本 | 0 | 0 |
| 最大动态轨迹 / 点数 | 0 / 0 | 27 / 1100 |
| 最大动态速度估计 | 0.000 m/s | 5.000 m/s |

## 验证结论

- 两次均到达远距离目标并返回 `SUCCEEDED code=4`；enabled 运行记录更长，是因为目标成功后按既定流程继续记录 45 s。
- 两次均出现非零左右推力，且终止样本左右均为 `0 N`；因此控制桥确实输出推进命令，并在导航完成后清零。
- enabled 运行中动态跟踪字段非零（峰值 27 条轨迹、1100 点），且自动障碍物轨迹事件完整记录；disabled 运行中动态跟踪字段保持为零。
- 两次 `safety_stop_samples=0`。tracker-enabled 的动态速度估计峰值为 `5.000 m/s`，高于障碍物配置速度 `0.60 m/s`，应视为跟踪器观测峰值/异常值，不应当作障碍物实际速度；实际障碍物轨迹由事件 CSV 记录为约 `20.092 s`。

## 原始数据与日志

- `report3/data/dynamic_ab_tracker_disabled_dynamic_20260829_2050_disabled.csv`
- `report3/data/dynamic_ab_tracker_disabled_dynamic_20260829_2050_disabled_obstacle_events.csv`
- `report3/configs/dynamic_ab_tracker_disabled_dynamic_20260829_2050_disabled/`
- `report3/logs/dynamic_ab_tracker_disabled_dynamic_20260829_2050_disabled_*.log`
- `report3/data/dynamic_ab_tracker_enabled_dynamic_20260829_2120_enabled.csv`
- `report3/data/dynamic_ab_tracker_enabled_dynamic_20260829_2120_enabled_obstacle_events.csv`
- `report3/configs/dynamic_ab_tracker_enabled_dynamic_20260829_2120_enabled/`
- `report3/logs/dynamic_ab_tracker_enabled_dynamic_20260829_2120_enabled_*.log`

## 环境收尾

实验结束后执行了 `./usv.sh stop` 和 `ros2 daemon stop`；随后进程复核无 Gazebo、RViz、ROS 2、Nav2、Point-LIO、tracker、bridge 或 cloud-filter 残留。
