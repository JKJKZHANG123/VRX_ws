# VRX USV Navigation Experiment Report

`report3/` 保存修复后 VRX 导航链路的原始 CSV、事件、派生指标、运行日志和
MATLAB 绘图脚本，避免与旧的 LIO/EKF 报告混用。字段含义见
`data/DATA_DICTIONARY.md`，汇总结果见 `data/metrics_summary.csv`。

## 已验证结果

### 静态单目标

`data/static_route_after_bt_fix_20260829_valid.csv`：`sydney_regatta` 世界，
目标 `(4, 0)`，403 个样本、80.6 s，`SUCCEEDED code=4`。最终误差
0.534 m，最大横向误差 0.236 m，最小路径净空 8.290 m，SafetyCloud
停车样本为 0；动作结束后推进器持续为零。

### M5 多航点

`data/m5_multigoal_20260829_valid.csv` 配套 `_events.csv`、`_waypoints.csv`
和 `logs/m5_multigoal_20260829_valid_*.log`。路线为：

```text
(1.5, 0.0) -> (3.0, 0.5) -> (4.0, 0.0)
```

三个航点均按顺序通过，动作接受后 24.2 s 成功。行驶距离 4.012 m，最大
横向误差 0.247 m，最小路径净空 6.823 m；终止误差 0.780 m，停车后最终
误差 0.481 m，无安全停车，成功后左右推进器保持为零。

运行绘图脚本：

```matlab
run('report3/matlab/plot_path_planning_comparison.m')
```

### M5 远距离单目标

`data/m5_long_route_30m_20260829.csv` 及配套 `*_events.csv`、`*_metrics.txt`
记录了 `sydney_regatta` 中从 `(0, 0)` 到 `(30, 0)` 的 30 m 验证。941 个
样本、188.2 s；Nav2 在 `logs/m5_long_route_30m_20260829_nav2.log` 中返回
`SUCCEEDED code=4`，终点误差 0.267 m，最小路径净空 8.038 m，SafetyCloud
停车样本为 0。左/右推进器各有 466 个非零样本，峰值分别为 64.226 N 和
61.936 N；到达后两侧推力均归零。该次原始 logger 未锁存目标/状态字段，目标
和终止结果以事件日志及 Nav2 日志交叉核验，汇总行已注明此限制。

## 无效记录

无 `_valid` 后缀的失败/重试 CSV 仅供审计。包括磁盘耗尽、ROS 参数语法错误、
多目标 BT 配置错误、脚本导入错误和错误 `yaw_sign`。这些记录均不得用于性能
对比；具体原因已写入汇总表的 `validity` 与 `notes`。

## 下一实验

下一阶段为固定条件的动态障碍物 tracker-disabled/enabled A/B。正式运行前，
先验证 Gazebo 障碍物可按同一轨迹自动运动，并为两次实验保存相同的配置快照。
每次重启前和实验结束后必须执行 `./usv.sh stop`、停止 ROS daemon，并用 `ps`
确认 Gazebo、ROS 2、RViz、Point-LIO、Nav2、过滤器和桥接进程全部结束。

### 动态障碍物 A/B

已完成自动障碍物运动下的 tracker-disabled / tracker-enabled 匹配验证。目标为
`camera_init (10,0,0)`，障碍物沿 `(6.5,-6.0)` 到 `(6.5,6.0)` 运动，配置速度
`0.60 m/s`。两次均返回 `SUCCEEDED code=4`，均记录到非零左右推力，并在终止
时清零；SafetyCloud 停车样本均为 0。完整对比见
`data/dynamic_ab_comparison_20260829.md`，原始 CSV、事件 CSV、配置快照和日志
按运行 ID 保存。

## 本阶段实验操作手册

### 1. 静态障碍物：规划路径与实际轨迹对比

静态实验脚本会在目标发布前生成一个固定障碍物，障碍物中心位于
`camera_init (5.0, 0.0)`，目标固定为 `(10.0, 0.0)`。`--static` 模式只执行
一次 Gazebo spawn，并在事件 CSV 中写入 `transform_frozen`、`spawned` 和
`static_ready`，不会把静态障碍物误记成动态运动。

```bash
./usv.sh stop
ros2 daemon stop
./usv.sh start
./experiments/run_static_obstacle_trial.sh 20260830_static01

MPLCONFIGDIR=/tmp/matplotlib-cache \
python3 experiments/compare_navigation_trials.py static \
  --prefix report3/data/static_obstacle_20260830_static01 \
  --scenario-events report3/data/static_obstacle_20260830_static01_scenario.csv \
  --goal 10,0 \
  --output report3/data/static_obstacle_20260830_static01_path_vs_trajectory.png
```

正式有效记录必须同时满足：`SUCCEEDED code=4`、目标误差可接受、规划路径文件
存在且至少包含一个有效路径快照、实际轨迹有明显位移，并且轨迹绕过
`(5.0,0.0)` 障碍物。脚本输出的 JSON/CSV 中重点查看
`latest_plan_length_m`、`actual_path_length_m`、`p95_plan_tracking_error_m`、
`min_scenario_clearance_m` 和 `safety_stop_samples`。历史
`verified3`、`final` 等失败或目标坐标污染记录仅用于审计，不能作为静态避障
结论。

### 2. 动态障碍物：tracker disabled/enabled 对比

两次运行应固定相同的世界、目标、障碍物模型和运动轨迹。运行器假设链路已经
启动，因此每次切换 tracker 都要先停止旧链路并用不同的环境变量重新启动：

```bash
# tracker disabled
./usv.sh stop && ros2 daemon stop
USV_DYNAMIC_TRACKER=false ./usv.sh start
./experiments/run_dynamic_ab_trial.sh tracker_disabled 20260830_dyn01_disabled

# tracker enabled
./usv.sh stop && ros2 daemon stop
USV_DYNAMIC_TRACKER=true ./usv.sh start
./experiments/run_dynamic_ab_trial.sh tracker_enabled 20260830_dyn01_enabled

MPLCONFIGDIR=/tmp/matplotlib-cache \
python3 experiments/compare_navigation_trials.py dynamic \
  --disabled-prefix report3/data/dynamic_ab_tracker_disabled_20260830_dyn01_disabled \
  --enabled-prefix report3/data/dynamic_ab_tracker_enabled_20260830_dyn01_enabled \
  --disabled-events report3/data/dynamic_ab_tracker_disabled_20260830_dyn01_disabled_obstacle_events.csv \
  --enabled-events report3/data/dynamic_ab_tracker_enabled_20260830_dyn01_enabled_obstacle_events.csv \
  --output report3/data/dynamic_ab_20260830_dyn01_comparison.png
```

动态图包含匹配航迹、横向误差/记录净空、tracker 轨迹与预测点数量、左右推力
和安全停车样本。旧版单文件 logger 与新版多 CSV recorder 的字段名已在离线
分析层统一；没有 planner snapshot 的旧动态记录会用直线路径横向误差替代计划
跟踪误差。`min_logged_path_clearance_m` 是记录器在时间局部障碍物云上的最小净空，
优先于把移动障碍物所有历史位置混在一起计算的几何距离；`max_dynamic_speed_mps`
只能作为 tracker 观测诊断，不能直接当作 Gazebo 障碍物真实速度。

所有正式实验结束后执行：

```bash
./usv.sh stop
ros2 daemon stop
ps -eo pid,ppid,stat,etime,args | grep -E \
  'gz|ros2|rviz|nav2|pointlio|point_lio|usv|gazebo|parameter_bridge|laserMapping|cloud_filter' \
  | grep -v grep
```
