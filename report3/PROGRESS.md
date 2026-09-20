# 当前项目进度交接记录

**记录日期：2026-08-29（Asia/Shanghai）**

## 已完成实验

### 静态单目标导航

有效数据：`data/static_route_after_bt_fix_20260829_valid.csv`。

- 世界：`sydney_regatta`；目标：`(4, 0, yaw=0)`，坐标系 `camera_init`。
- 403 个样本，80.6 s；Nav2 返回 `SUCCEEDED code=4`。
- 最终目标误差 0.534 m；最大横向误差 0.236 m。
- 最小路径净空 8.290 m；SafetyCloud 停车样本为 0。
- 成功后继续记录，确认左右推进器保持为零。

### M5 多航点导航

有效数据：`data/m5_multigoal_20260829_valid.csv`、`_events.csv` 和
`_waypoints.csv`；对应运行日志位于 `logs/m5_multigoal_20260829_valid_*.log`。

航点依次为 `(1.5,0.0) -> (3.0,0.5) -> (4.0,0.0)`。197 个样本、39.4 s，
动作接受后 24.2 s 返回 `SUCCEEDED code=4`。三个航点均按顺序进入
0.7/0.7/0.8 m 判定范围；行驶距离 4.012 m，最大横向误差 0.247 m，
最小路径净空 6.823 m。终止时误差 0.780 m，停车后最终误差 0.481 m；
无 SafetyCloud 停车，成功后两侧推进器均为零。

### M5 远距离单目标导航与推力

有效数据：`data/m5_long_route_30m_20260829.csv`、
`data/m5_long_route_30m_20260829_events.csv` 和
`data/m5_long_route_30m_20260829_metrics.txt`；配置快照位于
`configs/m5_long_route_30m_20260829/`，节点日志位于
`logs/m5_long_route_30m_20260829_*`。

目标为 `camera_init` 坐标系 `(30,0,yaw=0)`，941 个样本、188.2 s。船到达
`(30.259,0.065)`，最终误差 0.267 m，最小误差 0.032 m；Nav2 日志确认
`SUCCEEDED code=4`。行驶距离 56.229 m，最大横向偏差 3.673 m，最小路径
净空 8.038 m，SafetyCloud 停车为 0。左、右推进器各 466 个非零样本，峰值
分别为 64.226 N、61.936 N；目标完成后两侧均为 0 N，证明控制桥确实输出并
在终止时执行了安全清零。

注意：本次原始 logger 的 `goal_x/goal_y/goal_status` 未锁存，故目标和终止
状态以运行命令、事件 CSV 及 Nav2 日志交叉核验；该限制已写入
`data/metrics_summary.csv`，不应忽略。

## 已完成修复

- 单目标和 `NavigateThroughPoses` 使用独立行为树，避免 `{goal}` / `{goals}`
  黑板类型不匹配。
- Gazebo 多航点桥为 `PoseStamped` 填写时间戳，记录动作结果，并支持取消。
- 控制桥 `yaw_sign` 修正为 `+1.0`，且动作终止时立即清零推进器。
- 多航点命令脚本修正 `Odometry` 导入并在超时后取消动作。
- `usv.sh` 强化停止和残留进程清理；每次重启前仍必须人工复核进程。

## 无效记录

M5 的导入失败、错误单目标 BT、仅补时间戳的重试，以及
`yaw_sign=-1` 超时越界运行均保留用于审计，已在
`data/metrics_summary.csv` 标为 `invalid`，不得用于性能结论。

## 下一实验：动态障碍物 A/B

需进行 tracker-disabled 和 tracker-enabled 两次匹配实验。必须固定世界、
初始船位、目标、船速、障碍物几何体、障碍物轨迹和记录时长。每次记录：
航迹、终止状态、SafetyCloud 停车、最小净空、行驶距离、横向误差、路径长度、
动态轨迹数量/点数/速度及完整日志。正式执行前先实现并验证可重复的 Gazebo
移动障碍物生成与轨迹控制，不能用手动拖动物体代替。

## 每次运行的强制流程

```bash
./usv.sh stop
ros2 daemon stop
ps -eo pid,ppid,stat,etime,args | grep -E \
'gz|ros2|rviz|nav2|pointlio|point_lio|usv|gazebo|parameter_bridge|laserMapping|cloud_filter' \
| grep -v grep
```

只有最后一条无输出时才能重新执行 `./usv.sh start`。实验结束重复相同停止与
检查流程，并将 CSV、事件记录、参数快照和各节点日志保存到 `report3/`。

## 动态障碍物 A/B（已完成）

已完成固定条件的 tracker-disabled / tracker-enabled 匹配实验，原始数据、障碍物轨迹、参数快照和全节点日志均已保留。详细汇总见
`data/dynamic_ab_comparison_20260829.md` 和
`data/dynamic_ab_comparison_20260829.csv`；两次运行的指标也已追加到
`data/metrics_summary.csv`。

- 固定世界 `sydney_regatta`，目标为 `camera_init (10, 0, 0)`；测试路径为 10 m 远目标。
- 障碍物由脚本自动生成并沿 `(6.5,-6.0) -> (6.5,6.0)` 移动，配置速度 `0.60 m/s`、更新频率 `5 Hz`，实际运动约 20.1–20.3 s。
- disabled：471 样本、94.401 s，`SUCCEEDED code=4`；最小净空 1.400 m；左右推力峰值 63.874/57.145 N，分别有 239/238 个非零样本。
- enabled：952 样本、190.401 s，`SUCCEEDED code=4`；最小净空 1.503 m；左右推力峰值 64.494/63.653 N，分别有 325/325 个非零样本；动态跟踪峰值 27 条轨迹、1100 点。
- 两次均无 SafetyCloud 停车样本；导航成功后的最后记录中左右推力均为 `0 N`，确认控制桥输出过推力并在终止时安全清零。
- enabled 的跟踪器速度估计峰值为 5.0 m/s，高于障碍物设定速度；该值记录为观测异常/离群峰值，实际障碍物速度以事件 CSV 的轨迹记录为准。

实验完成后已执行 `./usv.sh stop`、`ros2 daemon stop`，并复核无 Gazebo、RViz、ROS 2、Nav2、Point-LIO、tracker、bridge 或 cloud-filter 残留进程。

## 2026-08-30 实验脚本收尾

已完成以下可复现实验基础设施：

- `dynamic_obstacle_scenario.py --static`：固定障碍物只 spawn 一次并写入
  `static_ready`，不再通过零长度动态轨迹模拟静态障碍物。
- `run_static_obstacle_trial.sh`：固定 `(5,0)` 障碍物和 `(10,0)` 目标，启动前
  检查 Nav2/action/目标订阅者，保存参数快照，并要求最终状态严格为
  `SUCCEEDED code=4`。
- `run_dynamic_ab_trial.sh`：动态 A/B 运行结束时同样要求 `SUCCEEDED code=4`，
  否则以非零状态退出，避免把超时或失败运行误纳入比较。
- `compare_navigation_trials.py`：动态图改为四面板，包含匹配航迹、横向误差与
  记录净空、tracker 轨迹/预测点数量、推力与安全停车样本；兼容旧版单 CSV logger。

历史动态 A/B 数据离线重绘已通过；当前历史结果仍以
`data/dynamic_ab_comparison_20260829.{csv,md}` 为准。静态障碍物正式有效运行尚未
产生新的结论：`verified3` 和 `final` 等历史记录存在 `ABORTED`、实际轨迹过短或
目标坐标污染，不得作为静态绕障成功证据。

本环境中直接启动 VRX 链路时，Gazebo/ROS 2 DDS socket 报告 `Operation not
permitted`，导致 Point-LIO 没有发布 `/aft_mapped_to_init`，因此本轮无法在受限
沙箱内完成新的静态船舶实跑；代码、启动门控和离线报告路径已准备好，需在允许
DDS/Gazebo 网络与图形设备的主机上按 `report3/README.md` 执行正式运行。
