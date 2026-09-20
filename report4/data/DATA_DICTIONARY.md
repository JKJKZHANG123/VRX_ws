# Report 4 数据字典

文件命名:`sudden_obstacle_<日期>_<run_id>[_valid]_<类型>.csv`

**只有带 `_valid` 后缀的记录可用于结论**;其余为失败/中间记录,仅供审计。

## `*_trajectory.csv`

固定 0.2 s 周期采样。位姿在 `camera_init`(Point-LIO 世界系)。

| 列 | 含义 |
|---|---|
| `t` | 相对记录起点的秒数(sim time) |
| `pose_x` / `pose_y` / `pose_yaw` | 船在 `camera_init` 的位姿。**会随 LIO 漂移** |
| `goal_x` / `goal_y` / `goal_active` | 当前目标及是否激活 |
| `goal_status` | 目标网关状态字符串(`ACCEPTED` / `SUCCEEDED code=4` 等) |
| `plan_id` | 规划快照编号。**只在收到 `/plan` 或 `/unsmoothed_plan` 时递增**,RPP 以 10 Hz 回显的 `/received_global_plan` 不再计数 |
| `plan_nearest_error_m` | 船到当前规划路径最近点的距离(规划跟踪误差) |
| `raw_obstacle_points` | `/usv/raw_obstacle_cloud` 点数 |
| `dynamic_obstacle_points` / `dynamic_tracks` / `dynamic_predicted_points` / `dynamic_max_speed_mps` | 动态追踪器输出(本实验未启用,恒为 0) |
| `safety_stop` | 桥的**警戒走廊**标志。**注意:1 不等于停车** —— 自 2026-09-01 起走廊只限速到 0.5 m/s,硬停由紧急包络负责 |
| `cmd_vx` / `cmd_wz` | Nav2 经 velocity_smoother 后的速度指令 |
| `left_thrust_n` / `right_thrust_n` | 实际下发的左右推进器推力 |

## `*_plans.csv`

每条规划路径的完整几何,逐点一行。

| 列 | 含义 |
|---|---|
| `plan_id` | 规划编号,与 trajectory 的 `plan_id` 对应 |
| `t` | 收到该规划的相对时刻 |
| `source` | `/plan`(规划器输出)、`/unsmoothed_plan`、`/received_global_plan`(控制器回显)、`/plan_smoothed` |
| `frame_id` | 原始消息坐标系 |
| `point_index` | 该点在路径中的序号 |
| `x` / `y` / `yaw` | 已换算到 `camera_init` 的位姿 |
| `raw_x` / `raw_y` / `raw_yaw` | 换算前的原始值 |

统计"重规划了几次"时**只应统计 `source == /plan`**。

## `*_obstacles.csv`

| 列 | 含义 |
|---|---|
| `t` | 采样时刻 |
| `source` | `raw`(原始 Gazebo 扫描)或 `dynamic` |
| `point_index` | 点序号 |
| `x` / `y` / `z` | **船体系 `wamv/wamv/base_link`** 坐标 |

## `*_scenario.csv`(障碍物注入事件)

| 列 | 含义 |
|---|---|
| `wall_time` / `sim_time` | 墙钟时间 / 仿真时间 |
| `event` | `start`、`waiting_for_motion`、`trigger_met`、`spawned` |
| `camera_x` / `camera_y` | ⚠️ **本实验中这两列存的是「世界系偏移」,不是 `camera_init` 坐标**。因为注入点改用 Gazebo 真值计算后,不再经过 `camera_init` |
| `world_x` / `world_y` | **Gazebo 世界真值坐标 —— 判断实际间距请用这一对** |
| `detail` | 触发时的船速、已行驶距离、世界系航向、校验后的实际间距 |

`min_scenario_clearance_m` 在 `_path_vs_trajectory.json` 中为 `nan`,原因是
`compare_navigation_trials.py` 按 `camera_init` 解释 scenario 事件。真实间距
(2.94 / 3.00 m)是另行用 Gazebo 真值计算的,以真值为准。

## `*_path_vs_trajectory.json`

派生指标。三个容易误读的量:

- `latest_plan_length_m` — **最后一次重规划的剩余残段**,不是全程路径长度。
- `mean_crosstrack_m` / `p95_crosstrack_m` — 相对起点→目标**直线**的偏移,
  即绕障所必需的横移量,**不是跟踪误差**。真正的跟踪误差是
  `mean_plan_tracking_error_m`。
- `min_scenario_clearance_m` — 本实验为 `nan`,见上。

## 坐标系与"是否碰撞"的判定规则

`camera_init` 会随 Point-LIO 漂移。sudden02 的教训:按 LIO 算船离障碍物
6.72 m,而 Gazebo 真值只有 1.99 m(箱子已嵌在船侧),误差 4.73 m。

**因此:任何关于碰撞、间距、避障成败的判定,必须用 Gazebo 世界真值**
(`gz model -m wamv -p` 与 `gz model -m <obstacle> -p`),
不能用 `_trajectory.csv` 里的 `pose_x/pose_y`。后者只适合看轨迹形状、
航向变化、跟踪误差这类**相对量**。
