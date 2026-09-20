# Report 3 Change Log

## 2026-08-29

- Added the valid post-BT-fix static-route dataset, metrics, and logs.
- Added the valid M5 `NavigateThroughPoses` dataset, event timeline, per-waypoint
  acceptance table, complete node logs, and derived traveled distance.
- Separated single-goal and multi-goal Nav2 behavior trees and corrected the
  control bridge yaw sign; verified zero thrust after terminal success.
- Retained and explicitly marked disk-full, parameter-syntax, import, BT, and
  incorrect-yaw-sign attempts as invalid audit records.
- Updated the data dictionary and MATLAB plotting script for static and M5 data.
- Defined the next controlled task as matched tracker-disabled/enabled dynamic-
  obstacle testing with an automated, repeatable Gazebo obstacle trajectory.

## 2026-09-01 静态避障:船在障碍物前停车 / 绕整圈的修复

有效记录:`static_plan_20260901_plan22`(`SUCCEEDED code=4`)。
plan01~plan21 为失败/中间记录,仅供审计,**不得用于性能结论**。

**9/2 可复现性验证:** `static_plan_20260902_plan23` 同几何独立复现,也是 `SUCCEEDED code=4`。
间距(Gazebo 真值):船头到障碍物 3.30 m,未进入任何惩罚环。两次航向变化 −27°/−20°(修复前 plan21 是 **−363°**),终点推力归零。解决了"单次成功、不排除偶然"的局限。

### 两项直接测量(先测再改,不是调参猜测)

- `data/turn_capability_20260901.csv` — 停掉 Nav2 与控制桥,直接给推进器加固定
  差动推力,实测船体响应 `wz = 0.00238 * dF`(8/14/25/40 N 四档,线性)。
  据此得出标定值 `k_yaw = 1/0.00238 ≈ 420`,原配置 80 只有需求的 19%。
- `data/planner_latency_20260901.csv` — 同一障碍物条件下调用
  `compute_path_to_pose` 25 次:确认响应 **0.001 s**、完整规划 **0.008 s**。
  **推翻了"规划器卡死"的假设**,`max_iterations` 300000→100000 的改动已回退。
- 换 Reeds-Shepp 前先验证倒退可行:开环实测 −57 N 下倒退 0.435 m/s,
  为前进能力的 79%(`experiments/measure_reverse_capability.py`)。

### 参数改动及其依据

| 参数 | 原值 | 新值 | 依据 |
|---|---|---|---|
| `k_yaw`(桥) | 80 | 420 | 实测船体偏航增益 |
| `max_yaw_difference_ratio`(桥) | 0.45 | 0.70 | 0.45 时转弯直径 20 m > 18 m 目标距离,几何不可行 |
| `minimum_turning_radius` | 2.5 | 7.0 | 与 0.70 差动上限下的可达半径 6.3 m 匹配 |
| `lookahead_dist` | 3.0 | 6.0 | 纯追踪稳定性要求 `Ld ≥ sqrt(2R)` |
| `xy_goal_tolerance` | 0.8 | 2.0 | 14 m 转弯直径命中 0.8 m 圆不可达;plan20 实测最近 1.78 m、容差内 0 样本、绕行 131.78 m 仍未终止 |
| `motion_model_for_search` | DUBIN | REEDS_SHEPP | Dubins 禁倒退,船头偏离目标时唯一合法解是绕 360°;plan21 规划路径在 12 m 与 50–58 m 间跳变并锁定绕圈解,跟踪误差仅 0.39 m(船忠实执行了规划的圈) |
| `allow_reversing`(RPP) | false | true | 否则 RPP 拒绝跟踪 Reeds-Shepp 的倒退段 |
| `footprint`(两张 costmap) | 5.9×3.2 | 4.9×2.2 | 去掉重复计入的 0.5 m 余量 |
| 警戒走廊行为(桥) | 清零推力 | 仅限速 0.5 m/s | 走廊是警戒距离而非碰撞距离;清零使 RPP 的直线接近段被误判为不安全 |
| `safety_emergency_half_width_m`(桥) | 1.8 | 1.3 | 侧向安全通过时箭体边缘落在 base_link \|y\|≈1.75 m,1.8 m 会误触发硬停 |

### plan21(DUBIN)与 plan22(REEDS_SHEPP)对比

| 指标 | plan21 | plan22 |
|---|---|---|
| 行驶距离 | 76.61 m | **25.79 m** |
| 累计航向变化 | **−363°**(整圈) | **−27°** |
| y 范围 | −19.36 … 0.79 | 0.05 … 3.75 |
| 倒退指令样本 | 0 | 71 |
| 船中心/船头最近障碍物 | 0.94 / 0.82 m | **2.66 / 2.59 m** |
| 规划路径 >40 m(绕圈特征) | 有 | **0 条** |
| 终点误差 | 1.78 m | 1.94 m |

### 记录工具修复(影响此前所有报告数字)

`experiments/record_nav_path.py` 原先把 RPP 以 10 Hz 回显的
`/received_global_plan` 也计为规划快照,使 `plan_snapshots` 虚高约 35 倍
(plan19:335 vs 真实 9),让正常的 0.25 Hz 重规划看起来像失控重规划。
现只有 `/plan` 与 `/unsmoothed_plan` 递增计数。**此前记录中的
`plan_snapshots` 数值均不可直接比较。**

### 指标读法提醒

`plan22_path_vs_trajectory.json` 中:

- `latest_plan_length_m: 4.96` 是**最后一次重规划的剩余残段**(t=96.9 s,
  已接近目标),不是全程路径;首次规划为 23.12 m,与实际 25.79 m 对应。
- `mean_crosstrack_m: 1.84` 是相对起点→目标**直线**的偏移,即绕障所必需的
  横移量,不是跟踪误差。真正的跟踪误差是 `mean_plan_tracking_error_m: 0.24`。

### 新增测量脚本

`experiments/measure_turn_capability.py`、`measure_reverse_capability.py`、
`probe_planner_latency.py`、`probe_goal_admissibility.py`、
`probe_start_occupancy.py`、`replay_corridor_gate.py`、
`run_static_plan_trial.sh`(几何可参数化,并在生成障碍物前先校验目标可受理)。
