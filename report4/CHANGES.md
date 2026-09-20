# Report 4 Change Log

## 2026-09-01 突现障碍物实验的建立与两个系统性缺陷修复

有效记录:`sudden_obstacle_20260901_sudden03_valid`(`SUCCEEDED code=4`)。
`sudden01` / `sudden02` 为无效记录,仅供审计。

### 新增实验设施

- `experiments/run_sudden_obstacle_trial.sh` — 突现障碍物试验运行器。与
  `run_static_plan_trial.sh` 的本质区别:**先发目标、等船巡航起来、再注入
  障碍物**,因此记录到的是真实规避瞬态而非预先算好的绕行路径。
- `experiments/spawn_obstacle_ahead.py` — 障碍物注入器。触发条件是船自身的
  速度与已行驶距离(而非固定 `sleep`),注入位置由 **Gazebo 世界真值**计算,
  并在生成前校验实际间距 ≥ 请求值的 90%,否则拒绝生成。

### 参数改动

| 参数 | 原值 | 新值 | 依据 |
|---|---|---|---|
| `default_server_timeout` | 20 | 500 | 单位是**毫秒**,20 ms 不够规划器确认 |
| `bt_loop_duration` | 10 | 100 | 同为毫秒,且是 BT tick 周期,须保持小 |
| `max_iterations` | 100000 | 300000 | **回退**:延迟实测推翻了"规划器卡死"假设 |

### 缺陷一:`default_server_timeout` 单位是毫秒

`nav2_behavior_tree/bt_action_server_impl.hpp:169` 把整数直接包进
`std::chrono::milliseconds`,所以配置里的 `20` 是 **20 毫秒**,不是 20 秒。

实测规划器确认耗时 1 ms、完整规划 3–18 ms
(`report3/data/planner_latency_20260901.csv`),正常 tick 能挤进 20 ms,最坏
情况不能 —— 因此中止是**间歇性**的:`report3` 的 plan17/plan22 跑通,
sudden01 在目标接受后 4 s 就报
`Timed out while waiting for action server to acknowledge goal request for
compute_path_to_pose` → `Goal failed`。

这个缺陷可能同时解释了 `report3` 阶段多次难以复现的中止。

### 缺陷二:注入坐标不能用会漂移的 LIO 位姿

sudden02 的注入点写在 `camera_init` 系,经**实验开始时冻结**的
`camera_init → world` 变换换算。船行驶 30 m 后 Point-LIO 已漂移,
"前方 8 m"落到真实世界变成**船体位置**,箱子生成在船身上。

更需要警惕的是**基于 LIO 的距离计算会掩盖事故**:

| | LIO 算出 | Gazebo 真值 |
|---|---|---|
| 船中心到障碍物 | 6.72 m | **1.99 m** |

误差 4.73 m。当时桥报的
`COLLISION EMERGENCY STOP: obstacle inside hull envelope (2379 points)`
才是正确的 —— 6.7 m 外的 1.2 m 箱子不可能产生 2379 个点。

**规则:凡判断"是否碰撞/间距多少",一律用 Gazebo 真值,不得用 LIO 位姿。**

### sudden03_valid 结果摘要

注入间距经真值校验为 **12.00 m**(请求 12.0 m)。

| 指标 | 数值 |
|---|---|
| 最终状态 | `SUCCEEDED code=4` |
| 行驶距离 | 25.72 m(目标 24 m) |
| 船中心 / 船头最近障碍物 | **2.94 / 3.00 m** |
| 进入致命环(1.70 m) | 否 |
| 进入 cost≥50 环(2.24 m) | 否 |
| 航向明确改变(>5°)用时 | **7.2 s** |
| 注入后安全限速样本 | 50 / 265(无停车死锁) |
| 终点推力 | L=0, R=0 |

### 待办

- **重复性验证**:目前只有一次有效运行,需在相同几何下重复 ≥3 次。
- **及时性边界**:只测了 12 m 一档,应按 10 / 8 / 6 m 递减扫描,确定多近会
  来不及规避。
- `compare_navigation_trials.py` 的 `min_scenario_clearance_m` 对本实验输出
  `nan`,因为它按 `camera_init` 读 scenario 事件,而本实验存的是世界系偏移。
  应让该脚本支持世界系 scenario 记录。
