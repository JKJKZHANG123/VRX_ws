# Report 4 — 航行中突现障碍物的实时规避实验

`report4/` 保存**动态突现障碍物**实验的原始 CSV、事件、派生指标和运行日志。
与 `report3/` 的静态实验区别在于:障碍物**不是**在发目标前就存在,而是在
无人船已经巡航起来之后才突然出现在正前方,因此记录到的是
「探测 → 重规划 → 执行规避」的真实瞬态,而不是一条预先算好的绕行路径。

有效记录:`sudden_obstacle_20260901_sudden03_valid`(`SUCCEEDED code=4`)。

## 实验设计

| 项 | 值 |
|---|---|
| 世界 | `sydney_regatta` |
| 目标 | `camera_init (24, 0)`,发目标时前方是空水面 |
| 障碍物 | 1.2×1.2×2.0 m 箱体,`z=1.0` |
| 注入时机 | 船速 ≥0.25 m/s **且** 已行驶 ≥4.0 m |
| 注入位置 | 船**当前航向正前方 12.0 m** |
| 注入坐标依据 | **Gazebo 世界真值**(见下方"关键修正") |

触发条件用的是船自身的运动量(速度 + 已行驶距离)而不是固定 `sleep`,
这样即使启动耗时有波动,注入几何仍然可复现。

## 关键修正:注入坐标必须用 Gazebo 真值,不能用 LIO

前两次尝试(sudden01/sudden02)**失败,且失败方式具有欺骗性**,必须记录:

注入脚本原先把目标点写在 `camera_init` 系,再用实验开始时**冻结**的
`camera_init → Gazebo world` 变换换算。但船行驶 30 m 后 Point-LIO 已经漂移,
那个冻结变换不再成立。于是"前方 8 m"这个意图,落到真实世界里变成了
**船体所在位置** —— 箱子直接生成在船身上,船被卡住。

更危险的是**基于 LIO 的距离计算会掩盖这个事故**:

| | LIO 坐标算出的 | **Gazebo 真值** |
|---|---|---|
| 船中心到障碍物 | 6.72 m | **1.99 m** |
| 障碍物在船体系 | — | x=−0.67, y=1.88(已在重心后方) |

按 LIO 算是"安全 6.72 m",按真值是"箱子嵌在船侧",误差 **4.73 m**。
当时桥的日志 `COLLISION EMERGENCY STOP: obstacle inside hull envelope
(2379 points)` 才是对的 —— 6.7 m 外的 1.2 m 箱子不可能产生 2379 个点。

**结论:凡是判断"有没有撞上"的量,一律必须用 Gazebo 真值,不能用 LIO 位姿。**
`experiments/spawn_obstacle_ahead.py` 现在直接用 `gz model -m wamv -p` 读真实
位姿和航向,在世界系里算注入点,并在生成前校验实际间距 ≥ 请求值的 90%,
否则拒绝生成。

## sudden03_valid 结果

注入几何(Gazebo 真值校验):

```
trigger_met  boat_world=(-529.752, 166.732)  yaw=1.338 rad  speed=0.56 m/s
spawned      obstacle_world=(-526.985, 178.409)
实测间距 12.00 m(请求 12.0 m)
```

导航结果:

| 指标 | 数值 |
|---|---|
| 最终状态 | **`SUCCEEDED code=4`** |
| 轨迹样本 | 347 |
| 行驶距离 | 25.72 m(目标距离 24 m) |
| **船中心最近障碍物** | **2.94 m** |
| **船头尖端最近障碍物** | **3.00 m** |
| 船头到箱体表面 | 2.40 m |
| 进入致命环(1.70 m)? | **否** |
| 进入 cost≥50 环(2.24 m)? | **否** |
| 规划跟踪误差 | p95 1.98 m |
| 真实重规划次数 | 14(仅统计 `/plan`) |
| 注入后安全限速样本 | 50 / 265 |
| 终点推力 | L=0, R=0 |

规避瞬态(以注入时刻为 0):

```
 +0.0s  x= 3.76 y=0.90  dyaw=  +0.0 deg   障碍物刚出现
 +2.0s  x= 4.65 y=1.14  dyaw=  +1.5 deg
 +6.0s  x= 6.37 y=1.56  dyaw=  -2.0 deg   进入警戒走廊,开始限速
 +8.0s  x= 7.15 y=1.83  dyaw=  -6.6 deg   明确转向
+12.0s  x= 8.92 y=2.06  dyaw= -17.2 deg
+16.0s  x=10.67 y=2.15  dyaw= -23.8 deg   已绕过
```

- **航向开始明确改变(>5°)用时 7.2 s**,此时船距障碍物约 8 m
- 注入后最大横向偏移 1.27 m
- 全程 `cmd_vx` 保持 0.60 → 进走廊后限到 0.50,**没有出现停车死锁**

## 无效记录

`sudden01`、`sudden02` 无 `_valid` 后缀,**仅供审计,不得用于规避性能结论**:

- `sudden01` — 目标接受后 4 s 即 `Timed out while waiting for action server to
  acknowledge goal request for compute_path_to_pose` → `Goal failed`。根因是
  `default_server_timeout` 的单位是**毫秒**(见下),配置值 20 意味着只给规划器
  20 ms 确认时间。
- `sudden02` — 注入点因 LIO 漂移落在船身上,箱子嵌进船体后硬停中止。

## 期间修复的两个系统性缺陷

**1. `default_server_timeout` 单位是毫秒,不是秒**

`nav2_behavior_tree/bt_action_server_impl.hpp:169`:

```cpp
default_server_timeout_ = std::chrono::milliseconds(default_server_timeout);
```

配置里的 `20` 是 **20 毫秒**。实测规划器确认耗时 1 ms、完整规划 3–18 ms
(`report3/data/planner_latency_20260901.csv`),正常情况能挤进 20 ms,但最坏
情况超时 —— 所以中止是**间歇性**的:report3 的 plan17/plan22 跑通了,
sudden01 开 4 秒就死。改为 `default_server_timeout: 500`、
`bt_loop_duration: 100`(后者是 BT 的 tick 周期,必须保持小,否则整棵树降频)。

此前曾把这归因为"规划器搜索卡死"并调低 `max_iterations`,已被延迟实测推翻并回退。

**2. 记录脚本虚报重规划次数约 35 倍**

`experiments/record_nav_path.py` 把 RPP 以 10 Hz 回显的
`/received_global_plan` 也计为规划快照。已修:只有 `/plan` 与
`/unsmoothed_plan` 递增计数。**`report3` 早期记录中的 `plan_snapshots`
数值不可与本报告直接比较。**

## 复现步骤

```bash
./usv.sh stop && ros2 daemon stop
./usv.sh start                      # 等 8 个进程全部就绪
./experiments/run_sudden_obstacle_trial.sh <RUN_ID> 24.0 0.0 12.0 0.0

MPLCONFIGDIR=/tmp/matplotlib-cache \
python3 experiments/compare_navigation_trials.py static \
  --prefix report4/data/sudden_obstacle_<RUN_ID> --goal 24,0 \
  --output report4/data/sudden_obstacle_<RUN_ID>_path_vs_trajectory.png
```

正式有效记录必须同时满足:`SUCCEEDED code=4`、注入间距经 Gazebo 真值校验
接近请求值、船头未进入 cost≥50 环、终点推力归零。

**每次实验前后都必须** `./usv.sh stop && ros2 daemon stop`,并用 `ps` 确认
gz / ros2 / rviz / nav2 / laserMapping / cloud_filter / bridge 全部退出;
残留的 launch 父进程会产生重复的 TF 与控制写入者。

## 已知限制

- **只有一次有效运行。** 尚未做重复性验证,不能排除这次是有利初始航向下的
  偶然成功。下一步应在相同几何下重复 3 次以上。
- 注入距离只测了 12 m 一档。真正的"及时性"边界(多近才来不及)需要按
  10 / 8 / 6 m 递减扫描才能确定。
- `_path_vs_trajectory.json` 里的 `min_scenario_clearance_m` 为 `nan`,因为
  该分析脚本按 `camera_init` 读 scenario 事件,而本实验的 scenario CSV 存的是
  世界系偏移。报告中的 2.94 / 3.00 m 是另行用 Gazebo 真值算的,以后者为准。
- `mean_crosstrack` 类指标是相对起点→目标**直线**的偏移,即绕障所需横移,
  不是跟踪误差。
