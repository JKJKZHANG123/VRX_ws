# USV LiDAR 感知、动态避障与导航控制系统技术文档

**版本日期：2026-08-29**  
**平台：ROS 2 Jazzy + Gazebo/VRX + WAM-V**

## 1. 系统目标与总体架构

本系统面向 VRX WAM-V 无人船，实现“LiDAR 感知—定位—障碍物建模—路径规划—推力控制—安全停车”的闭环导航。当前主要导航链路如下：

```text
Gazebo LiDAR/IMU
  → lidar_timestamp_adapter（有限点与逐点时间处理）
  → Point-LIO（定位、点云注册、TF）
  → usv_cloud_filter（水面/船体/噪声过滤）
  → raw obstacle / safety cloud / structure map
  → Nav2 costmap
  → Smac Hybrid-A*（全局规划）
  → Regulated Pure Pursuit（路径跟踪与碰撞预测）
  → velocity_smoother
  → control_bridge（速度到差动推力）
  → WAM-V 左右推进器
```

GPS/EKF 融合定位链路已单独验证，可通过 `usv_localization` 提供 `/odometry/filtered`；但当前 `usv_navigation` 配置以 Point-LIO 的 `/aft_mapped_to_init` 作为导航主里程计，避免重复 TF 和未稳定的速度状态进入控制桥。

## 2. 节点、坐标系与主要接口

| 功能 | 主要节点/包 | 输入 | 输出 |
|---|---|---|---|
| 点云时间适配 | `lidar_timestamp_adapter` | VRX 原始点云 | `/wamv/points_filtered` |
| LIO 定位建图 | `point_lio_ros2` | 点云、IMU | `/aft_mapped_to_init`、`/cloud_registered` |
| 点云过滤 | `usv_cloud_filter` | 注册点云、原始 LiDAR | `/usv/raw_obstacle_cloud`、`/usv/safety_cloud`、`/usv/structure_map_cloud` |
| 动态跟踪 | `dynamic_obstacle_tracker` | raw cloud、结构地图 | `/usv/dynamic_obstacle_cloud`、`/usv/dynamic_predictions`、`/usv/dynamic_metrics` |
| 导航规划控制 | `usv_navigation`/Nav2 | 点云、TF、目标 | `/plan`、`/cmd_vel_smoothed` |
| 推力与安全 | `usv_control_bridge` | 速度、SafetyCloud、目标状态 | 左/右推进器 thrust、`/usv/safety_stop` |
| 数据记录 | `nav_data_logger` | 位姿、路径、点云统计、推力 | `report3/data/*.csv` |

核心坐标系为 `camera_init`（LIO 世界系）和 `wamv/wamv/base_link`（船体系）。过滤后的近场点云发布在船体坐标系，使 Nav2 的量程裁剪和射线清除以“当前船位”为原点，而不是以世界原点为原点。

## 3. 感知与点云处理算法

### 3.1 LiDAR 时间与有限性处理

Gazebo 无回波射线会产生 `inf` 坐标；原始点云还缺少 Point-LIO 需要的逐点 `time` 字段。适配层删除非有限点，并依据 organized 点云的列号合成扫描内时间，避免 LIO 因未初始化时间字段出现同步死等、畸变校正异常或 `cos_sinc_sqrt` 崩溃。

### 3.2 水面与障碍物过滤

`usv_cloud_filter` 在转换到 `base_link` 后执行：

1. 删除非有限点；
2. 按高度带去除水面反射和低矮波花；
3. 去除 WAM-V 自身船体/甲板回波；
4. 以船体为中心裁剪距离窗口；
5. 0.1 m 体素降采样；
6. 半径离群点滤波，抑制孤立水花点。

同时生成三类数据：

- `raw_obstacle_cloud`：当前近场扫描，供 Nav2 实时标记和清除；
- `safety_cloud`：直接由原始 LiDAR 生成，绕过 LIO 累积地图，供独立安全门；
- `structure_map_cloud`：在启动预热后累计、经多帧确认并冻结的世界系静态结构地图。

### 3.3 动态障碍物跟踪与预测

跟踪器采用无机器学习依赖、易审计的轻量算法：

```text
当前扫描 − 冻结静态地图
  → XY 栅格 8 邻域连通聚类
  → 聚类质心观测
  → 最近邻轨迹关联
  → Alpha-Beta 匀速滤波
  → 8 s 短期轨迹预测
  → base_link 预测点云
```

对于观测位置 `z` 和轨迹状态 `(x,v)`，先按匀速模型预测，再用残差更新：

```text
x̂ = x + v·Δt
r = z − x̂
x ← x̂ + αr
v ← v + βr/Δt
```

当前关键参数包括：静态体素 0.75 m、聚类栅格 0.8 m、轨迹匹配半径 4 m、超时 1.5 s、确认帧数 2、`α=0.65`、`β=0.20`、预测时域 8 s。新轨迹保留 0.8 s，避免目标刚出现时因尚未形成速度估计而被忽略；预测点周围扩展 1.5 m 十字区域，再交由 costmap inflation 处理船体安全边界。

## 4. Nav2 规划与控制算法

- **代价地图**：30×30 m rolling local costmap、100×100 m rolling global costmap，实时 LiDAR 障碍层支持 marking/clearing；船体 footprint 约为 5.1×2.6 m。
- **全局规划**：Smac Hybrid-A*，Dubins 运动模型，最小转弯半径 2.5 m，允许在线未知区，但不允许穿越已观测障碍。
- **局部控制**：Regulated Pure Pursuit，目标巡航速度 0.6 m/s，按曲率、障碍代价和预计碰撞时间自动减速；禁止倒车和原地旋转式路径跟踪。
- **速度平滑**：20 Hz 限制速度、加速度和角速度变化。
- **推进器映射**：控制桥不使用有问题的速度 PID，而采用阻力模型前馈：

```text
F  = 51.3·vx + 72.4·vx·|vx|
dF = 80.0·wz·yaw_sign
left  = F − dF
right = F + dF
```

随后执行推力限幅、变化率限制和 2 N 死区。`cmd_vel` 超时、SafetyCloud 超时/异常、前向走廊出现至少 3 个障碍点，或目标进入终态时，左右推力均立即清零。

## 5. 运行工作流程与数据记录

1. 启动 VRX：`./sim.sh`；
2. 启动 Point-LIO：`./lio.sh`；
3. 按需启动定位：`./ekf.sh`；
4. 启动点云过滤、Nav2 和控制桥：`./usv.sh start`；
5. 通过 `/goal_pose` 或目标脚本发送单目标/多航点；
6. 记录位姿、目标状态、路径、点云数量、SafetyCloud、动态轨迹、速度指令和左右推力；
7. 结束后执行 `./usv.sh stop`、`ros2 daemon stop`，并用 `ps` 确认无 Gazebo、Nav2、LIO、tracker、bridge 残留。

动态障碍物实验使用 `experiments/dynamic_obstacle_scenario.py` 自动生成并移动障碍物，避免依赖 Gazebo GUI 手工添加。2026-08-29 的固定条件 A/B 实验中，目标为 `(10,0)`，障碍物沿 `(6.5,-6.0)` 到 `(6.5,6.0)`、配置速度 0.60 m/s 运动；tracker-disabled 和 enabled 均返回 `SUCCEEDED code=4`，最小净空分别为 1.400 m 和 1.503 m，均记录到非零推力，终止时左右推力均为 0 N。

## 6. 系统创新点

1. **面向移动船体的坐标系感知点云链路**：在 `base_link` 中完成高度、距离和自船体裁剪，解决世界系发布导致的量程/清除原点错误。
2. **规划器之外的独立安全闭环**：SafetyCloud 直接来自当前 LiDAR，定位或地图延迟不会使安全停车失效；感知失效按“停车”处理。
3. **静态地图冻结 + 动态残差跟踪**：先用多帧确认建立静态结构，再从实时扫描中剔除，降低岸线和固定浮标对动态检测的干扰。
4. **轻量、可解释的动态预测**：栅格聚类、最近邻和 Alpha-Beta 滤波无需深度模型或外部数据集，适合实时仿真和资源受限平台。
5. **基于船舶阻力模型的推力前馈**：绕开不可靠 EKF 速度状态，利用 Nav2 外环位置控制直接产生稳定推进力。
6. **可复现实验与证据链**：自动障碍物脚本、配置快照、事件 CSV、导航 CSV 和节点日志共同保存，支持 tracker 开关 A/B 对比。

## 7. 已修复问题与当前 Bug

### 已修复

- Point-LIO 输入 `inf` 点和缺失 `time` 字段导致的同步死等/断言崩溃；
- EKF 差分位姿纳秒级 `dt` 放大速度、零协方差引发 NaN；
- 单目标与多目标行为树黑板类型不匹配；
- `yaw_sign=-1` 导致转向方向错误；
- 目标完成后推进器未及时清零；
- `collision_monitor` 在仿真时间戳陈旧时阻塞 `/cmd_vel`，已改由 costmap、RPP 碰撞检测和控制桥安全门承担职责。

### 尚未关闭

1. **动态预测尚未接入正式 Nav2 costmap**：`nav2_params.yaml` 当前 observation source 仍为 `/usv/raw_obstacle_cloud`，而跟踪器发布 `/usv/dynamic_obstacle_cloud`。因此现有 A/B 主要证明实时 LiDAR + Nav2 重规划有效，不能声称已经验证“预测点云驱动的避障”。
2. **速度估计存在离群峰值**：enabled 实验记录到 5.0 m/s，明显高于障碍物设定的 0.60 m/s，可能与目标关联跳变、船体运动补偿或时间基准有关，应增加速度门限、轨迹质量评分和异常剔除。
3. **船体安全距离模型需重新标定**：Nav2 日志提示 inflation radius 1.5 m 小于 footprint 外接半径约 2.92 m；当前“最小路径净空”不能直接等价为真实船体碰撞裕度。
4. **统计样本不足**：动态 A/B 目前只有一组 disabled/enabled 配对，需多起点、多轨迹、多速度和重复试验后才能形成稳健结论。
5. **日志字段完整性需强化**：部分早期远距离实验未锁存目标和终止状态，后续应将目标事件、Nav2 action result 与采样数据统一关联。

## 8. 当前结论与后续建议

系统已经完成从 LiDAR 输入、LIO 定位、点云过滤、Nav2 规划到 WAM-V 推力输出的可运行闭环，并通过静态、多航点、30 m 远距离和自动动态障碍物场景验证了基本导航能力及终止清零机制。下一阶段的首要工作不是继续扩大场景，而是将 `/usv/dynamic_obstacle_cloud` 明确加入 local costmap，修复速度估计离群和 footprint/inflation 标定问题，再进行至少 5 组严格匹配的动态避障重复实验。

相关数据、参数快照和日志位于 `report3/data/`、`report3/configs/`、`report3/logs/`；实验汇总见 `report3/data/dynamic_ab_comparison_20260829.md` 和 `report3/PROGRESS.md`。
