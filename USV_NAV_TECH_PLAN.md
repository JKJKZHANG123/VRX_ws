# USV 自动避障与路径规划技术方案

> 生成日期: 2026-07-18
> 基础: Point-LIO (主工作区) + VRX 仿真 (vrx_ws), 均已在 PC 上编译通过 (x86-64)
> 前置文档: `MIGRATION_NOTES.md` (Jetson → PC 迁移记录)
> 本文档取代迁移文档"三、避障方案决策"一节, 是当前生效的技术路线

---

## 当前进度快照 (更新于 2026-08-29)

**已完成并验证:**
- **M0 VRX 基线** ✅ — RTF≈1.0, 传感器话题/频率核实, 32 线雷达 (点云 ~4.9Hz, 32线渲染代价, 已知可接受)
- **M1 时间戳适配 + Point-LIO** ✅ — 直行 30m LIO vs GPS 误差 0.17m; 静止漂移 0.16m; 里程计 ~1kHz
- **M2.5 相机感知层 (usv_perception)** ✅ 单节点验证 — 浮标检测(LiDAR融合门控,零树线误检)/彩色点云/水面过滤

- **M2 EKF 融合定位** ✅ (2026-08-10) — 三项全过 (完整链路 ekf+navsat+injector, gps_pos_var=0.1): ① 静止 max EKF **0.03m** (阈值 1.5m); ② 运动直行 26m, EKF 误差 **-0.12m (0%)**, 优于裸 LIO; ③ GPS 退化兜底: 切断 GPS 后靠 LIO 速度+IMU 死推, 25s 漂 0.54m 不发散。`/odometry/filtered` 有界、无 NaN、被 GPS 正确锚定。
- **M3 水面点云预处理** ✅ (2026-08-10) — 新包 `usv_cloud_filter` (C++/PCL); 输出点 z∈[0.207,2.945] 全落在 [0.2,3.0] 带内, 水面点(z≈0)彻底剔除, 障碍点保留; `/usv/costmap_cloud` 供 Nav2 STVL 消费。
- **M4 Nav2 集成** ✅ (2026-08-10) — 新包 `usv_navigation`; bt_navigator 接受目标并导航; Smac Hybrid-A* 产出全局路径(/plan @ 1Hz); MPPI 输出 /cmd_vel @ 5Hz (linear.x=0.58m/s); STVL costmap 无鬼影; 规划层验收通过 (未接控制桥, M5 让船真正移动)。
- **M5 控制桥** ✅ (2026-08-29，基础链路) — `cmd_vel_smoothed` 已转换为左右推进器推力；30 m 目标在 VRX 中返回 `SUCCEEDED code=4`，终点误差 0.267 m；左右推进器峰值 64.226/61.936 N，目标完成后均清零。动态障碍与更复杂避障仍未验收。

**进行中:** M5（静态/远距离与异常场景补测）  **未开始:** M6（动态避障） / M7（COLREGs）

**M2 实测结果 (2026-08-10) — 发散真凶与旧诊断完全不同:**

旧诊断 (时钟跳变 / IMU 加速度) 均非主因。逐层排查出的真实根因链 (从致命到次要):
1. **LIO 差分位姿融合的纳秒 dt 爆炸 (致命)**: Point-LIO 以 ~2.7kHz 发布 `/aft_mapped_to_init`, 约 2% 的帧时间戳间隔 <0.01ms (最小 **1ns**), 且约 1% 为 **dt==0**。robot_localization 差分模式算 `速度=位姿增量/dt`, dt→0 放大 10⁹ 倍 → 位置瞬间冲到百万米 / NaN, 滤波器一旦 NaN **永不自愈**。
2. **两个绝对位姿源零协方差 (奇异)**: `/aft_mapped_to_init` 只在 `odom_only=true` 才填协方差 (VRX 走建图模式 `odom_only=false`, 恒为 0); 原始 GPS `NavSatFix` 协方差也是 0 (type UNKNOWN)。零协方差=无穷置信 → EKF 更新步矩阵奇异 → NaN。
3. **单 EKF↔navsat 冷启动反馈环 (连带)**: navsat 需 `/odometry/filtered` 锚定世界系, EKF 又需 navsat 的 `/odometry/gps`; EKF 一 NaN, navsat 就吐出 9743m 垃圾偏移 (真实位移仅 2.7m) 再灌回 EKF。EKF 修好后此症状自动消失。

**修复 (全部落在新包 `usv_localization`, 零侵入 Point-LIO/仿真):**
- 新节点 `covariance_injector` (纯中继): ① 丢弃 LIO 的 **dt≤0 非单调帧** 和非有限帧; ② 给 LIO twist + GPS fix 注入对角协方差。
  `/aft_mapped_to_init`→`/aft_mapped_to_init/cov`, `/gps/fix`→`/gps/fix_cov`。
- `ekf.yaml`: odom0 从 **差分位姿改为速度融合** (`vx,vy,vyaw`, `odom0_differential:false`) —— 与差分同意图 (用 LIO 运动而非会漂的绝对位姿) 但无 dt 除法, LIO twist 干净 (±3m/s); GPS 绝对 XY 协方差收紧到 **0.25** (0.5m std) 作唯一位置锚; IMU 仅 `vyaw`。
- 新脚本 `./ekf.sh` (= `sim.sh`/`lio.sh` 风格)。冷启动顺序: `./sim.sh` → `./lio.sh` → `./ekf.sh`。
- 验收脚本 `src/lidar_timestamp_adapter/test/verify_m2_fusion.py <秒>` (静止 max EKF 误差 <1.5m 判 PASS)。

**运动 + 退化验证 (2026-08-10 补测):**
- 运动超调调参: `gps_pos_var` 从 0.25→**0.1** (GPS 位置锚定更硬)。cov=0.25 时直行超调 13% (+2.9m); 收紧到 0.1 后, 完整链路 (ekf+navsat 都在) 直行 26m 误差降到 **-0.12m (0%)**, 静止 0.03m —— 均优于裸 LIO。
  ⚠️ 教训: 补测中曾因后台进程管理失误 (pkill 自杀/exit 144) 跑在缺 navsat 的坏链路上, 一度得到"6%/0.86m"的假 PASS (静止时纯死推船不动也会假过)。已按 PID 清干净重起完整链路复测, 上述才是真值。**验证前务必确认 3 节点齐 (ekf+navsat+injector) 且 /odometry/gps 活着。**
- GPS 退化兜底 (杀 navsat 模拟 GPS 丢失): EKF 靠 LIO 速度 + IMU yaw-rate 死推, 25s 静止漂 0.54m, 平缓有界不发散 —— 融合定位的立身之本达标。
- 绝对航向仍不融合 (gz IMU yaw 相对出生朝向), 靠 yaw-rate 积分, 超长时间可能缓漂, 后续长航时任务需关注。

**工程环境重大变更 (2026-08-05):**
- **单工作区**: 原 `vrx_ws` 已合并进主工作区 `src/vrx/` (5 个 VRX 包), 旧 `vrx_ws/` 已删除。现在一次 `colcon build --symlink-install` + 一次 `source install/setup.bash` 管理全部 17 个包。
- **根治了离线启动崩溃**: 旧 vrx_ws 环境钩子把 Gazebo 资源路径写死成迁移前旧机器路径 (`~/vrx_ws`), 离线时找不到世界文件→联网下载 Fuel 世界→崩溃。合并重编译后路径正确。
- **isolated 布局的 model:// 修复**: 合并后船体网格 (WAM-V/引擎/螺旋桨) 加载失败, 因 isolated 布局下 `model://` 跨包解析断了。已在 `sim.sh` 里把各包 `share` 父目录加进 `GZ_SIM_RESOURCE_PATH` 修复。
- **一键脚本**: `./sim.sh` (仿真, 内置 NVIDIA 独显 offload) / `./lio.sh` (Point-LIO 链路) / `./boat.sh fwd 15` (手动驾船, 自动归零推力)。

---

## 一、总体架构

```
VRX 仿真 (Gazebo Harmonic 8.11 + ROS2 Jazzy)
 │  LiDAR: 16线 gpu_ray, 10Hz, ±15°垂直FOV, 130m  → .../lidar_wamv_sensor/scan/points
 │  IMU:   .../imu_wamv_sensor/imu
 │  GPS:   .../navsat/navsat (NavSatFix)
 ▼
① 时间戳适配层 (新节点 lidar_timestamp_adapter)          ← 第一步, 链路地基
 │  为无逐点时间戳的 Gazebo 点云合成 time 字段, 输出 Velodyne 格式
 ▼
② Point-LIO (lidar_type=2 VELO16, 新配置 vrx_wamv.yaml)
 │  输出: /aft_mapped_to_init (局部里程计) + /cloud_registered (配准单帧点云)
 ▼
③ robot_localization EKF (GNSS + IMU + LIO 融合定位)     ← 防开阔水面退化
 │  navsat_transform_node + ekf_node
 │  输出: /odometry/filtered (全局有界、高频平滑)
 ▼
④ 水面点云预处理 (新节点 usv_cloud_filter)
 │  高度带裁剪 + 水面反射/水花离群点过滤
 ▼
⑤ Nav2
 │  costmap_2d: STVL 时空体素层 (输入 = 过滤后的实时点云, 非累积地图)
 │  全局规划: Smac Planner (Hybrid-A* 或 2D)
 │  局部规划: MPPI 控制器
 │  输出: /cmd_vel
 ▼
⑥ 控制桥 (新节点 usv_control_bridge)
 │  cmd_vel → PID 速度闭环 (反馈来自 EKF) → 差速推力分配
 ▼
WAM-V 推进器: wamv/thrusters/left|right/thrust (Float64)
```

### 与旧方案 (MIGRATION_NOTES 三) 的差异

| 项 | 旧方案 | 新方案 | 原因 |
|---|---|---|---|
| 定位 | 仅 Point-LIO | GNSS+IMU+LIO 三源 EKF 融合 | LIO 在开阔水面必然退化, 不能作唯一定位源 |
| 局部规划器 | TEB 或 MPPI 二选一 | 明确 MPPI | Nav2 官方主力维护; 采样型 MPC 天然适配大惯性欠驱动船体 |
| costmap 输入 | 未明确 | 明确用实时单帧 `/cloud_registered` | 累积地图会给动态障碍留"鬼影", STVL 时间衰减机制才能清除 |
| 控制桥 | cmd_vel → 推力静态映射 | 加 PID 速度闭环再分配推力 | 船动力学响应慢, 开环映射会使 MPPI 预测失效 |
| 接入 Point-LIO | "remap 话题即可" | 需时间戳适配层 | Gazebo gpu_ray 点云无逐点 time 字段, Point-LIO 直接报错无法运行 |

---

## 二、模块详细设计

### ① 时间戳适配层 `lidar_timestamp_adapter` (新包, Python 或 C++)

**问题**: Point-LIO 逐点处理, PointCloud2 必须有逐点时间字段, 缺失时报
"Failed to find match for field 'time'"。已核实 VRX 的雷达是 Gazebo `gpu_ray`
传感器 (`wamv_gazebo/urdf/components/wamv_3d_lidar.xacro`), 输出不含该字段。

**方案**: 订阅 VRX 原始点云 → 按扫描模型为每个点合成时间戳 → 重发布为
Velodyne 点格式 (`x,y,z,intensity,time,ring`), 供 Point-LIO 以 `lidar_type=2`
(VELO16) 直接消费, 不改 Point-LIO 源码。

- 合成规则: 机械式雷达一帧扫描周期 T=0.1s (10Hz), 按点的水平方位角
  `atan2(y,x)` 线性摊开: `t_point = (azimuth - azimuth_start) / 2π × T`
  (相对帧头的偏移, float 秒, 与 VELO16 handler 的 `time` 字段约定一致)
- `ring` 字段: 由垂直角 `atan2(z, sqrt(x²+y²))` 映射到 0–15 (16 线, ±15° 均分)
- 顺带做 NaN/Inf 剔除 (Gazebo 无回波点)
- 仿真船速慢 (<5 m/s), 线性方位角模型引入的去畸变误差可忽略

**验收**: Point-LIO 正常启动无 time 字段报错; `/aft_mapped_to_init` 有稳定输出。

### ② Point-LIO 新配置 `config/vrx_wamv.yaml`

从 `velody16.yaml` 复制修改, 关键项:

```yaml
common:
  lid_topic: "/wamv/lidar_points_stamped"   # 适配层输出
  imu_topic: "/wamv/sensors/imu/imu_wamv_sensor/imu"  # 以实际话题为准, 启动后 ros2 topic list 核实
preprocess:
  lidar_type: 2        # VELO16
  scan_line: 16
  timestamp_unit: 0    # 秒 (适配层按秒填)
  blind: 1.0           # 船体自遮挡半径, 视 WAM-V 结构调
mapping:
  imu_time_inte: 按实测 IMU 频率填 (1/frequency, 启动后 ros2 topic hz 核实)
  satu_acc / satu_gyro: 仿真 IMU 无饱和, 给大值即可 (如 100)
  acc_norm: 9.81       # Gazebo IMU 输出 m/s²
  extrinsic_T / extrinsic_R: LiDAR↔IMU 外参, 从 WAM-V xacro 的安装位姿推算
                             (lidar 默认 x=0.7, z=1.8; IMU 安装位姿见 wamv_gazebo 传感器 xacro)
  det_range: 130.0     # 与仿真雷达 max_range 一致
  gravity_align: true
```

**注意**: 真机 L1 配置 (`unilidar_l1.yaml`) 不动, 仿真用独立配置 + 独立 launch
(`mapping_vrx.launch.py`), 保持真机/仿真双轨并存。

**已知风险 — 开阔水面退化**: 16 线且垂直 FOV 只有 ±15°, 打到水面的几何特征
极少。验证时选特征丰富的世界 (如 `sydney_regatta` 码头场景), 并把退化程度
量化 (见第四节验收指标)。退化正是③存在的理由。

### ③ 定位融合 `robot_localization` EKF

真机侧已有 `navsat_transform_node` 使用经验 (`my_mapping_launcher`), 仿真侧复用同一套思路:

- **ekf_node** (局部, odom frame): 融合 Point-LIO 里程计 (x,y,yaw, 高权重速度) + IMU (yaw rate, 加速度)
- **navsat_transform_node**: GPS NavSatFix + EKF 姿态 → UTM 系里程计
- **ekf_node** (全局, map frame): 再融合 GPS-UTM 位置, 提供全局有界定位
- 关键参数沿用真机经验: `use_odometry_yaw: true` (信 LIO 航向不信磁力计), `zero_altitude: true`
- LIO 退化时 GPS 兜底; LIO 正常时提供 GPS 达不到的高频平滑位姿
- 输出 `/odometry/filtered` 作为 Nav2 与控制桥的唯一位姿/速度源

TF 树 (仿真侧):
```
map (UTM 对齐) → odom → wamv/base_link → 各传感器 link
```
Point-LIO 的 `camera_init`/`aft_mapped` 通过静态 TF 桥入 (复用真机 TF 桥方案)。

### ④ 水面点云预处理 `usv_cloud_filter` (新节点)

输入 `/cloud_registered` (单帧、已配准), 输出给 costmap:

1. **高度带裁剪**: 保留水面以上 z ∈ [0.2, 3.0] m (map 系, 阈值可调) —
   剔除水面反射点 (z≈0 及以下) 和高空无关点
2. **离群点过滤**: 统计/半径离群滤波去水花、浪尖噪点
   (PCL `RadiusOutlierRemoval`, 半径 0.5m 内少于 3 邻居即弃)
3. **降采样**: 0.1m 体素, 减轻 costmap 负担

这是 USV 感知最关键的一环: 不做①, 水面反射会把整个 costmap 涂成障碍。

### ⑤ Nav2 配置要点

- **costmap_2d**:
  - 全局/局部 costmap 均用 **STVL** (spatio_temporal_voxel_layer),
    观测源 = ④输出的实时点云
  - STVL `decay_model: 0 (linear)`, `voxel_decay: ~15s` — 动态障碍 (他船) 移走后自动清除
  - 不用 Point-LIO 累积地图 (`/Laser_map`) 喂 costmap — 鬼影问题
  - inflation_layer: 膨胀半径按船长放大 (WAM-V ~4.9m 长, 建议 inflation ≥ 3m,
    cost_scaling 放缓 — 船不能急转, 需要提前绕)
- **全局规划**: Smac Planner。若控制桥/MPPI 调好后船能原地差速转向, 用 2D 版;
  否则 Hybrid-A* (带最小转弯半径约束) 更贴合船的运动能力
- **局部规划: MPPI 控制器**:
  - 运动模型: DiffDrive (WAM-V 双推差速, thruster_config: H)
  - `vx_max` 按仿真实测巡航速度定 (先保守 2 m/s); `vx_min` 允许小幅负值 (倒车制动)
  - `wz_max` 保守 (0.5 rad/s 起步), 惩罚项加大 `wz` 变化率权重 — 船不能急转
  - 预测时域拉长 (`time_steps × model_dt` ≥ 3–5s) — 大惯性平台需要看得远
- **恢复行为**: 禁用 spin/back_up 默认参数直接用 — 船上原地自旋和盲退都危险,
  先只留 wait, 后续按需定制

### ⑥ 控制桥 `usv_control_bridge` (新节点)

```
输入: /cmd_vel (Nav2 MPPI 输出, 期望 vx + wz)
反馈: /odometry/filtered (EKF 实测 vx + wz)
控制: 两路 PID
  surge PID:  vx_err → 总推力 F
  yaw   PID:  wz_err → 差动推力 ΔF
分配 (差速 H 构型):
  left  = F - ΔF
  right = F + ΔF
输出: wamv/thrusters/left/thrust, wamv/thrusters/right/thrust (Float64)
```

- 推力限幅 + 变化率限幅 (推进器响应模型保护)
- cmd_vel 超时 (>0.5s 无新指令) → 推力归零 (安全停船)
- PID 参数在 VRX 里用阶跃响应实验整定 (先 surge 后 yaw), 这层调好后
  Nav2/MPPI 的调参难度下降一个数量级
- 真机替换点: 只需换掉本节点的输出端 (推进器话题 → 真机 CAN/PWM), 上层全部复用

---

## 三、实施顺序 (里程碑)

严格按序推进 — **定位不稳时调规划器是浪费时间**。

| # | 里程碑 | 内容 | 验收标准 |
|---|---|---|---|
| M0 | VRX 基线 | PC 上跑 `sydney_regatta`/`stationkeeping_task` 完整 GUI; 记录内存/帧率/各话题频率 | 点云稳定 10Hz (Jetson 上仅 2Hz); 实际话题名清单落盘 |
| M1 | 时间戳适配 + Point-LIO | ①②: 适配层节点 + `vrx_wamv.yaml` + `mapping_vrx.launch.py` | Point-LIO 无报错运行; 手动开船绕码头一圈, 轨迹回到起点附近 (目测闭环) |
| M2 | EKF 融合定位 | ③: ekf + navsat_transform 配置 | `/odometry/filtered` 稳定; 人为遮蔽 GPS 或驶入开阔水面, 定位不发散 |
| M2.5 | 相机语义感知 | 新包 `usv_perception`: 浮标语义识别 + 彩色点云 + 水面过滤 (三方向, 见第二点五节) | ✅ 已完成: 浮标检出带 3D 定位且零树线误检; 彩色点云/障碍点云正常输出 |
| M3 | 点云预处理 | ④: usv_cloud_filter (与 M2.5 方向3 合并/复用) | ✅ 已完成: 输出点 z∈[0.207,2.945] 全落在 [0.2,3.0] 带内, 水面点(z≈0)彻底剔除, 障碍点保留 |
| M4 | Nav2 集成 | ⑤: costmap + Smac + MPPI 配置 (先用理想速度接口验证规划层) | ✅ 已完成: bt_navigator 接受目标; Smac Hybrid-A* 出全局路径 (/plan @ 1Hz); MPPI 输出 /cmd_vel @ 5Hz (linear.x=0.58m/s); costmap 无鬼影 |
| M5 | 控制桥 | ⑥: PID 闭环 + 推力分配; 接通完整链路 | 30 m 静态目标已到达且误差 < 5m；动态障碍绕行仍待验证 |
| M6 | 动态避障验证 | 仿真中加移动障碍船 | STVL 正确衰减旧障碍; 对迎面/横穿目标能提前绕行 |
| M7 | (可选) COLREGs | 避碰规则层 (右让、追越等) | 仿真验证阶段之后再做, 本轮不承诺 |

其后: 迁回真机 (方案B分布式: PC 跑仿真 → Jetson 跑算法, 或直接真机), 只需
替换传感器话题与控制桥输出端。

---

## 四、关键验收指标

- **点云频率**: ≥ 10Hz (M0)
- **LIO 质量**: 码头场景绕行 200m 回起点, 端点漂移 < 2m (M1);
  开阔水面记录退化表现 (协方差/漂移速率), 作为 M2 的对照
- **融合定位**: GPS 单点精度约束下全局误差有界; LIO 可用时输出平滑无跳变 (M2)
- **避障**: 静态障碍 100% 绕行不碰撞; 动态障碍在相对速度 ≤ 3m/s 下不碰撞 (M5/M6)
- **实时性**: 全链路 (Gazebo+LIO+EKF+Nav2) 在本机 (16G RAM / RTX 5070 8G) 内存 < 12G, Gazebo RTF > 0.8

---

## 四点五、M0 实测结果 (2026-07-19, 本机 PC)

**结论: M0 通过。** GUI 完整仿真 RTF≈1.0, 内存充裕, 传感器话题全部核实。

### 实际话题名 (以此为准, 与迁移文档记录的不同)

| 传感器 | 实际话题 | 实测频率 | frame_id |
|---|---|---|---|
| LiDAR 点云 | `/wamv/sensors/lidars/lidar_wamv_sensor/points` | ~7.2–8.6Hz (标称10Hz, 有抖动, 帧间隔偶达 0.6–1.3s) | `wamv/wamv/base_link/lidar_wamv_sensor` |
| IMU | `/wamv/sensors/imu/imu/data` | ~96Hz (标称100Hz) → `imu_time_inte: 0.01` | `wamv/wamv/imu_wamv_link/imu_wamv_sensor` |
| GPS | `/wamv/sensors/gps/gps/fix` | ~19Hz (标称20Hz) | — |
| 左/右推力 | `/wamv/thrusters/left\|right/thrust` (Float64) | 订阅端 | — |
| 转向 | `/wamv/thrusters/left\|right/pos` | 订阅端 | — |

### 点云格式 (逐字段核实)

- 尺寸: 1875×16 = 30000 点/帧, 0.96MB/帧, ~6.4MB/s
- 字段: `x,y,z,intensity`(float32) + **`ring`(uint16, offset 24) 已存在**
- **无 `time` 字段 — 确认适配层必要性**; 但 ring 已有, 适配层只需**追加 time 字段**,
  比原计划(重算 ring)更简单

### 资源占用 (sydney_regatta, 完整 GUI)

- RTF: **1.0** (Jetson 上点云仅 2Hz 的问题在 PC 上消失)
- 内存: 系统总占用 ~9.3G/15G (仿真本体约 3.5G), 余量足够再跑 LIO+EKF+Nav2
- GPU: NVIDIA 显存 ~3.0G/8G, 利用率 ~20%, 余量大

### 两个必须记住的运行事项

1. **必须用 PRIME offload 启动, 否则渲染落在核显上, 点云掉到 ~3.4Hz**:
   ```bash
   __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
     ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ```
   (建议后续写进 usv_bringup 的 launch 或 shell 包装)
2. **开发用 `sydney_regatta` 世界**: `stationkeeping_task` 是竞赛任务世界,
   任务超时后会自动 SIGINT 整个仿真 (M0 中实测发生); sydney_regatta 无任务
   计时, 且码头/岸线几何特征丰富, 适合 LIO 验证。

### 对后续里程碑的修正

- ~~适配层可能完全不需要~~ **M1 实测: 适配层必要, 但职责变了** (见 M1 实测一节):
  velodyne_handler 的无时间戳回退分支确实工作, 但 Gazebo 对无回波射线输出
  **inf 坐标点** (开阔水面下占比 60–80%), Point-LIO 无有限性检查, 直接吞掉后
  里程计完全无输出。已实现 `lidar_timestamp_adapter/cloud_filter` 节点:
  仅做非有限点过滤 + 直通其余字段, 时间戳合成仍交给 Point-LIO 回退分支
- `vrx_wamv.yaml` (M1): `imu_time_inte: 0.01`; `scan_line: 16`; SCAN_RATE 10;
  话题名按上表填
- 点云频率标称 10Hz 但实测均值 ~7–8.6Hz 且有抖动 — Point-LIO 按点时间处理,
  帧率抖动本身无害

## 四点六、M1 实测结果 (2026-07-19)

**状态: 通过。** VRX → 适配层 → Point-LIO 链路打通, 静止/直行/原地掉头/变向直行全程稳定。

### 交付物

- 新包 `src/lidar_timestamp_adapter` (Python, `cloud_filter` 节点):
  `/wamv/sensors/lidars/lidar_wamv_sensor/points` → `/wamv/points_filtered`
  1. **剔除非有限点**: Gazebo 对无回波射线输出 inf 坐标 (开阔水向占比 60–80%,
     且 is_dense 错标为 true), Point-LIO 无有限性检查
  2. **追加逐点 `time` 字段** (float32 秒): 利用输入云的 organized 结构
     (16×1875, 列号=方位角步), `t = col/width × 0.1s`
- `point_lio_ros2/config/vrx_wamv.yaml` (`lidar_type: 2`, `timestamp_unit: 0`,
  `scan_line: 16`, `imu_time_inte: 0.01`, `det_range: 130`, `blind: 2.0`,
  extrinsic_T [0.4, 0.2, 0.5] 由 xacro 推得) + `launch/mapping_vrx.launch.py`
  (含 cloud_filter + use_sim_time)

### 关键调试结论 (为什么两个字段都必须补)

最初只滤 inf 不补 time, 结果**不稳定复现**三种表现: 正常运行 / 静默卡死
(回调收数据但 sync_packages 永不放行) / `cos_sinc_sqrt` assert 崩溃。
根因: PCL `fromROSMsg` 对消息里不存在的字段**不清零** — velodyne 点结构的
`time` 成员是未初始化内存。`given_offset_time` 检测读到垃圾值: 垃圾 ≤0 时
走方位角回退 (碰巧正常); 垃圾 >0 时 `lidar_end_time` 被推到未来几百秒,
sync_packages 死等 IMU 追上 (卡死), 或畸变矫正吃到坏时间差 (崩溃)。
**之前记录的"运动中初始化崩溃"实为此内存问题, 与运动/退化无关。**
适配层显式写入 time 字段后 100% 稳定。

### 实测数据 (sydney_regatta, 复查后以第二轮为准)

第一轮 (初测):

| 场景 | 结果 |
|---|---|
| 静止 60s | 位姿抖动 < 0.05m |
| 全推力直行 35s (~65m) | LIO 61.6m vs GPS 64.7m, 误差 4.9% |
| 原地掉头 + 变向直行 | 无发散, 轨迹连续 |

第二轮 (2026-07-19 复查, 冷启动全链路重跑, 脚本存
`src/lidar_timestamp_adapter/test/verify_lio_motion.py`):

| 场景 | 结果 |
|---|---|
| 静止 20s | XY 漂移 0.21m (含波浪真实运动), 里程计 ~818Hz |
| 直行 30s (~55m) | **LIO 55.43m vs GPS 55.48m, 误差 0.05m (0.1%)** ✅ |
| 直行 30s Z 漂移 | 0.82m (较第一轮 5m 大幅收敛) |
| 掉头 + 变向直行 | 里程计存活, 轨迹连续 ✅ |
| 全程日志 | 0 错误 / 0 assert / 0 time 字段告警 ✅ |

### 复查时验证过的适配层假设 (脚本存 test/ 目录)

- `verify_layout.py`: 原始点云确为 organized 16×1875 行主序 —
  ring==行号 100% 匹配, 列号与方位角相关系数 1.000 → time 合成公式成立
- `verify_output.py`: 输出云 6 字段齐全, time ∈ [0, 0.0999]s, 0 个非有限点
- `timestamp_unit: 0` (SEC) 对应 `time_unit_scale=1e3` (preprocess.cpp:49),
  curvature 得 ms, 与 sync_packages 的 `/1000` 还原一致 — 单位链路核实无误

### 已知问题 (记入 M2 输入)

- 两轮位移误差差异大 (4.9% vs 0.1%): 第一轮驱动路径朝开阔水向, 特征更少;
  说明退化敏感度与航向强相关, M2 GPS 融合仍必要
- Z 向仍有缓漂 (~0.8m/55m), M2 沿用真机经验 `zero_altitude: true` 压平
- 本机内存: 仿真 + LIO 全链路后 available ~5.4G, 后续叠 Nav2 尚可, 但 RViz
  开启时需关注

### M1 后追加: 点云密度升级 (2026-07-19, 应可视化需求)

1. **雷达升级 32 线** (`wamv_gazebo.urdf.xacro` 两处 `16_beam`→`32_beam`):
   32 线 × 2187 采样 = 69984 点/帧 (原 30000), 垂直 FOV -10.7°..+31°
   (32 线预设上仰更多, 对岸上结构覆盖更好)。`vrx_wamv.yaml` 的 `scan_line`
   同步改 32。适配层按 width 动态计算, 无需改码, 已验证 (ring 0..31 匹配)
2. **Point-LIO 密度参数** (`mapping_vrx.launch.py`):
   `point_filter_num` 4→1, `filter_size_surf/map` 0.5→0.2
3. **回归测试全通过**: 直行 55.7m 误差 0.02m (0.0%), 静止漂移 0.14m,
   掉头存活; RTF 仍 1.0, 内存/GPU 余量充足; 过滤后有效点 ~23000/帧 (原 ~11500)
4. 注意: `wamv_gazebo` 在 vrx_ws 是 **merged 布局**, 单包重编译命令:
   `colcon build --merge-install --packages-select wamv_gazebo --symlink-install`;
   首次重编译需先删除 `build/wamv_gazebo` (残留 Jetson 路径的 CMake 缓存)

---

## 四点七、M2.5 相机语义感知实测结果 (2026-08-05)

**状态: 三个方向全部构建完成并对仿真实测通过。** 新包 `src/usv_perception`
(Python), 纯经典视觉 (OpenCV + numpy + scipy + PCL), **零下载、完全离线**。
不动真机的 `lidar_camera_fusion` 包, 真机/仿真双轨并存。

### 设计前提 (相机当"航道语义传感器", 不用深度通道)

- VRX 当前 WAM-V **只有 RGB 相机, 无深度相机** (sensor type=camera, 1280x720,
  水平 FOV 80°, 30Hz)。刻意用 RGB 语义, 回避深度相机在水面 (反光/无纹理) 和
  户外强光下的物理短板 + 仿真-真机 gap
- 测距/3D 一律交给激光雷达, 相机只提供颜色/语义
- VRX 提供 `.../optical/` 版相机话题, 自带标准光学系 TF (x右y下z前),
  外参直接从 tf2 取, 省掉真机版那套手工 90° 基准旋转标定

### 方向1: 浮标语义识别 + 3D 定位 (`buoy_detector.py`) ✅ 已验证

- HSV 颜色分割检测红/绿/黑/白/橙浮标 (海事航道 marker, COLREGs 语义输入)
- **关键调试结论**: 纯 2D HSV **误检严重** — 把远岸整条树线误判成 13-18 个
  "green" 浮标 (有标注图为证)。根因: 颜色空间里树叶和绿浮标重叠, 纯 2D 无法区分
- **修复 = LiDAR 融合门控**: 把雷达点投影到图像 (相机内参 K + tf2 外参),
  只保留"框内有雷达点支撑 + 水平距离 ∈ [1, 80]m + 高度 ∈ [-2.5, 2.0]m (LiDAR系)"
  的颜色块。树线超量程/超高度 → 自动滤除; 顺带用框内点算出浮标 3D 位置
- 修复后实测: **稳定检出 5 个真浮标 (红/绿/黑/橙), 各带 3D 位置 + 距离
  (24-28m), 树线零误检** (标注图二次确认)
- 输出: `/usv/buoy/image` (标注图) + `/usv/buoy/detections` (JSON, 类别+框+3D位置+距离)
  + `/usv/buoy/markers` (RViz MarkerArray)

### 方向2: LiDAR-相机彩色点云 (`cloud_colorizer.py`) ✅ 核心可用

- 基于真机版 `pointcloud_colorizer.py` 改造; 因 optical 系 TF, 砍掉手工基准旋转
- 逐帧着色正常 (`/usv/colored_cloud`); 累积彩色地图 (`/usv/colored_map`,
  TRANSIENT_LOCAL + 体素哈希) 需 Point-LIO 里程计 `/aft_mapped_to_init` 在跑
- **物理限制**: 只有相机 80° 视锥内的雷达点能着色 (雷达 360°), 逐帧着色点数少属正常

### 方向3: 水面点云过滤 (`water_filter.py`) ✅ 可用

- 高度带裁剪 (剔水面反射) + scipy cKDTree 半径离群滤波 (去水花) + 体素降采样
- 实测 69984 点 → 高度带 ~9000 → 输出 ~5000 障碍点 (`/usv/obstacle_cloud`)
- 与 ④ `usv_cloud_filter` (M3) 功能重叠, M3 可直接采用本节点或在此基础上扩展

### 启动与已知事项

- 启动: `ros2 launch usv_perception perception.launch.py` (需仿真在跑)
- HSV 阈值按 VRX 当前光照整定, 换世界/换光照需微调 (经典颜色分割固有特性)
- 今日为感知节点**单独**实测; 与定位链路 (Point-LIO / M2 融合) 的联调待做,
  方向2 累积地图的完整效果依赖定位链路

---

## 五、风险与预案

| 风险 | 影响 | 预案 |
|---|---|---|
| Point-LIO 开阔水面退化 | 里程计漂移/发散 | M2 的 EKF 融合本身就是预案; 极端时可降级为纯 GNSS+IMU 定位, LiDAR 只做感知 |
| 16线±15°FOV 特征过少, LIO 在仿真中不可用 | M1 受阻 | 改雷达 xacro 参数加密线束 (vertical_lasers/samples 可调); 或接受降级路线 (LiDAR 仅感知) |
| Gazebo 水动力模型与真机差异 | 控制桥 PID 参数不可迁移 | 架构上 PID 参数独立成 yaml, 真机重新整定即可, 结构不变 |
| MPPI 调参困难 | M5 延期 | 先用 Nav2 默认 DiffDrive 参数起步; 控制桥闭环质量优先于 MPPI 精调 |
| STVL 依赖未装 | M4 受阻 | `ros-jazzy-spatio-temporal-voxel-layer`; 若 Jazzy 源不可用则退回 voxel_layer + 短 observation_persistence |

---

## 六、新增代码清单 (全部纯新增, 零侵入现有包)

| 包/文件 | 类型 | 里程碑 |
|---|---|---|
| `src/lidar_timestamp_adapter/` | 新 ROS2 包 (建议 C++, 点云吞吐大) | M1 |
| `point_lio_ros2/config/vrx_wamv.yaml` | 新配置 | M1 |
| `point_lio_ros2/launch/mapping_vrx.launch.py` | 新 launch | M1 |
| `src/usv_localization/` (ekf/navsat 配置 + launch) | 新包 (纯配置) | M2 |
| `src/usv_perception/` (buoy_detector + cloud_colorizer + water_filter + config/launch) | 新 ROS2 包 (Python, 纯离线 OpenCV/scipy) | M2.5 ✅ |
| `src/usv_cloud_filter/` | 新 ROS2 包 (C++/PCL) | M3 ✅ |
| `src/usv_navigation/` (Nav2 参数 + launch) | 新包 (纯配置) | M4 ✅ |
| `src/usv_control_bridge/` | 新 ROS2 包 | M5 |
| `src/usv_bringup/` (全链路一键 launch, 仿 all_in_one 风格) | 新包 | M5 |

---

## 当前实施快照（2026-08-28，动态避障第一版）

在静态路径安全阶段基础上，已加入 `dynamic_obstacle_tracker`：

```text
/usv/raw_obstacle_cloud
  -> camera_init 坐标变换
  -> /usv/structure_map_cloud 静态结构剔除
  -> XY 8邻域栅格聚类
  -> 最近邻关联
  -> alpha-beta 匀速跟踪
  -> 8 s 预测
  -> /usv/dynamic_obstacle_cloud (base_link)
  -> local_costmap marking-only
```

当前参数是保守的 VRX 初始值：聚类最少 3 点、匹配门限 4m、超时 1.5s、
速度阈值 0.15m/s、预测 8s。raw LiDAR 仍负责实时 marking+clearing，SafetyCloud
急停路径未被替换。该版本已通过语法、3 项纯算法测试、YAML/启动解析和
`colcon build --packages-select usv_navigation usv_cloud_filter`，但还没有在
VRX 中运行移动障碍物 A/B 实验，因此不能把动态预测效果称为已验证。

下一步实验必须固定世界、起点、目标、船速和移动障碍轨迹，分别关闭/开启
`dynamic_obstacle_tracker`，导出 `nav_log.csv`，比较 `safety_stop`、最小间距、
路径长度、横向误差、目标结果、`dynamic_track_count`、
`dynamic_point_count` 和 `dynamic_max_speed`。通过后再增加 CPA/TCPA 或 COLREGs；
不要在第一版尚未验证时直接切换 MPPI。

### 2026-08-29 远距离 M5 补测

在关闭动态跟踪器的完整链路上执行了 30 m 单目标验证。原始 CSV、事件、派生
指标、参数快照和节点日志均保存在 `report3/`；Nav2 成功且控制桥左右推力
均有非零输出，终止后归零。下一步仍是固定条件的 tracker-disabled/enabled
动态障碍 A/B，不把本次静态结果解读为动态避障通过。

## 2026-08-30 静态/动态避障实验交付状态

- **静态障碍物路径对比**：实验运行器和离线报告链路已完成。固定障碍物中心为
  `camera_init (5,0)`，目标为 `(10,0)`；`dynamic_obstacle_scenario.py --static`
  只生成一次障碍物并记录 `static_ready`，随后由 recorder 保存 Nav2 规划路径和
  实际 USV 轨迹。正式有效记录仍要求 `SUCCEEDED code=4`、规划快照有效、轨迹绕开
  障碍物，历史 `verified3/final` 不计入结论。
- **动态障碍物避障对比**：tracker-disabled/enabled A/B 已有有效历史数据，比较器
  兼容旧版 logger 单 CSV，并输出四面板图和统一指标。enabled 只能在动态轨迹数量、
  预测点以及 `/usv/dynamic_obstacle_cloud` costmap 输入均有效时解释为预测驱动避障；
  tracker 速度峰值仅作观测诊断。
- **当前限制**：受限执行环境的 DDS/Gazebo socket 权限会阻止 Point-LIO 启动，新的
  静态正式运行需在具备 Gazebo/ROS 2 网络权限的环境执行。
