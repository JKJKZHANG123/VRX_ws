# 项目迁移记录 (Jetson Orin Nano → PC)

> 生成日期: 2026-07-17
> 用途: 记录本次会话的全部工作、决策与待办, 便于在 PC 上继续 (尤其是 VRX 仿真迁移)

---

## 一、硬件与系统环境

### Jetson Orin Nano (当前主机)
- 型号: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit **Super**
- 系统: Ubuntu 24.04.4 LTS, **X11** + GNOME
- ROS: **Jazzy**
- 内存: **7.4G** (关键瓶颈, 见 VRX 部分)
- 功耗档: MAXN_SUPER
- 无 NVENC 硬件编码器 (只有 NVDEC 解码), Orin Nano 官方确认无硬编
- IP: 192.168.1.26 (WiFi wlP1p1s0)
- 串口: `/dev/ttyUSB0` = 激光雷达 (CP2102), 数传模块用 USB 转串口枚举为 ttyUSB1

### 传感器 (真机)
- LiDAR: Unitree Unilidar L1 (10Hz+, `/unilidar/cloud` + `/unilidar/imu`)
- 相机: Orbbec Astra Pro Plus (深度相机, USB 视频接口, 非串口)
- GPS: NMEA 串口 (已从主流程剥离)
- 数传模块: USB 转串口

---

## 二、主项目 `~/Desktop/lidar_project`

### 核心: Point-LIO (LiDAR-惯性里程计)
- 里程计话题: **`/aft_mapped_to_init`** (nav_msgs/Odometry) — 注意不是 `/Odometry`
- 全局点云: `/cloud_registered` (camera_init 坐标系)
- 累积地图: `/Laser_map`
- PCD 保存: `unilidar_l1.yaml` 里 `pcd_save_en: true`, **只在 laserMapping 收到 SIGINT 优雅退出时写盘** (`laserMapping.cpp:1312`), 运行时 `ros2 param set` 无效

### 本次会话新增的功能包 (纯新增, 零侵入现有代码)

1. **`lidar_camera_fusion`** — 点云 RGB 着色
   - `pointcloud_colorizer.py`: LiDAR 点云投影到相机图像取色
   - 逐帧输出 `/colored_cloud` (雷达系) + 累积输出 `/colored_map` (全局系 camera_init)
   - 累积用 `/aft_mapped_to_init` 位姿变换 + 体素降采样(0.05m) + TRANSIENT_LOCAL QoS
   - **内置 LiDAR→相机光学系 90° 基准旋转** (解决坐标轴约定不同导致的颜色错乱)
   - `roll/pitch/yaw` 参数现在是"度", 且是在基准旋转上的微调
   - **外参标定未完成**: 平移/旋转微调还是占位值, 颜色会偏。标定方式: rqt_reconfigure 实时拖滑块 (参数已加 range 描述符 + 动态回调, 改参数会自动清空累积地图重建)

2. **`visual_loop_closure`** — 视觉回环检测
   - `visual_slam_node.py` + `pose_graph.py` (SE(3) Gauss-Newton + LM阻尼) + `keyframe_database.py` (ORB特征 + 汉明匹配 + Lowe ratio)
   - 回环约束用 identity (检测到"同一地点"则两帧位姿应重合)
   - 话题: `/optimized_path`, `/loop_status`, `/loop_markers`

### 展示脚本 `demo.sh` (项目根目录)
三个场景:
- 场景1: Point-LIO 无GPS → `mapping_no_gps.launch.py` (RViz: demo_point_lio.rviz)
- 场景2: RGB上色无回环 → `with_camera.launch.py loop_closure:=false` (RViz: demo_rgb_color.rviz, RGB8 预设)
- 场景3: 合并展示 (白色+彩色同屏) → `with_camera.launch.py loop_closure:=false rviz_config:=demo_combined.rviz`

### 相机工作区迁移 (已完成)
- 原 `camera_ws` 嵌套独立工作区 → `OrbbecSDK_ROS2` 移至主 `src/`, 现为主工作区正式成员
- 三个包: `orbbec_camera` / `orbbec_camera_msgs` / `orbbec_description` (自带 arm64 SDK)
- `depth_reader.py` → `src/camera_tools/`
- 现在只需 `source install/setup.bash` 即可用相机

### 数传通信 `src/Tools/serial_agent.py`
- 原 `serial_comm.py` 保留未动
- 智能版: PC 发命令 → Jetson 执行 ROS2 操作 + 主动推送状态
- 命令: start1/2/3, stop(SIGINT优雅停+保存PCD), status, pose, mapsize, savepcd, help
- **点云无法通过数传传** (带宽 11KB/s vs 点云 8MB/s, 差700倍), 只能传位姿/状态文本
- ⚠️ 无鉴权, 演示可用, 公共环境有风险

---

## 三、避障方案决策 (本次会话讨论结论)

### 平台: 无人船 (USV)
- 水面平面运动 → **规划层用 2D** (确定)
- 感知层可用 3D 建图后投影降维成 2D costmap

### 推荐算法栈
```
Point-LIO (位姿+点云)
  → ① 点云预处理 (水面反射/水花噪声过滤 + 高度裁剪) ← USV 最关键
  → ② Nav2 costmap_2d (推荐 STVL 时空体素层)
  → ③ 局部规划器: TEB 或 MPPI (不用DWA — USV惯性大/欠驱动)
  → ④ 控制桥接 (Nav2速度 → 推进器/CAN)
```
- 框架: **Nav2**
- USV 特有坑: Point-LIO 开阔水面会退化(缺几何特征)、动态障碍物、COLREGs避碰规则

### 待确认
- 驱动方式 (差速/欠驱动/全向) — 影响 Nav2 运动学配置和控制桥

---

## 四、VRX 仿真 `~/vrx_ws` (关键: 需迁移到 PC)

### 现状
- VRX (Gazebo Harmonic 8.11 + ROS2 Jazzy), 已编译安装成功
- 12 个任务世界齐全
- src 里还有 **LIO-SAM** (未编译), 决定: 仿真用 **Point-LIO** 保持与真机一致, 不用 LIO-SAM

### ⚠️ 实测结论: Jetson 跑完整 VRX 内存不够 (这是迁移到 PC 的核心原因)
带 GUI 完整仿真实测:
| 指标 | 实测 | 评价 |
|---|---|---|
| 内存 | 6.9G/7.4G, 剩336MB, SWAP已用1.8G | 🔴 几乎耗尽 |
| LiDAR点云 | ~2Hz (真机10Hz+) | 🟡 偏低 |
| CPU | 6核~70% | 🟡 |
| GPU | 64% | 🟢 有余量 |

- **瓶颈是内存 (8G板子硬限制), 不是算力**
- 再叠加 Point-LIO + Nav2 几乎必然 OOM 崩溃
- headless 模式 (`headless:=true`) 可省2-3G, 但仍紧张

### 在 PC 上继续的建议
1. **PC 需要独立 NVIDIA 显卡** (Gazebo 3D 渲染 + 水面仿真吃 GPU/显存)
2. PC 内存充足 → 可开 GUI 完整仿真, 无 Jetson 的内存瓶颈
3. 迁移方式二选一:
   - **方案A (全在PC)**: VRX + Point-LIO + Nav2 全部在 PC 上跑, 最简单, 适合算法验证
   - **方案B (分布式)**: PC 跑 VRX 仿真, Jetson 跑 Point-LIO+Nav2, 通过 ROS2 DDS 网络通信, 真实测 Jetson 算力
4. VRX 关键传感器话题 (已验证正常发布):
   - LiDAR点云: `.../lidar_wamv_sensor/scan/points` (PointCloud2)
   - IMU: `.../imu_wamv_sensor/imu`
   - GPS: `.../navsat/navsat` (NavSatFix)
   - 相机: `.../front_left_camera_sensor/image` 等
   - 推进器控制: `wamv/thrusters/left|right/thrust` (Float64), `.../pos` (转向)
   - 默认船: WAM-V, 差速双推进器 (thruster_config: H)

### VRX 启动命令
```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
# 完整(PC): ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task
# headless: ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task headless:=true
```

### VRX 待办 (迁移到 PC 后)
1. [ ] PC 上确认 VRX 完整 GUI 能流畅跑 (帧率/内存)
2. [ ] 重新测 LiDAR 点云频率 (Jetson上仅2Hz, PC应更高)
3. [ ] 接入 Point-LIO (用 VRX 的 LiDAR+IMU 话题, 需 remap 到 Point-LIO 期望的话题名)
4. [ ] 确认 Point-LIO 里程计在仿真水面正常 (注意开阔水面退化问题)
5. [ ] 上 Nav2: costmap + TEB 局部规划器
6. [ ] 写水面点云预处理节点
7. [ ] 写 Nav2速度 → WAM-V推进器 控制桥接节点
8. [ ] 确认真机驱动方式, 配 Nav2 运动学

---

## 五、其他已完成的杂项
- NoMachine 9.8.2 远程桌面已装 (端口4000, 局域网直连比Todesk好, NX协议对RViz 3D支持好)
  - ⚠️ 拔HDMI后远程无画面 → 需HDMI假负载(dummy plug)或配虚拟显示器
  - 跨网可用 Tailscale 组网
- 关闭了 iPhone-hotspot 开机自动连接 (`nmcli connection modify iPhone-hotspot connection.autoconnect no`), 根因是它 autoconnect-priority=100 最高, 非脚本
