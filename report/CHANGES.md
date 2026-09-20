# 当前改动说明

## 1. VRX 点击平面与 LiDAR 可见性

文件：`src/vrx/vrx_gz/worlds/sydney_regatta.sdf`

为了让 Gazebo 空水面可以点击，增加了仅供 GUI 拾取的 `pick_plane`。它没有碰撞和物理作用，并设置 `visibility_flags=8`；WAM-V GPU LiDAR 使用掩码 7，因此该平面不会进入激光点云，避免形成一整块假水面。

## 2. Point-LIO 输入预处理

文件：`src/lidar_timestamp_adapter/lidar_timestamp_adapter/cloud_filter_node.py`

- 删除 `NaN/Inf` 无回波点；
- 按组织点云列号补充 0～0.1 s 的 `time` 字段；
- 使用 `sensor_z_min=-1.5 m` 删除 LiDAR 坐标系中主要位于约 `-1.8 m` 的水面回波，同时保留较低的浮标回波。

对应启动参数在 `src/point_lio_ros2/launch/mapping_vrx.launch.py`，Point-LIO 使用 VRX IMU 传播、`point_filter_num=2`、表面/地图体素尺寸 `0.3 m`、LiDAR 协方差 `0.01`。

## 3. 障碍物点云坐标与链路拆分

文件：`src/usv_cloud_filter/src/obstacle_filter.cpp`、`config/cloud_filter_params.yaml`

点云先从 `camera_init` 转换到 `wamv/wamv/base_link`，再进行高度和距离过滤。当前主要参数为：

```yaml
raw_z_min: 0.10
structure_z_min: 0.10
z_min: 0.10
max_range: 35.0
voxel_size: 0.1
outlier_radius: 1.0
outlier_min_neighbors: 1
```

输出被拆成：

- `/usv/structure_cloud`：Point-LIO 累积结构云；
- `/usv/raw_obstacle_cloud`：原始 LiDAR 的局部障碍云；
- `/usv/safety_cloud`：实时安全监测云。

在船体坐标系中水面约为 `z=0`，因此 `z<0.10 m` 的水面和水花被去除；浮标、岸线等高于水面的结构仍可保留。

## 4. Nav2 与控制链路

文件：`src/usv_navigation/config/nav2_params.yaml`、`launch/navigation.launch.py`

- 局部/全局 costmap 使用滚动窗口；
- 障碍物最大观测距离 35 m，清除射线最大距离 40 m；
- 高度范围放宽到 `-1～5 m`，具体水面裁剪由前级船体坐标过滤完成；
- 膨胀半径调整为 5 m，给岸线和浮标留下安全裕度；
- 禁止 `feature_nav_node` 与 Nav2 同时发布速度指令，Nav2 作为唯一控制来源，避免原地转圈和错误直冲岸线。

## 5. 数据记录

`src/lidar_timestamp_adapter/test/measure_stationary_lio.py` 记录静止漂移；`src/usv_navigation/usv_navigation/nav_data_logger.py` 记录目标、规划路径、速度、横向误差、障碍点数、各阶段点云数和安全停止状态。当前历史导航日志已复制为 `data/historical_nav_log.csv`，但它不是本次 LIO 静止 A/B 主对比数据。

本次没有引入新的 SLAM 或路径规划算法，主要是点云预处理、坐标系修正、传感器链路拆分和参数调优。

## 6. SafetyCloud / Nav2 Transform 启动屏障（2026-08-27）

本次运行日志表明，Nav2 激活后没有持续的 Transform 错误；唯一的异常发生在 `obstacle_filter` 启动后的首个原始 LiDAR 数据包：其 TF listener 尚未收到 `camera_init -> aft_mapped -> wamv/wamv/base_link`，因此首帧被安全丢弃。随后 `/usv/safety_cloud` 持续约 3 Hz 发布，消息坐标系为 `wamv/wamv/base_link`，Nav2 local/global costmap 均为 `active`，且 costmap 正常发布。

为消除该启动竞态，修改根目录 `usv.sh`：

1. 在启动 `obstacle_filter` 前等待完整 TF 链可查询；
2. 启动过滤器后不再使用固定 `sleep 3` 判断就绪，而是等待 `/usv/safety_cloud` 首帧；
3. 首帧未到达时不启动 Nav2，避免在安全点云或 TF 尚未准备好时激活 costmap；
4. 清理 Point-LIO RViz TF 面板中过期的未命名 WAM-V frame 条目，保留实际使用的 `camera_init`、`aft_mapped` 和 `wamv/wamv/base_link`。

本次没有加入新的导航/定位算法，也没有产生新的算法性能数据；这是启动时序与可视化配置修复。验证重点为：SafetyCloud 首帧存在、`frame_id` 正确、`camera_init -> wamv/wamv/base_link` 可查询，以及 Nav2 costmap lifecycle 为 `active`。
## 7. SafetyCloud/Nav2 运行核验与数据更新（2026-08-27）

针对“RViz 已无报错但日志看起来等待较久”的疑问，本次直接对现有启动日志做了快速统计，没有继续长时间等待，也没有发送导航目标。结果保存在 `report/data/safetycloud_nav2_runtime_validation_20260827.csv`：

- `nav2.log` 中 Transform 等待/`Invalid frame ID` 各 5 条，集中在 Nav2 激活初期约 2 秒；随后日志显示 `Managed nodes are active`，因此这是启动 TF 缓存竞态，不是持续故障。
- `cloud_filter.log` 只有 1 条首帧 TF skip，之后连续 66 帧 SafetyCloud 正常输出；点数中位数 3548，范围 3350～3742，输出 frame 为 `wamv/wamv/base_link`。
- 过滤器有效输出统计 65 帧，最终点数中位数 427，说明水面过滤没有把链路删空。
- Nav2 日志另有 1 条 Smac 膨胀层建议和 1 条 BT 参数警告；它们不是 SafetyCloud/Transform 报错。本次不改规划算法，避免把已验证的版本再次扰动。

因此，现有数据相较未改进基线更适合 MATLAB 对比：既有 Point-LIO 静止 A/B 数据，也新增了 SafetyCloud/TF/Costmap 启动运行证据。


## 8. 最优候选复核（2026-08-27）

为避免把“当前参考版本”误称为“所有历史数据中的绝对最优”，新增：

- `report/data/improved_lio_stationary_after_zupt_candidate_20260827.csv`：从已有 `data/lio_stationary_after_zupt_20260827.csv` 纳入报告；
- `report/data/lio_candidate_comparison_20260827.csv`：按统一公式比较基线、漂移修复中间版本、ZUPT 候选和当前最终参考版本。

结论是：当前 `final_optimized` 在 P95 漂移上略优，且已配套完成 SafetyCloud/Nav2 验证；已有 ZUPT 候选在末值漂移、最大漂移、相对轨迹 RMSE 和 Z 变化上更优，但没有在本次完整 VRX 启动链路中重新验证。因此本次不直接改启动配置，只把它标记为下一轮复测候选。

## 9. EKF 静止/运行对比实验（2026-08-28）

本轮没有改动已验证的 Nav2 控制和 TF 主链，只启动了一个被动的 `robot_localization` 平面 EKF 用于测量。输入包括带协方差的 Point-LIO 里程计、GPS/navsat 和 IMU；`two_d_mode=true`，外部 EKF 设置 `publish_tf=false`，所以不会改变当前 Nav2 使用的 Point-LIO 定位。

记录器 `experiments/record_ekf_ab.py` 使用 BEST_EFFORT/KEEP_LAST(1) 读取高频 LIO，后台持续 spin，避免高频队列积压造成“旧 LIO 对新真值”的假误差；同时将 Gazebo ENU 真值旋转到 `camera_init`，并保留世界系 EKF 误差和相对轨迹字段。

完成的四个文件已复制到 `report/data/`：静止前后、运行前后各一份；统计见 `ekf_ab_metrics_summary.csv`，MATLAB 图见 `matlab/plot_ekf_ab_comparison.m`。本轮所有记录中 `safety_stop=0`、NaN 计数为 0、时间戳倒退为 0。

结果不能简单表述为“EKF 全面替代 LIO”：静止时 EKF 明显更稳定；运行时有 EKF 试验中 EKF 跟踪真值明显优于同时采集的 LIO，但无 EKF 和有 EKF 是两次独立冷启动，船的实际航程不同。EKF 仍应先作为评估/候选定位源，接管 Nav2 前还需同一轨迹、多次重复及碰撞安全验证。
