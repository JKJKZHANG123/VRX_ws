# EKF 对比实验数据字典

## 位置与轨迹字段

| 字段 | 含义 |
|---|---|
| `elapsed_s` | 从本次记录开始计算的经过时间，单位 s |
| `truth_x_rel_m`, `truth_y_rel_m` | Gazebo 真值相对起点位置，已转换到 `camera_init` 坐标系 |
| `lio_x_rel_m`, `lio_y_rel_m` | Point-LIO 相对起点位置 |
| `ekf_x_rel_m`, `ekf_y_rel_m` | EKF 世界系相对起点位置 |
| `ekf_camera_x_rel_m`, `ekf_camera_y_rel_m` | 转换到 `camera_init` 后的 EKF 相对轨迹，用于与 LIO/真值叠加绘图 |
| `lio_xy_drift_m` | Point-LIO 相对本次记录起点的平面漂移 |
| `ekf_xy_drift_m` | EKF 相对本次记录起点的平面漂移 |
| `lio_truth_xy_error_m` | Point-LIO 与 Gazebo 真值的平面距离误差 |
| `ekf_truth_xy_error_m` | EKF 与 Gazebo 真值的平面距离误差 |

## 姿态、速度和传感器字段

- `lio_yaw_rad`、`ekf_yaw_rad`：航向角，单位 rad。
- `lio_vx_mps`、`ekf_vx_mps`：估计纵向速度，单位 m/s。
- `lio_wz_radps`、`ekf_wz_radps`：估计角速度，单位 rad/s。
- `cmd_vx_mps`、`cmd_wz_radps`：控制链输出的速度指令。
- `gps_lat`、`gps_lon`：GPS 纬度和经度。
- `imu_gz_radps`：IMU Z 轴角速度。

## 诊断字段

- `filtered_points`、`registered_points`：各点云阶段点数。
- `safety_cloud_points`、`raw_obstacle_points`：安全点云和原始障碍点云点数。
- `safety_stop`：安全停止状态，1 表示触发，0 表示未触发。
- `nan_count`：实际收到的定位消息中检测到非有限值的次数。
- `stamp_backwards`：检测到消息时间戳倒退的次数。

无 EKF 基线中的 `ekf_*` 字段为 NaN 是预期占位。汇总文件中的 `final`、`max`、`p95`、`rmse` 分别表示末值、最大值、95 分位数和均方根。
