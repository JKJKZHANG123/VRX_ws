#!/usr/bin/env bash
# 一键启动 M2 融合定位链路 (ekf_node + navsat_transform + odom->camera_init TF 桥)
# 冷启动顺序 (关键): 先 ./sim.sh, 再 ./lio.sh, 最后 ./ekf.sh —— 三者一次起齐,
# 中途不要单独重启仿真 (gz 时钟跳变会让 EKF 预测步 dt 爆炸, /odometry/filtered 发散)。
# 用法:
#   ./ekf.sh
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/install/setup.bash"

exec ros2 launch usv_localization localization.launch.py
