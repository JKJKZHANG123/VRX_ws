#!/usr/bin/env bash
# M5 控制桥启动脚本
# 用法: ./bridge.sh
# 需要: ./sim.sh + ./lio.sh + ./ekf.sh + ./nav.sh 都在跑
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/install/setup.bash"
exec ros2 launch usv_control_bridge control_bridge.launch.py
