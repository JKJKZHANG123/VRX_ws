#!/usr/bin/env bash
# 一键启动 Point-LIO 建图链路 (适配层 + Point-LIO + RViz)
# 用法:
#   ./lio.sh              # 带 RViz
#   ./lio.sh norviz       # 不带 RViz
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/install/setup.bash"

RVIZ=true
[ "${1:-}" = "norviz" ] && RVIZ=false

exec ros2 launch point_lio mapping_vrx.launch.py rviz:=$RVIZ
