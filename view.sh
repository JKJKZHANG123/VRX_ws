#!/usr/bin/env bash
# 3D 导航视图 — RViz 里看立体点云 + 路径 + 障碍方块
# 用法:  ./view.sh [2d]      # 默认为 3D 融合视图; 传 2d 用旧平面视图
# 需要: usv.sh 在跑 (nav2 + costmap_3d_markers + lio)
# 必须带独显 offload: RViz 在核显上渲染 costmap 会 GLSL shader 崩溃
# (active samplers with a different type -> 地图/点云黑屏)
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/install/setup.bash"

CFG="nav3d_view.rviz"
[[ "${1:-}" = "2d" ]] && CFG="nav_view.rviz"

RVIZ_CFG="$(dirname "$0")/install/usv_navigation/share/usv_navigation/config/${CFG}"
exec env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  rviz2 -d "$RVIZ_CFG"