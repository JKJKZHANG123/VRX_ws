#!/usr/bin/env bash
# 一键启动 VRX 仿真 (自动配置 NVIDIA 独显渲染 + ROS 环境)
# 用法:
#   ./sim.sh                      # 默认 sydney_regatta 世界
#   ./sim.sh stationkeeping_task  # 指定其他世界
WORLD=${1:-sydney_regatta}

source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/install/setup.bash"

# 强制 Gazebo 用 NVIDIA 独显渲染 (核显跑 gpu_lidar 点云频率会掉到 ~3Hz)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# isolated-install 布局下, 每个包的网格在各自的 install/<pkg>/share 里。
# model://<pkg>/... 需要 GZ_SIM_RESOURCE_PATH 含指向各包 share 父目录的条目,
# 否则 WAM-V 船体/引擎/螺旋桨等 .dae 网格在 GUI 里加载失败 (纯外观, 不影响传感器)。
INSTALL_DIR="$(cd "$(dirname "$0")/install" && pwd)"
for pkg_share in "$INSTALL_DIR"/*/share; do
    [ -d "$pkg_share" ] && export GZ_SIM_RESOURCE_PATH="$pkg_share:${GZ_SIM_RESOURCE_PATH:-}"
done

# 让 Gazebo GUI 加载我们的点击插件 (libgz_click_to_goal.so)
export GZ_GUI_PLUGIN_PATH="$INSTALL_DIR/usv_click_gazebo/lib/usv_click_gazebo:${GZ_GUI_PLUGIN_PATH:-}"

# 显式用独显跑, 确保 nvidia-smi 能看到 gz 进程 (避免跑核显卡顿)
exec env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch vrx_gz competition.launch.py world:="$WORLD"
