#!/usr/bin/env bash
# ==========================================================
# LiDAR 项目展示脚本
#   场景1: Point-LIO 无 GPS 建图 (纯激光惯性里程计)
#   场景2: RGB 彩色点云建图无回环 (激光几何 + 相机颜色)
# ==========================================================
set -e

WS="$(cd "$(dirname "$0")" && pwd)"
cd "$WS"

# ---- source 环境 ----
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash"

echo "=========================================="
echo "  LiDAR 项目展示"
echo "=========================================="
echo "  1) Point-LIO 无 GPS 建图"
echo "       激光雷达 + IMU → Point-LIO → RViz"
echo "       (白色累积地图 + 绿色轨迹 + 里程计)"
echo ""
echo "  2) RGB 彩色建图 (无回环)"
echo "       激光雷达 + 相机 → 着色 → 彩色地图 → RViz"
echo "       (彩色累积地图 /colored_map)"
echo ""
echo "  3) 合并展示: 白色地图 + RGB彩色地图 一起显示"
echo "       同一套系统, RViz 同时显示两种地图"
echo "       (可用左侧勾选框切换对比)"
echo ""
echo "  q) 退出"
echo "=========================================="
read -rp "选择场景 [1/2/3/q]: " choice

case "$choice" in
  1)
    echo ""
    echo ">>> 启动场景1: Point-LIO 无 GPS 建图"
    echo ">>> (Ctrl+C 停止)"
    echo ""
    exec ros2 launch my_mapping_launcher mapping_no_gps.launch.py
    ;;
  2)
    echo ""
    echo ">>> 启动场景2: RGB 彩色建图 (无回环)"
    echo ">>> 回环检测已关闭 (loop_closure:=false)"
    echo ">>> (Ctrl+C 停止)"
    echo ""
    exec ros2 launch my_mapping_launcher with_camera.launch.py loop_closure:=false
    ;;
  3)
    echo ""
    echo ">>> 启动场景3: 合并展示 (白色地图 + RGB彩色地图)"
    echo ">>> 同一套系统, 回环关闭, RViz 同时显示两种地图"
    echo ">>> (Ctrl+C 停止)"
    echo ""
    exec ros2 launch my_mapping_launcher with_camera.launch.py \
        loop_closure:=false rviz_config:=demo_combined.rviz
    ;;
  q|Q)
    echo "退出"
    exit 0
    ;;
  *)
    echo "无效选择: $choice"
    exit 1
    ;;
esac
