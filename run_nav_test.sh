#!/usr/bin/env bash
# ============================================================================
# USV 导航对比测试运行器  (改进前 / 改进后 -> MATLAB 数据)
# ----------------------------------------------------------------------------
# 用法:
#   ./run_nav_test.sh OUT_CSV GOAL_X GOAL_Y [DURATION_S] [YAW_SIGN]
#
# 前置条件: 完整链路已起 (./usv.sh start, 注意 usv.sh 现在不含 EKF 步)。
# 本脚本只负责: 起一个 nav_data_logger 写 CSV, 发一个 Nav2 目标, 等船跑
# DURATION 秒 (或到达), 然后停止 logger。不启动/不停止仿真或导航。
#
# 典型流程:
#   1) ./usv.sh start
#   2) 让它建一点图 (等 20~30s)
#   3) ./run_nav_test.sh data/baseline_go20.csv 20 0 40      # 改进前
#   4) (改 config / 重启 nav) ./run_nav_test.sh data/improved_go20.csv 20 0 40  # 改进后
#
# 注意: 发目标用 camera_init 系 (与 ./goto.sh 一致)。
# ============================================================================
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null

# ROS setup scripts may reference optional unset tracing variables.  Enable
# nounset only after both environments have been sourced.
set -u

# Do not start a measurement until the Nav2 lifecycle stack and both action
# servers are genuinely ready.  A running launch process alone is not enough.
check_nav2_ready() {
  local nodes actions node
  nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
  for node in /planner_server /controller_server /bt_navigator \
              /behavior_server /waypoint_follower /velocity_smoother; do
    grep -qx "$node" <<<"$nodes" || return 1
    timeout 5 ros2 lifecycle get "$node" 2>/dev/null | grep -q 'active \[3\]' || return 1
  done
  actions="$(timeout 5 ros2 action list 2>/dev/null || true)"
  grep -qx '/navigate_to_pose' <<<"$actions" || return 1
  grep -qx '/navigate_through_poses' <<<"$actions" || return 1
  return 0
}

wait_for_nav2_ready() {
  local max_wait="${1:-30}" waited=0
  while ! check_nav2_ready; do
    if (( waited >= max_wait )); then
      echo "[test] ERROR: Nav2 未处于 active，或 action server 未就绪" >&2
      return 1
    fi
    sleep 2
    waited=$((waited + 2))
    echo -n "."
  done
  echo " ✓ Nav2 ready"
}

latest_goal_status() {
  timeout 4 ros2 topic echo /usv/goal_status --once --field data 2>/dev/null |
    tr -d '\r"' | sed -e '/^[[:space:]]*$/d' -e '/^---$/d' | tail -1
}

wait_for_goal_result() {
  local baseline="$1" max_wait="${2:-15}" waited=0 status
  while (( waited < max_wait )); do
    status="$(latest_goal_status || true)"
    if [[ -n "$status" && "$status" != "$baseline" ]] &&
       grep -qE 'QUEUED|ACCEPTED|EXECUTING|CANCELING|SUCCEEDED|CANCELED|ABORTED|FAILED|REJECTED' <<<"$status"; then
      printf '%s\n' "$status"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

cleanup_logger() {
  if [[ -n "${LOGGER_PID:-}" ]] && kill -0 "$LOGGER_PID" 2>/dev/null; then
    kill "$LOGGER_PID" 2>/dev/null || true
    wait "$LOGGER_PID" 2>/dev/null || true
  fi
}
trap cleanup_logger EXIT INT TERM

OUT_CSV="${1:-data/nav_log.csv}"
GX="${2:-20}"
GY="${3:-0}"
DUR="${4:-40}"
YAW_SIGN="${5:-1.0}"

mkdir -p "$(dirname "$OUT_CSV")"

if ! wait_for_nav2_ready 30; then
  exit 2
fi

if ! timeout 5 ros2 topic info /goal_pose 2>/dev/null |
     grep -qE 'Subscription count: [1-9][0-9]*'; then
  echo "[test] ERROR: /goal_pose 没有订阅者，目标不会进入导航链路" >&2
  exit 2
fi

# yaw 角 -> 四元数
YAW="${6:-0}"
QZ=$(python3 -c "import math,sys; print(round(math.sin(float(sys.argv[1])/2),6))" "$YAW")
QW=$(python3 -c "import math,sys; print(round(math.cos(float(sys.argv[1])/2),6))" "$YAW")

echo -e "\033[1;36m[test] 启动 nav_data_logger -> $OUT_CSV\033[0m"
ros2 run usv_navigation nav_data_logger --ros-args -p out_csv:="$OUT_CSV" >/dev/null 2>&1 &
LOGGER_PID=$!

# 给 logger 一点时间订阅
sleep 3

gateway_log="${OUT_CSV%.csv}_goal_gateway.log"
echo -e "\033[1;36m[test] 发送导航目标 ($GX, $GY)  yaw=$YAW  yaw_sign=$YAW_SIGN\033[0m"
if ! python3 "$SCRIPT_DIR/experiments/publish_goal_once.py" \
    --x "$GX" --y "$GY" --yaw "$YAW" --timeout-s 15 >"$gateway_log" 2>&1; then
  cat "$gateway_log" >&2
  echo "[test] ERROR: 目标发布后没有收到新的 /usv/goal_status，停止本次测试" >&2
  exit 3
fi
cat "$gateway_log"
goal_status="$(sed -n 's/^STATUS://p' "$gateway_log" | tail -1)"
if [[ -z "$goal_status" ]]; then
  echo "[test] ERROR: 目标网关未返回状态，停止本次测试" >&2
  exit 3
fi
echo "[test] goal_status: $goal_status"
if grep -qE 'REJECTED|FAILED|ABORTED|CANCELED' <<<"$goal_status"; then
  echo "[test] ERROR: Nav2 未接受可执行目标；停止本次测试，不进入计时阶段" >&2
  exit 4
fi

echo -e "\033[1;36m[test] 航行 $DUR 秒 (Ctrl-C 提前结束)...\033[0m"
# 打印实时航迹概要
T_END=$(( $(date +%s) + DUR ))
while [ "$(date +%s)" -lt "$T_END" ]; do
  live_status="$(latest_goal_status || true)"
  if grep -qE 'SUCCEEDED|CANCELED|ABORTED|FAILED|REJECTED' <<<"$live_status"; then
    echo "  goal_status: $live_status"
    break
  fi
  pos=$(timeout 3 ros2 topic echo /aft_mapped_to_init --once --field pose.pose.position 2>/dev/null | tr '\n' ' ')
  echo "  t=$((T_END-$(date +%s)))s  pos: $pos"
  sleep 5
done

echo -e "\033[1;36m[test] 停止 logger\033[0m"
cleanup_logger
echo -e "\033[1;32m[test] 完成 -> $OUT_CSV\033[0m"
