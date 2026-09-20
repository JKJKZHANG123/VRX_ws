#!/usr/bin/env bash
# 让船导航到指定坐标 (坐标系: camera_init — Point-LIO 的世界系)
# 用法:
#   ./goto.sh 20 10          # 开到 x=20, y=10 (保持当前航向)
#   ./goto.sh 20 10 1.57     # 开到 x=20, y=10 并朝向 1.57rad
#   ./goto.sh here            # 显示当前位置
#   ./goto.sh cancel          # 取消当前导航任务

source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$(dirname "$0")/install/setup.bash" 2>/dev/null

GREEN='\033[0;32m'; CYAN='\033[0;36m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Do not publish a goal merely because a launch parent exists.  Verify every
# managed Nav2 node is active and both action servers are present first.
check_nav2_ready() {
  local nodes actions node state
  nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
  for node in /planner_server /controller_server /bt_navigator \
              /behavior_server /waypoint_follower /velocity_smoother; do
    grep -qx "$node" <<<"$nodes" || return 1
    state="$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || true)"
    grep -q 'active \[3\]' <<<"$state" || return 1
  done
  actions="$(timeout 5 ros2 action list 2>/dev/null || true)"
  grep -qx '/navigate_to_pose' <<<"$actions" || return 1
  grep -qx '/navigate_through_poses' <<<"$actions" || return 1
  return 0
}

wait_nav2_ready() {
  local max_wait="${1:-45}" waited=0
  echo -e "${CYAN}[gate] 确认 Nav2 lifecycle/action 就绪...${NC}"
  while ! check_nav2_ready; do
    if (( waited >= max_wait )); then
      echo -e "${RED}[gate] Nav2 未就绪，拒绝发送目标。${NC}" >&2
      return 1
    fi
    sleep 2
    waited=$((waited + 2))
  done
  echo -e "${GREEN}[gate] Nav2 已确认：6 个节点 active [3]，两个 action server 存在。${NC}"
}

case "${1:-}" in

  here|pos|position)
    echo -e "${CYAN}当前位置 (camera_init):${NC}"
    timeout 5 ros2 topic echo /aft_mapped_to_init --once --field pose.pose.position 2>/dev/null \
      | grep -E "x:|y:" | awk '{print "  " $1, $2}'
    ;;

  cancel|stop)
    echo -e "${YELLOW}取消导航...${NC}"
    timeout 8 ros2 topic pub --once /cancel_navigation std_msgs/msg/Empty \
      "{}" > /dev/null 2>&1
    echo -e "${GREEN}已发送取消指令${NC}"
    ;;

  ''|--help|-h)
    echo "用法:"
    echo "  ./goto.sh <x> <y>           开到坐标 (x, y)  [camera_init 系，保持当前航向]"
    echo "  ./goto.sh <x> <y> <yaw>     开到坐标并指定到达朝向(弧度)"
    echo "  ./goto.sh here              显示当前位置"
    echo "  ./goto.sh cancel            取消当前导航"
    echo ""
    echo "示例:"
    echo "  ./goto.sh 20 0              向正前方20米"
    echo "  ./goto.sh 15 15             斜向右前方"
    echo "  ./goto.sh 0 0               开回出发点"
    ;;

  *)
    X="${1}"
    Y="${2:-0}"
    YAW="${3:-0}"

    wait_nav2_ready 45 || exit 2
    goal_info="$(timeout 5 ros2 topic info /goal_pose 2>/dev/null || true)"
    if ! grep -qE 'Subscription count: [1-9][0-9]*' <<<"$goal_info"; then
      echo -e "${RED}[gate] /goal_pose 没有订阅者，拒绝发送目标。${NC}" >&2
      exit 2
    fi
    echo -e "${GREEN}[gate] /goal_pose 订阅者已确认。${NC}"

    cur="$(timeout 5 ros2 topic echo /aft_mapped_to_init --once --field pose.pose.position 2>/dev/null | tr '\n' ' ')"
    echo -e "${CYAN}当前位置 (camera_init): ${cur}${NC}"
    echo -e "${GREEN}导航目标: x=${X}  y=${Y}  朝向=${YAW}rad${NC}"

    # publish_goal_once drains the transient-local old status and waits for a
    # fresh status_seq, so an old SUCCEEDED cannot be mistaken for this goal.
    gateway_log="${TMPDIR:-/tmp}/goto_goal_$$.log"
    if ! python3 "$SCRIPT_DIR/experiments/publish_goal_once.py" \
        --topic /goal_pose --frame-id camera_init --x "$X" --y "$Y" \
        --yaw "$YAW" --timeout-s 15 --hold-s 0.5 >"$gateway_log" 2>&1; then
      cat "$gateway_log" >&2
      rm -f "$gateway_log"
      echo -e "${RED}[gate] 目标状态未刷新，认为发送失败。${NC}" >&2
      exit 3
    fi
    cat "$gateway_log"
    status="$(sed -n 's/^STATUS://p' "$gateway_log" | tail -1)"
    rm -f "$gateway_log"
    if [[ -z "$status" ]] || grep -qE 'REJECTED|FAILED|ABORTED|CANCELED' <<<"$status"; then
      echo -e "${RED}[gate] Nav2 没有接受可执行目标：${status:-无状态}${NC}" >&2
      exit 4
    fi
    echo -e "${GREEN}[gate] 目标状态已二次确认：${status}${NC}"
    # DDS graph discovery can briefly return an incomplete action list just
    # after Nav2 accepts a goal. Retry the full readiness gate instead of
    # declaring Nav2 dead from one transient query.
    if ! wait_nav2_ready 20; then
      echo -e "${RED}[gate] 目标已接受，但 Nav2 在发布后未能连续通过就绪复核。${NC}" >&2
      exit 5
    fi

    # Do not return immediately after ACCEPTED: observe the status again for a
    # short window so an immediate planner/controller ABORTED is reported to
    # the caller rather than being mistaken for a successful goto submission.
    post_status="$status"
    post_deadline=$((SECONDS + 8))
    while (( SECONDS < post_deadline )); do
      observed="$(timeout 5 ros2 topic echo /usv/goal_status --once --field data 2>/dev/null         | tr -d '\r"' | sed -e '/^[[:space:]]*$/d' -e '/^---$/d' | tail -1 || true)"
      if [[ -n "$observed" ]]; then
        post_status="$observed"
        echo -e "${CYAN}[gate] 目标状态复核：${post_status}${NC}"
        if grep -qE 'SUCCEEDED' <<<"$post_status"; then
          echo -e "${GREEN}✓ 目标已完成，Nav2 仍处于 active。${NC}"
          exit 0
        fi
        if grep -qE 'ABORTED|FAILED|REJECTED|CANCELED|CANCELLED' <<<"$post_status"; then
          echo -e "${RED}[gate] 目标在复核窗口内进入失败/取消终态：${post_status}${NC}" >&2
          exit 6
        fi
      fi
      sleep 1
    done
    echo -e "${GREEN}✓ Nav2 已连续复核 active，目标仍在执行：${post_status}${NC}"
    ;;
esac
