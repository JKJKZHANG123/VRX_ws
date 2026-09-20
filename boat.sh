#!/usr/bin/env bash
# WAM-V 手动驾驶助手。用法:
#   ./boat.sh fwd 15        # 直行 15 秒后自动归零
#   ./boat.sh fwd 15 500    # 直行 15 秒, 推力 500
#   ./boat.sh left 8        # 左转 8 秒 (右推左反推) 后归零
#   ./boat.sh right 8       # 右转 8 秒后归零
#   ./boat.sh back 10       # 倒车 10 秒后归零
#   ./boat.sh stop          # 立即双侧归零 (船仍会滑行一段, 无刹车)
# 注意: Gazebo 推进器保持最后收到的指令, 所以每个动作结束都必须归零 —
# 本脚本已自动处理。
source /opt/ros/jazzy/setup.bash 2>/dev/null

L=/wamv/thrusters/left/thrust
R=/wamv/thrusters/right/thrust
MSG=std_msgs/msg/Float64

pub_once() { ros2 topic pub --once "$1" $MSG "{data: $2}" >/dev/null; }

zero() {
  pub_once $L 0.0
  pub_once $R 0.0
  echo "[boat] thrusters zeroed (hull will coast to a stop)"
}

drive() { # drive <left> <right> <sec>
  local lv=$1 rv=$2 sec=$3
  echo "[boat] L=$lv R=$rv for ${sec}s ..."
  timeout "$sec" ros2 topic pub -r 10 $L $MSG "{data: $lv}" >/dev/null &
  timeout "$sec" ros2 topic pub -r 10 $R $MSG "{data: $rv}" >/dev/null
  wait
  zero
}

CMD=${1:-help}
SEC=${2:-10}
THRUST=${3:-300}

case "$CMD" in
  fwd)   drive "$THRUST" "$THRUST" "$SEC" ;;
  back)  drive "-$THRUST" "-$THRUST" "$SEC" ;;
  left)  drive "-$THRUST" "$THRUST" "$SEC" ;;
  right) drive "$THRUST" "-$THRUST" "$SEC" ;;
  stop)  zero ;;
  *) grep '^#' "$0" | head -10; exit 1 ;;
esac
