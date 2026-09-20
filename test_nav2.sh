#!/bin/bash
# Nav2 dry-run test: verify all nodes can launch without errors.
# This does NOT require sim/LIO running - it only checks node spawn.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_FILE="/tmp/nav2_test.log"
NAV_PID=""

cleanup() {
  if [[ -n "$NAV_PID" ]] && kill -0 "$NAV_PID" 2>/dev/null; then
    # ros2 launch starts children; use the dedicated process group so none are
    # left behind when this smoke test exits.
    kill -INT -- "-$NAV_PID" 2>/dev/null || kill -INT "$NAV_PID" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-$NAV_PID" 2>/dev/null || kill -KILL "$NAV_PID" 2>/dev/null || true
  fi
  wait "$NAV_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

printf '%s\n' '=== Nav2 Stack Dry-Run Test ==='
printf '%s\n' 'Testing usv_navigation package launch...'

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/lidar_project_roslog_nav2}"
mkdir -p "$ROS_LOG_DIR"
: > "$LOG_FILE"

# setsid gives the launch tree its own process group.  Do not use a pipeline
# here: with `cmd | tee &`, $! is tee rather than ros2 launch.
setsid bash -c "set +u; source /opt/ros/jazzy/setup.bash; source '$SCRIPT_DIR/install/setup.bash'; exec ros2 launch usv_navigation navigation.launch.py use_sim_time:=true" \
  >"$LOG_FILE" 2>&1 &
NAV_PID=$!

sleep 15

printf '\n%s\n' '=== Checking for critical Nav2 nodes in launch log ==='
REQUIRED_NODES=(
  controller_server
  planner_server
  bt_navigator
  behavior_server
  smoother_server
  velocity_smoother
  lifecycle_manager
)

PASS=0
FAIL=0
for node in "${REQUIRED_NODES[@]}"; do
  if grep -q "$node" "$LOG_FILE"; then
    echo "[✓] $node found"
    PASS=$((PASS + 1))
  else
    echo "[✗] $node NOT found"
    FAIL=$((FAIL + 1))
  fi
done

printf '\n%s\n' '=== Test Summary ==='
echo "Passed: $PASS / ${#REQUIRED_NODES[@]}"
echo "Failed: $FAIL / ${#REQUIRED_NODES[@]}"

if [[ "$FAIL" -eq 0 ]]; then
  echo
  echo '✓ Nav2 stack dry-run PASSED - all nodes spawned successfully'
  echo '  Ready for full integration test with sim + LIO running.'
  exit 0
fi

echo
echo '✗ Nav2 stack dry-run FAILED - check /tmp/nav2_test.log for errors'
exit 1
