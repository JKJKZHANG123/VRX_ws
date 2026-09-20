#!/usr/bin/env bash
# Run a gated static-obstacle trial against an already-started USV chain.
# Usage: ./experiments/run_static_obstacle_trial.sh [run_id]
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

RUN_ID="${1:-$(date +%Y%m%d_%H%M%S)}"
BASE="static_obstacle_${RUN_ID}"
DATA_DIR="$ROOT/report3/data"
LOG_DIR="$ROOT/report3/logs"
CONFIG_DIR="$ROOT/report3/configs/$BASE"
PREFIX="$DATA_DIR/$BASE"
SCENARIO="$DATA_DIR/${BASE}_scenario.csv"
PREFLIGHT="$LOG_DIR/${BASE}_preflight.log"
REC_LOG="$LOG_DIR/${BASE}_recorder.log"
SCEN_LOG="$LOG_DIR/${BASE}_scenario.log"
GOAL_LOG="$DATA_DIR/${BASE}_goal_gateway.log"
STATUS_LOG="$LOG_DIR/${BASE}_status.log"
mkdir -p "$DATA_DIR" "$LOG_DIR" "$CONFIG_DIR"
cp "$ROOT/src/usv_navigation/config/nav2_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/src/usv_navigation/config/dynamic_obstacle_tracker.yaml" "$CONFIG_DIR/"
cp "$ROOT/experiments/models/dynamic_obstacle/model.sdf" "$CONFIG_DIR/"
cat > "$CONFIG_DIR/trial.txt" <<META
mode=static_obstacle
run_id=$RUN_ID
world=sydney_regatta
goal_camera_init=10.0,0.0,0.0
obstacle_model=$BASE
obstacle_center_camera_init=7.0,0.0
obstacle_mode=static
obstacle_z_m=1.0
started_wall_time=$(date --iso-8601=seconds)
META

for f in "${PREFIX}_trajectory.csv" "${PREFIX}_plans.csv" "${PREFIX}_obstacles.csv" \
         "${PREFIX}_events.csv" "$SCENARIO" "$PREFLIGHT" "$REC_LOG" "$SCEN_LOG" \
         "$GOAL_LOG" "$STATUS_LOG"; do
  test ! -e "$f" || { echo "refusing to overwrite output: $f" >&2; exit 3; }
done

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
}

wait_nav2_ready() {
  local max_wait="${1:-60}" waited=0
  while ! check_nav2_ready; do
    if (( waited >= max_wait )); then
      echo "Nav2 preflight failed after ${max_wait}s" >&2
      return 1
    fi
    sleep 2
    waited=$((waited + 2))
  done
}

wait_goal_subscriber() {
  local max_wait="${1:-30}" waited=0 info
  while true; do
    info="$(timeout 5 ros2 topic info /goal_pose 2>/dev/null || true)"
    if grep -qE 'Subscription count: [1-9][0-9]*' <<<"$info"; then return 0; fi
    if (( waited >= max_wait )); then return 1; fi
    sleep 2
    waited=$((waited + 2))
  done
}

{
  echo "wall_time=$(date --iso-8601=seconds)"
  echo "stage=preflight_before_recorder"
  wait_nav2_ready 60
  wait_goal_subscriber 30
  echo "nav2=active_nodes_and_actions_verified"
  echo "goal_pose=subscriber_verified"
} >"$PREFLIGHT"
cat "$PREFLIGHT"

REC_PID=""
cleanup_recorder() {
  if [[ -n "$REC_PID" ]] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
}
trap cleanup_recorder EXIT INT TERM

# This is the experiment recorder script, not a guessed ros2 executable name.
python3 "$ROOT/experiments/record_nav_path.py" \
  --out-prefix "$PREFIX" --period 0.2 --record-clouds \
  --cloud-period 1.0 --max-cloud-points 300 \
  >"$REC_LOG" 2>&1 &
REC_PID=$!
sleep 4
kill -0 "$REC_PID" 2>/dev/null || { echo "path recorder exited early" >&2; exit 5; }
test -s "${PREFIX}_trajectory.csv" || { echo "path recorder did not create trajectory CSV" >&2; exit 5; }

# Spawn a stationary 1.2 m box directly on the nominal route before sending the
# goal. The event file is the authoritative obstacle-center record.  The box
# is at x=7.0 m (not x=5.0 m) to provide stopping/turning margin while the
# direct emergency envelope is being validated; this does not replace the
# collision gate.
python3 "$ROOT/experiments/dynamic_obstacle_scenario.py" \
  --static --world sydney_regatta --model-name "$BASE" --out-events "$SCENARIO" \
  --start-x 7.0 --start-y 0.0 --end-x 7.0 --end-y 0.0 \
  --speed 0.6 --update-hz 5.0 --frame-yaw 1.0 --z 1.0 \
  >"$SCEN_LOG" 2>&1
sleep 3

test -s "$SCENARIO" || { echo "scenario did not create event CSV" >&2; exit 6; }
{
  echo "stage=preflight_before_goal"
  wait_nav2_ready 30
  wait_goal_subscriber 20
  echo "nav2=active_nodes_and_actions_verified_again"
  echo "goal_pose=subscriber_verified_again"
} >>"$PREFLIGHT"

python3 "$ROOT/experiments/publish_goal_once.py" \
  --topic /goal_pose --frame-id camera_init --x 10.0 --y 0.0 \
  --yaw 0.0 --timeout-s 20 --hold-s 2.0 \
  >"$GOAL_LOG" 2>&1
cat "$GOAL_LOG"
FRESH_STATUS="$(sed -n 's/^STATUS://p' "$GOAL_LOG" | tail -1)"
if [[ -z "$FRESH_STATUS" ]] || grep -qE 'REJECTED|FAILED|ABORTED|CANCELED' <<<"$FRESH_STATUS"; then
  echo "goal was not accepted: ${FRESH_STATUS:-no fresh status}" >&2
  exit 7
fi
printf '%s fresh_goal_status=%s\n' "$(date --iso-8601=seconds)" "$FRESH_STATUS" >>"$STATUS_LOG"

# Never assume a transient-local terminal status belongs to this run. We first
# observed a fresh ACCEPTED/EXECUTING status above, then wait for terminal.
END=$((SECONDS + 210))
FINAL_STATUS="$FRESH_STATUS"
while (( SECONDS < END )); do
  CURRENT="$(timeout 5 ros2 topic echo /usv/goal_status --once --field data 2>/dev/null \
    | tr -d '\r\"' | sed -e '/^[[:space:]]*$/d' -e '/^---$/d' | tail -1 || true)"
  if [[ -n "$CURRENT" ]]; then
    FINAL_STATUS="$CURRENT"
    printf '%s %s\n' "$(date --iso-8601=seconds)" "$CURRENT" >>"$STATUS_LOG"
    if grep -qE 'SUCCEEDED|CANCELED|ABORTED|FAILED|REJECTED' <<<"$CURRENT"; then break; fi
  fi
  sleep 3
done
sleep 8
cleanup_recorder
REC_PID=""
trap - EXIT INT TERM
printf 'completed_wall_time=%s\nfinal_status=%s\n' \
  "$(date --iso-8601=seconds)" "$FINAL_STATUS" >> "$CONFIG_DIR/trial.txt"
if ! grep -qE '^SUCCEEDED( |$)' <<<"$FINAL_STATUS" || ! grep -qE 'code=4( |$)' <<<"$FINAL_STATUS"; then
  echo "static obstacle trial did not finish with SUCCEEDED code=4: $FINAL_STATUS" >&2
  exit 8
fi
printf 'run=%s\nfinal_status=%s\npreflight=%s\nconfig=%s\n' "$BASE" "$FINAL_STATUS" "$PREFLIGHT" "$CONFIG_DIR"
