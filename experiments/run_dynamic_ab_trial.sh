#!/usr/bin/env bash
# Run one matched dynamic-obstacle navigation trial against an already-started chain.
# Usage: ./experiments/run_dynamic_ab_trial.sh tracker_disabled|tracker_enabled [run_id]
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
# ROS setup files may read optional unset tracing variables.
set -u

MODE="${1:?mode must be tracker_disabled or tracker_enabled}"
case "$MODE" in
  tracker_disabled|tracker_enabled) ;;
  *) echo "invalid mode: $MODE" >&2; exit 2 ;;
esac
RUN_ID="${2:-$(date +%Y%m%d_%H%M%S)}"
BASE="dynamic_ab_${MODE}_${RUN_ID}"
DATA_DIR="$ROOT/report3/data"
LOG_DIR="$ROOT/report3/logs"
CONFIG_DIR="$ROOT/report3/configs/$BASE"
CSV="$DATA_DIR/$BASE.csv"
EVENTS="$DATA_DIR/${BASE}_obstacle_events.csv"
LOGGER_LOG="$LOG_DIR/${BASE}_logger.log"
GOAL_LOG="$LOG_DIR/${BASE}_goal.log"
SCENARIO_LOG="$LOG_DIR/${BASE}_scenario.log"
STATUS_LOG="$LOG_DIR/${BASE}_status.log"

mkdir -p "$DATA_DIR" "$LOG_DIR" "$CONFIG_DIR"
for output in "$CSV" "$EVENTS" "$LOGGER_LOG" "$GOAL_LOG" "$SCENARIO_LOG"; do
  if [[ -e "$output" ]]; then
    echo "refusing to overwrite existing output: $output" >&2
    exit 3
  fi
done

cp "$ROOT/src/usv_navigation/config/nav2_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/src/usv_navigation/config/dynamic_obstacle_tracker.yaml" "$CONFIG_DIR/"
cp "$ROOT/experiments/models/dynamic_obstacle/model.sdf" "$CONFIG_DIR/"
cat > "$CONFIG_DIR/trial.txt" <<META
mode=$MODE
run_id=$RUN_ID
world=sydney_regatta
goal_camera_init=10.0,0.0,0.0
obstacle_model=dynamic_obstacle_ab
obstacle_start_camera_init=6.5,-6.0
obstacle_end_camera_init=6.5,6.0
obstacle_speed_mps=0.60
obstacle_update_hz=5.0
obstacle_z_m=1.0
post_motion_record_s=45
started_wall_time=$(date --iso-8601=seconds)
META

LOGGER_PID=""
GOAL_PUB_PID=""
cleanup_logger() {
  if [[ -n "$LOGGER_PID" ]] && kill -0 "$LOGGER_PID" 2>/dev/null; then
    kill "$LOGGER_PID" 2>/dev/null || true
    wait "$LOGGER_PID" 2>/dev/null || true
  fi
  if [[ -n "$GOAL_PUB_PID" ]] && kill -0 "$GOAL_PUB_PID" 2>/dev/null; then
    kill -INT "$GOAL_PUB_PID" 2>/dev/null || true
    wait "$GOAL_PUB_PID" 2>/dev/null || true
  fi
}
trap cleanup_logger EXIT INT TERM

ros2 run usv_navigation nav_data_logger --ros-args -p out_csv:="$CSV" \
  >"$LOGGER_LOG" 2>&1 &
LOGGER_PID=$!
sleep 3

# Send exactly one goal after confirming click_to_goal is subscribed.  A retry
# after Nav2 accepts the action preempts the active goal and manufactures an
# ABORTED result, so this block deliberately never republishes the goal.
accepted=false
deadline=$((SECONDS + 25))
while (( SECONDS < deadline )); do
  topic_info="$(timeout 4 ros2 topic info /goal_pose 2>/dev/null || true)"
  if grep -Eq 'Subscription count: [1-9][0-9]*' <<<"$topic_info"; then
    accepted=true
    break
  fi
  sleep 0.5
done
if [[ "$accepted" != true ]]; then
  echo "goal publisher subscriber was not ready within 25 seconds" >&2
  exit 5
fi

printf '%s sending one goal after subscriber readiness\n' \
  "$(date --iso-8601=seconds)" >> "$GOAL_LOG"
python3 "$ROOT/experiments/publish_goal_once.py" \
  --topic /goal_pose --frame-id camera_init --x 10.0 --y 0.0 --hold-s 3.0 \
  >>"$GOAL_LOG" 2>&1

# Wait for acceptance without publishing another goal.
accepted=false
deadline=$((SECONDS + 25))
while (( SECONDS < deadline )); do
  status="$(timeout 4 ros2 topic echo /usv/goal_status --once --field data 2>/dev/null | tr -d '\r"' | sed -e '/^[[:space:]]*$/d' -e '/^---$/d' | tail -1 || true)"
  printf '%s %s\n' "$(date --iso-8601=seconds)" "$status" >> "$STATUS_LOG"
  if [[ "$status" == *ACCEPTED* || "$status" == *EXECUTING* ]]; then
    accepted=true
    break
  fi
  if [[ "$status" == *ABORTED* || "$status" == *FAILED* || "$status" == *CANCELED* || "$status" == *REJECTED* || "$status" == *rejected* ]]; then
    echo "goal failed before obstacle spawn: $status" >&2
    exit 4
  fi
  sleep 0.5
done
if [[ "$accepted" != true ]]; then
  echo "goal was not accepted within 25 seconds" >&2
  exit 5
fi

python3 "$ROOT/experiments/dynamic_obstacle_scenario.py" \
  --world sydney_regatta \
  --model-name dynamic_obstacle_ab \
  --out-events "$EVENTS" \
  --start-x 6.5 --start-y -6.0 \
  --end-x 6.5 --end-y 6.0 \
  --speed 0.60 --update-hz 5.0 --frame-yaw 1.0 --z 1.0 \
  >"$SCENARIO_LOG" 2>&1

sleep 45
final_status="$(timeout 4 ros2 topic echo /usv/goal_status --once --field data 2>/dev/null | tr -d '\r"' | sed -e '/^[[:space:]]*$/d' -e '/^---$/d' | tail -1 || true)"
printf '%s FINAL %s\n' "$(date --iso-8601=seconds)" "$final_status" >> "$STATUS_LOG"

cleanup_logger
LOGGER_PID=""
trap - EXIT INT TERM

if ! grep -qE '^SUCCEEDED( |$)' <<<"$final_status" || ! grep -qE 'code=4( |$)' <<<"$final_status"; then
  echo "dynamic obstacle trial did not finish with SUCCEEDED code=4: $final_status" >&2
  exit 8
fi

for name in nav2 bridge cloud_filter sim lio gz_click clock_watchdog; do
  if [[ -f "$ROOT/.usv_logs/$name.log" ]]; then
    cp "$ROOT/.usv_logs/$name.log" "$LOG_DIR/${BASE}_${name}.log"
  fi
done
printf 'completed_wall_time=%s\nfinal_status=%s\n' \
  "$(date --iso-8601=seconds)" "$final_status" >> "$CONFIG_DIR/trial.txt"
printf '%s\n' "$BASE"
