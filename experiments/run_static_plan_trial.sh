#!/usr/bin/env bash
# Static-obstacle "plan around it and drive" trial against a running chain.
#
# Unlike run_static_obstacle_trial.sh (obstacle x=7.0, goal x=10.0 hardcoded),
# the geometry is parameterized here, because those fixed values are what made
# the earlier runs unusable:
#   * The goal gateway rejects a goal whose 2.8 m check disk touches any cell
#     with cost >= 50.  Inflation puts cost >= 50 roughly 2.7 m out from the
#     1.2 m box, so the goal must sit >= ~5.5 m from the obstacle centre.
#   * The goal disk must also fit inside the 30 m local costmap window, which
#     caps a straight-ahead goal near x = 11 m from a start close to origin.
# Usage: run_static_plan_trial.sh RUN_ID [OBS_X] [OBS_Y] [GOAL_X] [GOAL_Y]
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

RUN_ID="${1:?run_id required}"
OBS_X="${2:-5.0}"
OBS_Y="${3:-0.0}"
GOAL_X="${4:-11.0}"
GOAL_Y="${5:-0.0}"
WAIT_S="${WAIT_S:-240}"

BASE="static_plan_${RUN_ID}"
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
NAV_LOG="$LOG_DIR/${BASE}_nav2_tail.log"
mkdir -p "$DATA_DIR" "$LOG_DIR" "$CONFIG_DIR"

for f in "${PREFIX}_trajectory.csv" "${PREFIX}_plans.csv" "$SCENARIO" "$GOAL_LOG"; do
  test ! -e "$f" || { echo "refusing to overwrite output: $f" >&2; exit 3; }
done

cp "$ROOT/src/usv_navigation/config/nav2_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/src/usv_cloud_filter/config/cloud_filter_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/src/usv_control_bridge/config/bridge_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/experiments/models/dynamic_obstacle/model.sdf" "$CONFIG_DIR/"
cat > "$CONFIG_DIR/trial.txt" <<META
mode=static_plan_and_drive
run_id=$RUN_ID
world=sydney_regatta
goal_camera_init=${GOAL_X},${GOAL_Y},0.0
obstacle_center_camera_init=${OBS_X},${OBS_Y}
obstacle_mode=static
obstacle_box_m=1.2x1.2x2.0
obstacle_z_m=1.0
started_wall_time=$(date --iso-8601=seconds)
META

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
}

wait_nav2_ready() {
  local max_wait="${1:-60}" waited=0
  while ! check_nav2_ready; do
    (( waited >= max_wait )) && { echo "Nav2 preflight failed" >&2; return 1; }
    sleep 2; waited=$((waited + 2))
  done
}

wait_goal_subscriber() {
  local max_wait="${1:-30}" waited=0 info
  while true; do
    info="$(timeout 5 ros2 topic info /goal_pose 2>/dev/null || true)"
    grep -qE 'Subscription count: [1-9]' <<<"$info" && return 0
    (( waited >= max_wait )) && return 1
    sleep 2; waited=$((waited + 2))
  done
}

{
  echo "wall_time=$(date --iso-8601=seconds)"
  echo "geometry obstacle=(${OBS_X},${OBS_Y}) goal=(${GOAL_X},${GOAL_Y})"
  wait_nav2_ready 90
  wait_goal_subscriber 30
  echo "nav2=verified_active"
} >"$PREFLIGHT"
cat "$PREFLIGHT"

# Confirm the gateway will actually take this goal BEFORE spawning anything.
# A rejected goal was the formal05 failure mode; catching it here keeps the
# world clean instead of leaving an orphan obstacle model behind.
python3 "$ROOT/experiments/probe_goal_admissibility.py" --settle-s 12 \
  >"$LOG_DIR/${BASE}_admissibility.log" 2>&1 || true
cat "$LOG_DIR/${BASE}_admissibility.log"
REC_PID=""
cleanup() {
  if [[ -n "$REC_PID" ]] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Recorder first, so the pre-goal baseline and the spawn are both captured.
python3 "$ROOT/experiments/record_nav_path.py" \
  --out-prefix "$PREFIX" --period 0.2 --record-clouds \
  --cloud-period 1.0 --max-cloud-points 300 \
  >"$REC_LOG" 2>&1 &
REC_PID=$!
# Poll instead of testing once after a fixed sleep. record_nav_path.py writes
# the CSV header immediately but only flushes on its first timer tick, so a
# single check at 4 s raced the flush and failed a healthy run (plan08).
rec_ready=0
for _ in $(seq 1 30); do
  kill -0 "$REC_PID" 2>/dev/null || { echo "recorder exited early" >&2; exit 5; }
  if [[ -s "${PREFIX}_trajectory.csv" ]]; then rec_ready=1; break; fi
  sleep 1
done
(( rec_ready == 1 )) || { echo "recorder wrote no CSV within 30s" >&2; exit 5; }

python3 "$ROOT/experiments/dynamic_obstacle_scenario.py" \
  --static --world sydney_regatta --model-name "$BASE" --out-events "$SCENARIO" \
  --start-x "$OBS_X" --start-y "$OBS_Y" --end-x "$OBS_X" --end-y "$OBS_Y" \
  --speed 0.6 --update-hz 5.0 --frame-yaw 1.0 --z 1.0 \
  >"$SCEN_LOG" 2>&1
test -s "$SCENARIO" || { echo "scenario wrote no events" >&2; exit 6; }
# Let the obstacle be observed and marked before the goal is judged.
sleep 8

python3 "$ROOT/experiments/publish_goal_once.py" \
  --topic /goal_pose --frame-id camera_init --x "$GOAL_X" --y "$GOAL_Y" \
  --yaw 0.0 --timeout-s 20 --hold-s 2.0 \
  >"$GOAL_LOG" 2>&1 || true
cat "$GOAL_LOG"
FRESH="$(sed -n 's/^STATUS://p' "$GOAL_LOG" | tail -1)"
if [[ -z "$FRESH" ]] || grep -qiE 'reject|FAILED|ABORTED|CANCELED' <<<"$FRESH"; then
  echo "goal not accepted: ${FRESH:-none}" >&2
  echo "goal_not_accepted=${FRESH:-none}" >>"$CONFIG_DIR/trial.txt"
  exit 7
fi
printf '%s fresh=%s\n' "$(date --iso-8601=seconds)" "$FRESH" >>"$STATUS_LOG"

END=$((SECONDS + WAIT_S))
FINAL="$FRESH"
while (( SECONDS < END )); do
  CUR="$(timeout 5 ros2 topic echo /usv/goal_status --once --field data 2>/dev/null \
    | tr -d '\r"' | sed -e '/^[[:space:]]*$/d' -e '/^---$/d' | tail -1 || true)"
  if [[ -n "$CUR" ]]; then
    FINAL="$CUR"
    printf '%s %s\n' "$(date --iso-8601=seconds)" "$CUR" >>"$STATUS_LOG"
    grep -qE 'SUCCEEDED|CANCELED|ABORTED|FAILED' <<<"$CUR" && break
  fi
  sleep 3
done
sleep 6
cleanup; REC_PID=""; trap - EXIT INT TERM

# Keep the planner-side reason for a non-success next to the data.
tail -n 400 "$ROOT/.usv_logs/nav2.log" >"$NAV_LOG" 2>/dev/null || true

printf 'completed_wall_time=%s\nfinal_status=%s\n' \
  "$(date --iso-8601=seconds)" "$FINAL" >>"$CONFIG_DIR/trial.txt"
python3 - "$PREFIX" <<'PY'
import csv, sys
p = sys.argv[1]
rows = list(csv.DictReader(open(p + '_trajectory.csv')))
xs = [float(r['pose_x']) for r in rows]
ys = [float(r['pose_y']) for r in rows]
dist = sum(((xs[i]-xs[i-1])**2 + (ys[i]-ys[i-1])**2) ** 0.5
           for i in range(1, len(xs)))
plans = set()
with open(p + '_plans.csv') as f:
    for r in csv.DictReader(f):
        plans.add(r['plan_id'])
print(f'samples={len(rows)} traveled={dist:.2f}m '
      f'x={min(xs):.2f}..{max(xs):.2f} y={min(ys):.2f}..{max(ys):.2f} '
      f'plan_snapshots={len(plans)}')
PY
printf 'run=%s\nfinal_status=%s\nprefix=%s\n' "$BASE" "$FINAL" "$PREFIX"
grep -qE '^SUCCEEDED' <<<"$FINAL" || { echo "NOT SUCCEEDED" >&2; exit 8; }
