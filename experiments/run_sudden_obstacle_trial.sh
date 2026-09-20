#!/usr/bin/env bash
# Sudden-obstacle reaction trial: the boat is already under way at cruise speed
# when an obstacle appears directly ahead of it.
#
# This differs from run_static_plan_trial.sh in the one way that matters: there,
# the obstacle exists before the goal is sent, so Nav2's first plan already
# routes around it and nothing is ever "reacted" to. Here the goal is sent into
# clear water, the boat accelerates, and the obstacle is spawned only once the
# vessel has closed to a chosen distance -- so the recorded data shows the
# detection-to-avoidance transient rather than a pre-planned detour.
#
# Usage: run_sudden_obstacle_trial.sh RUN_ID [GOAL_X] [GOAL_Y] [TRIGGER_DIST_M] [LATERAL_OFFSET_M]
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

RUN_ID="${1:?run_id required}"
GOAL_X="${2:-24.0}"
GOAL_Y="${3:-0.0}"
# Spawn the obstacle this far ahead of the boat's CURRENT position, on its
# current heading. 8 m at 0.6 m/s leaves ~13 s to react, and is outside the
# bridge's 6.5 m warning corridor so the spawn itself does not trip a stop.
TRIGGER_AHEAD="${4:-8.0}"
# Small lateral offset so the obstacle blocks the route without being a perfectly
# symmetric head-on case (which has no preferred turn direction).
LATERAL="${5:-0.0}"
WAIT_S="${WAIT_S:-300}"

BASE="sudden_obstacle_${RUN_ID}"
DATA_DIR="$ROOT/report4/data"
LOG_DIR="$ROOT/report4/logs"
CONFIG_DIR="$ROOT/report4/configs/$BASE"
PREFIX="$DATA_DIR/$BASE"
SCENARIO="$DATA_DIR/${BASE}_scenario.csv"
PREFLIGHT="$LOG_DIR/${BASE}_preflight.log"
REC_LOG="$LOG_DIR/${BASE}_recorder.log"
SCEN_LOG="$LOG_DIR/${BASE}_scenario.log"
GOAL_LOG="$DATA_DIR/${BASE}_goal_gateway.log"
STATUS_LOG="$LOG_DIR/${BASE}_status.log"
NAV_LOG="$LOG_DIR/${BASE}_nav2_tail.log"
BRIDGE_LOG="$LOG_DIR/${BASE}_bridge_tail.log"
TRIGGER_LOG="$LOG_DIR/${BASE}_trigger.log"
mkdir -p "$DATA_DIR" "$LOG_DIR" "$CONFIG_DIR"

for f in "${PREFIX}_trajectory.csv" "${PREFIX}_plans.csv" "$SCENARIO" "$GOAL_LOG"; do
  test ! -e "$f" || { echo "refusing to overwrite output: $f" >&2; exit 3; }
done

cp "$ROOT/src/usv_navigation/config/nav2_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/src/usv_control_bridge/config/bridge_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/src/usv_cloud_filter/config/cloud_filter_params.yaml" "$CONFIG_DIR/"
cp "$ROOT/experiments/models/dynamic_obstacle/model.sdf" "$CONFIG_DIR/"
cat > "$CONFIG_DIR/trial.txt" <<META
mode=sudden_obstacle_reaction
run_id=$RUN_ID
world=sydney_regatta
goal_camera_init=${GOAL_X},${GOAL_Y},0.0
trigger_spawn_ahead_m=${TRIGGER_AHEAD}
trigger_lateral_offset_m=${LATERAL}
obstacle_box_m=1.2x1.2x2.0
obstacle_z_m=1.0
note=obstacle spawned only after the boat is under way; not present in first plan
started_wall_time=$(date --iso-8601=seconds)
META

check_nav2_ready() {
  local nodes node state actions
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

{
  echo "wall_time=$(date --iso-8601=seconds)"
  echo "goal=(${GOAL_X},${GOAL_Y}) trigger_ahead=${TRIGGER_AHEAD}m lateral=${LATERAL}m"
  waited=0
  while ! check_nav2_ready; do
    (( waited >= 90 )) && { echo "Nav2 preflight failed" >&2; exit 1; }
    sleep 2; waited=$((waited + 2))
  done
  echo "nav2=verified_active"
} >"$PREFLIGHT"
cat "$PREFLIGHT"

REC_PID=""
cleanup() {
  if [[ -n "$REC_PID" ]] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

python3 "$ROOT/experiments/record_nav_path.py" \
  --out-prefix "$PREFIX" --period 0.2 --record-clouds \
  --cloud-period 1.0 --max-cloud-points 300 \
  >"$REC_LOG" 2>&1 &
REC_PID=$!
rec_ready=0
for _ in $(seq 1 30); do
  kill -0 "$REC_PID" 2>/dev/null || { echo "recorder exited early" >&2; exit 5; }
  if [[ -s "${PREFIX}_trajectory.csv" ]]; then rec_ready=1; break; fi
  sleep 1
done
(( rec_ready == 1 )) || { echo "recorder wrote no CSV within 30s" >&2; exit 5; }

# Goal FIRST, into water the planner sees as clear.
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

# Wait until the boat is genuinely under way before injecting the obstacle,
# then spawn it ahead of the vessel's live pose. The trigger is distance
# travelled, not a fixed sleep, so the spawn geometry is repeatable.
python3 "$ROOT/experiments/spawn_obstacle_ahead.py" \
  --ahead-m "$TRIGGER_AHEAD" --lateral-m "$LATERAL" \
  --min-speed 0.25 --min-travel-m 4.0 --timeout-s 120 \
  --model-name "$BASE" --out-events "$SCENARIO" \
  >"$TRIGGER_LOG" 2>&1 || { echo "obstacle injection failed" >&2; cat "$TRIGGER_LOG" >&2; exit 6; }
cat "$TRIGGER_LOG"

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

tail -n 400 "$ROOT/.usv_logs/nav2.log"  >"$NAV_LOG"    2>/dev/null || true
tail -n 200 "$ROOT/.usv_logs/bridge.log" >"$BRIDGE_LOG" 2>/dev/null || true

printf 'completed_wall_time=%s\nfinal_status=%s\n' \
  "$(date --iso-8601=seconds)" "$FINAL" >>"$CONFIG_DIR/trial.txt"

python3 - "$PREFIX" "$SCENARIO" <<'PY'
import csv, math, sys
prefix, scenario = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(prefix + '_trajectory.csv')))
def f(r, k):
    v = r[k]
    return float(v) if v not in ('', 'None') else 0.0
ox = oy = None
t_spawn = None
try:
    for r in csv.DictReader(open(scenario)):
        if r['event'] in ('spawned', 'static_ready'):
            ox, oy = float(r['camera_x']), float(r['camera_y'])
            t_spawn = float(r['sim_time'])
            break
except Exception:
    pass
xs = [f(r, 'pose_x') for r in rows]
ys = [f(r, 'pose_y') for r in rows]
dist = sum(math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1]) for i in range(1, len(xs)))
line = f'samples={len(rows)} traveled={dist:.2f}m x={min(xs):.2f}..{max(xs):.2f} y={min(ys):.2f}..{max(ys):.2f}'
if ox is not None:
    cd = min(math.hypot(f(r,'pose_x')-ox, f(r,'pose_y')-oy) for r in rows)
    bd = min(math.hypot(f(r,'pose_x')+2.6*math.cos(f(r,'pose_yaw'))-ox,
                        f(r,'pose_y')+2.6*math.sin(f(r,'pose_yaw'))-oy) for r in rows)
    line += f' | obstacle=({ox:.2f},{oy:.2f}) closest_centre={cd:.2f}m closest_bow={bd:.2f}m'
print(line)
PY
printf 'run=%s\nfinal_status=%s\nprefix=%s\n' "$BASE" "$FINAL" "$PREFIX"
grep -qE '^SUCCEEDED' <<<"$FINAL" || { echo "NOT SUCCEEDED" >&2; exit 8; }
