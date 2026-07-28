#!/usr/bin/env bash
# CellOps (WorkCell Part E) — one-command Nav2 evaluation run.
#
# Launches the TB3 world (headless, retried) + Nav2 on the SLAM-produced map,
# seeds AMCL, then sends a goal sequence and reports a success rate — the same
# "run N trials, report a number" protocol as the other WorkCell parts.
#
# Usage (from Windows):
#   wsl -d Ubuntu-24.04 -- bash -l /mnt/c/.../scripts/run_nav2.sh
# Env: RECORD=1 -> record run data; GOALS="x y label\n..." -> override goal list
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_YAML="${MAP_YAML:-$HOME/maps/tb3_world.yaml}"
LOG_DIR="/tmp/cellops"
RESULTS="$LOG_DIR/nav2_results.csv"

# AMCL runs with set_initial_pose:false on Jazzy — it sits dead until a pose is
# published. NOTE: slam_toolbox anchors the map frame at the robot's STARTING
# pose, so the spawn point is the map origin (0,0), not Gazebo world coords.
INIT_X="${INIT_X:-0.0}"
INIT_Y="${INIT_Y:-0.0}"

# Goal list "x y label" in MAP frame, produced by pick_goals.py from the saved
# map (highest-clearance cell per angular sector) — no hand-guessed coordinates.
DEFAULT_GOALS=(
  "2.556 2.298 north-east"
  "1.456 2.298 north"
  "0.006 0.548 west"
  "1.456 -1.252 south"
  "2.606 -1.302 south-east"
  "0.0 0.0 home"
)

set +u; source /opt/ros/jazzy/setup.bash; set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
source "$HERE/lib_common.sh"
mkdir -p "$LOG_DIR"

cleanup() {
  log "tearing down"
  safe_pkill "record_run.py"
  safe_pkill "navigation2.launch.py"
  safe_pkill "component_container|rviz2"
  teardown_world
}
trap cleanup EXIT

[[ -f "$MAP_YAML" ]] || { log "ERROR: map not found at $MAP_YAML — run run_mapping.sh first"; exit 1; }
cleanup

log "=== 1/4  Gazebo: turtlebot3_world (headless) ==="
launch_world "$LOG_DIR/gazebo_nav.log" || exit 1

log "=== 2/4  Nav2 on $MAP_YAML ==="
nohup ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:="$MAP_YAML" \
  > "$LOG_DIR/nav2.log" 2>&1 < /dev/null &
disown
wait_for_active /amcl 120 || { tail -n 40 "$LOG_DIR/nav2.log"; exit 1; }

# Seed AMCL IMMEDIATELY after it activates — the navigation stack's
# global_costmap blocks its own activation on the base_link->map transform,
# which only exists once AMCL has a pose. Seeding late deadlocks the bringup.
log "=== 3/4  seeding AMCL at ($INIT_X, $INIT_Y) ==="
for _ in 1 2 3; do
  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $INIT_X, y: $INIT_Y, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}" \
    >/dev/null 2>&1
  sleep 2
done

# Localization proof: AMCL broadcasts map->odom on /tf. (/amcl_pose is
# latched-once so late echo sees nothing; and piping echo into grep starves on
# stdout buffering — capture to a file, then grep the file.)
localized=0
t0=$SECONDS
while (( SECONDS - t0 < 60 )); do
  timeout 12 ros2 topic echo /tf > "$LOG_DIR/tf_probe.txt" 2>/dev/null
  if grep -q 'frame_id: map' "$LOG_DIR/tf_probe.txt"; then
    localized=1; break
  fi
done
[[ $localized == 1 ]] || { log "ERROR: never localized"; tail -n 40 "$LOG_DIR/nav2.log"; exit 1; }
log "localized (map frame is being broadcast)"
wait_for_active /bt_navigator 120 || { tail -n 40 "$LOG_DIR/nav2.log"; exit 1; }

if [[ "${RECORD:-0}" == "1" ]]; then
  log "=== recorder on -> $LOG_DIR/nav_record.jsonl ==="
  nohup python3 "$HERE/record_run.py" --out "$LOG_DIR/nav_record.jsonl" \
    --duration 1800 > "$LOG_DIR/recorder.log" 2>&1 < /dev/null &
  disown
fi

log "=== 4/4  navigating goals ==="
# start/end epochs let the offline renderer align goals with recorder samples
echo "goal,x,y,result,seconds,start_epoch,end_epoch" > "$RESULTS"
if [[ -n "${GOALS:-}" ]]; then
  mapfile -t GOAL_LIST <<< "$GOALS"
else
  GOAL_LIST=("${DEFAULT_GOALS[@]}")
fi

pass=0; total=0
for entry in "${GOAL_LIST[@]}"; do
  read -r gx gy label <<< "$entry"
  total=$((total + 1))
  log "goal $total: $label ($gx, $gy)"
  start=$(date +%s)
  out=$(timeout 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: $gx, y: $gy, z: 0.0}, orientation: {w: 1.0}}}}" \
        2>&1 | tail -n 20)
  secs=$(( $(date +%s) - start ))
  if grep -q "Goal finished with status: SUCCEEDED" <<< "$out"; then
    result=SUCCEEDED; pass=$((pass + 1))
  elif grep -q "ABORTED" <<< "$out";   then result=ABORTED
  elif grep -q "CANCELED" <<< "$out";  then result=CANCELED
  elif grep -q "REJECTED" <<< "$out";  then result=REJECTED
  else result=TIMEOUT; fi
  log "  -> $result in ${secs}s"
  echo "$label,$gx,$gy,$result,$secs,$start,$(date +%s)" >> "$RESULTS"
  sleep 3
done

log "=== RESULT: $pass/$total goals reached ==="
column -s, -t "$RESULTS"
log "results written to $RESULTS"
[[ $pass -eq $total ]]
