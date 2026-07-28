#!/usr/bin/env bash
# CellOps (WorkCell Part E) — one-command SLAM mapping session.
#
# Launches the TurtleBot3 Gazebo world (headless, with bad-start retries) +
# slam_toolbox, drives the robot autonomously (explore_drive.py), then saves
# and sanity-checks the occupancy map.
#
# Usage (from Windows):
#   wsl -d Ubuntu-24.04 -- bash -l /mnt/c/.../scripts/run_mapping.sh [DRIVE_SECONDS]
# Env: RECORD=1 -> also record run data for render_gif.py
set -uo pipefail

DRIVE_SECONDS="${1:-240}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="$HOME/maps"
MAP_NAME="tb3_world"
LOG_DIR="/tmp/cellops"

# ROS setup.bash reads unbound vars — relax -u just for the source.
set +u; source /opt/ros/jazzy/setup.bash; set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
source "$HERE/lib_common.sh"
mkdir -p "$MAP_DIR" "$LOG_DIR"

cleanup() {
  log "tearing down (gz + ros nodes)"
  safe_pkill "record_run.py"
  safe_pkill "explore_drive.py"
  safe_pkill "slam_toolbox"
  teardown_world
}
trap cleanup EXIT
cleanup   # clean slate: kill leftovers from any previous run

log "=== 1/5  Gazebo: turtlebot3_world (model=$TURTLEBOT3_MODEL, headless) ==="
launch_world "$LOG_DIR/gazebo.log" || exit 1

log "=== 2/5  slam_toolbox (online_async, use_sim_time) ==="
nohup ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  > "$LOG_DIR/slam.log" 2>&1 < /dev/null &
disown
wait_for_topic /map 120 || { log "slam_toolbox failed — see $LOG_DIR/slam.log"; tail -n 30 "$LOG_DIR/slam.log"; exit 1; }

if [[ "${RECORD:-0}" == "1" ]]; then
  log "=== recorder on -> $LOG_DIR/slam_record.jsonl ==="
  nohup python3 "$HERE/record_run.py" --out "$LOG_DIR/slam_record.jsonl" \
    --duration $((DRIVE_SECONDS + 120)) > "$LOG_DIR/recorder.log" 2>&1 < /dev/null &
  disown
fi

log "=== 3/5  exploring for ${DRIVE_SECONDS}s ==="
python3 "$HERE/explore_drive.py" --duration "$DRIVE_SECONDS" 2>&1 | tee "$LOG_DIR/drive.log"
drive_rc=${PIPESTATUS[0]}
if [[ $drive_rc -ne 0 ]]; then
  log "ERROR: explore driver exited $drive_rc (sim freeze watchdog?) — aborting run"
  exit 1
fi

log "=== 4/5  saving map -> $MAP_DIR/$MAP_NAME ==="
# Must run while slam_toolbox is still alive: map_saver subscribes to /map.
# The default save_map_timeout (2 s) is SHORTER than slam_toolbox's map publish
# interval — whether the save works is then a coin flip on phase. Widen the
# window and retry; and verify FRESHNESS below (a stale .pgm from a previous
# run otherwise passes every existence check).
run_start_epoch=$(date +%s)
saved=0
for attempt in 1 2 3; do
  if ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" \
       --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0 \
       2>&1 | tee "$LOG_DIR/map_saver.log" | grep -q "Map saved successfully"; then
    saved=1; break
  fi
  log "map_saver attempt $attempt failed, retrying"
  sleep 3
done
[[ $saved == 1 ]] || { log "ERROR: map_saver failed 3x"; exit 1; }

log "=== 5/5  result ==="
pgm_mtime=$(stat -c %Y "$MAP_DIR/$MAP_NAME.pgm" 2>/dev/null || echo 0)
if (( pgm_mtime < run_start_epoch - 30 )); then
  log "ERROR: $MAP_NAME.pgm is STALE (mtime predates this run) — save silently failed"
  exit 1
fi
if [[ -f "$MAP_DIR/$MAP_NAME.pgm" && -f "$MAP_DIR/$MAP_NAME.yaml" ]]; then
  ls -lh "$MAP_DIR/$MAP_NAME".{pgm,yaml}
  # Occupancy breakdown straight from the PGM: a map that is ~all-unknown means
  # SLAM ran but saw nothing (bad QoS / no scans), which otherwise looks like success.
  python3 - "$MAP_DIR/$MAP_NAME.pgm" <<'PY'
import sys
path = sys.argv[1]
with open(path, 'rb') as f:
    data = f.read()
parts, idx = [], 0
while len(parts) < 4:
    while idx < len(data) and data[idx:idx+1].isspace():
        idx += 1
    if data[idx:idx+1] == b'#':
        while idx < len(data) and data[idx] != 0x0A:
            idx += 1
        continue
    start = idx
    while idx < len(data) and not data[idx:idx+1].isspace():
        idx += 1
    parts.append(data[start:idx])
idx += 1
w, h = int(parts[1]), int(parts[2])
px = data[idx:idx + w * h]
free = sum(1 for b in px if b > 250)
occ = sum(1 for b in px if b < 5)
unknown = len(px) - free - occ
print(f"map {w}x{h}: free={free} ({100*free/len(px):.1f}%) "
      f"occupied={occ} ({100*occ/len(px):.1f}%) unknown={unknown} ({100*unknown/len(px):.1f}%)")
print("VERDICT:", "map looks substantive" if free > 0.05 * len(px)
      else "SUSPICIOUS - almost nothing mapped")
PY
  log "MAPPING COMPLETE"
else
  log "ERROR: map files not written"
  exit 1
fi
