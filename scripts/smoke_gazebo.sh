#!/usr/bin/env bash
# CellOps — first-light test: does the TurtleBot3 world actually run under WSLg,
# and does the ros_gz bridge produce live sensor data?
#
# Answers the three questions the whole project depends on:
#   1. does gz sim start at all (OGRE2 under WSLg is the known crash risk)
#   2. is /scan publishing real laser data
#   3. does the robot respond to TwistStamped on /cmd_vel (odom must change)
set -uo pipefail

LOG_DIR=/tmp/cellops
mkdir -p "$LOG_DIR"
# ROS's setup.bash reads unbound vars (AMENT_TRACE_SETUP_FILES) — it aborts
# under `set -u`, so relax the option just for the source.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL=burger

log() { echo "[$(date +%H:%M:%S)] $*"; }
cleanup() {
  pkill -f turtlebot3_world 2>/dev/null
  pkill -f "ros_gz_bridge|parameter_bridge" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  sleep 2
}
trap cleanup EXIT
cleanup

log "launching turtlebot3_world (GALLIUM_DRIVER=${GALLIUM_DRIVER:-unset})"
nohup ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
  > "$LOG_DIR/smoke_gazebo.log" 2>&1 < /dev/null &
disown

log "waiting up to 120s for /scan to publish"
ok=0
for i in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -qx /scan; then
    if timeout 8 ros2 topic echo /scan --once > "$LOG_DIR/scan_sample.txt" 2>&1; then
      ok=1; break
    fi
  fi
  sleep 3
done

if [ "$ok" -ne 1 ]; then
  log "FAIL: /scan never published. Last 40 log lines:"
  tail -n 40 "$LOG_DIR/smoke_gazebo.log"
  exit 1
fi
log "PASS: /scan is live"

echo
log "topics:"
ros2 topic list | sed 's/^/    /'

echo
log "scan sanity (finite ranges mean the lidar sees the world):"
python3 - "$LOG_DIR/scan_sample.txt" <<'PY'
import re, sys
txt = open(sys.argv[1]).read()
m = re.search(r'ranges:\s*\n((?:\s*-\s*[^\n]+\n)+)', txt)
if not m:
    print("    could not parse ranges"); raise SystemExit
vals = []
for line in m.group(1).splitlines():
    v = line.strip().lstrip('-').strip()
    try: vals.append(float(v))
    except ValueError: pass
finite = [v for v in vals if v == v and v not in (float('inf'),) and 0 < v < 100]
print(f"    {len(vals)} beams, {len(finite)} finite")
if finite:
    print(f"    min={min(finite):.3f} m  max={max(finite):.3f} m")
print("    VERDICT:", "lidar sees geometry" if len(finite) > 10 else "SUSPICIOUS - almost no returns")
PY

echo
log "odom before motion:"
timeout 8 ros2 topic echo /odom --once 2>/dev/null | grep -A3 'position:' | head -4

log "commanding TwistStamped forward for 3s"
timeout 12 ros2 topic pub -r 10 -t 30 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.15}}}' >/dev/null 2>&1
timeout 8 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/TwistStamped '{}' >/dev/null 2>&1
sleep 1

log "odom after motion (x should have increased ~0.4m):"
timeout 8 ros2 topic echo /odom --once 2>/dev/null | grep -A3 'position:' | head -4

echo
log "SMOKE TEST DONE"
