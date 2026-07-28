#!/usr/bin/env bash
# CellOps — bring up world+Nav2, seed AMCL, then dump exactly what is on /tf
# and what AMCL says, so the localization probe can be built on facts.
set -uo pipefail
LOG_DIR=/tmp/cellops
mkdir -p "$LOG_DIR"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export TURTLEBOT3_MODEL=burger
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() { echo "[$(date +%H:%M:%S)] $*"; }

_ancestors() { local p=$$; while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do echo "$p"; p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' '); done; }
safe_pkill() { local pat="$1" pid skip; skip=" $(_ancestors | tr '\n' ' ') "; for pid in $(pgrep -f "$pat" 2>/dev/null); do [[ "$skip" == *" $pid "* ]] && continue; kill "$pid" 2>/dev/null; done; }
cleanup() { safe_pkill "navigation2.launch.py"; safe_pkill "component_container|rviz2"; safe_pkill "headless_world"; safe_pkill "ros_gz_bridge|parameter_bridge"; safe_pkill "gz sim"; safe_pkill "robot_state_publisher"; sleep 2; }
trap cleanup EXIT
cleanup

log "world up"
nohup ros2 launch "$HERE/headless_world.launch.py" > "$LOG_DIR/dbg_gz.log" 2>&1 < /dev/null & disown
sleep 12

log "nav2 up"
nohup ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True map:="$HOME/maps/tb3_world.yaml" \
  > "$LOG_DIR/dbg_nav2.log" 2>&1 < /dev/null & disown

for i in $(seq 1 40); do
  state=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -n1)
  [[ "$state" == active* ]] && break
  sleep 3
done
log "amcl state: $state"

log "seeding initialpose"
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}' >/dev/null 2>&1
sleep 5

log "dumping 15s of /tf"
timeout 15 ros2 topic echo /tf > "$LOG_DIR/tf_dump.txt" 2>&1
echo "--- frame_id counts in /tf dump ---"
grep 'frame_id' "$LOG_DIR/tf_dump.txt" | sort | uniq -c | sort -rn
echo "--- /tf_static one sample ---"
timeout 10 ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable --once 2>/dev/null | grep -E 'frame_id' | head -8

echo "--- amcl-related log lines (last 15) ---"
grep -E '\[amcl\]' "$LOG_DIR/dbg_nav2.log" | tail -n 15 | tr -d '\r'

echo "--- tf2_echo map->odom, 10s, unbuffered ---"
timeout 10 stdbuf -oL ros2 run tf2_ros tf2_echo map odom 2>&1 | head -n 8

echo "--- lifecycle summary ---"
for n in /amcl /bt_navigator /planner_server /controller_server; do
  printf '%-20s %s\n' "$n" "$(timeout 6 ros2 lifecycle get $n 2>/dev/null | head -n1)"
done
log "debug done"
