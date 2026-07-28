#!/usr/bin/env bash
# CellOps — shared orchestration helpers, sourced by run_mapping.sh / run_nav2.sh.
#
# Hard-won rules encoded here (each one cost a debugging session):
#  * `pkill -f` matches this script's own path — safe_pkill skips self+ancestors.
#  * Topic waits must count WALL CLOCK, not loop iterations: each probe can take
#    ~11 s (discovery + echo timeout), so an iteration-counted "120 s" wait
#    silently becomes 8 minutes.
#  * The TB3 sim under WSL is intermittently flaky at startup (robot spawns,
#    bridge up, yet a topic like /odom never carries data). Detect early
#    (/scan AND /odom are HARD gates) and relaunch instead of limping on —
#    a missing /odom guarantees SLAM produces nothing 2 minutes later.

log() { echo "[$(date +%H:%M:%S)] $*"; }

_ancestors() {
  local p=$$
  while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do
    echo "$p"
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  done
}

safe_pkill() {
  local pat="$1" pid skip
  skip=" $(_ancestors | tr '\n' ' ') "
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    [[ "$skip" == *" $pid "* ]] && continue
    kill "$pid" 2>/dev/null
  done
}

teardown_world() {
  safe_pkill "headless_world"
  safe_pkill "ros_gz_bridge|parameter_bridge"
  safe_pkill "gz sim"
  safe_pkill "robot_state_publisher"
  sleep 2
}

# wait_for_topic <topic> <timeout_s> — true once the topic exists AND delivers
# a message. Wall-clock timed via $SECONDS.
wait_for_topic() {
  local topic="$1" timeout="${2:-90}" t0=$SECONDS
  log "waiting for $topic (timeout ${timeout}s)"
  while (( SECONDS - t0 < timeout )); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      if timeout 8 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
        log "  $topic is live ($((SECONDS - t0))s)"
        return 0
      fi
    fi
    sleep 3
  done
  log "ERROR: $topic never became live (${timeout}s)"
  return 1
}

# wait_for_active <lifecycle_node> <timeout_s>
wait_for_active() {
  local node="$1" timeout="${2:-120}" t0=$SECONDS state=""
  log "waiting for $node to be active (timeout ${timeout}s)"
  while (( SECONDS - t0 < timeout )); do
    state=$(timeout 8 ros2 lifecycle get "$node" 2>/dev/null | head -n1)
    if [[ "$state" == active* ]]; then
      log "  $node is active ($((SECONDS - t0))s)"
      return 0
    fi
    sleep 3
  done
  log "ERROR: $node never became active (last state: ${state:-none})"
  return 1
}

# launch_world <logfile> [attempts=3] — start the headless TB3 world and verify
# BOTH /scan and /odom deliver data; teardown + relaunch on a bad start.
launch_world() {
  local logfile="$1" attempts="${2:-3}" here
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  for a in $(seq 1 "$attempts"); do
    log "launching headless TB3 world (attempt $a/$attempts)"
    nohup ros2 launch "$here/headless_world.launch.py" \
      > "$logfile" 2>&1 < /dev/null &
    disown
    if wait_for_topic /scan 90 && wait_for_topic /odom 45; then
      log "world is up (scan + odom live)"
      return 0
    fi
    log "bad world start — tearing down for retry"
    tail -n 15 "$logfile" | sed 's/^/    /'
    teardown_world
  done
  log "ERROR: world failed to start after $attempts attempts"
  return 1
}
