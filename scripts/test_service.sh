#!/usr/bin/env bash
# CellOps — round-trip test of the C++ grasp service over ROS 2.
set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/ros2_ws/install/setup.bash"; set -u
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib_common.sh"

cleanup() { safe_pkill "grasp_service_node"; }
trap cleanup EXIT
cleanup

log "starting C++ grasp_service_node"
nohup ros2 run cellops_grasp grasp_service_node > /tmp/cellops/service.log 2>&1 < /dev/null &
disown
sleep 4

rc_all=0
for i in 0 5 11; do
  log "--- client call, cloud $i ---"
  python3 /mnt/c/Users/sohai/robotics/cellops/scripts/grasp_client.py \
    /mnt/c/Users/sohai/robotics/cellops/testdata "$i" || rc_all=1
done

log "--- service log ---"
grep -E 'solved|ready' /tmp/cellops/service.log | tr -d '\r'
exit $rc_all
