#!/usr/bin/env bash
# CellOps — build the colcon workspace inside WSL.
#
# Sources live in the repo on /mnt/c (canonical, git-tracked); building there
# is slow through the 9P mount, so rsync them to ext4 (~/ros2_ws) and build
# natively. Usage:
#   wsl -d Ubuntu-24.04 -- bash -l /mnt/c/Users/sohai/robotics/cellops/scripts/build_ws.sh
set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; set -u

REPO=/mnt/c/Users/sohai/robotics/cellops
WS="$HOME/ros2_ws"

echo "=== sync sources -> $WS/src ==="
mkdir -p "$WS/src"
rsync -a --delete "$REPO/ros2_ws/src/" "$WS/src/"

echo "=== colcon build (Release) ==="
cd "$WS"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -n 20
build_rc=${PIPESTATUS[0]}
if [[ $build_rc -ne 0 ]]; then
  echo "BUILD FAILED (rc=$build_rc) — dumping last errors:"
  grep -RhoE 'error[: ].*' log/latest_build 2>/dev/null | sort -u | tail -n 30
  exit "$build_rc"
fi

echo "=== smoke: executables exist ==="
set +u; source "$WS/install/setup.bash"; set -u
ls -l "$WS"/install/cellops_grasp/lib/cellops_grasp/
ros2 interface show cellops_interfaces/srv/EstimateCubePose | head -n 12
echo "=== BUILD OK ==="
