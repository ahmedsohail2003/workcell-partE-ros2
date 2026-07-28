#!/usr/bin/env bash
# CellOps (WorkCell Part E) — provision WSL2 Ubuntu 24.04 with
# ROS 2 Jazzy + Gazebo Harmonic (ros_gz) + TurtleBot3 + slam_toolbox + Nav2.
#
# Run ONCE, either inside the Ubuntu window:
#   sudo bash /mnt/c/Users/sohai/robotics/cellops/scripts/wsl_setup.sh
# or non-interactively from Windows (no password needed — WSL grants root):
#   wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/Users/sohai/robotics/cellops/scripts/wsl_setup.sh
#
# Idempotent: safe to re-run if it fails partway.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

log() { echo; echo "=== $* ==="; }

[ "$(id -u)" -eq 0 ] || { echo "ERROR: run with sudo"; exit 1; }

log "Locale + base tools"
apt-get update -y
apt-get install -y locales curl software-properties-common mesa-utils
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

log "Ubuntu suites sanity check (Jazzy docs gotcha: ros-dev-tools needs noble-updates)"
SRC=/etc/apt/sources.list.d/ubuntu.sources
if [ -f "$SRC" ] && ! grep -q "noble-updates" "$SRC"; then
  echo "WARNING: $SRC lacks noble-updates/noble-backports Suites;"
  echo "ros-dev-tools may hit dependency conflicts. Inspect it manually."
fi

log "ROS 2 apt source (ros2-apt-source .deb — the only maintained method since June 2025)"
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

log "Full upgrade BEFORE installing ROS (mixed-version breakage guard)"
apt-get update -y
apt-get full-upgrade -y

log "ROS 2 Jazzy desktop + Gazebo Harmonic (ros_gz vendor pkgs) + dev tools (~4-5 GB)"
apt-get install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-dev-tools

log "TurtleBot3 sim + slam_toolbox + Nav2"
apt-get install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-navigation2 \
  ros-jazzy-turtlebot3-teleop \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup

log "ROS env for EVERY shell incl. non-interactive 'wsl -- bash -lc' (~/.bashrc is NOT read there)"
cat > /etc/profile.d/99-ros.sh <<'EOF'
# CellOps: ROS 2 Jazzy env — profile.d so login non-interactive shells get it too
if [ -n "${BASH_VERSION:-}" ]; then
  source /opt/ros/jazzy/setup.bash
fi
export TURTLEBOT3_MODEL=burger
# WSLg GPU: Mesa defaults to llvmpipe (software) on this machine; forcing the
# d3d12 gallium driver resolves to the real RTX 4050 via WSL's D3D12 layer.
# Verified with `glxinfo -B`: llvmpipe -> "D3D12 (NVIDIA GeForce RTX 4050)".
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GALLIUM_DRIVER=d3d12
EOF

log "User workspace dirs"
# When invoked via 'wsl -u root' there is no SUDO_USER; fall back to the
# WSL default user (uid 1000), not root, so dirs land in the right home.
U=${SUDO_USER:-$(getent passwd 1000 | cut -d: -f1)}
U=${U:-root}
UH=$(getent passwd "$U" | cut -d: -f6)
mkdir -p "$UH/maps" "$UH/ros2_ws/src"
chown -R "$U:$U" "$UH/maps" "$UH/ros2_ws"

log "Verify install"
set +u
source /opt/ros/jazzy/setup.bash
set -u
for pkg in turtlebot3_gazebo slam_toolbox nav2_bringup ros_gz_sim; do
  printf '%-16s -> %s\n' "$pkg" "$(ros2 pkg prefix "$pkg")"
done
gz sim --versions 2>/dev/null | head -n1 | sed 's/^/gz-sim version: /' || echo "gz version check skipped"
df -h / | tail -n1

echo
echo "=== SETUP COMPLETE ==="
