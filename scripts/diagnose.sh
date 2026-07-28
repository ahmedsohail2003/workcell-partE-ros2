#!/usr/bin/env bash
# CellOps — environment diagnostics. Run as a FILE (never as an inline
# `wsl -- bash -lc "..."` string): PowerShell expands $VARs inside double
# quotes before wsl.exe sees them, which silently corrupts inline commands.
echo "=== shell env ==="
echo "ROS_DISTRO=[${ROS_DISTRO:-UNSET}]"
echo "TURTLEBOT3_MODEL=[${TURTLEBOT3_MODEL:-UNSET}]"
echo "MESA_D3D12_DEFAULT_ADAPTER_NAME=[${MESA_D3D12_DEFAULT_ADAPTER_NAME:-UNSET}]"
echo "AMENT_PREFIX_PATH=[${AMENT_PREFIX_PATH:-UNSET}]"
echo "ros2 -> $(command -v ros2 || echo MISSING)"
echo "gz   -> $(command -v gz   || echo MISSING)"

echo
echo "=== is /etc/profile.d/99-ros.sh sourced by this shell? ==="
if [ -f /etc/profile.d/99-ros.sh ]; then
  echo "file exists"
  bash -lc 'echo "  nested login shell ROS_DISTRO=[${ROS_DISTRO:-UNSET}]"'
else
  echo "file MISSING"
fi

echo
echo "=== manual source test ==="
if source /opt/ros/jazzy/setup.bash 2>/tmp/source_err.txt; then
  echo "source OK -> ROS_DISTRO=[${ROS_DISTRO:-UNSET}]"
else
  echo "source FAILED:"; cat /tmp/source_err.txt
fi

echo
echo "=== GPU / rendering ==="
echo "WSL driver dir:"
ls /usr/lib/wsl/lib/ 2>/dev/null | head -20 || echo "  /usr/lib/wsl/lib MISSING"
echo "ld.so.conf.d wsl entry:"
cat /etc/ld.so.conf.d/ld.wsl.conf 2>/dev/null || echo "  no ld.wsl.conf"
echo "d3d12 gallium driver present:"
ls /usr/lib/x86_64-linux-gnu/dri/ 2>/dev/null | grep -Ei 'd3d12|zink|dzn' || echo "  no d3d12/zink dri module found"
echo "default renderer:"
glxinfo -B 2>/dev/null | grep -E 'Device:|OpenGL renderer' || echo "  glxinfo failed"
echo "forced GALLIUM_DRIVER=d3d12:"
GALLIUM_DRIVER=d3d12 glxinfo -B 2>&1 | grep -E 'Device:|OpenGL renderer|Error|error' | head -5
echo "vulkan (dzn) check:"
command -v vulkaninfo >/dev/null && (vulkaninfo --summary 2>/dev/null | grep -E 'deviceName|driverName' | head -4) || echo "  vulkaninfo not installed"

echo
echo "=== display ==="
echo "DISPLAY=[${DISPLAY:-UNSET}] WAYLAND_DISPLAY=[${WAYLAND_DISPLAY:-UNSET}]"
ls -d /mnt/wslg 2>/dev/null && echo "  wslg mount present" || echo "  NO /mnt/wslg"
