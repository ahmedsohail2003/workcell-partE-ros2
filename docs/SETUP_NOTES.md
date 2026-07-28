# Setup notes — verified install plan (research pass, 2026-07-28)

Internal working notes; condense before publishing.

# ROS2 Jazzy + Gazebo Harmonic + TurtleBot3 SLAM/Nav2 under WSL2 — Merged Install & Run Plan

Target: Windows 11 Home 25H2 (build 26200), 16 GB RAM, RTX 4050 6 GB, ~42 GB free disk. WSL is **not yet installed** (verified live on this machine). All Linux steps driven non-interactively via `wsl.exe`. Route chosen end-to-end: **ROBOTIS TurtleBot3 stack (apt) + slam_toolbox + turtlebot3_navigation2** — do NOT mix with the Nav2-native `tb3_simulation_launch.py` ecosystem (different cmd_vel semantics, see Gotcha #10).

---

## 1. WSL2 install + .wslconfig

```powershell
# 1. (ADMIN PowerShell) first-time install — expect UAC + REBOOT prompt
wsl --install -d Ubuntu-24.04

# 2. after reboot: Ubuntu console auto-launches -> create Linux username/password interactively (one-time, cannot be scripted)

# 3. verify modern WSL (need >= 2.4.10 for the tar-based Ubuntu-24.04 image; also shows WSLg + Direct3D versions)
wsl --version
wsl --status

# 4. write global config (non-admin PowerShell)
Set-Content -Path "$env:USERPROFILE\.wslconfig" -Encoding ascii -Value @'
[wsl2]
memory=8GB
swap=8GB

[experimental]
autoMemoryReclaim=dropCache
sparseVhd=true
'@
wsl --shutdown   # then wait ~8s before next wsl command so config applies

# 5. make wsl.exe's own messages parseable from scripts (modern WSL only; pre-install stub ignores it — verified)
$env:WSL_UTF8='1'   # per-session; or setx WSL_UTF8 1 once
```

**Conflict flagged — memory sizing:** one researcher suggested `memory=12GB`; another (with the full MS wsl-config doc, ms.date 2026-04-15) recommends `memory=8GB` (the default 50%, made explicit) + `swap=8GB`. **Pick 8GB** — 12GB would starve Windows-side recording tools on a 16 GB machine, and 4–6 GB is the realistic gz+Nav2+rviz2 footprint. Set `sparseVhd=true` **before** installing ROS (VHDX never auto-shrinks; ~42 GB free disk).

Post-install plumbing (run once, in order):

```powershell
# base update + GPU sanity check
wsl -d Ubuntu-24.04 -- bash -lc "sudo apt-get update && sudo apt-get full-upgrade -y && sudo apt-get install -y mesa-utils"
wsl -d Ubuntu-24.04 -- bash -lc "glxinfo -B | grep -E 'renderer|OpenGL version'"   # want: D3D12 (NVIDIA GeForce RTX 4050...) — llvmpipe = passthrough broken

# pin the NVIDIA adapter (Optimus laptop) — profile.d so bash -lc sees it
wsl -d Ubuntu-24.04 -- bash -lc "echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' | sudo tee /etc/profile.d/98-wslg-gpu.sh"

# exit-code propagation self-test (expect 42)
wsl -d Ubuntu-24.04 -- bash -lc "exit 42"; echo $LASTEXITCODE
```

Ubuntu-24.04 lands at `%LOCALAPPDATA%\wsl\{guid}\ext4.vhdx` (new tar-based format, NOT the old Packages\Canonical... path). systemd is on by default (nothing to configure; ROS doesn't need it anyway).

---

## 2. ROS 2 Jazzy + Gazebo Harmonic apt setup (current 2026 method)

The **only** maintained path is the `ros2-apt-source` .deb (current release 1.2.0, 2026-04-23). Any tutorial with `curl ... ros.key` + manual `ros2.list` has been dead since the June 2025 GPG key expiration. Jazzy is LTS (supported to May 2029), latest patch release jazzy/2026-06-18, no outstanding repo-wide regressions.

```bash
# locale
locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# repos
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl

# ros2-apt-source .deb — the export and the curl MUST run in the SAME bash -lc invocation
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Ubuntu 24.04 Suites check (documented Jazzy gotcha) — must include noble-updates noble-backports before ros-dev-tools
grep Suites /etc/apt/sources.list.d/ubuntu.sources

# upgrade BEFORE installing ROS (docs admonition — stale base = mixed-version breakage)
sudo apt update && sudo apt upgrade -y

# install: desktop does NOT include Gazebo; desktop + ros-gz is the lean recommended combo
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-jazzy-ros-gz     # pulls Gazebo Harmonic as gz vendor packages from packages.ros.org — NO osrfoundation repo needed
sudo apt install -y ros-dev-tools

source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker  # smoke test (pair with: ros2 run demo_nodes_py listener)
```

If the Suites check lacks the entries: edit to `Suites: noble noble-updates noble-backports` then `sudo apt clean && sudo apt update && sudo apt full-upgrade -y`.

Then make sourcing work for non-interactive `bash -lc` (**NOT** ~/.bashrc — see Gotcha #2):

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "echo 'source /opt/ros/jazzy/setup.bash' | sudo tee /etc/profile.d/99-ros.sh"
wsl -d Ubuntu-24.04 -- bash -lc "printenv MESA_D3D12_DEFAULT_ADAPTER_NAME; ros2 --help >/dev/null && echo ROS_OK"
```

Also persist `export TURTLEBOT3_MODEL=burger` in a profile.d file (e.g. append to `/etc/profile.d/99-ros.sh`) — it is required in *every* shell (Section 3).

Non-interactive driving: docs commands omit `-y`; always add it (and `DEBIAN_FRONTEND=noninteractive` for the big installs) or apt hangs.

---

## 3. TurtleBot3 packages — apt (source build NOT required)

All five ROBOTIS packages are RELEASED for Jazzy at **2.3.7** (verified on index.ros.org): `turtlebot3`, `turtlebot3_simulations`, `turtlebot3_gazebo`, `turtlebot3_cartographer`, `turtlebot3_navigation2`. The jazzy branch is fully ported to new Gazebo (gz-sim8/Harmonic) via ros_gz — no gazebo_ros anywhere. The ROBOTIS e-manual still shows a source build (`git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git` + `colcon build --symlink-install`) but that predates the deb release; apt 2.3.7 == the `jazzy` branch (if you ever do build from source, the branch is **`jazzy`**, not `new-gazebo`).

```bash
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-cartographer ros-jazzy-turtlebot3-navigation2 ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-navigation2 ros-jazzy-turtlebot3-teleop ros-jazzy-teleop-twist-keyboard ros-jazzy-ros-gz
```

Key facts:
- `export TURTLEBOT3_MODEL=burger` is **required** (spawn + robot_state_publisher launch files do `os.environ['TURTLEBOT3_MODEL']` — KeyError crash if unset). Burger is the lightest choice for this hardware.
- No `GZ_SIM_RESOURCE_PATH` export needed — `turtlebot3_world.launch.py` sets it itself.
- Bridge (`turtlebot3_burger_bridge.yaml`): clock, joint_states, odom, tf (carries odom->base_footprint), scan, imu GZ→ROS; **cmd_vel ROS→GZ as `geometry_msgs/msg/TwistStamped`** (not Twist!).
- `use_sim_time` defaults true in the world launch; /clock is bridged.

---

## 4. SLAM mapping session

**SLAM engine choice (flagged, not a true conflict):** the e-manual default is Cartographer (`ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True`, released for Jazzy 2.0.9003), but both researchers who examined it recommend **slam_toolbox** (2.8.5, actively maintained, Nav2's official tutorial choice, jazzy defaults already match TB3: `/scan`, `base_footprint`, `odom`, mode mapping — zero param overrides needed). **Use slam_toolbox.**

```powershell
# ---- terminal A: TB3 world in Gazebo (server runs headless -s; GUI is a separate -g client) ----
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'

# ---- terminal B: SLAM ----
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true'
```

For orchestration, background long-running launches with the full redirect pattern (they block the `wsl.exe` call otherwise):

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "nohup ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py >/tmp/gz.log 2>&1 </dev/null & disown; echo started"
```

**Non-interactive driving — MUST be TwistStamped** (plain Twist silently does nothing on Jazzy TB3):

```powershell
# 5 s forward
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic pub -r 10 -t 50 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.15}}}"'
# 3 s turn
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic pub -r 10 -t 30 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {angular: {z: 0.5}}}"'
# explicit stop (gz diff-drive keeps executing the last command)
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic pub -1 /cmd_vel geometry_msgs/msg/TwistStamped "{}"'
```

Interactive alternatives (need a real TTY — cannot run through `wsl -- bash -lc` pipes): `ros2 run turtlebot3_teleop teleop_keyboard` (publishes TwistStamped on jazzy) or `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true`.

**Save the map (while the SLAM node is still alive — map_saver subscribes to /map):**

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 run nav2_map_server map_saver_cli -f $HOME/maps/tb3_world --ros-args -p use_sim_time:=true'
# optional: slam_toolbox pose-graph serialization (for continued mapping only — NOT loadable by AMCL/map_server)
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: \"$HOME/maps/tb3_world_posegraph\"}"'
```

---

## 5. Nav2 on the saved map

Kill SLAM first, keep the Gazebo world running.

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/tb3_world.yaml'
```

AMCL will **not** localize until it gets an initial pose (`set_initial_pose: false` in the jazzy params). Non-interactive version (TB3 world spawn = x=-2.0, y=-0.5, verified from the launch file):

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"'
```

Send a goal from CLI (action interface = scriptable, gives result/feedback):

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.5}, orientation: {w: 1.0}}}}" --feedback'
```

Topic alternative (same path rviz2's Nav2 Goal button uses — bt_navigator subscribes to hardcoded `goal_pose`):

```
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.5}, orientation: {w: 1.0}}}'
```

Interactive alternative: rviz2 2D Pose Estimate + Nav2 Goal (rviz2 launches with `navigation2.launch.py`).

Teardown: `wsl -d Ubuntu-24.04 -- bash -lc "pkill -f 'ros2|gz sim' || true"`

*(Nav2-native alternative, only if wanted separately: `sudo apt install 'ros-jazzy-nav2-minimal-tb*' && ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True` — its own sandbox world/map, waffle model, unstamped Twist. Do not mix with the ROBOTIS route.)*

---

## 6. Demo recording under WSLg

**Conflict flagged:** one researcher proposed Xbox Game Bar (Win+Alt+R) as the simplest capture; another **verified** Game Bar cannot record WSLg windows — they're hosted by msrdc (RDP/RAIL) and Game Bar reports "this game doesn't support recording" (open request microsoft/wslg#1037). **Game Bar is out.** Linux-side Wayland recorders (wf-recorder, grim, kooha) also don't work (WSLg's Weston-RDP compositor exposes no wlr-screencopy or portal).

What works, in order of preference:
1. **ScreenToGif** (Windows) — region capture straight to editable GIF; ideal for portfolio GIFs.
2. **Windows 11 Snipping Tool recorder** (Win+Shift+R, region → mp4).
3. **OBS Studio display-capture** + crop (window-capture of msrdc windows may be flaky — use display capture).
4. **ffmpeg gdigrab** region capture + two-pass GIF palette:

```powershell
ffmpeg -f gdigrab -framerate 30 -offset_x 100 -offset_y 100 -video_size 1280x720 -i desktop -t 30 demo.mp4
ffmpeg -i demo.mp4 -vf 'fps=12,scale=960:-1:flags=lanczos,palettegen' palette.png
ffmpeg -i demo.mp4 -i palette.png -filter_complex 'fps=12,scale=960:-1:flags=lanczos[x];[x][1:v]paletteuse' demo.gif
```

No-live-capture alternative (robust for headless runs): bag-record, replay later into rviz2, record the replay:

```powershell
wsl -d Ubuntu-24.04 -- bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 bag record -o navdemo /tf /tf_static /map /scan /amcl_pose /plan /cmd_vel /goal_pose'
# later: ros2 bag play --clock + rviz2 (nav2_bringup rviz_launch.py has the preconfigured Nav2 view)
```

Also unverified-but-promising: Gazebo GUI's built-in VideoRecorder plugin (records the 3D viewport to mp4 from inside gz sim — sidesteps screen capture for the Gazebo half).

---

## 7. GOTCHAS (deduped, ranked by likelihood of biting us)

1. **cmd_vel is TwistStamped, not Twist** — Jazzy TB3 bridge takes `geometry_msgs/msg/TwistStamped`; `turtlebot3_navigation2` sets `enable_stamped_cmd_vel: true` everywhere. Plain Twist (generic teleop_twist_keyboard, naive `ros2 topic pub`) **silently does nothing** — robot never moves, no error. This will hit our non-interactive driving first.
2. **`~/.bashrc` is useless for `wsl -- bash -lc`** — it's a login *non-interactive* shell; Ubuntu's .bashrc returns early. The classic `echo source ... >> ~/.bashrc` silently fails → "ros2: command not found". Use `/etc/profile.d/99-ros.sh` or source inline in every command string.
3. **`TURTLEBOT3_MODEL` must exist in every shell** — hard KeyError crash in spawn/robot_state_publisher launch files. Every `wsl -- bash -lc` is a fresh shell; persist via profile.d and/or export inline.
4. **Env vars don't persist across separate `wsl -- bash -lc` calls** — notably `ROS_APT_SOURCE_VERSION` export + the curl download must be one invocation.
5. **Background processes holding the wsl.exe pipe block or die** — always `nohup ... >log 2>&1 </dev/null & disown`. VM shuts down ~60 s (vmIdleTimeout) after the last Linux process exits; `wsl -d Ubuntu-24.04 --exec dbus-launch true` pins it when idle.
6. **apt hangs on prompts non-interactively** — docs omit `-y`; add it plus `DEBIAN_FRONTEND=noninteractive` for big installs.
7. **AMCL needs an initial pose** (`set_initial_pose: false`) — Nav2 sits dead on a saved map until /initialpose is published (spawn = -2.0, -0.5) or amcl params overridden.
8. **Gazebo GUI segfault risk under WSLg** — turtlebot3_simulations issue #247 (open): OGRE2/EGL segfault without working GPU accel; also gz-sim#2670 black-screen and #2873 mesh memory leak on WSL2. Server is already headless (`-s`); demo can run server-only. Fallback: `LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb`; on Optimus set `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`; never set `LIBGL_ALWAYS_INDIRECT`; verify with `glxinfo -B` (llvmpipe = passthrough broken → update *Windows* NVIDIA driver, never install a Linux driver in WSL).
9. **Xbox Game Bar cannot record WSLg windows** (wslg#1037); Linux Wayland recorders don't work either. Plan recording per Section 6.
10. **Don't mix ecosystems** — ROBOTIS route (stamped cmd_vel, TB3 worlds) vs Nav2-native `tb3_simulation_launch.py` (unstamped, own sandbox, needs `ros-jazzy-nav2-minimal-tb*`, ignores turtlebot3_gazebo). Pick one end-to-end. `tb3_loopback_simulation` is not Gazebo at all (no physics/sensors).
11. **Old-style apt key setup is dead** (GPG key expired June 2025) — only the ros2-apt-source .deb works; it auto-rotates keys via apt upgrade.
12. **`ros-jazzy-desktop` does NOT include Gazebo** — need `ros-jazzy-ros-gz`; and do NOT add packages.osrfoundation.org for Jazzy+Harmonic (vendor packages come from packages.ros.org; mixing repos risks conflicting duplicate Gazebo libs).
13. **Ubuntu 24.04 Suites gotcha** — if `ubuntu.sources` lacks noble-updates/noble-backports, `ros-dev-tools` hits dependency conflicts (documented in Jazzy docs). Check first; run `sudo apt upgrade` *before* installing ros-jazzy-*.
14. **Map saving traps** — `map_saver_cli` must run while SLAM is alive; pass `-p use_sim_time:=true`; slam_toolbox `.posegraph/.data` is NOT loadable by AMCL/map_server (need the .pgm/.yaml); thresholds sometimes need `--occ 0.65 --free 0.25`.
15. **Disk pressure** — VHDX grows (~12–20 GB for the full stack incl. Ubuntu base; ROS desktop+ros-gz alone ~4–5 GB apt-installed [community ballpark, undocumented]) and never auto-shrinks; swap.vhdx adds up to `swap=` on top. sparseVhd from day one; ~42 GB free is enough but not roomy.
16. **wsl.exe's own output is UTF-16LE** (verified locally; the pre-install stub even ignores WSL_UTF8) — set `WSL_UTF8=1` after modern WSL installs, or `[Console]::OutputEncoding = [System.Text.Encoding]::Unicode`, before parsing `wsl --status/--list`. Linux process output is normal UTF-8.
17. **The 8-second rule** — .wslconfig edits apply only after full VM stop (`wsl --shutdown`, wait, relaunch); an immediate relaunch silently uses stale settings.
18. **teleop needs a TTY** — keyboard teleop can't run through non-interactive pipes; use the TwistStamped `topic pub` burst pattern (and always publish an explicit zero-stop).
19. **Standalone `robot_state_publisher.launch.py` defaults `use_sim_time` false** — fine when launching the world file (which passes it through); only a trap if launched standalone.
20. **docs.ros.org is behind Anubis anti-bot** in 2026 — scripted lookups should use raw.githubusercontent.com mirrors (ros2/ros2_documentation jazzy branch).
21. **Harmless log noise** — `gz_frame_id element not defined` SDF warnings, QT binding-loop warnings; also open sim-fidelity issues (#255–258: no accel limiting, odom inconsistencies, z-lift in reverse) are cosmetic, not blockers.

---

## 8. UNCERTAINTIES needing live verification once WSL is up

1. **Exit-code propagation** — `wsl -d Ubuntu-24.04 -- bash -lc "exit 42"; echo $LASTEXITCODE` must print 42 before orchestration scripts trust it (only stub codes 1/50 verified pre-install).
2. **GPU passthrough actually works** — `glxinfo -B` must show `D3D12 (NVIDIA GeForce RTX 4050...)`; a reported Ubuntu 24.04+NVIDIA llvmpipe failure mode exists; fix is per-machine (Windows driver update).
3. **Gazebo GUI vs segfault #247 on this machine** — whether the Harmonic GUI renders under WSLg on the NVIDIA path or we must go headless/software; also whether rviz2 needs `LIBGL_ALWAYS_SOFTWARE=1` for meshes (wslg#554).
4. **Whether zero-stamp TwistStamped from `ros2 topic pub` is accepted** by the ros_gz bridge (inference from bridge config — stamp appears ignored, but untested).
5. **`teleop_twist_keyboard -p stamped:=true`** — README documents the param, but its presence in the jazzy 2.4.1 *binary* is unconfirmed (fallback: turtlebot3_teleop, which is confirmed TwistStamped).
6. **Recording toolchain specifics** — ffmpeg gdigrab against msrdc windows by title, Snipping Tool recorder output quality, ScreenToGif behavior on WSLg windows, Gazebo Harmonic GUI VideoRecorder plugin availability, x11grab against Xwayland `DISPLAY=:0`. All plausible, none verified.
7. **Distro installs after initial WSL enablement need no admin/reboot** — consistent with docs wording, not explicitly stated.
8. **Actual disk usage** of the full stack in the VHDX (estimates 12–20 GB are undocumented) and whether `wsl --manage Ubuntu-24.04 --set-sparse true` is available in the installed WSL version.
9. **fuel.gazebosim.org access on first world load** — TB3 models ship in the deb, but ground plane/sun may attempt a Fuel fetch (expect short stall, not failure, if offline — unverified).
10. **Background persistence with systemd enabled** — an old (2023) report said systemd broke background-process VM pinning; considered fixed in current WSL, verify the nohup pattern survives `wsl.exe` returning.
11. **`ros2 bag play --clock` nuances** with rviz2 use_sim_time for the replay-recording pipeline.
12. **map_saver default thresholds** on the tb3_world map — check the .pgm before wiring Nav2; adjust `--occ/--free` if needed.