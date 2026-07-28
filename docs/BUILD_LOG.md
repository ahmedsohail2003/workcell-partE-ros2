# Build log — CellOps (WorkCell Part E)

Chronological engineering log. Newest entries at the bottom.

## 2026-07-28 — kickoff

- Machine recon: Windows 11, ~48 GB free disk (enough for WSL2 + ROS 2 stack,
  est. 12–15 GB), WSL2 **not yet installed**.
- Decision: ROS 2 **Jazzy Jalisco** on **Ubuntu 24.04 (WSL2)** with the paired
  **Gazebo (Harmonic)** — not Gazebo Classic, which is EOL.
- Repo scaffolded on the Windows side (`robotics/cellops`) as a staging area;
  the colcon workspace will live inside the WSL filesystem for build speed.
- Research pass launched to pin down 2026-current install specifics
  (ros-apt-source method, TurtleBot3 Jazzy deb availability vs. source build,
  WSLg rendering path for OGRE2/RViz2, non-interactive `wsl.exe` driving).

## 2026-07-28 — research pass complete → install plan locked

Full verified plan in [SETUP_NOTES.md](SETUP_NOTES.md). Decisions and key findings:

- **TurtleBot3 needs NO source build**: all five ROBOTIS packages released for
  Jazzy at 2.3.7 (apt), fully ported to new Gazebo (gz-sim8/Harmonic) via
  `ros_gz` — the e-manual's source-build instructions are outdated.
- **SLAM engine: `slam_toolbox`** (not the e-manual's Cartographer) — Nav2's
  official choice, and its Jazzy defaults already match TB3 (`/scan`,
  `base_footprint`, `odom`) with zero param overrides.
- **`cmd_vel` is `TwistStamped` on Jazzy TB3** — plain `Twist` silently does
  nothing (bridge + `enable_stamped_cmd_vel: true`). All scripted driving must
  publish TwistStamped and end with an explicit zero-stop.
- **Old apt-key setup is dead** (GPG key expired June 2025); only the
  `ros2-apt-source` .deb works. Gazebo Harmonic comes as vendor packages from
  packages.ros.org via `ros-jazzy-ros-gz` — do NOT add the osrfoundation repo.
- **Non-interactive driving contract**: `wsl -d Ubuntu-24.04 -- bash -lc` is a
  login *non-interactive* shell → `~/.bashrc` is skipped; ROS env goes in
  `/etc/profile.d/99-ros.sh` (incl. `TURTLEBOT3_MODEL=burger`, required or
  launch files KeyError-crash). Long-running launches need
  `nohup ... >log 2>&1 </dev/null & disown`.
- **Recording**: Xbox Game Bar cannot capture WSLg windows (wslg#1037) and
  Linux Wayland recorders don't work under Weston-RDP. Plan: ScreenToGif /
  Snipping Tool recorder / ffmpeg gdigrab; Gazebo's built-in VideoRecorder as
  a candidate for the sim viewport.
- Wrote `scripts/wsl_setup.sh` (idempotent, run once via sudo) + `~/.wslconfig`
  (memory=8GB, swap=8GB, sparseVhd — set BEFORE first boot so the VHDX stays
  sparse from day one).
- Known risk to verify live: Gazebo GUI under WSLg may segfault without GPU
  accel (turtlebot3_simulations#247); server runs headless (`-s`) regardless;
  fallback `LIBGL_ALWAYS_SOFTWARE=1`. Verify `glxinfo -B` shows the RTX 4050
  via D3D12, not llvmpipe.

## 2026-07-28 — WSL2 up; provisioning launched

- `wsl --install -d Ubuntu-24.04` → WSL 2.7.11, kernel 6.18.33.2, WSLg 1.0.73.2,
  Direct3D 1.611. Distro VERSION 2 ✅.
- The interactive first-run password never got set (`passwd -S` → `L`, locked).
  Not a blocker: `wsl -u root` grants root with no password, so provisioning runs
  non-interactively. `wsl_setup.sh` was adjusted to resolve the target user from
  uid 1000 when `SUDO_USER` is absent (root-direct invocation).
- **Uncertainty #1 resolved**: exit codes propagate correctly through
  `wsl -d Ubuntu-24.04 -- bash -lc "exit 42"` → `$LASTEXITCODE = 42`. Orchestration
  scripts can trust them.
- Windows-side files must be written **LF-only** — normalized after every write
  (a stray CR makes bash fail with unreadable `$'\r'` errors).

### Design: scripted exploration instead of teleop

`explore_drive.py` is a reactive `rclpy` node rather than a fixed sequence of
`ros2 topic pub` bursts. Open-loop driving hits the tb3_world cylinders within a
few metres; this closes the loop on `/scan` (drive while the front sector is
clear, turn toward the roomier side otherwise, occasional random turn so the
robot doesn't orbit the same loop). Two traps designed around up front:
`TwistStamped` on `/cmd_vel` (plain `Twist` is silently ignored), and a
BEST_EFFORT subscription QoS for `/scan` (a default RELIABLE subscription never
matches Gazebo's publisher — no scans, no error).

`run_mapping.sh` verifies the saved `.pgm` occupancy breakdown rather than just
checking the file exists — an all-unknown map is the failure mode that otherwise
looks exactly like success.

### Design: C++ grasp service (Artifact 3)

Port target is `graspsight/src/graspsight/pose.py` — self-contained and a good
fit for Eigen: 2D Kabsch via SVD + a table-constrained ICP loop (coarse yaw
sweep init, trimmed correspondences, closed-form increment per iteration).
Plan: export a fixed set of segmented clouds from Part C on the Windows side,
serve them to a `rclcpp` node exposing an `EstimateCubePose` service, and report
**both** correctness parity (pose/yaw/RMSE vs the Python reference) and a
speedup number. The Python side uses scipy's C-backed `cKDTree`, so a fair
comparison needs a real KD-tree in C++ — writing one from scratch keeps the
"from scratch" through-line of the series and avoids a PCL dependency.

## 2026-07-28 — SLAM ✅ and Nav2 ✅ (6/6 goals)

**SLAM**: first end-to-end mapping run succeeded — 240 s autonomous exploration
(1199 scans processed), map saved: 112×103 @ 5 cm, 67.3% free / 7.1% occupied /
25.6% unknown (the unknown is the bounding-box corners outside the hexagonal
wall). All nine cylinders resolved in their 3×3 grid; walls closed cleanly.

**Nav2**: `run_nav2.sh` → **6/6 goals SUCCEEDED** (12–50 s each), goals derived
from the map itself by `pick_goals.py` (max-clearance cell per angular sector —
no hand-guessed coordinates), AMCL seeded at the spawn, return-home included.

The debugging trail to get there — each of these is a silent-failure mode:

1. **Gazebo GUI segfault under WSLg** (intermittent, in the NVIDIA D3D12
   driver `libnvwgf2umx.so`; the known turtlebot3_simulations#247 mode). The
   stock TB3 launch marks the GUI *required*, so a GUI crash kills the physics
   server too. Fix: `headless_world.launch.py` — the stock launch minus the
   GUI client. Orchestrated runs never need a GUI.
2. **`pkill -f` self-kill**: patterns like "nav2" match the orchestration
   script's own path (`run_nav2.sh`) — the cleanup SIGTERMed its own process
   group. Fix: `safe_pkill` (skips the current PID and its ancestors).
3. **Seed-before-bringup ordering**: Nav2's global_costmap blocks its own
   activation waiting for the `base_link→map` transform, which only exists
   after AMCL has an initial pose. Waiting for `bt_navigator` active before
   seeding = deadlock; the lifecycle manager aborts bringup. Fix: seed AMCL
   the moment `/amcl` is active, then wait for the rest of the stack.
4. **Latched topics can't be probed by late joiners**: `/amcl_pose` and `/map`
   publish once with transient-local durability — a late `ros2 topic echo`
   receives nothing, forever. Probes must use lifecycle states
   (`ros2 lifecycle get`) or the continuous `/tf` stream.
5. **Pipe buffering starves grep**: `ros2 topic echo | grep -q` can time out
   even while the topic broadcasts at 4 Hz (stdout block-buffering). Fix:
   capture to a file, then grep the file.
6. AMCL's "Failed to transform initial pose in time … extrapolation into the
   future" warning (a ~10 ms race) is **cosmetic** — it still sets the pose,
   confirmed by the following "Setting pose" log line.
7. `source /opt/ros/jazzy/setup.bash` aborts under `set -u`
   (AMENT_TRACE_SETUP_FILES unbound) — wrap the source in `set +u` / `set -u`.

Results CSV in `results/nav2_results.csv`. Remaining: demo GIFs (attach GUI /
rviz2 for recording), C++ grasp-service node, README, publish.

## 2026-07-28 — demo pipeline, sim-flakiness hardening, C++ node built

**Demos = data, not pixels.** Screen-recording WSLg is the least reliable part
of this whole stack, so demos are rendered offline instead: `record_run.py`
(rclpy) samples /map, /tf pose, /scan, /plan to JSONL during runs;
`render_gif.py` (Windows venv, PIL) draws the frames. First SLAM GIF rendered
(63 frames, 0.6 MB, map-growth + robot trace + HUD).

**Two more silent-failure modes found and fixed:**

8. **`map_saver_cli`'s default 2 s timeout vs slam_toolbox's ~5 s map publish
   interval** — whether a save works is a phase coin-flip; worse, a failed save
   leaves the PREVIOUS run's .pgm in place and every existence check passes on
   stale data. Fix: `save_map_timeout:=15.0`, 3 retries, and an mtime freshness
   gate that fails the run if the .pgm predates it.
9. **Gazebo can freeze mid-run under WSL2** (gz-transport goes silent —
   `NodeShared::Publish() Error: Interrupted system call` — no crash, no exit).
   Recorded evidence: robot pose bit-identical from t=130 s to t=240 s while
   wall clock advanced; slam_toolbox kept republishing its last map, the driver
   kept publishing into a dead sim (its duration used wall clock). Fixes:
   scan-staleness watchdog in `explore_drive.py` (12 s without /scan → abort
   exit 2), PIPESTATUS propagation in run_mapping.sh, and `launch_world` with
   /scan+/odom hard gates + teardown-and-relaunch retries in lib_common.sh.
   (An earlier run also produced a world where /odom NEVER carried data despite
   a clean spawn and bridge — same resilience layer catches that at t=0.)

**wait_for_topic wall-clock bug**: the original loop counted iterations
(`sleep 3` × 40 = "120 s") but each iteration can spend ~11 s in discovery +
echo timeouts — a "120 s" gate silently waited 8+ minutes. All waits now use
`$SECONDS` elapsed time.

**C++ grasp node (Artifact 3) built and benchmarked (naive port):**
`cellops_interfaces` (EstimateCubePose.srv) + `cellops_grasp` (from-scratch
KD-tree, Eigen Kabsch/ICP port of Part C's pose.py, rclcpp service node,
standalone benchmark). Fixtures: 12 segmented clouds exported through the REAL
Part C perception pipeline (`make_testdata.py`). First numbers: Python
reference ~17–22 ms/solve; naive C++ port ~6–8 ms (≈2.8×) — scipy's C-backed
cKDTree means vectorized NumPy is no strawman. Optimization applied: rigid
transforms preserve distances, so ICP correspondences can query ONE static
model-frame KD-tree against inverse-transformed observations (zero per-iter
tree rebuilds) + iterative cache-friendly search. Re-benchmark pending on a
quiet machine.
