#!/usr/bin/env python3
"""Export segmented cube clouds from GraspSight (Part C) as benchmark fixtures.

Runs the real Part C perception pipeline (MuJoCo render -> backproject ->
RANSAC table fit -> DBSCAN clustering -> target selection) and saves the
segmented cluster of each trial as cloud_XX.csv, plus truth.json with the
ground-truth cube pose and the fitted table z. These fixtures feed:

  * python_ref.py   — the ORIGINAL pose.py estimator (timing + results)
  * pose_benchmark  — the C++ port (timing + results)
  * parity_report.py — joins all three into the benchmark table

Run with the Windows robotics venv from the cellops repo root:
    python scripts/make_testdata.py [n_trials]
"""
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO.parent / "graspsight" / "src"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from graspsight.camera import Intrinsics, backproject, cam_to_world  # noqa: E402
from graspsight.env import GraspEnv  # noqa: E402
from graspsight.pointcloud import extract_objects, select_target  # noqa: E402

n_trials = int(sys.argv[1]) if len(sys.argv) > 1 else 12
out_dir = REPO / "testdata"
out_dir.mkdir(exist_ok=True)

env = GraspEnv(seed=42)
K = Intrinsics.from_fovy(env.camera_fovy(), width=640, height=480)
cam_pos, R_wc = env.gt_camera_extrinsics()
rng = np.random.default_rng(7)

truth = []
saved = 0
attempt = 0
while saved < n_trials and attempt < n_trials * 3:
    attempt += 1
    env.reset()
    key = env.model.key("rest")
    env.data.qpos[:6] = key.qpos[:6]
    env.data.ctrl[:6] = key.ctrl[:6]
    mujoco.mj_forward(env.model, env.data)

    obs = env.rgbd()
    pts_cam, pix = backproject(obs["depth"], K)
    pts_w = cam_to_world(pts_cam, cam_pos, R_wc)
    colors = obs["rgb"][pix[:, 1], pix[:, 0]]
    clusters, plane = extract_objects(pts_w, colors, rng=rng)
    target = select_target(clusters)
    if target is None:
        print(f"attempt {attempt}: no target cluster, reseeding")
        continue

    table_z = float(-plane.d / plane.normal[2])
    gt_pos, gt_yaw = env.gt_block_pose("block_red")

    name = f"cloud_{saved:02d}.csv"
    np.savetxt(out_dir / name, target.points, delimiter=",", fmt="%.6f")
    truth.append({
        "cloud": name,
        "n_points": int(len(target.points)),
        "table_z": table_z,
        "gt_x": float(gt_pos[0]), "gt_y": float(gt_pos[1]),
        "gt_z": float(gt_pos[2]), "gt_yaw": float(gt_yaw),
    })
    print(f"{name}: {len(target.points):4d} pts  table_z={table_z:.4f}  "
          f"gt=({gt_pos[0]:.4f}, {gt_pos[1]:.4f})  yaw={np.rad2deg(gt_yaw):.1f} deg")
    saved += 1

table_zs = [t["table_z"] for t in truth]
meta = {
    "trials": truth,
    "table_z_mean": float(np.mean(table_zs)),
    "table_z_spread": float(np.max(table_zs) - np.min(table_zs)),
}
(out_dir / "truth.json").write_text(json.dumps(meta, indent=2))
# per-cloud table_z sidecar for the C++ benchmark — both implementations must
# see the identical table plane or the comparison isn't apples-to-apples
(out_dir / "table_z.csv").write_text(
    "".join(f"{t['cloud']},{t['table_z']:.9f}\n" for t in truth))
print(f"\nwrote {saved} clouds + truth.json  "
      f"(table_z mean {meta['table_z_mean']:.5f}, spread {meta['table_z_spread'] * 1000:.3f} mm)")
