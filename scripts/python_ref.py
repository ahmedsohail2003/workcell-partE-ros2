#!/usr/bin/env python3
"""Python reference results for the pose-estimator benchmark (runs in WSL).

Imports the ORIGINAL, unmodified graspsight/pose.py from the Part C repo
(mounted at /mnt/c) and runs it on the exported cloud fixtures with the same
per-cloud table_z the C++ benchmark uses — same code, same data, same OS as
the C++ side, so the timing comparison is apples-to-apples.

Usage (WSL): python3 python_ref.py <testdata_dir> [reps=20] > python_ref.json
"""
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, "/mnt/c/Users/sohai/robotics/graspsight/src")

import numpy as np  # noqa: E402

from graspsight.pose import estimate_cube_pose  # noqa: E402

testdata = Path(sys.argv[1])
reps = int(sys.argv[2]) if len(sys.argv) > 2 else 20

meta = json.loads((testdata / "truth.json").read_text())
out = []
for trial in meta["trials"]:
    pts = np.loadtxt(testdata / trial["cloud"], delimiter=",")
    times = []
    pose = None
    for _ in range(reps):
        t0 = time.perf_counter()
        pose = estimate_cube_pose(pts, table_z=trial["table_z"])
        times.append((time.perf_counter() - t0) * 1000.0)
    out.append({
        "cloud": trial["cloud"],
        "n_points": int(len(pts)),
        "x": float(pose.position[0]), "y": float(pose.position[1]),
        "yaw": float(pose.yaw), "rmse": float(pose.rmse),
        "iters": int(pose.n_iters),
        "median_ms": round(statistics.median(times), 3),
    })
    print(f"{trial['cloud']}: {statistics.median(times):7.2f} ms  "
          f"iters={pose.n_iters}", file=sys.stderr)

print(json.dumps(out, indent=1))
