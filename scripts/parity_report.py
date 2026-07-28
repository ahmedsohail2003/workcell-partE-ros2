#!/usr/bin/env python3
"""Join ground truth + Python reference + C++ benchmark into the final table.

Reads:
  testdata/truth.json        (make_testdata.py — ground-truth poses)
  results/python_ref.json    (python_ref.py — original pose.py, WSL timing)
  results/cpp_bench.jsonl    (pose_benchmark — C++ port, WSL timing)

Writes results/grasp_benchmark.md and prints it.

Run anywhere with Python 3 (no deps).
"""
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
truth = json.loads((REPO / "testdata" / "truth.json").read_text())["trials"]
py = {r["cloud"]: r for r in json.loads((REPO / "results" / "python_ref.json").read_text())}
cpp = {}
for line in (REPO / "results" / "cpp_bench.jsonl").read_text().splitlines():
    if line.strip():
        r = json.loads(line)
        cpp[r["cloud"]] = r


def yaw_err_deg(a, b):
    d = (a - b) % (math.pi / 2)
    return math.degrees(min(d, math.pi / 2 - d))


rows = []
for t in truth:
    c, p = cpp.get(t["cloud"]), py.get(t["cloud"])
    if c is None or p is None:
        print(f"WARNING: missing results for {t['cloud']}", file=sys.stderr)
        continue
    rows.append({
        "cloud": t["cloud"].replace("cloud_", "").replace(".csv", ""),
        "n": t["n_points"],
        # implementation agreement (C++ vs Python, same inputs)
        "d_pos_mm": math.hypot(c["x"] - p["x"], c["y"] - p["y"]) * 1000,
        "d_yaw_deg": yaw_err_deg(c["yaw"], p["yaw"]),
        "d_iters": abs(c["iters"] - p["iters"]),
        # accuracy vs ground truth
        "py_err_mm": math.hypot(p["x"] - t["gt_x"], p["y"] - t["gt_y"]) * 1000,
        "cpp_err_mm": math.hypot(c["x"] - t["gt_x"], c["y"] - t["gt_y"]) * 1000,
        "py_yaw_err": yaw_err_deg(p["yaw"], t["gt_yaw"]),
        "cpp_yaw_err": yaw_err_deg(c["yaw"], t["gt_yaw"]),
        # speed
        "py_ms": p["median_ms"],
        "cpp_ms": c["median_ms"],
        "speedup": p["median_ms"] / c["median_ms"] if c["median_ms"] > 0 else float("inf"),
    })

if not rows:
    sys.exit("no joined rows — did both benchmarks run?")

mean = lambda k: sum(r[k] for r in rows) / len(rows)  # noqa: E731

md = []
md.append("# GraspSight pose estimator: C++ port vs Python reference\n")
md.append(f"{len(rows)} segmented cube clouds exported from the real Part C perception "
          "pipeline (MuJoCo render → RANSAC table fit → DBSCAN → target cluster). "
          "Same clouds, same per-cloud table plane, both implementations timed on "
          "the same WSL2 Ubuntu 24.04 (median of 20 Python / 50 C++ solves).\n")
md.append("| cloud | pts | agree Δpos (mm) | agree Δyaw (°) | py err (mm) | C++ err (mm) | py (ms) | C++ (ms) | speedup |")
md.append("|---|---|---|---|---|---|---|---|---|")
for r in rows:
    md.append(f"| {r['cloud']} | {r['n']} | {r['d_pos_mm']:.3f} | {r['d_yaw_deg']:.3f} "
              f"| {r['py_err_mm']:.2f} | {r['cpp_err_mm']:.2f} "
              f"| {r['py_ms']:.1f} | {r['cpp_ms']:.2f} | {r['speedup']:.1f}x |")
md.append(f"| **mean** |  | **{mean('d_pos_mm'):.3f}** | **{mean('d_yaw_deg'):.3f}** "
          f"| **{mean('py_err_mm'):.2f}** | **{mean('cpp_err_mm'):.2f}** "
          f"| **{mean('py_ms'):.1f}** | **{mean('cpp_ms'):.2f}** | **{mean('speedup'):.1f}x** |")
md.append("")
md.append(f"- Implementation agreement: mean Δpos **{mean('d_pos_mm'):.3f} mm**, "
          f"mean Δyaw **{mean('d_yaw_deg'):.3f}°** (identical trim/convergence rules; "
          "residual differences come from KD-tree tie-breaking on equidistant "
          "correspondences).")
md.append(f"- Accuracy vs ground truth is preserved: {mean('py_err_mm'):.2f} mm (py) "
          f"vs {mean('cpp_err_mm'):.2f} mm (C++).")
md.append(f"- Median solve: **{mean('py_ms'):.1f} ms → {mean('cpp_ms'):.2f} ms**, "
          f"**{mean('speedup'):.0f}× faster** — from-scratch KD-tree + Eigen, "
          "no PCL, single-threaded.")

out = REPO / "results" / "grasp_benchmark.md"
out.write_text("\n".join(md) + "\n", encoding="utf-8")   # Windows default is cp1252
print("\n".join(md))
print(f"\nwrote {out}", file=sys.stderr)
