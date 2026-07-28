# GraspSight pose estimator: C++ port vs Python reference

12 segmented cube clouds exported from the real Part C perception pipeline (MuJoCo render → RANSAC table fit → DBSCAN → target cluster). Same clouds, same per-cloud table plane, both implementations timed on the same WSL2 Ubuntu 24.04 (median of 20 Python / 50 C++ solves).

| cloud | pts | agree Δpos (mm) | agree Δyaw (°) | py err (mm) | C++ err (mm) | py (ms) | C++ (ms) | speedup |
|---|---|---|---|---|---|---|---|---|
| 00 | 169 | 0.000 | 0.000 | 0.76 | 0.76 | 11.2 | 3.35 | 3.3x |
| 01 | 159 | 0.000 | 0.000 | 0.34 | 0.34 | 12.7 | 3.46 | 3.7x |
| 02 | 194 | 0.000 | 0.000 | 0.58 | 0.58 | 12.5 | 4.05 | 3.1x |
| 03 | 208 | 0.000 | 0.000 | 0.62 | 0.62 | 14.3 | 4.55 | 3.1x |
| 04 | 235 | 0.000 | 0.000 | 0.30 | 0.30 | 18.2 | 5.58 | 3.3x |
| 05 | 220 | 0.000 | 0.000 | 0.43 | 0.43 | 20.5 | 5.28 | 3.9x |
| 06 | 222 | 0.000 | 0.000 | 0.30 | 0.30 | 14.0 | 4.71 | 3.0x |
| 07 | 228 | 0.000 | 0.000 | 0.29 | 0.29 | 16.1 | 5.09 | 3.2x |
| 08 | 172 | 0.001 | 0.000 | 0.77 | 0.77 | 12.1 | 3.24 | 3.7x |
| 09 | 230 | 0.001 | 0.000 | 0.57 | 0.57 | 10.8 | 4.42 | 2.4x |
| 10 | 200 | 0.000 | 0.000 | 0.36 | 0.36 | 15.2 | 4.70 | 3.2x |
| 11 | 148 | 0.000 | 0.000 | 0.12 | 0.12 | 10.9 | 3.50 | 3.1x |
| **mean** |  | **0.000** | **0.000** | **0.45** | **0.45** | **14.0** | **4.33** | **3.3x** |

- Implementation agreement: mean Δpos **0.000 mm**, mean Δyaw **0.000°** (identical trim/convergence rules; residual differences come from KD-tree tie-breaking on equidistant correspondences).
- Accuracy vs ground truth is preserved: 0.45 mm (py) vs 0.45 mm (C++).
- Median solve: **14.0 ms → 4.33 ms**, **3× faster** — from-scratch KD-tree + Eigen, no PCL, single-threaded.
