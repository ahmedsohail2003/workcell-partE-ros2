#!/usr/bin/env python3
"""Choose Nav2 goal poses from a saved occupancy map, instead of guessing them.

Two reasons not to hardcode goals in world coordinates:

1. `slam_toolbox` anchors the map frame at the robot's *starting pose*, so map
   coordinates are offset from Gazebo world coordinates by the spawn point.
2. A goal that lands inside an obstacle (or in never-observed space) is rejected
   by the planner, which looks like a navigation failure but is really a bad
   test fixture.

So: take the distance transform of free space, then pick the highest-clearance
cell in each angular sector around the map centroid. Every goal is therefore
provably reachable-looking (open, well clear of walls) and the set is spread
around the map rather than clustered.

Usage (Windows robotics venv):
    python pick_goals.py maps/tb3_world.pgm maps/tb3_world.yaml [n_goals]
"""
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

# TurtleBot3 Burger footprint radius (~0.105 m) plus a margin for costmap
# inflation — goals closer than this to anything get rejected or replanned.
MIN_CLEARANCE_M = 0.35

COMPASS = ["east", "north-east", "north", "north-west",
           "west", "south-west", "south", "south-east"]


def compass_label(deg: float) -> str:
    """Nearest compass point to a bearing in degrees (0 = +x/east, CCW)."""
    return COMPASS[int(((deg % 360.0) + 22.5) // 45.0) % 8]


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    parts, idx = [], 0
    while len(parts) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx] != 0x0A:
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        parts.append(data[start:idx])
    idx += 1
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)


def read_yaml(path: Path) -> dict:
    out = {}
    for line in path.read_text().splitlines():
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        out[k.strip()] = v.strip()
    return out


def main():
    pgm = Path(sys.argv[1])
    yml = Path(sys.argv[2])
    n_goals = int(sys.argv[3]) if len(sys.argv) > 3 else 5

    img = read_pgm(pgm)
    meta = read_yaml(yml)
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    h, w = img.shape

    # Unknown counts as blocked: a goal in never-observed space is not a fair
    # navigation trial.
    free = img >= 250
    clearance = ndimage.distance_transform_edt(free) * res

    valid = clearance >= MIN_CLEARANCE_M
    if not valid.any():
        print("ERROR: no cell has enough clearance", file=sys.stderr)
        sys.exit(1)

    rows, cols = np.nonzero(valid)
    cy, cx = rows.mean(), cols.mean()

    # Bias toward the outer ring: goals near the centroid are trivially short
    # trips and don't exercise planning around the obstacle grid.
    ang = np.degrees(np.arctan2(-(rows - cy), cols - cx)) % 360.0
    radius = np.hypot(rows - cy, cols - cx)
    r_max = radius.max() if radius.max() > 0 else 1.0

    n_sectors = max(1, n_goals)
    width = 360.0 / n_sectors
    goals = []
    for s in range(n_sectors):
        lo, hi = s * width, (s + 1) * width
        sel = (ang >= lo) & (ang < hi)
        if not sel.any():
            continue
        # Score = clearance, weighted toward the perimeter.
        score = clearance[rows[sel], cols[sel]] * (0.5 + 0.5 * radius[sel] / r_max)
        k = int(np.argmax(score))
        r, c = int(rows[sel][k]), int(cols[sel][k])
        # PGM rows run top-down; map y runs bottom-up.
        gx = ox + (c + 0.5) * res
        gy = oy + (h - 1 - r + 0.5) * res
        # Label from the chosen point's actual bearing, not the sector index —
        # with n_goals != 8 the two disagree and the labels become nonsense.
        label = compass_label(float(ang[sel][k]))
        goals.append((gx, gy, label, float(clearance[r, c])))

    print(f"# map {w}x{h} @ {res} m/pix, origin ({ox}, {oy})", file=sys.stderr)
    print(f"# max clearance in map: {clearance.max():.2f} m", file=sys.stderr)
    for gx, gy, name, clr in goals:
        print(f"# {name:<12} clearance {clr:.2f} m", file=sys.stderr)
    # stdout: the GOALS payload consumed by run_nav2.sh (one "x y label" per line)
    for gx, gy, name, _ in goals:
        print(f"{gx:.3f} {gy:.3f} {name}")


if __name__ == "__main__":
    main()
