#!/usr/bin/env python3
"""Render demo GIFs from record_run.py streams — CellOps (WorkCell Part E).

Draws portfolio-quality frames from recorded run data (map, robot trace, lidar,
plans, goals) and assembles them into GIFs. Runs on the Windows side with the
shared robotics venv (numpy + imageio + pillow); no ROS needed here.

Modes:
  slam  — the map growing as the robot explores (map snapshots + trace + lidar)
  nav   — goal-to-goal navigation on the finished map (plan lines + goal HUD)

Usage:
  python render_gif.py slam <record.jsonl> <out.gif>
  python render_gif.py nav  <record.jsonl> <out.gif> --map-pgm maps/tb3_world.pgm \
      --map-yaml maps/tb3_world.yaml --results results/nav2_results.csv
"""

import argparse
import csv
import json
import math
from pathlib import Path

import imageio.v3 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SCALE = 7            # px per map cell at 0.05 m/cell -> 140 px/m
TRACE = (46, 134, 222)
TRACE_OLD = (150, 190, 235)
ROBOT = (232, 89, 12)
LIDAR = (231, 76, 60)
PLAN = (39, 174, 96)
GOAL = (142, 68, 173)
HUD_BG = (24, 28, 34)
HUD_FG = (235, 238, 242)

C_FREE = (245, 245, 245)
C_OCC = (25, 28, 36)
C_UNK = (148, 156, 168)


def load_font(size: int):
    for name in ("consola.ttf", "arial.ttf", "DejaVuSansMono.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def read_stream(path: Path):
    events = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                break  # torn final line from SIGTERM — fine
    return events


def grid_to_rgb(data, w, h):
    """Occupancy int8 grid -> RGB image array (row 0 = map origin = image bottom)."""
    g = np.array(data, dtype=np.int16).reshape(h, w)
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:] = C_UNK
    img[g == 0] = C_FREE
    img[(g > 0) & (g <= 100)] = C_OCC
    img[(g > 0) & (g < 65)] = (90, 98, 110)   # low-confidence obstacles, lighter
    return np.flipud(img)                      # image row 0 = top = max y


class MapCanvas:
    """Fixed world->pixel transform sized to the FINAL map, so early frames
    (when SLAM's map is still small) render into a stable, non-jumping canvas."""

    def __init__(self, res, w, h, ox, oy):
        self.res, self.w, self.h, self.ox, self.oy = res, w, h, ox, oy
        self.W, self.H = w * SCALE, h * SCALE

    def to_px(self, x, y):
        c = (x - self.ox) / self.res
        r = (y - self.oy) / self.res
        return c * SCALE, self.H - r * SCALE

    def base_image(self, map_evt=None):
        img = Image.new("RGB", (self.W, self.H), C_UNK)
        if map_evt is not None:
            rgb = grid_to_rgb(map_evt["data"], map_evt["w"], map_evt["h"])
            tile = Image.fromarray(rgb).resize(
                (map_evt["w"] * SCALE, map_evt["h"] * SCALE), Image.NEAREST)
            # paste at this snapshot's origin offset within the final canvas
            dx = round((map_evt["ox"] - self.ox) / self.res) * SCALE
            dy_bottom = round((map_evt["oy"] - self.oy) / self.res) * SCALE
            dy = self.H - dy_bottom - tile.height
            img.paste(tile, (dx, dy))
        return img


def draw_robot(d: ImageDraw.ImageDraw, cv: MapCanvas, x, y, yaw, r_px=9):
    cx, cy = cv.to_px(x, y)
    pts = []
    for ang, rad in ((0, 1.6 * r_px), (2.5, r_px), (-2.5, r_px)):
        a = yaw + ang
        pts.append((cx + rad * math.cos(a), cy - rad * math.sin(a)))
    d.polygon(pts, fill=ROBOT, outline=(120, 40, 0))


def draw_trace(d: ImageDraw.ImageDraw, cv: MapCanvas, poses, color, width=3):
    if len(poses) < 2:
        return
    d.line([cv.to_px(p[0], p[1]) for p in poses], fill=color, width=width, joint="curve")


def draw_lidar(d: ImageDraw.ImageDraw, cv: MapCanvas, scan):
    x, y, yaw = scan["pose"]
    a = scan["amin"]
    for r in scan["ranges"]:
        if r is not None and 0.05 < r < 4.0:
            px, py = cv.to_px(x + r * math.cos(yaw + a), y + r * math.sin(yaw + a))
            d.ellipse([px - 1.5, py - 1.5, px + 1.5, py + 1.5], fill=LIDAR)
        a += scan["ainc"]


def hud(img: Image.Image, lines, font):
    d = ImageDraw.Draw(img, "RGBA")
    pad, lh = 10, font.size + 6
    wmax = max(d.textlength(t, font=font) for t in lines)
    d.rectangle([8, 8, 8 + wmax + 2 * pad, 8 + lh * len(lines) + pad],
                fill=HUD_BG + (215,))
    for i, t in enumerate(lines):
        d.text((8 + pad, 8 + pad // 2 + i * lh), t, fill=HUD_FG, font=font)


def finalize(frames, out, fps=10, hold_last=15):
    frames = frames + [frames[-1]] * hold_last
    iio.imwrite(out, [np.asarray(f) for f in frames],
                duration=int(1000 / fps), loop=0)
    print(f"wrote {out}: {len(frames)} frames @ {fps} fps, "
          f"{Path(out).stat().st_size / 1e6:.1f} MB")


# ----------------------------------------------------------------- slam mode
def render_slam(events, out):
    maps = [e for e in events if e["kind"] == "map"]
    poses = [e for e in events if e["kind"] == "pose"]
    scans = [e for e in events if e["kind"] == "scan"]
    if not maps or not poses:
        raise SystemExit(f"not enough data: {len(maps)} maps, {len(poses)} poses")
    last = maps[-1]
    cv = MapCanvas(last["res"], last["w"], last["h"], last["ox"], last["oy"])
    font = load_font(max(13, cv.W // 46))
    t0 = poses[0]["t"]

    # one frame per map snapshot, robot state = nearest samples at that time
    frames = []
    for i, m in enumerate(maps):
        img = cv.base_image(m)
        d = ImageDraw.Draw(img)
        trail = [p["xyyaw"] for p in poses if p["t"] <= m["t"]]
        draw_trace(d, cv, trail, TRACE)
        scan = min(scans, key=lambda s: abs(s["t"] - m["t"])) if scans else None
        if scan and abs(scan["t"] - m["t"]) < 3.0:
            draw_lidar(d, cv, scan)
        if trail:
            draw_robot(d, cv, *trail[-1])
        pct_known = 100.0 * sum(1 for v in m["data"] if v >= 0) / len(m["data"])
        hud(img, [
            "WorkCell Part E - SLAM (slam_toolbox, ROS 2 Jazzy)",
            f"t = {m['t'] - t0:5.0f} s    map {m['w']}x{m['h']} @ {m['res']:.2f} m",
            f"explored: {pct_known:4.1f} %    autonomous scan-guided drive",
        ], font)
        frames.append(img)
    finalize(frames, out, fps=8)


# ------------------------------------------------------------------ nav mode
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


def render_nav(events, out, map_pgm, map_yaml, results_csv):
    poses = [e for e in events if e["kind"] == "pose"]
    plans = [e for e in events if e["kind"] == "plan"]
    if not poses:
        raise SystemExit("no pose data recorded")

    meta = {}
    for line in Path(map_yaml).read_text().splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]

    pgm = read_pgm(Path(map_pgm))
    h, w = pgm.shape
    cv = MapCanvas(res, w, h, ox, oy)
    font = load_font(max(13, cv.W // 46))

    base = np.empty((h, w, 3), dtype=np.uint8)
    base[:] = C_UNK
    base[pgm >= 250] = C_FREE
    base[pgm <= 100] = C_OCC
    base_img = Image.fromarray(base).resize((cv.W, cv.H), Image.NEAREST)

    goals = []
    with open(results_csv) as f:
        for row in csv.DictReader(f):
            goals.append({
                "label": row["goal"], "x": float(row["x"]), "y": float(row["y"]),
                "result": row["result"], "secs": row["seconds"],
                "t0": float(row["start_epoch"]), "t1": float(row["end_epoch"]),
            })

    def active_goal(t):
        for i, g in enumerate(goals):
            if g["t0"] <= t <= g["t1"] + 2:
                return i
        return None

    frames = []
    step = max(1, len(poses) // 260)          # ~260 frames cap
    n_pass = sum(1 for g in goals if g["result"] == "SUCCEEDED")
    for k in range(0, len(poses), step):
        p = poses[k]
        img = base_img.copy()
        d = ImageDraw.Draw(img)
        gi = active_goal(p["t"])

        # completed goal markers stay visible
        for j, g in enumerate(goals):
            if g["t1"] < p["t"]:
                gx, gy = cv.to_px(g["x"], g["y"])
                ok = g["result"] == "SUCCEEDED"
                d.ellipse([gx - 7, gy - 7, gx + 7, gy + 7],
                          outline=PLAN if ok else ROBOT, width=3)

        trail = [q["xyyaw"] for q in poses[:k + 1]]
        draw_trace(d, cv, trail, TRACE_OLD, width=2)
        if gi is not None:
            seg = [q["xyyaw"] for q in poses[:k + 1] if q["t"] >= goals[gi]["t0"]]
            draw_trace(d, cv, seg, TRACE, width=4)
            # latest plan published before this frame
            live = [pl for pl in plans if pl["t"] <= p["t"] + 0.5]
            if live and live[-1]["t"] >= goals[gi]["t0"] - 1:
                d.line([cv.to_px(x, y) for x, y in live[-1]["points"]],
                       fill=PLAN, width=2)
            gx, gy = cv.to_px(goals[gi]["x"], goals[gi]["y"])
            r = 9
            d.line([gx - r, gy, gx + r, gy], fill=GOAL, width=3)
            d.line([gx, gy - r, gx, gy + r], fill=GOAL, width=3)
            d.ellipse([gx - r, gy - r, gx + r, gy + r], outline=GOAL, width=2)

        draw_robot(d, cv, *poses[k]["xyyaw"])
        done = sum(1 for g in goals if g["t1"] < p["t"])
        lines = ["WorkCell Part E - Nav2 on the SLAM map (AMCL localization)"]
        if gi is not None:
            g = goals[gi]
            lines.append(f"goal {gi + 1}/{len(goals)}: {g['label']}  "
                         f"({g['x']:.2f}, {g['y']:.2f})")
        lines.append(f"reached: {done}/{len(goals)}"
                     + (f"    final: {n_pass}/{len(goals)} SUCCEEDED"
                        if done == len(goals) else ""))
        hud(img, lines, font)
        frames.append(img)
    finalize(frames, out, fps=12)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["slam", "nav"])
    p.add_argument("record")
    p.add_argument("out")
    p.add_argument("--map-pgm")
    p.add_argument("--map-yaml")
    p.add_argument("--results")
    args = p.parse_args()

    events = read_stream(Path(args.record))
    print(f"{len(events)} events from {args.record}")
    if args.mode == "slam":
        render_slam(events, args.out)
    else:
        for req in ("map_pgm", "map_yaml", "results"):
            if getattr(args, req) is None:
                raise SystemExit(f"--{req.replace('_', '-')} required for nav mode")
        render_nav(events, args.out, args.map_pgm, args.map_yaml, args.results)


if __name__ == "__main__":
    main()
