#!/usr/bin/env python3
"""Run-data recorder for demo rendering — CellOps (WorkCell Part E).

Samples the live ROS graph during a SLAM or Nav2 run and appends compact JSON
lines to a file; `render_gif.py` (Windows side) turns that stream into demo
GIFs offline. This replaces screen-recording entirely: the WSLg GUI stack is
the one unreliable component in the setup (intermittent D3D12 segfaults), so
demos are rendered from data instead of pixels.

Records:
  map    — every /map update (SLAM: the growing map; Nav2: unused, static map)
  pose   — map->base_footprint from tf at 5 Hz (robot trajectory)
  scan   — /scan at ~2 Hz, with the pose at sample time (lidar overlay)
  plan   — every /plan update (Nav2 global path)

Each line: {"t": <wall time>, "kind": ..., ...}. Writes are line-buffered and
the file is valid at any prefix, so SIGTERM at teardown loses nothing.

Usage: python3 record_run.py --out /tmp/cellops/run_record.jsonl [--duration 600]
"""

import argparse
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class RunRecorder(Node):
    def __init__(self, args):
        super().__init__("cellops_run_recorder")
        self.f = open(args.out, "w", buffering=1)  # line-buffered
        self.deadline = time.monotonic() + args.duration
        self.last_scan_write = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(OccupancyGrid, "map", self.on_map, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, scan_qos)
        self.create_subscription(Path, "plan", self.on_plan, 10)
        self.create_timer(0.2, self.on_pose_timer)   # 5 Hz pose samples
        self.n = {"map": 0, "pose": 0, "scan": 0, "plan": 0}
        self.get_logger().info(f"recording to {args.out} for {args.duration:.0f}s")

    def write(self, kind: str, payload: dict):
        payload["t"] = time.time()
        payload["kind"] = kind
        self.f.write(json.dumps(payload) + "\n")
        self.n[kind] += 1
        if time.monotonic() > self.deadline:
            self.get_logger().info(f"duration reached, counts={self.n}")
            raise SystemExit(0)

    def current_pose(self):
        """Robot pose in the map frame, or None before localization."""
        try:
            tfs = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        tr, q = tfs.transform.translation, tfs.transform.rotation
        return [round(tr.x, 4), round(tr.y, 4),
                round(yaw_from_quat(q.x, q.y, q.z, q.w), 4)]

    def on_pose_timer(self):
        pose = self.current_pose()
        if pose is not None:
            self.write("pose", {"xyyaw": pose})

    def on_map(self, msg: OccupancyGrid):
        info = msg.info
        self.write("map", {
            "res": info.resolution,
            "w": info.width, "h": info.height,
            "ox": info.origin.position.x, "oy": info.origin.position.y,
            # int8 occupancy values, row-major from the origin corner
            "data": list(msg.data),
        })

    def on_scan(self, msg: LaserScan):
        now = time.monotonic()
        if now - self.last_scan_write < 0.5:   # ~2 Hz is plenty for rendering
            return
        pose = self.current_pose()
        if pose is None:
            return
        self.last_scan_write = now
        self.write("scan", {
            "amin": msg.angle_min, "ainc": msg.angle_increment,
            "ranges": [round(r, 3) if math.isfinite(r) else None for r in msg.ranges],
            "pose": pose,
        })

    def on_plan(self, msg: Path):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
               for p in msg.poses]
        if pts:
            self.write("plan", {"points": pts})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", required=True)
    p.add_argument("--duration", type=float, default=900.0)
    args = p.parse_args()

    rclpy.init()
    node = RunRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info(f"final counts: {node.n}")
        node.f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
