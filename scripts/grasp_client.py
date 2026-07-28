#!/usr/bin/env python3
"""Round-trip demo client for the C++ grasp service (runs in WSL).

Loads an exported Part C cloud fixture, packs it into a PointCloud2, calls
/estimate_cube_pose on the C++ node, and prints the response next to the
ground truth — proving the full ROS 2 service path (serialization included),
not just the estimator math.

Usage: python3 grasp_client.py <testdata_dir> [cloud_index]
"""
import json
import struct
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from cellops_interfaces.srv import EstimateCubePose


def make_cloud_msg(points, frame_id="world") -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id)
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate("xyz")
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    buf = bytearray()
    for x, y, z in points:
        buf += struct.pack("<fff", x, y, z)
    msg.data = bytes(buf)
    return msg


def main():
    testdata = Path(sys.argv[1])
    idx = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    meta = json.loads((testdata / "truth.json").read_text())
    trial = meta["trials"][idx]

    points = []
    for line in (testdata / trial["cloud"]).read_text().splitlines():
        if line.strip():
            points.append([float(v) for v in line.split(",")])

    rclpy.init()
    node = Node("grasp_client")
    client = node.create_client(EstimateCubePose, "estimate_cube_pose")
    if not client.wait_for_service(timeout_sec=15.0):
        print("FAIL: service not available")
        sys.exit(1)

    req = EstimateCubePose.Request()
    req.cloud = make_cloud_msg(points)
    req.table_z = trial["table_z"]
    req.half_extent = 0.012
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    res = future.result()
    if res is None:
        print("FAIL: no response")
        sys.exit(1)

    p = res.grasp_pose.pose.position
    import math
    yaw_err = (res.yaw - trial["gt_yaw"]) % (math.pi / 2)
    yaw_err = min(yaw_err, math.pi / 2 - yaw_err)
    pos_err = math.hypot(p.x - trial["gt_x"], p.y - trial["gt_y"]) * 1000
    print(f"cloud       : {trial['cloud']} ({trial['n_points']} pts)")
    print(f"estimate    : ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})  yaw {math.degrees(res.yaw):6.2f} deg")
    print(f"ground truth: ({trial['gt_x']:.4f}, {trial['gt_y']:.4f})  yaw {math.degrees(trial['gt_yaw']):6.2f} deg")
    print(f"errors      : pos {pos_err:.2f} mm   yaw {math.degrees(yaw_err):.2f} deg (mod 90)")
    print(f"fit         : rmse {res.rmse * 1000:.2f} mm, {res.iters} iters, {res.solve_ms:.2f} ms solve")
    print("ROUND-TRIP OK" if pos_err < 5.0 else "ROUND-TRIP SUSPICIOUS")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
