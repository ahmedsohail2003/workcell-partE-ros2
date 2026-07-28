#!/usr/bin/env python3
"""Reactive exploration driver for TurtleBot3 mapping runs (CellOps / WorkCell Part E).

Drives the robot around the Gazebo world autonomously so `slam_toolbox` sees
enough of it to close a map — no keyboard teleop, so the whole mapping session
runs non-interactively from a single command.

Why not open-loop `ros2 topic pub` bursts: a fixed forward/turn sequence drives
straight into the tb3_world cylinders within a few metres. This node closes the
loop on /scan instead: drive forward while the front sector is clear, otherwise
turn toward whichever side has more room, with an occasional random turn so the
robot doesn't settle into a limit cycle around the same loop of the world.

IMPORTANT (ROS 2 Jazzy): TurtleBot3's ros_gz bridge takes
`geometry_msgs/msg/TwistStamped` on /cmd_vel, NOT plain `Twist` — publishing
Twist here fails *silently* (robot never moves, no error anywhere).

Usage (inside WSL, after sourcing ROS):
    python3 explore_drive.py --duration 180
"""

import argparse
import math
import random
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def sector_min(scan: LaserScan, center_deg: float, half_width_deg: float) -> float:
    """Smallest valid range in an angular sector centred on `center_deg`.

    Returns +inf when the sector holds no valid returns (open space or dropout),
    which the caller treats as "clear" — the conservative direction here, since
    a dropout in front still gets caught by the neighbouring beams.
    """
    n = len(scan.ranges)
    if n == 0:
        return math.inf

    best = math.inf
    lo = math.radians(center_deg - half_width_deg)
    hi = math.radians(center_deg + half_width_deg)
    for i, r in enumerate(scan.ranges):
        # Drop inf/nan and out-of-spec returns before comparing.
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        ang = scan.angle_min + i * scan.angle_increment
        # Wrap to [-pi, pi) so a sector straddling 0 (the front) works.
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        if lo <= ang <= hi:
            best = min(best, r)
    return best


class ExplorerDriver(Node):
    def __init__(self, args):
        super().__init__("cellops_explore_driver")

        self.duration = args.duration
        self.fwd_speed = args.fwd_speed
        self.turn_speed = args.turn_speed
        self.stop_dist = args.stop_dist
        self.side_clearance = args.side_clearance

        # Sensor data QoS: Gazebo's scan publisher is BEST_EFFORT. A default
        # RELIABLE subscription silently never matches it -> no scans, ever.
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, "scan", self.on_scan, scan_qos)
        self.pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        self.scan = None
        self.scans_seen = 0
        self.turn_dir = 0.0        # 0 = driving forward; ±1 = turning
        self.turn_until = 0.0      # sim-time deadline for the current turn
        self.start_time = None
        self.last_report = 0.0
        # Watchdog: Gazebo under WSL can freeze mid-run (pose/tf/scan all stop
        # while wall time marches on). Without this, the driver keeps publishing
        # into a dead sim for minutes and the run "succeeds" with half the data.
        self.last_scan_wall = time.monotonic()

        self.create_timer(0.1, self.tick)  # 10 Hz control loop
        self.get_logger().info(
            f"explore driver up: {self.duration:.0f}s, fwd={self.fwd_speed} m/s, "
            f"stop_dist={self.stop_dist} m — waiting for /scan"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.scans_seen += 1
        self.last_scan_wall = time.monotonic()

    def publish(self, lin: float, ang: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        self.pub.publish(msg)

    def stop(self):
        # gz's diff-drive keeps executing the last command forever; an explicit
        # zero is the only thing that actually halts the robot.
        for _ in range(5):
            self.publish(0.0, 0.0)

    def tick(self):
        if self.scan is None:
            return

        if time.monotonic() - self.last_scan_wall > 12.0:
            self.get_logger().error(
                "no /scan for 12s (wall) — simulator froze or died, aborting")
            self.stop()
            raise SystemExit(2)

        t = self.now_sec()
        if self.start_time is None:
            self.start_time = t
            self.get_logger().info("first scan received — driving")

        elapsed = t - self.start_time
        if elapsed >= self.duration:
            self.stop()
            self.get_logger().info(
                f"done: drove {elapsed:.0f}s, {self.scans_seen} scans — stopping"
            )
            raise SystemExit(0)

        if elapsed - self.last_report >= 15.0:
            self.last_report = elapsed
            self.get_logger().info(f"exploring... {elapsed:.0f}/{self.duration:.0f}s")

        front = sector_min(self.scan, 0.0, 25.0)
        left = sector_min(self.scan, 60.0, 30.0)
        right = sector_min(self.scan, -60.0, 30.0)

        # Mid-turn: keep turning until the deadline and the front is clear again.
        if self.turn_dir != 0.0:
            if t < self.turn_until or front < self.stop_dist * 1.2:
                self.publish(0.0, self.turn_dir * self.turn_speed)
                return
            self.turn_dir = 0.0

        if front < self.stop_dist:
            # Turn toward the roomier side; tie-break randomly so symmetric
            # corners don't lock the robot into a back-and-forth.
            if abs(left - right) < 0.1:
                self.turn_dir = random.choice((-1.0, 1.0))
            else:
                self.turn_dir = 1.0 if left > right else -1.0
            self.turn_until = t + random.uniform(0.8, 2.0)
            self.publish(0.0, self.turn_dir * self.turn_speed)
            return

        # Occasional random turn: pure wall-following retraces one loop of the
        # world forever and leaves the interior unmapped.
        if random.random() < 0.004:
            self.turn_dir = random.choice((-1.0, 1.0))
            self.turn_until = t + random.uniform(0.5, 1.5)
            return

        # Gentle steer away from a wall we're running alongside.
        steer = 0.0
        if left < self.side_clearance:
            steer -= 0.4
        if right < self.side_clearance:
            steer += 0.4

        # Ease off the throttle as the front closes in.
        speed = self.fwd_speed
        if front < self.stop_dist * 2.0:
            speed *= 0.6
        self.publish(speed, steer * self.turn_speed)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=180.0, help="drive time in sim seconds")
    p.add_argument("--fwd-speed", type=float, default=0.16, help="m/s (burger max 0.22)")
    p.add_argument("--turn-speed", type=float, default=0.9, help="rad/s (burger max 2.84)")
    p.add_argument("--stop-dist", type=float, default=0.42, help="front obstacle threshold (m)")
    p.add_argument("--side-clearance", type=float, default=0.35, help="side steer-away threshold (m)")
    p.add_argument("--seed", type=int, default=0, help="RNG seed for reproducible runs")
    args = p.parse_args()

    random.seed(args.seed)
    rclpy.init()
    node = ExplorerDriver(args)
    code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as exc:  # normal completion (0) or watchdog abort (2)
        code = int(exc.code or 0)
    except Exception as exc:  # noqa: BLE001 - report then still stop the robot
        node.get_logger().error(f"driver failed: {exc}")
        code = 1
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
