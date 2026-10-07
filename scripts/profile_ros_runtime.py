#!/usr/bin/env python3
"""Measure live EdgePick topic freshness and callback periods.

This is an observation-only tool. It does not publish commands and can run
next to ``edgepick_system.launch.py`` in mock, MuJoCo, or real mode. The
header-age values are meaningful when the camera and host use the same ROS
clock; inter-arrival periods are measured from the local monotonic clock.
"""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

try:
    import rclpy
    from edgepick_interfaces.msg import VLAAction
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
except ImportError as exc:  # Allow ``--help`` before the ROS workspace is sourced.
    rclpy = None
    Node = object
    VLAAction = Image = JointState = None
    qos_profile_sensor_data = None
    ROS_IMPORT_ERROR = exc
else:
    ROS_IMPORT_ERROR = None


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))]


def describe(samples: list[float]) -> dict[str, float | int | None]:
    if not samples:
        return {"count": 0, "p50": None, "p95": None, "min": None, "max": None}
    return {
        "count": len(samples),
        "p50": statistics.median(samples),
        "p95": percentile(samples, 0.95),
        "min": min(samples),
        "max": max(samples),
    }


class RuntimeProfiler(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("edgepick_runtime_profiler")
        self.started = time.monotonic()
        self.duration = args.duration
        self.stats: dict[str, dict[str, list[float] | float | int]] = {}
        self._subscribe("rgb", args.image_topic, Image)
        self._subscribe("depth", args.depth_topic, Image)
        self._subscribe("joints", args.joint_topic, JointState)
        self._subscribe("proposal", args.proposal_topic, VLAAction)

    def _subscribe(self, name: str, topic: str, message_type: type) -> None:
        self.stats[name] = {"count": 0, "last_wall": 0.0, "periods": [], "ages": []}
        self.create_subscription(
            message_type,
            topic,
            lambda message, key=name: self._record(key, message),
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"profiling {name}: {topic}")

    def _record(self, name: str, message) -> None:
        now_wall = time.monotonic()
        entry = self.stats[name]
        previous = float(entry["last_wall"])
        if previous > 0:
            period_ms = (now_wall - previous) * 1000.0
            if math.isfinite(period_ms):
                entry["periods"].append(period_ms)  # type: ignore[union-attr]
        entry["last_wall"] = now_wall
        entry["count"] = int(entry["count"]) + 1

        header = getattr(message, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            return
        ros_now = self.get_clock().now().nanoseconds / 1e9
        age_ms = (ros_now - (stamp.sec + stamp.nanosec * 1e-9)) * 1000.0
        # A negative value usually means mixed wall/sim clocks; omit it from
        # the report instead of presenting a misleading freshness result.
        if math.isfinite(age_ms) and age_ms >= 0:
            entry["ages"].append(age_ms)  # type: ignore[union-attr]

    def finished(self) -> bool:
        return time.monotonic() - self.started >= self.duration

    def report(self) -> dict:
        result = {
            "duration_s": self.duration,
            "topics": {},
        }
        for name, entry in self.stats.items():
            result["topics"][name] = {
                "messages": entry["count"],
                "period_ms": describe(entry["periods"]),
                "header_age_ms": describe(entry["ages"]),
            }
        return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--proposal-topic", default="/edgepick/vla/proposal")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be > 0")
    if rclpy is None:
        parser.error(
            "ROS 2 Python packages are unavailable; source the EdgePick workspace "
            f"before profiling ({ROS_IMPORT_ERROR})"
        )

    rclpy.init()
    node = RuntimeProfiler(args)
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    encoded = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
