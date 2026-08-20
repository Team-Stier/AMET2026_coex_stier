#!/usr/bin/env python3
"""Read-only RViz bridge for the SIM-only control autotune session.

This node deliberately has no ControllerCore dependency and creates publishers
only for ``/debug/*`` topics.  It never publishes vehicle commands.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
from typing import Deque, List, Optional, Sequence, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


DEFAULT_PATH = Path("/tmp/amet_merged_fillet_route.json")
DEFAULT_POSE_URL = "http://localhost/sim/api/pose"
DEFAULT_FRAME = "map"
DEFAULT_RATE_HZ = 10.0
DEFAULT_MAX_TRAJECTORY_POINTS = 4000
RESET_JUMP_THRESHOLD_M = 0.30


def load_waypoints(path: Path) -> List[Tuple[float, float]]:
    with path.open("r", encoding="utf-8") as source:
        payload = json.load(source)
    values = payload.get("waypoints")
    if not isinstance(values, list) or len(values) < 2:
        raise ValueError("mock path JSON has no usable waypoints")
    points = [(float(value[0]), float(value[1])) for value in values]
    if not all(math.isfinite(x) and math.isfinite(y) for x, y in points):
        raise ValueError("mock path contains non-finite coordinates")
    return points


class SimControlVisualizer(Node):
    """Publish world-absolute debug geometry without affecting control."""

    def __init__(
        self,
        path_file: Path,
        pose_url: str,
        frame_id: str,
        rate_hz: float,
        max_trajectory_points: int,
        api_timeout_s: float,
    ) -> None:
        super().__init__("sim_control_visualizer")
        self.frame_id = frame_id
        self.pose_url = pose_url
        self.api_timeout_s = api_timeout_s
        self.waypoints = load_waypoints(path_file)
        self.trajectory: Deque[PoseStamped] = deque(
            maxlen=max_trajectory_points
        )
        self.previous_xy: Optional[Tuple[float, float]] = None
        self.failure_count = 0

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        live_qos = QoSProfile(depth=10)
        live_qos.reliability = ReliabilityPolicy.RELIABLE

        # These are the only publishers in this process.  In particular there
        # are no /speed or /steering publishers.
        self.mock_path_pub = self.create_publisher(
            PathMessage, "/debug/mock_path", latched_qos
        )
        self.sim_pose_pub = self.create_publisher(
            PoseStamped, "/debug/sim_pose", live_qos
        )
        self.trajectory_pub = self.create_publisher(
            PathMessage, "/debug/actual_trajectory", live_qos
        )

        self.mock_path_message = self._make_mock_path()
        self.mock_path_pub.publish(self.mock_path_message)
        self.path_timer = self.create_timer(2.0, self._publish_mock_path)
        self.pose_timer = self.create_timer(1.0 / rate_hz, self._update_pose)
        self.get_logger().info(
            f"publishing {len(self.waypoints)} path points in frame "
            f"'{self.frame_id}' at {rate_hz:.1f} Hz"
        )

    def _make_mock_path(self) -> PathMessage:
        message = PathMessage()
        message.header.frame_id = self.frame_id
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def _publish_mock_path(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.mock_path_message.header.stamp = stamp
        for pose in self.mock_path_message.poses:
            pose.header.stamp = stamp
        self.mock_path_pub.publish(self.mock_path_message)

    def _read_pose(self) -> Tuple[float, float, float]:
        try:
            with urlopen(self.pose_url, timeout=self.api_timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"SIM pose request failed: {error}") from error
        values = float(payload["x"]), float(payload["y"]), float(payload["yaw"])
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("SIM pose contains a non-finite value")
        return values

    def _update_pose(self) -> None:
        try:
            x, y, yaw = self._read_pose()
        except Exception as error:
            self.failure_count += 1
            if self.failure_count == 1 or self.failure_count % 50 == 0:
                self.get_logger().warning(str(error))
            return
        self.failure_count = 0

        current_xy = (x, y)
        if self.previous_xy is not None:
            jump = math.hypot(
                current_xy[0] - self.previous_xy[0],
                current_xy[1] - self.previous_xy[1],
            )
            if jump > RESET_JUMP_THRESHOLD_M:
                self.trajectory.clear()
                self.get_logger().info(
                    f"SIM reset/teleport detected ({jump:.3f} m); "
                    "cleared actual trajectory"
                )
        self.previous_xy = current_xy

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.trajectory.append(pose)
        self.sim_pose_pub.publish(pose)

        trajectory = PathMessage()
        trajectory.header.stamp = stamp
        trajectory.header.frame_id = self.frame_id
        trajectory.poses = list(self.trajectory)
        self.trajectory_pub.publish(trajectory)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", type=Path, default=DEFAULT_PATH)
    parser.add_argument("--pose-url", default=DEFAULT_POSE_URL)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument(
        "--max-trajectory-points",
        type=int,
        default=DEFAULT_MAX_TRAJECTORY_POINTS,
    )
    parser.add_argument("--api-timeout-s", type=float, default=0.5)
    args, ros_args = parser.parse_known_args(argv)
    args.ros_args = ros_args
    if not 1.0 <= args.rate_hz <= 30.0:
        parser.error("--rate-hz must be within 1..30 Hz")
    if not 100 <= args.max_trajectory_points <= 10000:
        parser.error("--max-trajectory-points must be within 100..10000")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    rclpy.init(args=args.ros_args)
    node = SimControlVisualizer(
        path_file=args.path,
        pose_url=args.pose_url,
        frame_id=args.frame_id,
        rate_hz=args.rate_hz,
        max_trajectory_points=args.max_trajectory_points,
        api_timeout_s=args.api_timeout_s,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
