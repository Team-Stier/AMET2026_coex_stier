#!/usr/bin/env python3
"""Drive a PhysiCar simulator route slowly for repeatable map collection.

This is intentionally a SIM-only adapter. It reads ground-truth world pose from
the simulator API, follows an existing waypoint JSON with ControllerCore, and
publishes only /speed, /steering, and a fixed /camera/pan command. Sensor and
SLAM recording remain separate so the source bag is preserved unchanged.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Sequence
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

CONTROL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(CONTROL_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_PACKAGE_ROOT))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

from control.controller_core import ControllerCore  # noqa: E402
from control.models import (  # noqa: E402
    AdaptiveControlConfig,
    ControllerConfig,
    PIDConfig,
    PathPoint,
    PurePursuitConfig,
    VehicleState,
)
from control.pid import PIDController  # noqa: E402
from control.pure_pursuit import PurePursuit  # noqa: E402

DEFAULT_PATH = CONTROL_PACKAGE_ROOT / "config" / "sim_mapping_waypoints.json"
DEFAULT_SIM_API = "http://localhost/sim/api"
DUPLICATE_ENDPOINT_TOLERANCE_M = 1.0e-6
WHEELBASE_M = 0.18
STEERING_LIMIT_RAD = 0.3491


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def distance(first: PathPoint, second: PathPoint) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def load_path(path_file: Path) -> tuple[list[PathPoint], dict[str, object]]:
    payload = json.loads(path_file.read_text(encoding="utf-8"))
    values = payload.get("waypoints")
    if not isinstance(values, list) or len(values) < 3:
        raise ValueError("waypoint JSON must contain at least three points")
    points = [PathPoint(float(value[0]), float(value[1])) for value in values]
    if not all(math.isfinite(point.x) and math.isfinite(point.y) for point in points):
        raise ValueError("waypoint JSON contains a non-finite coordinate")
    if distance(points[0], points[-1]) <= DUPLICATE_ENDPOINT_TOLERANCE_M:
        points.pop()
    if len(points) < 3:
        raise ValueError("closed waypoint path has fewer than three logical points")
    metadata = payload.get("metadata")
    return points, metadata if isinstance(metadata, dict) else {}


def path_length(points: Sequence[PathPoint]) -> float:
    return sum(
        distance(points[index], points[(index + 1) % len(points)])
        for index in range(len(points))
    )


class SimApi:
    def __init__(self, base_url: str, timeout_sec: float) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec

    def get(self, endpoint: str) -> dict[str, object]:
        try:
            with urlopen(
                f"{self.base_url}/{endpoint.lstrip('/')}",
                timeout=self.timeout_sec,
            ) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"simulator API request failed: {error}") from error
        if not isinstance(payload, dict):
            raise RuntimeError(f"simulator API {endpoint} returned non-object JSON")
        return payload

    def pose(self) -> tuple[float, float, float]:
        payload = self.get("pose")
        try:
            pose = float(payload["x"]), float(payload["y"]), float(payload["yaw"])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(f"invalid simulator pose: {payload}") from error
        if not all(math.isfinite(value) for value in pose):
            raise RuntimeError(f"non-finite simulator pose: {payload}")
        return pose


class MappingDriveNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "sim_waypoint_mapping_drive",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ],
        )
        self.speed_pub = self.create_publisher(Float64, "/speed", 10)
        self.steering_pub = self.create_publisher(Float64, "/steering", 10)
        self.camera_pan_pub = self.create_publisher(Float64, "/camera/pan", 10)
        self.sim_pose_pub = self.create_publisher(
            PoseStamped, "/mapping/sim_pose", 10
        )
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.measured_speed_m_s: float | None = None
        self.last_odom_monotonic: float | None = None

    def on_odom(self, message: Odometry) -> None:
        speed = float(message.twist.twist.linear.x)
        if math.isfinite(speed):
            self.measured_speed_m_s = speed
            self.last_odom_monotonic = time.monotonic()

    def publish(self, speed_m_s: float, steering_rad: float, camera_pan_rad: float) -> None:
        speed = Float64()
        speed.data = float(speed_m_s)
        steering = Float64()
        steering.data = float(steering_rad)
        camera_pan = Float64()
        camera_pan.data = float(camera_pan_rad)
        self.speed_pub.publish(speed)
        self.steering_pub.publish(steering)
        self.camera_pan_pub.publish(camera_pan)

    def publish_sim_pose(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "sim_world"
        message.pose.position.x = float(x_m)
        message.pose.position.y = float(y_m)
        message.pose.orientation.z = math.sin(yaw_rad * 0.5)
        message.pose.orientation.w = math.cos(yaw_rad * 0.5)
        self.sim_pose_pub.publish(message)

    def safe_stop(self, repetitions: int = 12) -> None:
        for _ in range(repetitions):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)


def make_controller(target_speed_m_s: float) -> ControllerCore:
    pure_pursuit_config = PurePursuitConfig(
        wheelbase_m=WHEELBASE_M,
        lookahead_distance_m=0.30,
        max_steering_rad=STEERING_LIMIT_RAD,
        closed_loop=True,
    )
    pid_config = PIDConfig(
        kp=0.0,
        ki=0.0,
        kd=0.0,
        output_min=-0.2,
        output_max=0.2,
        integral_min=-1.0,
        integral_max=1.0,
    )
    adaptive_config = AdaptiveControlConfig(
        enabled=True,
        preview_distance_m=1.0,
        min_lookahead_m=0.25,
        max_lookahead_m=0.40,
        curvature_reference_inv_m=2.0,
        max_lateral_acceleration_m_s2=0.45,
        min_speed_limit_m_s=min(0.30, target_speed_m_s),
        max_speed_limit_m_s=target_speed_m_s,
    )
    return ControllerCore(
        PurePursuit(pure_pursuit_config),
        PIDController(pid_config),
        ControllerConfig(
            longitudinal_pid_enabled=False,
            max_speed_m_s=target_speed_m_s,
            adaptive_control=adaptive_config,
        ),
    )


def nearest_index(x_m: float, y_m: float, points: Sequence[PathPoint]) -> int:
    return min(
        range(len(points)),
        key=lambda index: math.hypot(points[index].x - x_m, points[index].y - y_m),
    )


def preflight(
    node: MappingDriveNode,
    api: SimApi,
    points: Sequence[PathPoint],
    metadata: dict[str, object],
    maximum_path_error_m: float,
) -> int:
    status = api.get("status")
    if not status.get("running"):
        raise RuntimeError(f"simulator is not running: {status}")
    expected_world = metadata.get("world")
    actual_world = status.get("current")
    if expected_world and expected_world != actual_world:
        raise RuntimeError(
            f"waypoint world {expected_world!r} does not match simulator {actual_world!r}"
        )

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and node.measured_speed_m_s is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.measured_speed_m_s is None:
        raise RuntimeError("no /odom sample received during preflight")
    if node.count_subscribers("/speed") < 1 or node.count_subscribers("/steering") < 1:
        raise RuntimeError("vehicle command subscribers are not connected")

    x_m, y_m, yaw_rad = api.pose()
    start_index = nearest_index(x_m, y_m, points)
    start_error = math.hypot(points[start_index].x - x_m, points[start_index].y - y_m)
    if start_error > maximum_path_error_m:
        raise RuntimeError(
            f"vehicle is {start_error:.3f} m from the waypoint path; "
            f"limit is {maximum_path_error_m:.3f} m"
        )
    following = points[(start_index + 1) % len(points)]
    path_yaw = math.atan2(
        following.y - points[start_index].y,
        following.x - points[start_index].x,
    )
    heading_error = abs(normalize_angle(path_yaw - yaw_rad))
    if heading_error > math.radians(60.0):
        raise RuntimeError(
            f"vehicle heading differs from waypoint direction by "
            f"{math.degrees(heading_error):.1f} degrees"
        )
    return start_index


def drive(args: argparse.Namespace) -> None:
    points, metadata = load_path(args.path)
    route_length_m = path_length(points)
    api = SimApi(args.sim_api, args.api_timeout_sec)
    node = MappingDriveNode()
    controller = make_controller(args.target_speed_m_s)
    started: float | None = None
    maximum_error_m = 0.0
    try:
        previous_index = preflight(
            node, api, points, metadata, args.maximum_path_error_m
        )
        progressed_points = 0
        target_points = args.laps * len(points)
        maximum_forward_jump = max(10, len(points) // 20)
        started = time.monotonic()
        previous_tick = started
        next_tick = started
        next_report = started
        timeout_sec = args.timeout_sec or (
            args.laps * route_length_m / max(args.target_speed_m_s, 0.1) * 2.0 + 30.0
        )
        print(
            f"mapping drive started: points={len(points)} length={route_length_m:.3f} m "
            f"laps={args.laps} target_speed={args.target_speed_m_s:.2f} m/s "
            f"start_index={previous_index}",
            flush=True,
        )

        while progressed_points < target_points:
            now = time.monotonic()
            if now - started > timeout_sec:
                raise RuntimeError(f"mapping drive exceeded {timeout_sec:.1f} s timeout")
            rclpy.spin_once(node, timeout_sec=0.0)
            if (
                node.last_odom_monotonic is None
                or now - node.last_odom_monotonic > 1.0
            ):
                raise RuntimeError("/odom became stale during mapping drive")
            x_m, y_m, yaw_rad = api.pose()
            node.publish_sim_pose(x_m, y_m, yaw_rad)
            current_speed = float(node.measured_speed_m_s or 0.0)
            dt = max(1.0e-4, now - previous_tick)
            previous_tick = now
            result = controller.update(
                VehicleState(x_m, y_m, yaw_rad, current_speed),
                points,
                args.target_speed_m_s,
                dt,
            )
            current_index = result.pure_pursuit.nearest_index
            path_error_m = math.hypot(
                result.pure_pursuit.nearest_point.x - x_m,
                result.pure_pursuit.nearest_point.y - y_m,
            )
            maximum_error_m = max(maximum_error_m, path_error_m)
            if path_error_m > args.maximum_path_error_m:
                raise RuntimeError(
                    f"path error reached {path_error_m:.3f} m at index {current_index}"
                )
            if abs(result.pure_pursuit.alpha_rad) > math.pi / 2.0:
                raise RuntimeError("Pure Pursuit target moved behind the vehicle")

            forward = (current_index - previous_index) % len(points)
            if 0 < forward <= maximum_forward_jump:
                progressed_points += forward
            previous_index = current_index
            node.publish(
                result.speed_command_m_s,
                result.steering_rad,
                args.camera_pan_rad,
            )

            if now >= next_report:
                print(
                    f"progress={progressed_points / len(points):.2f}/{args.laps} laps "
                    f"index={current_index} error={path_error_m:.3f} m "
                    f"max_error={maximum_error_m:.3f} m "
                    f"command={result.speed_command_m_s:.3f} m/s",
                    flush=True,
                )
                next_report = now + 5.0

            next_tick += 1.0 / args.control_rate_hz
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)

        elapsed = time.monotonic() - started
        print(
            f"mapping drive complete: laps={progressed_points / len(points):.2f} "
            f"elapsed={elapsed:.1f} s maximum_path_error={maximum_error_m:.3f} m",
            flush=True,
        )
    finally:
        node.safe_stop()
        node.destroy_node()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", type=Path, default=DEFAULT_PATH)
    parser.add_argument("--sim-api", default=DEFAULT_SIM_API)
    parser.add_argument("--laps", type=int, default=2)
    parser.add_argument("--target-speed-m-s", type=float, default=0.55)
    parser.add_argument("--maximum-path-error-m", type=float, default=0.20)
    parser.add_argument("--camera-pan-rad", type=float, default=0.0)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--api-timeout-sec", type=float, default=0.8)
    parser.add_argument("--timeout-sec", type=float)
    args = parser.parse_args(argv)
    if not args.path.is_file():
        parser.error(f"waypoint path does not exist: {args.path}")
    if args.laps < 1:
        parser.error("--laps must be at least one")
    if not 0.05 <= args.target_speed_m_s <= 0.8:
        parser.error("--target-speed-m-s must be within 0.05..0.8")
    if args.maximum_path_error_m <= 0.0:
        parser.error("--maximum-path-error-m must be positive")
    if args.control_rate_hz <= 0.0 or args.api_timeout_sec <= 0.0:
        parser.error("control rate and API timeout must be positive")
    if args.timeout_sec is not None and args.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be positive")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    try:
        drive(args)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except Exception as error:
        print(f"mapping drive failed: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
