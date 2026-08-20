#!/usr/bin/env python3
"""Unattended, SIM-only ControllerCore parameter tuning for PhysiCar.

The production controller modules remain ROS independent.  This executable is
an intentionally separate adapter: it reads world pose from the simulator API,
measured speed from ``/odom``, and publishes ``/speed`` and ``/steering``.

Every exit path attempts a repeated zero-speed/zero-steering safe stop.  Results
are appended and fsynced after every experiment so an interrupted session keeps
all completed data.
"""

from __future__ import annotations

import argparse
import atexit
import csv
import fcntl
import json
import logging
import math
import os
from pathlib import Path
import signal
import statistics
import sys
import tempfile
import time
import traceback
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


CONTROL_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(CONTROL_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_PACKAGE_ROOT))

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
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


DEFAULT_PATH = Path("/tmp/amet_merged_fillet_route.json")
DEFAULT_OUTPUT_DIR = Path("/tmp/amet_autotune")
DEFAULT_SIM_API = "http://localhost/sim/api"

WHEELBASE_M = 0.18
BASE_LOOKAHEAD_M = 0.45
STEERING_LIMIT_RAD = 0.3491
PLANNER_TARGET_SPEED_M_S = 0.80
MIN_SPEED_LIMIT_M_S = 0.30
MAX_SPEED_LIMIT_M_S = 0.80
PID_OUTPUT_LIMIT_M_S = 0.20

PATH_ERROR_STOP_M = 0.30
START_PATH_DISTANCE_LIMIT_M = 0.16
START_HEADING_ERROR_LIMIT_RAD = math.radians(50.0)
STOPPED_SPEED_LIMIT_M_S = 0.035
ODOM_STALE_TIMEOUT_S = 1.0
TARGET_BEHIND_GRACE_S = 0.8
POSE_FAILURE_LIMIT = 3
CONTROL_EXCEPTION_LIMIT = 2
RUN_TIMEOUT_LIMIT = 2
SATURATION_EPSILON_RAD = 1.0e-4

BASELINE_HISTORICAL = {
    "lap_time": 46.9,
    "mean_path_error": 0.025,
    "max_path_error": 0.050,
    "mean_target_command": 0.690,
    "max_preview_curvature": 1.800,
    "max_abs_steering": math.radians(19.7),
}

NUMERIC_METRICS = (
    "lap_time",
    "elapsed_time",
    "mean_path_error",
    "rms_path_error",
    "p95_path_error",
    "max_path_error",
    "mean_abs_steering",
    "p95_abs_steering",
    "max_abs_steering",
    "mean_abs_steering_rate",
    "p95_steering_rate",
    "max_steering_rate",
    "mean_curvature",
    "max_preview_curvature",
    "mean_lookahead",
    "min_lookahead_observed",
    "max_lookahead_observed",
    "mean_target_command",
    "min_command",
    "max_command",
    "measured_speed_mean",
    "measured_speed_error",
    "mean_abs_measured_speed_error",
    "score",
)

CSV_FIELDS = (
    "session_id",
    "run_id",
    "timestamp_utc",
    "phase",
    "experiment_type",
    "label",
    "min_lookahead",
    "max_lookahead",
    "curvature_reference",
    "preview_distance",
    "max_lateral_acceleration",
    "pid_enabled",
    "kp",
    "ki",
    "kd",
    "success",
    "lap_completed",
    "lap_time",
    "elapsed_time",
    "mean_path_error",
    "rms_path_error",
    "p95_path_error",
    "max_path_error",
    "mean_abs_steering",
    "p95_abs_steering",
    "max_abs_steering",
    "steering_saturation_count",
    "mean_abs_steering_rate",
    "p95_steering_rate",
    "max_steering_rate",
    "mean_curvature",
    "max_preview_curvature",
    "mean_lookahead",
    "min_lookahead_observed",
    "max_lookahead_observed",
    "mean_target_command",
    "min_command",
    "max_command",
    "measured_speed_mean",
    "measured_speed_error",
    "mean_abs_measured_speed_error",
    "path_error_stop",
    "target_behind",
    "timeout",
    "pose_api_error",
    "other_exception",
    "score",
    "step_score",
    "termination_hint",
    "error_message",
)

RAW_FIELDS = (
    "elapsed_s",
    "segment",
    "x_m",
    "y_m",
    "yaw_rad",
    "measured_speed_m_s",
    "planner_target_m_s",
    "effective_target_m_s",
    "speed_command_m_s",
    "steering_rad",
    "steering_rate_rad_s",
    "path_error_m",
    "nearest_index",
    "lap_progress_points",
    "target_index",
    "target_alpha_rad",
    "preview_curvature_inv_m",
    "lookahead_m",
    "speed_limit_m_s",
    "pid_error",
    "pid_output",
)


class SafetyAbort(RuntimeError):
    """Raised when further autonomous motion is not considered safe."""


class RunAbort(RuntimeError):
    """A classified experiment failure."""

    def __init__(self, failure: str, message: str, fatal: bool = False) -> None:
        super().__init__(message)
        self.failure = failure
        self.fatal = fatal


@dataclass(frozen=True)
class TrialConfig:
    min_lookahead: float = 0.25
    max_lookahead: float = 0.45
    curvature_reference: float = 2.0
    preview_distance: float = 1.0
    max_lateral_acceleration: float = 0.8
    pid_enabled: bool = False
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0

    def key(self) -> Tuple[Any, ...]:
        return (
            round(self.min_lookahead, 4),
            round(self.max_lookahead, 4),
            round(self.curvature_reference, 4),
            round(self.preview_distance, 4),
            round(self.max_lateral_acceleration, 4),
            self.pid_enabled,
            round(self.kp, 5),
            round(self.ki, 5),
            round(self.kd, 5),
        )

    def short_name(self) -> str:
        pid = (
            f"PID({self.kp:.3f},{self.ki:.3f},{self.kd:.3f})"
            if self.pid_enabled
            else "PID_OFF"
        )
        return (
            f"L{self.min_lookahead:.3f}-{self.max_lookahead:.3f}_"
            f"C{self.curvature_reference:.2f}_P{self.preview_distance:.2f}_"
            f"A{self.max_lateral_acceleration:.2f}_{pid}"
        )


BASELINE_CONFIG = TrialConfig()


class SimApi:
    def __init__(self, base_url: str, timeout_s: float = 0.8) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s

    def _request(
        self, endpoint: str, method: str = "GET", payload: Any = None
    ) -> Dict[str, Any]:
        data = None
        headers: Dict[str, str] = {}
        if method == "POST":
            data = json.dumps(payload if payload is not None else {}).encode(
                "utf-8"
            )
            headers["Content-Type"] = "application/json"
        request = Request(
            f"{self.base_url}/{endpoint.lstrip('/')}",
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                body = response.read().decode("utf-8")
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            raise RuntimeError(
                f"SIM API {method} {endpoint} failed: {error}"
            ) from error
        if not body.strip():
            return {}
        decoded = json.loads(body)
        if not isinstance(decoded, dict):
            raise RuntimeError(f"SIM API {endpoint} returned non-object JSON")
        return decoded

    def status(self) -> Dict[str, Any]:
        return self._request("status")

    def pose(self) -> Tuple[float, float, float]:
        result = self._request("pose")
        try:
            values = float(result["x"]), float(result["y"]), float(result["yaw"])
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(f"invalid SIM pose response: {result}") from error
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"non-finite SIM pose response: {result}")
        return values

    def reset(self) -> None:
        self._request("reset", method="POST")

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self._request(
            "pose",
            method="POST",
            payload={"x": x, "y": y, "yaw": yaw},
        )


class AutotuneNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_control_autotune")
        self.speed_pub = self.create_publisher(Float64, "/speed", 10)
        self.steering_pub = self.create_publisher(Float64, "/steering", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, 10
        )
        self.speed_m_s: Optional[float] = None
        self.odom_received_monotonic: Optional[float] = None

    def _on_odom(self, message: Odometry) -> None:
        speed = float(message.twist.twist.linear.x)
        if math.isfinite(speed):
            self.speed_m_s = speed
            self.odom_received_monotonic = time.monotonic()

    def publish(self, speed_m_s: float, steering_rad: float) -> None:
        speed_message = Float64()
        speed_message.data = float(speed_m_s)
        steering_message = Float64()
        steering_message.data = float(steering_rad)
        self.speed_pub.publish(speed_message)
        self.steering_pub.publish(steering_message)


class ResultStore:
    def __init__(self, output_dir: Path, session_id: str) -> None:
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.session_id = session_id
        self.runs_csv = output_dir / "runs.csv"
        self.runs_jsonl = output_dir / "runs.jsonl"
        self.best_path = output_dir / "best_so_far.json"
        self.final_summary_path = output_dir / "final_summary.txt"
        self.best_config_path = output_dir / "best_config.json"
        self.findings_path = output_dir / "findings.txt"
        self.lock_file = (output_dir / ".autotune.lock").open("a+")
        try:
            fcntl.flock(
                self.lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB
            )
        except BlockingIOError as error:
            raise RuntimeError(
                f"another autotune process holds {output_dir}/.autotune.lock"
            ) from error
        self.lock_file.seek(0)
        self.lock_file.truncate()
        self.lock_file.write(f"pid={os.getpid()} session={session_id}\n")
        self.lock_file.flush()
        os.fsync(self.lock_file.fileno())
        self.next_run_id = self._discover_next_run_id()
        self.best_score = -math.inf

    def _discover_next_run_id(self) -> int:
        maximum = 0
        if self.runs_jsonl.exists():
            with self.runs_jsonl.open("r", encoding="utf-8") as source:
                for line in source:
                    try:
                        maximum = max(maximum, int(json.loads(line)["run_id"]))
                    except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                        continue
        return maximum + 1

    def allocate_run_id(self) -> int:
        run_id = self.next_run_id
        self.next_run_id += 1
        return run_id

    @staticmethod
    def _fsync_file(file_object: Any) -> None:
        file_object.flush()
        os.fsync(file_object.fileno())

    def append(self, record: Dict[str, Any]) -> None:
        csv_exists = self.runs_csv.exists() and self.runs_csv.stat().st_size > 0
        with self.runs_csv.open("a", newline="", encoding="utf-8") as target:
            writer = csv.DictWriter(
                target, fieldnames=CSV_FIELDS, extrasaction="ignore"
            )
            if not csv_exists:
                writer.writeheader()
            writer.writerow(
                {
                    field: _csv_value(record.get(field))
                    for field in CSV_FIELDS
                }
            )
            self._fsync_file(target)
        with self.runs_jsonl.open("a", encoding="utf-8") as target:
            target.write(json.dumps(record, sort_keys=True, allow_nan=False))
            target.write("\n")
            self._fsync_file(target)

        score = record.get("score")
        if (
            record.get("experiment_type") == "lap"
            and record.get("lap_completed")
            and isinstance(score, (float, int))
            and score > self.best_score
        ):
            self.best_score = float(score)
            best = {
                "session_id": self.session_id,
                "updated_at": utc_now(),
                "run_id": record["run_id"],
                "score": score,
                "configuration": {
                    key: record[key]
                    for key in (
                        "min_lookahead",
                        "max_lookahead",
                        "curvature_reference",
                        "preview_distance",
                        "max_lateral_acceleration",
                        "pid_enabled",
                        "kp",
                        "ki",
                        "kd",
                    )
                },
                "metrics": {
                    key: record.get(key)
                    for key in NUMERIC_METRICS
                    if record.get(key) is not None
                },
            }
            atomic_json_write(self.best_path, best)

    def raw_writer(self, run_id: int) -> "RawWriter":
        return RawWriter(self.output_dir / f"run_{run_id:04d}.csv")

    def write_finding(self, message: str) -> None:
        with self.findings_path.open("a", encoding="utf-8") as target:
            target.write(f"[{utc_now()}] {message.rstrip()}\n")
            self._fsync_file(target)


class RawWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.target = path.open("x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.target, fieldnames=RAW_FIELDS, extrasaction="ignore"
        )
        self.writer.writeheader()
        self.count = 0

    def write(self, sample: Dict[str, Any]) -> None:
        self.writer.writerow(
            {field: _csv_value(sample.get(field)) for field in RAW_FIELDS}
        )
        self.count += 1
        if self.count % 10 == 0:
            self.target.flush()

    def close(self) -> None:
        if self.target.closed:
            return
        self.target.flush()
        os.fsync(self.target.fileno())
        self.target.close()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        return f"{value:.9g}"
    return value


def atomic_json_write(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", dir=path.parent, delete=False, encoding="utf-8"
    ) as target:
        temporary = Path(target.name)
        json.dump(payload, target, indent=2, sort_keys=True, allow_nan=False)
        target.write("\n")
        target.flush()
        os.fsync(target.fileno())
    os.replace(temporary, path)


def mean(values: Sequence[float]) -> Optional[float]:
    return sum(values) / len(values) if values else None


def rms(values: Sequence[float]) -> Optional[float]:
    return math.sqrt(sum(value * value for value in values) / len(values)) \
        if values else None


def percentile(values: Sequence[float], probability: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def maximum(values: Sequence[float]) -> Optional[float]:
    return max(values) if values else None


def minimum(values: Sequence[float]) -> Optional[float]:
    return min(values) if values else None


def population_stddev(values: Sequence[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def distance(first: Tuple[float, float], second: Tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def point_segment_distance(
    point: Tuple[float, float],
    start: PathPoint,
    end: PathPoint,
) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared <= 1.0e-18:
        return math.hypot(point[0] - start.x, point[1] - start.y)
    projection = (
        (point[0] - start.x) * dx + (point[1] - start.y) * dy
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest_x = start.x + projection * dx
    closest_y = start.y + projection * dy
    return math.hypot(point[0] - closest_x, point[1] - closest_y)


def local_path_error(
    x: float,
    y: float,
    nearest_index: int,
    path: Sequence[PathPoint],
) -> float:
    count = len(path)
    previous = path[(nearest_index - 1) % count]
    current = path[nearest_index]
    following = path[(nearest_index + 1) % count]
    point = (x, y)
    return min(
        point_segment_distance(point, previous, current),
        point_segment_distance(point, current, following),
    )


def load_path(path: Path) -> Tuple[List[PathPoint], Dict[str, Any]]:
    with path.open("r", encoding="utf-8") as source:
        payload = json.load(source)
    values = payload.get("waypoints")
    if not isinstance(values, list) or len(values) < 4:
        raise ValueError("mock path must contain at least four waypoints")
    points = [PathPoint(float(value[0]), float(value[1])) for value in values]
    if distance((points[0].x, points[0].y), (points[-1].x, points[-1].y)) \
            <= 1.0e-9:
        points.pop()
    if len(points) < 3:
        raise ValueError("mock path has fewer than three logical points")
    for point in points:
        if not (math.isfinite(point.x) and math.isfinite(point.y)):
            raise ValueError("mock path contains a non-finite waypoint")
    metadata = payload.get("metadata", {})
    if metadata.get("cell_outside_count") not in (None, 0):
        raise ValueError("mock path metadata reports cellOutside != 0")
    if float(metadata.get("measured_max_curvature_inv_m", 0.0)) > 1.85:
        raise ValueError("mock path curvature exceeds the validated SIM bound")
    return points, metadata


def config_record(config: TrialConfig) -> Dict[str, Any]:
    return asdict(config)


def make_core(config: TrialConfig) -> ControllerCore:
    pure_pursuit = PurePursuit(
        PurePursuitConfig(
            wheelbase_m=WHEELBASE_M,
            lookahead_distance_m=BASE_LOOKAHEAD_M,
            max_steering_rad=STEERING_LIMIT_RAD,
            closed_loop=True,
        )
    )
    pid = PIDController(
        PIDConfig(
            kp=config.kp,
            ki=config.ki,
            kd=config.kd,
            output_min=-PID_OUTPUT_LIMIT_M_S,
            output_max=PID_OUTPUT_LIMIT_M_S,
            integral_min=-0.50,
            integral_max=0.50,
        )
    )
    adaptive = AdaptiveControlConfig(
        enabled=True,
        preview_distance_m=config.preview_distance,
        min_lookahead_m=config.min_lookahead,
        max_lookahead_m=config.max_lookahead,
        curvature_reference_inv_m=config.curvature_reference,
        max_lateral_acceleration_m_s2=config.max_lateral_acceleration,
        min_speed_limit_m_s=MIN_SPEED_LIMIT_M_S,
        max_speed_limit_m_s=MAX_SPEED_LIMIT_M_S,
    )
    return ControllerCore(
        pure_pursuit,
        pid,
        ControllerConfig(
            longitudinal_pid_enabled=config.pid_enabled,
            max_speed_m_s=MAX_SPEED_LIMIT_M_S,
            adaptive_control=adaptive,
        ),
    )


def empty_record(
    session_id: str,
    run_id: int,
    phase: str,
    experiment_type: str,
    label: str,
    config: TrialConfig,
) -> Dict[str, Any]:
    record: Dict[str, Any] = {
        "session_id": session_id,
        "run_id": run_id,
        "timestamp_utc": utc_now(),
        "phase": phase,
        "experiment_type": experiment_type,
        "label": label,
        **config_record(config),
        "success": False,
        "lap_completed": False,
        "path_error_stop": False,
        "target_behind": False,
        "timeout": False,
        "pose_api_error": False,
        "other_exception": False,
        "steering_saturation_count": 0,
        "error_message": "",
    }
    for metric in NUMERIC_METRICS:
        record.setdefault(metric, None)
    return record


def aggregate_records(records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    aggregate: Dict[str, Any] = {
        "runs": len(records),
        "successful_runs": sum(bool(record.get("success")) for record in records),
        "completed_laps": sum(
            bool(record.get("lap_completed")) for record in records
        ),
    }
    for metric in NUMERIC_METRICS:
        values = [
            float(record[metric])
            for record in records
            if isinstance(record.get(metric), (float, int))
        ]
        aggregate[metric] = mean(values)
    aggregate["steering_saturation_count"] = sum(
        int(record.get("steering_saturation_count") or 0)
        for record in records
    )
    return aggregate


def score_lap(record: Dict[str, Any], baseline: Dict[str, Any]) -> float:
    if not record.get("lap_completed"):
        return -10000.0
    score = 100.0
    weighted_metrics = (
        ("mean_path_error", 10.0),
        ("rms_path_error", 10.0),
        ("p95_path_error", 12.0),
        ("max_path_error", 22.0),
        ("mean_abs_steering", 3.0),
        ("p95_abs_steering", 4.0),
        ("max_abs_steering", 8.0),
        ("p95_steering_rate", 8.0),
        ("max_steering_rate", 4.0),
        ("lap_time", 5.0),
        ("mean_abs_measured_speed_error", 2.0),
    )
    for metric, weight in weighted_metrics:
        base = baseline.get(metric)
        value = record.get(metric)
        if not isinstance(base, (float, int)) or not isinstance(
            value, (float, int)
        ):
            continue
        denominator = max(abs(float(base)), 1.0e-4)
        improvement = (float(base) - float(value)) / denominator
        score += weight * max(-2.0, min(1.0, improvement))

    base_max_steering = float(
        baseline.get("max_abs_steering") or STEERING_LIMIT_RAD
    )
    steering = float(record.get("max_abs_steering") or STEERING_LIMIT_RAD)
    score += max(
        -12.0,
        min(12.0, 4.0 * (base_max_steering - steering) / 0.01),
    )
    steering_fraction = steering / STEERING_LIMIT_RAD
    if steering_fraction >= 0.98:
        score -= 25.0
    elif steering_fraction >= 0.96:
        score -= 10.0

    saturation_count = int(record.get("steering_saturation_count") or 0)
    if saturation_count:
        score -= 100.0 + 10.0 * min(saturation_count, 10)

    base_max_error = float(baseline.get("max_path_error") or 0.05)
    if float(record.get("max_path_error") or math.inf) > max(
        base_max_error * 1.15, base_max_error + 0.008
    ):
        score -= 50.0
    base_rate = float(baseline.get("p95_steering_rate") or 0.1)
    if float(record.get("p95_steering_rate") or math.inf) > base_rate * 1.30:
        score -= 20.0
    return score


def config_from_record(record: Dict[str, Any]) -> TrialConfig:
    return TrialConfig(
        min_lookahead=float(record["min_lookahead"]),
        max_lookahead=float(record["max_lookahead"]),
        curvature_reference=float(record["curvature_reference"]),
        preview_distance=float(record["preview_distance"]),
        max_lateral_acceleration=float(record["max_lateral_acceleration"]),
        pid_enabled=bool(record["pid_enabled"]),
        kp=float(record["kp"]),
        ki=float(record["ki"]),
        kd=float(record["kd"]),
    )


class Autotuner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.started_monotonic = time.monotonic()
        self.experiment_deadline = self.started_monotonic + (
            args.experiment_budget_min * 60.0
        )
        self.hard_deadline = self.started_monotonic + args.total_budget_min * 60.0
        self.session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.store = ResultStore(args.output_dir, self.session_id)
        self.logger = configure_logging(args.output_dir / "autotune.log")
        self.path, self.path_metadata = load_path(args.path)
        self.api = SimApi(args.sim_api, timeout_s=args.api_timeout_s)
        self.abort_requested = False
        self.shutdown_signal: Optional[int] = None
        self.node: Optional[AutotuneNode] = None
        self.records: List[Dict[str, Any]] = []
        self.baseline_records: List[Dict[str, Any]] = []
        self.baseline_reference: Optional[Dict[str, Any]] = None
        self.configs: Dict[Tuple[Any, ...], TrialConfig] = {
            BASELINE_CONFIG.key(): BASELINE_CONFIG
        }
        self.tested_lap_keys: set[Tuple[Any, ...]] = set()
        self.controller_exception_streak = 0
        self.timeout_streak = 0
        self.plateau_detected = False
        self.pid_step_records: List[Dict[str, Any]] = []
        self.selected_pid_config = BASELINE_CONFIG
        self.final_choices: Dict[str, TrialConfig] = {}
        self.termination_reason = "SAFETY_ABORT"
        self.termination_detail = "tuning did not reach the execution phase"

        rclpy.init(
            args=None,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        self.node = AutotuneNode()
        self._install_signal_handlers()
        atexit.register(self._atexit_stop)

    def _install_signal_handlers(self) -> None:
        def handle(signum: int, _frame: Any) -> None:
            self.abort_requested = True
            self.shutdown_signal = signum

        signal.signal(signal.SIGINT, handle)
        signal.signal(signal.SIGTERM, handle)

    def _atexit_stop(self) -> None:
        try:
            self.safe_stop("atexit")
        except Exception:
            pass

    def close(self) -> None:
        self.safe_stop("shutdown")
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if rclpy.ok():
            rclpy.shutdown()

    def log(self, message: str, *args: Any) -> None:
        self.logger.info(message, *args)

    def check_abort(self, allow_experiment_deadline: bool = False) -> None:
        if self.abort_requested:
            signal_name = (
                signal.Signals(self.shutdown_signal).name
                if self.shutdown_signal is not None
                else "external request"
            )
            raise SafetyAbort(f"received {signal_name}")
        now = time.monotonic()
        if now >= self.hard_deadline:
            raise RunAbort("timeout", "hard wall-clock budget reached")
        if not allow_experiment_deadline and now >= self.experiment_deadline:
            raise RunAbort("timeout", "experiment budget reached")

    def remaining_experiment_s(self) -> float:
        return self.experiment_deadline - time.monotonic()

    def spin_once(self, timeout_s: float = 0.0) -> None:
        if self.node is None:
            return
        rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def spin_sleep(self, duration_s: float, honor_budget: bool = True) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            if self.abort_requested:
                raise SafetyAbort("external shutdown requested")
            if honor_budget and time.monotonic() >= self.hard_deadline:
                raise RunAbort("timeout", "hard wall-clock budget reached")
            self.spin_once(min(0.05, max(0.0, end - time.monotonic())))

    def safe_stop(self, context: str) -> bool:
        if self.node is None or not rclpy.ok():
            return False
        succeeded = True
        try:
            for _ in range(self.args.safe_stop_repetitions):
                self.node.publish(0.0, 0.0)
                self.spin_once(0.0)
                time.sleep(self.args.safe_stop_interval_s)
            if (
                self.node.speed_pub.get_subscription_count() < 1
                or self.node.steering_pub.get_subscription_count() < 1
            ):
                succeeded = False
                self.logger.error("safe stop has no command subscribers: %s", context)
        except Exception:
            succeeded = False
            self.logger.exception("safe stop failed: %s", context)
        return succeeded

    def preflight(self) -> None:
        assert self.node is not None
        self.log(
            "session=%s branch-required=feature/jaehyeok-control path=%s points=%d",
            self.session_id,
            self.args.path,
            len(self.path),
        )
        status = self.api.status()
        if not status.get("running"):
            raise SafetyAbort(f"SIM is not running: {status}")
        self.api.pose()
        deadline = time.monotonic() + 8.0
        while self.node.speed_m_s is None and time.monotonic() < deadline:
            self.spin_once(0.1)
        if self.node.speed_m_s is None:
            raise SafetyAbort("/odom produced no samples during preflight")
        if self.node.speed_pub.get_subscription_count() < 1:
            raise SafetyAbort("/speed has no subscriber")
        if self.node.steering_pub.get_subscription_count() < 1:
            raise SafetyAbort("/steering has no subscriber")
        if not self.safe_stop("preflight"):
            raise SafetyAbort("preflight safe stop could not be confirmed")
        self.log(
            "preflight PASS sim=%s odom_speed=%.4f speed_subscribers=%d "
            "steering_subscribers=%d",
            status.get("current"),
            self.node.speed_m_s,
            self.node.speed_pub.get_subscription_count(),
            self.node.steering_pub.get_subscription_count(),
        )

    def wait_until_stopped(self, timeout_s: float = 4.0) -> None:
        assert self.node is not None
        deadline = time.monotonic() + timeout_s
        stable_since: Optional[float] = None
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            speed = self.node.speed_m_s
            if speed is not None and abs(speed) <= STOPPED_SPEED_LIMIT_M_S:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.5:
                    return
            else:
                stable_since = None
            if self.abort_requested:
                raise SafetyAbort("shutdown requested while waiting for stop")
        raise SafetyAbort(
            f"vehicle did not stop below {STOPPED_SPEED_LIMIT_M_S:.3f} m/s"
        )

    def nearest_path_index(self, x: float, y: float) -> int:
        return min(
            range(len(self.path)),
            key=lambda index: math.hypot(
                self.path[index].x - x, self.path[index].y - y
            ),
        )

    def path_heading(self, index: int) -> float:
        following = self.path[(index + 1) % len(self.path)]
        current = self.path[index]
        return math.atan2(following.y - current.y, following.x - current.x)

    def verify_start_pose(self) -> Tuple[float, float, float, int]:
        x, y, yaw = self.api.pose()
        index = self.nearest_path_index(x, y)
        path_distance = math.hypot(
            self.path[index].x - x, self.path[index].y - y
        )
        heading_error = abs(normalize_angle(yaw - self.path_heading(index)))
        if path_distance > START_PATH_DISTANCE_LIMIT_M:
            raise SafetyAbort(
                f"start pose is {path_distance:.3f} m from path "
                f"(limit {START_PATH_DISTANCE_LIMIT_M:.3f} m)"
            )
        if heading_error > START_HEADING_ERROR_LIMIT_RAD:
            raise SafetyAbort(
                f"start heading error is {math.degrees(heading_error):.1f} deg"
            )
        return x, y, yaw, index

    def prepare_run(self, teleport_index: Optional[int] = None) -> int:
        if not self.safe_stop("before reset"):
            raise SafetyAbort("safe stop failed before reset")
        self.wait_until_stopped()
        try:
            self.api.reset()
            self.spin_sleep(self.args.reset_settle_s)
            if teleport_index is not None:
                point = self.path[teleport_index]
                self.api.set_pose(
                    point.x,
                    point.y,
                    self.path_heading(teleport_index),
                )
                self.spin_sleep(0.6)
        except Exception as error:
            raise SafetyAbort(f"SIM reset/pose API failed: {error}") from error
        if not self.safe_stop("after reset"):
            raise SafetyAbort("safe stop failed after reset")
        self.wait_until_stopped()
        _, _, _, start_index = self.verify_start_pose()
        assert self.node is not None
        if (
            self.node.odom_received_monotonic is None
            or time.monotonic() - self.node.odom_received_monotonic
            > ODOM_STALE_TIMEOUT_S
        ):
            raise SafetyAbort("/odom is stale after reset")
        return start_index

    def run_lap(
        self,
        config: TrialConfig,
        phase: str,
        label: str,
        mark_tested: bool = True,
    ) -> Dict[str, Any]:
        assert self.node is not None
        run_id = self.store.allocate_run_id()
        record = empty_record(
            self.session_id, run_id, phase, "lap", label, config
        )
        raw = self.store.raw_writer(run_id)
        self.configs[config.key()] = config
        if mark_tested:
            self.tested_lap_keys.add(config.key())
        self.log("RUN %04d START %s %s", run_id, phase, config.short_name())

        path_errors: List[float] = []
        steerings: List[float] = []
        steering_rates: List[float] = []
        curvatures: List[float] = []
        lookaheads: List[float] = []
        commands: List[float] = []
        measured_speeds: List[float] = []
        speed_errors: List[float] = []
        completion = False
        fatal = False
        started: Optional[float] = None
        previous_loop: Optional[float] = None
        previous_steering: Optional[float] = None
        previous_index: Optional[int] = None
        progress = 0.0
        behind_duration = 0.0
        pose_failure_streak = 0
        saturation_count = 0

        try:
            start_index = self.prepare_run()
            previous_index = start_index
            core = make_core(config)
            started = time.monotonic()
            previous_loop = started
            next_tick = started

            while True:
                self.check_abort()
                now = time.monotonic()
                elapsed = now - started
                if elapsed >= self.args.lap_timeout_s:
                    raise RunAbort("timeout", "lap timeout reached")
                self.spin_once(0.0)
                if (
                    self.node.odom_received_monotonic is None
                    or now - self.node.odom_received_monotonic
                    > ODOM_STALE_TIMEOUT_S
                ):
                    raise RunAbort("pose_api_error", "/odom became stale", True)
                try:
                    x, y, yaw = self.api.pose()
                    pose_failure_streak = 0
                except Exception as error:
                    pose_failure_streak += 1
                    self.node.publish(0.0, 0.0)
                    if pose_failure_streak >= POSE_FAILURE_LIMIT:
                        raise RunAbort(
                            "pose_api_error",
                            f"persistent pose API failure: {error}",
                            True,
                        ) from error
                    self.spin_sleep(1.0 / self.args.control_rate_hz)
                    continue

                dt = max(1.0e-4, now - previous_loop)
                previous_loop = now
                measured_speed = float(self.node.speed_m_s or 0.0)
                state = VehicleState(x=x, y=y, yaw=yaw, speed=measured_speed)
                try:
                    result = core.update(
                        state,
                        self.path,
                        PLANNER_TARGET_SPEED_M_S,
                        dt,
                    )
                except Exception as error:
                    raise RunAbort(
                        "other_exception", f"ControllerCore exception: {error}"
                    ) from error

                nearest_index = result.pure_pursuit.nearest_index
                path_error = local_path_error(x, y, nearest_index, self.path)
                if path_error > PATH_ERROR_STOP_M:
                    raise RunAbort(
                        "path_error_stop",
                        f"path error {path_error:.3f} m exceeds "
                        f"{PATH_ERROR_STOP_M:.3f} m",
                        True,
                    )

                if abs(result.pure_pursuit.alpha_rad) > math.pi / 2.0:
                    behind_duration += dt
                    if behind_duration >= TARGET_BEHIND_GRACE_S:
                        raise RunAbort(
                            "target_behind",
                            "Pure Pursuit target remained behind vehicle",
                            True,
                        )
                else:
                    behind_duration = 0.0

                if previous_index is not None:
                    count = len(self.path)
                    signed_delta = (
                        (nearest_index - previous_index + count / 2.0) % count
                    ) - count / 2.0
                    if abs(signed_delta) <= max(10.0, count * 0.08):
                        progress += signed_delta
                previous_index = nearest_index

                steering_rate = 0.0
                if previous_steering is not None:
                    steering_rate = abs(
                        result.steering_rad - previous_steering
                    ) / dt
                    steering_rates.append(steering_rate)
                previous_steering = result.steering_rad
                if abs(result.steering_rad) >= (
                    STEERING_LIMIT_RAD - SATURATION_EPSILON_RAD
                ):
                    saturation_count += 1

                adaptive = result.adaptive
                curvature = adaptive.curvature_inv_m if adaptive else 0.0
                lookahead = (
                    adaptive.lookahead_distance_m
                    if adaptive
                    else BASE_LOOKAHEAD_M
                )
                speed_limit = (
                    adaptive.speed_limit_m_s
                    if adaptive
                    else PLANNER_TARGET_SPEED_M_S
                )
                effective_target = min(PLANNER_TARGET_SPEED_M_S, speed_limit)

                self.node.publish(
                    result.speed_command_m_s,
                    result.steering_rad,
                )
                path_errors.append(path_error)
                steerings.append(abs(result.steering_rad))
                curvatures.append(curvature)
                lookaheads.append(lookahead)
                commands.append(result.speed_command_m_s)
                measured_speeds.append(measured_speed)
                speed_errors.append(result.speed_command_m_s - measured_speed)
                raw.write(
                    {
                        "elapsed_s": elapsed,
                        "segment": "lap",
                        "x_m": x,
                        "y_m": y,
                        "yaw_rad": yaw,
                        "measured_speed_m_s": measured_speed,
                        "planner_target_m_s": PLANNER_TARGET_SPEED_M_S,
                        "effective_target_m_s": effective_target,
                        "speed_command_m_s": result.speed_command_m_s,
                        "steering_rad": result.steering_rad,
                        "steering_rate_rad_s": steering_rate,
                        "path_error_m": path_error,
                        "nearest_index": nearest_index,
                        "lap_progress_points": progress,
                        "target_index": result.pure_pursuit.target_index,
                        "target_alpha_rad": result.pure_pursuit.alpha_rad,
                        "preview_curvature_inv_m": curvature,
                        "lookahead_m": lookahead,
                        "speed_limit_m_s": speed_limit,
                        "pid_error": result.pid.error if result.pid else None,
                        "pid_output": result.pid.output if result.pid else None,
                    }
                )

                start_distance = min(
                    (nearest_index - start_index) % len(self.path),
                    (start_index - nearest_index) % len(self.path),
                )
                if (
                    progress >= len(self.path) * 0.98
                    and start_distance <= max(10, int(len(self.path) * 0.025))
                    and elapsed >= 20.0
                ):
                    completion = True
                    break

                next_tick += 1.0 / self.args.control_rate_hz
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    self.spin_sleep(delay)
                elif delay < -0.5:
                    next_tick = time.monotonic()
        except RunAbort as error:
            record[error.failure] = True
            record["error_message"] = str(error)
            fatal = error.fatal
        except SafetyAbort as error:
            record["other_exception"] = True
            record["error_message"] = str(error)
            fatal = True
        except Exception as error:
            record["other_exception"] = True
            record["error_message"] = f"unexpected: {error}"
            self.logger.error("RUN %04d traceback:\n%s", run_id, traceback.format_exc())
        finally:
            safe = self.safe_stop(f"after run {run_id}")
            raw.close()
            if not safe:
                fatal = True
                record["other_exception"] = True
                record["error_message"] = (
                    record["error_message"] + "; " if record["error_message"] else ""
                ) + "safe stop failed"

        elapsed_time = (
            time.monotonic() - started if started is not None else 0.0
        )
        record.update(
            {
                "success": completion,
                "lap_completed": completion,
                "lap_time": elapsed_time if completion else None,
                "elapsed_time": elapsed_time,
                "mean_path_error": mean(path_errors),
                "rms_path_error": rms(path_errors),
                "p95_path_error": percentile(path_errors, 0.95),
                "max_path_error": maximum(path_errors),
                "mean_abs_steering": mean(steerings),
                "p95_abs_steering": percentile(steerings, 0.95),
                "max_abs_steering": maximum(steerings),
                "steering_saturation_count": saturation_count,
                "mean_abs_steering_rate": mean(steering_rates),
                "p95_steering_rate": percentile(steering_rates, 0.95),
                "max_steering_rate": maximum(steering_rates),
                "mean_curvature": mean(curvatures),
                "max_preview_curvature": maximum(curvatures),
                "mean_lookahead": mean(lookaheads),
                "min_lookahead_observed": minimum(lookaheads),
                "max_lookahead_observed": maximum(lookaheads),
                "mean_target_command": mean(commands),
                "min_command": minimum(commands),
                "max_command": maximum(commands),
                "measured_speed_mean": mean(measured_speeds),
                "measured_speed_error": mean(speed_errors),
                "mean_abs_measured_speed_error": mean(
                    [abs(value) for value in speed_errors]
                ),
            }
        )
        if self.baseline_reference is not None:
            record["score"] = score_lap(record, self.baseline_reference)
        self.store.append(record)
        self.records.append(record)
        self.log(
            "RUN %04d END complete=%s time=%s mean_err=%s max_err=%s "
            "max_steer_deg=%s sat=%d score=%s failure=%s",
            run_id,
            completion,
            _fmt(record.get("lap_time")),
            _fmt(record.get("mean_path_error")),
            _fmt(record.get("max_path_error")),
            _fmt_degrees(record.get("max_abs_steering")),
            saturation_count,
            _fmt(record.get("score")),
            record.get("error_message") or "none",
        )

        if saturation_count:
            risk = sharp_curve_lookahead(config)
            self.log(
                "over-tuning guard: reject candidate and do not locally refine "
                "saturation point at sharp-lookahead %.3f m",
                risk,
            )
        if record.get("other_exception"):
            self.controller_exception_streak += 1
        else:
            self.controller_exception_streak = 0
        if record.get("timeout"):
            self.timeout_streak += 1
        else:
            self.timeout_streak = 0
        if self.controller_exception_streak >= CONTROL_EXCEPTION_LIMIT:
            fatal = True
            record["termination_hint"] = "repeated controller exception"
        if self.timeout_streak >= RUN_TIMEOUT_LIMIT:
            fatal = True
            record["termination_hint"] = "repeated lap timeout"
        if fatal:
            raise SafetyAbort(
                f"run {run_id} requires safety abort: {record['error_message']}"
            )
        return record

    def run_phase0(self) -> None:
        self.log("PHASE 0 baseline validation")
        for repeat in range(2):
            self.baseline_records.append(
                self.run_lap(
                    BASELINE_CONFIG,
                    "phase0_baseline",
                    f"baseline_validation_{repeat + 1}",
                    mark_tested=False,
                )
            )
        self.baseline_reference = aggregate_records(self.baseline_records)
        for record in self.baseline_records:
            record["score"] = score_lap(record, self.baseline_reference)
        atomic_json_write(
            self.store.output_dir / "baseline_validation.json",
            {
                "historical": BASELINE_HISTORICAL,
                "measured": self.baseline_reference,
                "passed": self.baseline_is_valid(),
            },
        )
        if not self.baseline_is_valid():
            raise SafetyAbort(
                "baseline validation did not match safe/repeatable bounds"
            )
        self.log("PHASE 0 PASS baseline=%s", self.baseline_reference)

    def baseline_is_valid(self) -> bool:
        if len(self.baseline_records) != 2:
            return False
        if not all(record.get("lap_completed") for record in self.baseline_records):
            return False
        if any(
            int(record.get("steering_saturation_count") or 0) > 0
            for record in self.baseline_records
        ):
            return False
        laps = [float(record["lap_time"]) for record in self.baseline_records]
        means = [
            float(record["mean_path_error"]) for record in self.baseline_records
        ]
        maxima = [
            float(record["max_path_error"]) for record in self.baseline_records
        ]
        return (
            all(35.0 <= value <= 65.0 for value in laps)
            and max(means) <= 0.060
            and max(maxima) <= 0.100
            and (max(laps) - min(laps)) / mean(laps) <= 0.15
            and max(means) - min(means) <= 0.025
            and max(maxima) - min(maxima) <= 0.040
        )

    def candidate_allowed(self, config: TrialConfig) -> bool:
        if config.key() in self.tested_lap_keys:
            return False
        return True

    def enough_time_for_lap(self, reserve_s: float = 0.0) -> bool:
        return self.remaining_experiment_s() > (
            self.args.lap_timeout_s + self.args.reset_settle_s + 5.0 + reserve_s
        )

    def run_candidates(
        self,
        configs: Iterable[TrialConfig],
        phase: str,
        label_prefix: str,
    ) -> List[Dict[str, Any]]:
        records = []
        for index, config in enumerate(configs, start=1):
            if not self.enough_time_for_lap():
                self.log("%s stopped at experiment deadline", phase)
                break
            if not self.candidate_allowed(config):
                continue
            record = self.run_lap(
                config, phase, f"{label_prefix}_{index:02d}"
            )
            records.append(record)
        return records

    def run_phase1(self) -> List[Dict[str, Any]]:
        self.log("PHASE 1 lateral coarse-to-fine")
        coarse = self.run_candidates(
            coarse_lateral_candidates(),
            "phase1_lateral_coarse",
            "lateral_coarse",
        )
        top = select_top_configs(coarse, 3, self.baseline_reference)
        fine_candidates = fine_lateral_candidates(
            top, self.tested_lap_keys, limit=18
        )
        fine = self.run_candidates(
            fine_candidates,
            "phase1_lateral_fine",
            "lateral_fine",
        )
        all_records = coarse + fine
        self.plateau_detected = convergence_plateau(all_records)
        self.log(
            "PHASE 1 END runs=%d plateau=%s top=%s",
            len(all_records),
            self.plateau_detected,
            [config.short_name() for config in select_top_configs(
                all_records, 3, self.baseline_reference
            )],
        )
        return all_records

    def run_phase2(
        self, lateral_records: Sequence[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        self.log("PHASE 2 adaptive speed coarse-to-fine")
        lateral_top = select_top_configs(
            lateral_records, 3, self.baseline_reference
        )
        if not lateral_top:
            lateral_top = [BASELINE_CONFIG]
        candidates: List[TrialConfig] = []
        for lateral in lateral_top:
            for acceleration in (0.60, 0.70, 0.90, 1.00):
                candidates.append(
                    replace(lateral, max_lateral_acceleration=acceleration)
                )
        coarse = self.run_candidates(
            candidates,
            "phase2_speed_coarse",
            "speed_coarse",
        )
        combined = list(lateral_records) + coarse
        top_speed = select_top_configs(combined, 2, self.baseline_reference)
        fine_configs: List[TrialConfig] = []
        for config in top_speed:
            for delta in (-0.05, 0.05):
                acceleration = round(config.max_lateral_acceleration + delta, 2)
                if 0.60 <= acceleration <= 1.00:
                    fine_configs.append(
                        replace(config, max_lateral_acceleration=acceleration)
                    )
        fine = self.run_candidates(
            unique_configs(fine_configs),
            "phase2_speed_fine",
            "speed_fine",
        )
        phase_records = coarse + fine
        self.log("PHASE 2 END runs=%d", len(phase_records))
        return phase_records

    def find_longest_straight_start(self) -> int:
        best_length = 0.0
        best_start = 0
        current_start: Optional[int] = None
        current_length = 0.0
        count = len(self.path)
        for step in range(count * 2):
            index = step % count
            previous = self.path[(index - 1) % count]
            current = self.path[index]
            following = self.path[(index + 1) % count]
            curvature = triangle_curvature(previous, current, following)
            if curvature < 0.05:
                if current_start is None:
                    current_start = step
                    current_length = 0.0
                current_length += distance(
                    (current.x, current.y), (following.x, following.y)
                )
                if (
                    current_length > best_length
                    and step - current_start < count
                ):
                    best_length = current_length
                    best_start = current_start % count
            else:
                current_start = None
                current_length = 0.0
        self.log(
            "PID step start index=%d straight_length=%.3f m",
            best_start,
            best_length,
        )
        return best_start

    def run_step_response(
        self, config: TrialConfig, label: str, teleport_index: int
    ) -> Dict[str, Any]:
        assert self.node is not None
        run_id = self.store.allocate_run_id()
        record = empty_record(
            self.session_id,
            run_id,
            "phase3_pid_step",
            "pid_step",
            label,
            config,
        )
        raw = self.store.raw_writer(run_id)
        self.configs[config.key()] = config
        samples: List[Dict[str, Any]] = []
        path_errors: List[float] = []
        steerings: List[float] = []
        steering_rates: List[float] = []
        commands: List[float] = []
        measured_speeds: List[float] = []
        speed_errors: List[float] = []
        curvatures: List[float] = []
        lookaheads: List[float] = []
        saturation_count = 0
        completed = False
        fatal = False
        started: Optional[float] = None
        self.log("RUN %04d START PID STEP %s", run_id, config.short_name())

        segments = [
            ("hold_0.00", 0.00, 2.0),
            ("0.00_to_0.30", 0.30, 4.0),
            ("0.30_to_0.50", 0.50, 4.0),
            ("0.50_to_0.67", 0.67, 4.0),
            ("0.67_to_0.80", 0.80, 4.0),
            ("0.80_to_0.50", 0.50, 4.0),
            ("0.50_to_0.00", 0.00, 4.0),
        ]
        boundaries: List[Tuple[float, float, str, float]] = []
        cursor = 0.0
        for name, target, duration in segments:
            boundaries.append((cursor, cursor + duration, name, target))
            cursor += duration

        try:
            self.prepare_run(teleport_index=teleport_index)
            core = make_core(config)
            started = time.monotonic()
            previous_loop = started
            previous_steering: Optional[float] = None
            next_tick = started
            while True:
                self.check_abort()
                now = time.monotonic()
                elapsed = now - started
                if elapsed >= cursor:
                    completed = True
                    break
                segment_name = segments[-1][0]
                planner_target = segments[-1][1]
                for begin, end, name, target in boundaries:
                    if begin <= elapsed < end:
                        segment_name = name
                        planner_target = target
                        break
                self.spin_once(0.0)
                if (
                    self.node.odom_received_monotonic is None
                    or now - self.node.odom_received_monotonic
                    > ODOM_STALE_TIMEOUT_S
                ):
                    raise RunAbort("pose_api_error", "/odom became stale", True)
                try:
                    x, y, yaw = self.api.pose()
                except Exception as error:
                    raise RunAbort(
                        "pose_api_error", f"pose API failed: {error}", True
                    ) from error
                dt = max(1.0e-4, now - previous_loop)
                previous_loop = now
                measured_speed = float(self.node.speed_m_s or 0.0)
                state = VehicleState(x, y, yaw, measured_speed)
                try:
                    result = core.update(
                        state, self.path, planner_target, dt
                    )
                except Exception as error:
                    raise RunAbort(
                        "other_exception", f"ControllerCore exception: {error}"
                    ) from error
                path_error = local_path_error(
                    x, y, result.pure_pursuit.nearest_index, self.path
                )
                if path_error > PATH_ERROR_STOP_M:
                    raise RunAbort(
                        "path_error_stop",
                        f"PID step path error reached {path_error:.3f} m",
                        True,
                    )
                if abs(result.pure_pursuit.alpha_rad) > math.pi / 2.0:
                    raise RunAbort(
                        "target_behind", "PID step target is behind", True
                    )
                steering_rate = 0.0
                if previous_steering is not None:
                    steering_rate = abs(
                        result.steering_rad - previous_steering
                    ) / dt
                    steering_rates.append(steering_rate)
                previous_steering = result.steering_rad
                if abs(result.steering_rad) >= (
                    STEERING_LIMIT_RAD - SATURATION_EPSILON_RAD
                ):
                    saturation_count += 1
                adaptive = result.adaptive
                speed_limit = adaptive.speed_limit_m_s
                effective_target = min(planner_target, speed_limit)
                self.node.publish(
                    result.speed_command_m_s, result.steering_rad
                )
                sample = {
                    "elapsed_s": elapsed,
                    "segment": segment_name,
                    "x_m": x,
                    "y_m": y,
                    "yaw_rad": yaw,
                    "measured_speed_m_s": measured_speed,
                    "planner_target_m_s": planner_target,
                    "effective_target_m_s": effective_target,
                    "speed_command_m_s": result.speed_command_m_s,
                    "steering_rad": result.steering_rad,
                    "steering_rate_rad_s": steering_rate,
                    "path_error_m": path_error,
                    "nearest_index": result.pure_pursuit.nearest_index,
                    "target_index": result.pure_pursuit.target_index,
                    "target_alpha_rad": result.pure_pursuit.alpha_rad,
                    "preview_curvature_inv_m": adaptive.curvature_inv_m,
                    "lookahead_m": adaptive.lookahead_distance_m,
                    "speed_limit_m_s": speed_limit,
                    "pid_error": result.pid.error if result.pid else None,
                    "pid_output": result.pid.output if result.pid else None,
                }
                samples.append(sample)
                raw.write(sample)
                path_errors.append(path_error)
                steerings.append(abs(result.steering_rad))
                commands.append(result.speed_command_m_s)
                measured_speeds.append(measured_speed)
                speed_errors.append(effective_target - measured_speed)
                curvatures.append(adaptive.curvature_inv_m)
                lookaheads.append(adaptive.lookahead_distance_m)
                next_tick += 1.0 / self.args.control_rate_hz
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    self.spin_sleep(delay)
        except RunAbort as error:
            record[error.failure] = True
            record["error_message"] = str(error)
            fatal = error.fatal
        except SafetyAbort as error:
            record["other_exception"] = True
            record["error_message"] = str(error)
            fatal = True
        except Exception as error:
            record["other_exception"] = True
            record["error_message"] = f"unexpected: {error}"
            self.logger.error(
                "RUN %04d PID traceback:\n%s", run_id, traceback.format_exc()
            )
        finally:
            safe = self.safe_stop(f"after PID step {run_id}")
            raw.close()
            if not safe:
                fatal = True
                record["other_exception"] = True
                record["error_message"] += "; safe stop failed"

        step_metrics = analyze_step_response(samples, boundaries)
        elapsed_time = time.monotonic() - started if started else 0.0
        record.update(
            {
                "success": completed,
                "lap_completed": False,
                "elapsed_time": elapsed_time,
                "mean_path_error": mean(path_errors),
                "rms_path_error": rms(path_errors),
                "p95_path_error": percentile(path_errors, 0.95),
                "max_path_error": maximum(path_errors),
                "mean_abs_steering": mean(steerings),
                "p95_abs_steering": percentile(steerings, 0.95),
                "max_abs_steering": maximum(steerings),
                "steering_saturation_count": saturation_count,
                "mean_abs_steering_rate": mean(steering_rates),
                "p95_steering_rate": percentile(steering_rates, 0.95),
                "max_steering_rate": maximum(steering_rates),
                "mean_curvature": mean(curvatures),
                "max_preview_curvature": maximum(curvatures),
                "mean_lookahead": mean(lookaheads),
                "min_lookahead_observed": minimum(lookaheads),
                "max_lookahead_observed": maximum(lookaheads),
                "mean_target_command": mean(commands),
                "min_command": minimum(commands),
                "max_command": maximum(commands),
                "measured_speed_mean": mean(measured_speeds),
                "measured_speed_error": mean(speed_errors),
                "mean_abs_measured_speed_error": mean(
                    [abs(value) for value in speed_errors]
                ),
                "step_metrics": step_metrics,
                "step_score": step_metrics.get("step_score")
                if completed
                else None,
            }
        )
        self.store.append(record)
        self.records.append(record)
        self.pid_step_records.append(record)
        self.log(
            "RUN %04d END PID success=%s step_score=%s error=%s",
            run_id,
            completed,
            _fmt(record.get("step_score")),
            record.get("error_message") or "none",
        )
        if fatal:
            raise SafetyAbort(
                f"PID step run {run_id} requires safety abort: "
                f"{record['error_message']}"
            )
        return record

    def run_phase3(
        self, candidate_records: Sequence[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        self.log("PHASE 3 longitudinal PID step response")
        best_configs = select_top_configs(
            candidate_records, 1, self.baseline_reference
        )
        base = best_configs[0] if best_configs else BASELINE_CONFIG
        base = replace(base, pid_enabled=False, kp=0.0, ki=0.0, kd=0.0)
        start_index = self.find_longest_straight_start()
        configs = [base] + [
            replace(base, pid_enabled=True, kp=kp)
            for kp in (0.05, 0.10, 0.20, 0.30)
        ]
        initial: List[Dict[str, Any]] = []
        for index, config in enumerate(configs):
            if self.remaining_experiment_s() < 50.0:
                break
            initial.append(
                self.run_step_response(
                    config, f"pid_step_coarse_{index:02d}", start_index
                )
            )
        successful = [
            record for record in initial
            if record.get("success")
            and isinstance(record.get("step_score"), (float, int))
        ]
        off = next(
            (record for record in successful if not record["pid_enabled"]),
            None,
        )
        p_records = [record for record in successful if record["pid_enabled"]]
        fine_records: List[Dict[str, Any]] = []
        if off is not None and p_records:
            best_p = min(p_records, key=lambda value: value["step_score"])
            if best_p["step_score"] < off["step_score"] * 0.97:
                best_kp = float(best_p["kp"])
                fine_configs = [
                    replace(base, pid_enabled=True, kp=max(0.01, best_kp - 0.025)),
                    replace(base, pid_enabled=True, kp=best_kp + 0.025),
                    replace(base, pid_enabled=True, kp=best_kp, ki=0.01),
                    replace(base, pid_enabled=True, kp=best_kp, ki=0.02),
                ]
                metrics = best_p.get("step_metrics", {})
                if float(metrics.get("mean_overshoot") or 0.0) > 0.02:
                    fine_configs.append(
                        replace(base, pid_enabled=True, kp=best_kp, kd=0.005)
                    )
                    self.log(
                        "Kd evidence gate opened: P-only overshoot %.4f m/s",
                        metrics.get("mean_overshoot"),
                    )
                for index, config in enumerate(unique_configs(fine_configs)):
                    if self.remaining_experiment_s() < 50.0:
                        break
                    fine_records.append(
                        self.run_step_response(
                            config, f"pid_step_fine_{index:02d}", start_index
                        )
                    )

        all_step = initial + fine_records
        usable = [
            record for record in all_step
            if record.get("success")
            and isinstance(record.get("step_score"), (float, int))
        ]
        off_record = next(
            (record for record in usable if not record["pid_enabled"]), None
        )
        on_records = [record for record in usable if record["pid_enabled"]]
        selected = base
        pid_laps: List[Dict[str, Any]] = []
        if off_record is not None and on_records:
            best_on = min(on_records, key=lambda value: value["step_score"])
            if best_on["step_score"] <= off_record["step_score"] * 0.97:
                selected = config_from_record(best_on)
                for repeat in range(2):
                    if not self.enough_time_for_lap():
                        break
                    pid_laps.append(
                        self.run_lap(
                            selected,
                            "phase3_pid_lap",
                            f"pid_on_lap_{repeat + 1}",
                            mark_tested=False,
                        )
                    )
                off_lap = best_completed_record(
                    candidate_records, base.key()
                )
                if not pid_lap_improves(pid_laps, off_lap):
                    self.log("PID ON rejected by lap-level safety/performance gate")
                    selected = base
            else:
                self.log("PID OFF retained: no >=3%% step-response improvement")
        self.selected_pid_config = selected
        atomic_json_write(
            self.store.output_dir / "pid_comparison.json",
            {
                "selected": config_record(selected),
                "step_runs": all_step,
                "pid_lap_runs": pid_laps,
            },
        )
        return pid_laps

    def select_categories(
        self, records: Sequence[Dict[str, Any]]
    ) -> Dict[str, TrialConfig]:
        valid = eligible_records(records, self.baseline_reference)
        if not valid:
            return {
                "original_baseline": BASELINE_CONFIG,
                "best_tracking": BASELINE_CONFIG,
                "best_stability": BASELINE_CONFIG,
                "best_balanced": BASELINE_CONFIG,
            }
        best_tracking = min(valid, key=tracking_objective)
        best_stability = min(valid, key=stability_objective)
        best_balanced = max(valid, key=lambda record: record.get("score", -math.inf))
        return {
            "original_baseline": BASELINE_CONFIG,
            "best_tracking": config_from_record(best_tracking),
            "best_stability": config_from_record(best_stability),
            "best_balanced": config_from_record(best_balanced),
        }

    def run_phase4(self) -> TrialConfig:
        self.log("PHASE 4 final repeated validation")
        lap_records = [
            record for record in self.records
            if record.get("experiment_type") == "lap"
        ]
        choices = self.select_categories(lap_records)
        self.final_choices = choices
        unique = unique_configs(choices.values())
        validation: Dict[Tuple[Any, ...], List[Dict[str, Any]]] = {
            config.key(): [] for config in unique
        }
        for config in unique:
            repeats = 2
            for repeat in range(repeats):
                if not self.enough_time_for_lap(reserve_s=30.0):
                    break
                validation[config.key()].append(
                    self.run_lap(
                        config,
                        "phase4_validation",
                        f"final_{config.short_name()}_{repeat + 1}",
                        mark_tested=False,
                    )
                )

        ranked = ranked_configs(lap_records, self.baseline_reference)
        ordered_candidates = [choices["best_balanced"]]
        ordered_candidates.extend(config for config, _ in ranked)
        ordered_candidates.append(BASELINE_CONFIG)
        ordered_candidates = unique_configs(ordered_candidates)
        baseline_validation = validation.get(BASELINE_CONFIG.key(), [])

        for candidate in ordered_candidates[:5]:
            candidate_runs = validation.setdefault(candidate.key(), [])
            while len(candidate_runs) < 3 and self.enough_time_for_lap(
                reserve_s=15.0
            ):
                candidate_runs.append(
                    self.run_lap(
                        candidate,
                        "phase4_validation",
                        f"final_candidate_extra_{len(candidate_runs) + 1}",
                        mark_tested=False,
                    )
                )
            if candidate.key() == BASELINE_CONFIG.key():
                baseline_validation = candidate_runs
            if not baseline_validation:
                baseline_validation = self.baseline_records
            if final_validation_passes(candidate_runs, baseline_validation):
                self.log(
                    "PHASE 4 PASS selected=%s validation_laps=%d",
                    candidate.short_name(),
                    len(candidate_runs),
                )
                self.final_choices["best_balanced"] = candidate
                return candidate
            self.log("PHASE 4 candidate validation failed: %s", candidate.short_name())
        raise SafetyAbort("no candidate passed final repeatability validation")

    def run(self) -> None:
        self.preflight()
        if self.args.preflight_only:
            self.termination_reason = "OPTIMUM_CONVERGED"
            self.termination_detail = "preflight-only check completed"
            return
        self.run_phase0()
        lateral = self.run_phase1()
        speed = self.run_phase2(lateral)
        candidate_records = list(self.baseline_records) + lateral + speed
        pid_laps = self.run_phase3(candidate_records)
        candidate_records.extend(pid_laps)
        selected = self.run_phase4()
        elapsed = time.monotonic() - self.started_monotonic
        if time.monotonic() >= self.experiment_deadline:
            self.termination_reason = "TIME_BUDGET_EXHAUSTED"
            self.termination_detail = (
                "experiment deadline reached; final safe ranking was produced"
            )
        else:
            self.termination_reason = "OPTIMUM_CONVERGED"
            plateau = (
                "recent candidate scores plateaued below 1% improvement; "
                if self.plateau_detected
                else "scheduled coarse-to-fine search completed; "
            )
            self.termination_detail = (
                plateau
                + f"{selected.short_name()} passed three repeatability laps; "
                + f"elapsed={elapsed / 60.0:.1f} min"
            )

    def write_final_outputs(self) -> None:
        lap_records = [
            record for record in self.records
            if record.get("experiment_type") == "lap"
        ]
        completed = [record for record in lap_records if record.get("lap_completed")]
        ranking = ranked_configs(lap_records, self.baseline_reference)
        selected = self.final_choices.get("best_balanced", BASELINE_CONFIG)
        selected_records = [
            record for record in completed
            if config_from_record(record).key() == selected.key()
            and record.get("phase") == "phase4_validation"
        ]
        if not selected_records:
            selected_records = [
                record for record in completed
                if config_from_record(record).key() == selected.key()
            ]
        selected_aggregate = aggregate_records(selected_records)
        baseline_final = [
            record for record in completed
            if config_from_record(record).key() == BASELINE_CONFIG.key()
            and record.get("phase") == "phase4_validation"
        ]
        if not baseline_final:
            baseline_final = self.baseline_records
        baseline_aggregate = aggregate_records(baseline_final)

        best_tracking = self.final_choices.get("best_tracking", BASELINE_CONFIG)
        best_stability = self.final_choices.get("best_stability", BASELINE_CONFIG)
        best_config_payload = {
            "termination_reason": self.termination_reason,
            "termination_detail": self.termination_detail,
            "session_id": self.session_id,
            "configuration": config_record(selected),
            "validation_metrics": selected_aggregate,
            "baseline_metrics": baseline_aggregate,
            "best_tracking_configuration": config_record(best_tracking),
            "best_stability_configuration": config_record(best_stability),
            "pid_selected": selected.pid_enabled,
        }
        atomic_json_write(self.store.best_config_path, best_config_payload)

        elapsed_minutes = (time.monotonic() - self.started_monotonic) / 60.0
        failed = len(self.records) - sum(
            bool(record.get("success")) for record in self.records
        )
        lines = [
            "PhysiCar SIM Controller Autotune Final Summary",
            "=" * 48,
            f"session_id: {self.session_id}",
            f"termination_reason: {self.termination_reason}",
            f"elapsed_tuning_time: {elapsed_minutes:.1f} min",
            f"total_experiments: {len(self.records)}",
            f"lap_experiments: {len(lap_records)}",
            f"successful_laps: {len(completed)}",
            f"failed_runs: {failed}",
            f"reason: {self.termination_detail}",
            "",
            "Baseline",
            "--------",
            format_config(BASELINE_CONFIG),
            format_metrics(baseline_aggregate),
            "",
            "Selected configurations",
            "-----------------------",
            f"best_tracking: {format_config(best_tracking)}",
            f"best_stability: {format_config(best_stability)}",
            f"best_balanced: {format_config(selected)}",
            "",
            "Best balanced validation",
            "------------------------",
            format_metrics(selected_aggregate),
            improvement_summary(baseline_aggregate, selected_aggregate),
            "",
            "PID OFF vs PID ON",
            "------------------",
            pid_summary(self.pid_step_records, self.selected_pid_config),
            "",
            "Ranking (configuration aggregates; higher score is better)",
            "----------------------------------------------------------",
            ranking_table(ranking[:12]),
            "",
            "Safety policy",
            "-------------",
            f"path_error_stop_threshold_m: {PATH_ERROR_STOP_M:.3f}",
            f"steering_limit_rad: {STEERING_LIMIT_RAD:.4f}",
            "safe stop published /speed=0 and /steering=0 before/after every run.",
        ]
        with self.store.final_summary_path.open("w", encoding="utf-8") as target:
            target.write("\n".join(lines).rstrip() + "\n")
            target.flush()
            os.fsync(target.fileno())


def configure_logging(path: Path) -> logging.Logger:
    logger = logging.getLogger(f"sim_control_autotune.{os.getpid()}")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    formatter = logging.Formatter(
        "%(asctime)sZ %(levelname)s %(message)s", datefmt="%Y-%m-%dT%H:%M:%S"
    )
    file_handler = logging.FileHandler(path, mode="a", encoding="utf-8")
    file_handler.setFormatter(formatter)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    return logger


def triangle_curvature(
    first: PathPoint, middle: PathPoint, last: PathPoint
) -> float:
    ab = distance((first.x, first.y), (middle.x, middle.y))
    bc = distance((middle.x, middle.y), (last.x, last.y))
    ca = distance((last.x, last.y), (first.x, first.y))
    denominator = ab * bc * ca
    if denominator <= 1.0e-12:
        return 0.0
    twice_area = abs(
        (middle.x - first.x) * (last.y - first.y)
        - (middle.y - first.y) * (last.x - first.x)
    )
    return 2.0 * twice_area / denominator


def sharp_curve_lookahead(config: TrialConfig) -> float:
    normalized = min(1.0, 1.8 / config.curvature_reference)
    return config.max_lookahead - normalized * (
        config.max_lookahead - config.min_lookahead
    )


def coarse_lateral_candidates() -> List[TrialConfig]:
    candidates: List[TrialConfig] = []
    for value in (0.35, 0.30, 0.20):
        candidates.append(replace(BASELINE_CONFIG, min_lookahead=value))
    for value in (0.55, 0.50, 0.40):
        candidates.append(replace(BASELINE_CONFIG, max_lookahead=value))
    for value in (2.2, 1.8, 1.5):
        candidates.append(replace(BASELINE_CONFIG, curvature_reference=value))
    for value in (1.2, 0.8):
        candidates.append(replace(BASELINE_CONFIG, preview_distance=value))
    candidates.extend(
        [
            TrialConfig(0.30, 0.55, 2.2, 1.2),
            TrialConfig(0.30, 0.50, 2.2, 1.2),
            TrialConfig(0.25, 0.50, 1.8, 1.2),
            TrialConfig(0.20, 0.50, 1.8, 1.0),
            TrialConfig(0.20, 0.40, 1.5, 0.8),
            TrialConfig(0.35, 0.55, 2.2, 1.2),
        ]
    )
    return unique_configs(candidates)


def fine_lateral_candidates(
    top: Sequence[TrialConfig],
    tested: set[Tuple[Any, ...]],
    limit: int,
) -> List[TrialConfig]:
    candidates: List[TrialConfig] = []
    for config in top:
        variants = []
        for delta in (-0.025, 0.025):
            variants.append(
                replace(
                    config,
                    min_lookahead=round(config.min_lookahead + delta, 3),
                )
            )
            variants.append(
                replace(
                    config,
                    max_lookahead=round(config.max_lookahead + delta, 3),
                )
            )
        for delta in (-0.10, 0.10):
            variants.append(
                replace(
                    config,
                    curvature_reference=round(
                        config.curvature_reference + delta, 2
                    ),
                )
            )
            variants.append(
                replace(
                    config,
                    preview_distance=round(config.preview_distance + delta, 2),
                )
            )
        for variant in variants:
            if not (
                0.20 <= variant.min_lookahead <= 0.35
                and 0.40 <= variant.max_lookahead <= 0.55
                and variant.min_lookahead <= variant.max_lookahead
                and 1.5 <= variant.curvature_reference <= 2.2
                and 0.8 <= variant.preview_distance <= 1.2
            ):
                continue
            if variant.key() not in tested:
                candidates.append(variant)
    return unique_configs(candidates)[:limit]


def unique_configs(configs: Iterable[TrialConfig]) -> List[TrialConfig]:
    result: List[TrialConfig] = []
    seen: set[Tuple[Any, ...]] = set()
    for config in configs:
        if config.key() in seen:
            continue
        seen.add(config.key())
        result.append(config)
    return result


def eligible_records(
    records: Sequence[Dict[str, Any]], baseline: Optional[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    if baseline is None:
        return []
    baseline_max_error = float(baseline.get("max_path_error") or 0.05)
    baseline_rate = float(baseline.get("p95_steering_rate") or 1.0)
    return [
        record
        for record in records
        if record.get("lap_completed")
        and not int(record.get("steering_saturation_count") or 0)
        and float(record.get("max_abs_steering") or math.inf)
        < STEERING_LIMIT_RAD - SATURATION_EPSILON_RAD
        and float(record.get("max_path_error") or math.inf)
        <= max(baseline_max_error * 1.12, baseline_max_error + 0.006)
        and float(record.get("p95_steering_rate") or math.inf)
        <= max(baseline_rate * 1.30, baseline_rate + 0.05)
    ]


def select_top_configs(
    records: Sequence[Dict[str, Any]],
    count: int,
    baseline: Optional[Dict[str, Any]],
) -> List[TrialConfig]:
    valid = eligible_records(records, baseline)
    ordered = sorted(valid, key=lambda value: value.get("score", -math.inf), reverse=True)
    return unique_configs(config_from_record(record) for record in ordered)[:count]


def convergence_plateau(records: Sequence[Dict[str, Any]]) -> bool:
    scores = [
        float(record["score"])
        for record in records
        if record.get("lap_completed")
        and isinstance(record.get("score"), (float, int))
    ]
    if len(scores) < 10:
        return False
    prior_best = max(scores[:-8])
    recent_best = max(scores[-8:])
    improvement = max(0.0, recent_best - prior_best) / max(abs(prior_best), 1.0)
    return improvement < 0.01


def tracking_objective(record: Dict[str, Any]) -> float:
    return (
        float(record["mean_path_error"])
        + float(record["rms_path_error"])
        + float(record["p95_path_error"])
        + 2.0 * float(record["max_path_error"])
    )


def stability_objective(record: Dict[str, Any]) -> float:
    return (
        0.5 * float(record["mean_abs_steering"])
        + float(record["p95_abs_steering"])
        + 2.0 * float(record["max_abs_steering"])
        + 0.05 * float(record["p95_steering_rate"])
        + 0.02 * float(record["max_steering_rate"])
        + 5.0 * float(record["max_path_error"])
    )


def analyze_step_response(
    samples: Sequence[Dict[str, Any]],
    boundaries: Sequence[Tuple[float, float, str, float]],
) -> Dict[str, Any]:
    steps: List[Dict[str, Any]] = []
    previous_target = 0.0
    for begin, end, name, nominal_target in boundaries[1:]:
        segment = [
            sample for sample in samples
            if begin <= float(sample["elapsed_s"]) < end
        ]
        if not segment:
            continue
        speeds = [float(sample["measured_speed_m_s"]) for sample in segment]
        commands = [float(sample["speed_command_m_s"]) for sample in segment]
        targets = [float(sample["effective_target_m_s"]) for sample in segment]
        amplitude = nominal_target - previous_target
        low = previous_target + 0.10 * amplitude
        high = previous_target + 0.90 * amplitude
        crossing_10 = crossing_time(segment, low, amplitude >= 0.0)
        crossing_90 = crossing_time(segment, high, amplitude >= 0.0)
        response_time = None
        if crossing_10 is not None and crossing_90 is not None:
            response_time = max(0.0, crossing_90 - crossing_10)
        steady_count = max(1, len(segment) // 4)
        steady_speeds = speeds[-steady_count:]
        steady_targets = targets[-steady_count:]
        steady_commands = commands[-steady_count:]
        steady_errors = [
            target - speed
            for target, speed in zip(steady_targets, steady_speeds)
        ]
        # Measure overshoot only after an upward transition and undershoot only
        # after a downward transition. Counting the pre-step value would add
        # the requested step amplitude itself to every gain candidate.
        overshoot = (
            max(0.0, max(speeds) - nominal_target)
            if amplitude > 0.0
            else 0.0
        )
        undershoot = (
            max(0.0, nominal_target - min(speeds))
            if amplitude < 0.0
            else 0.0
        )
        tolerance = max(0.02, abs(amplitude) * 0.05)
        settling = settling_time(segment, tolerance)
        steps.append(
            {
                "transition": name,
                "from_m_s": previous_target,
                "to_m_s": nominal_target,
                "rise_time_s": response_time if amplitude > 0.0 else None,
                "fall_time_s": response_time if amplitude < 0.0 else None,
                "overshoot_m_s": overshoot,
                "undershoot_m_s": undershoot,
                "steady_state_error_m_s": mean(steady_errors),
                "settling_time_s": settling,
                "speed_oscillation_m_s": population_stddev(steady_speeds),
                "command_oscillation_m_s": population_stddev(steady_commands),
            }
        )
        previous_target = nominal_target
    steady_errors = [
        abs(float(step["steady_state_error_m_s"] or 0.0)) for step in steps
    ]
    overshoots = [float(step["overshoot_m_s"]) for step in steps]
    undershoots = [float(step["undershoot_m_s"]) for step in steps]
    settling_values = [
        float(step["settling_time_s"])
        if step["settling_time_s"] is not None
        else 4.0
        for step in steps
    ]
    speed_oscillations = [float(step["speed_oscillation_m_s"]) for step in steps]
    command_oscillations = [
        float(step["command_oscillation_m_s"]) for step in steps
    ]
    step_score = (
        float(mean(steady_errors) or 0.0)
        + 0.5 * float(mean(overshoots) or 0.0)
        + 0.5 * float(mean(undershoots) or 0.0)
        + 0.02 * float(mean(settling_values) or 0.0)
        + 2.0 * float(mean(speed_oscillations) or 0.0)
        + float(mean(command_oscillations) or 0.0)
    )
    return {
        "steps": steps,
        "mean_abs_steady_state_error": mean(steady_errors),
        "mean_overshoot": mean(overshoots),
        "mean_undershoot": mean(undershoots),
        "mean_settling_time": mean(settling_values),
        "mean_speed_oscillation": mean(speed_oscillations),
        "mean_command_oscillation": mean(command_oscillations),
        "step_score": step_score,
    }


def crossing_time(
    segment: Sequence[Dict[str, Any]], threshold: float, rising: bool
) -> Optional[float]:
    for sample in segment:
        speed = float(sample["measured_speed_m_s"])
        if (rising and speed >= threshold) or (not rising and speed <= threshold):
            return float(sample["elapsed_s"])
    return None


def settling_time(
    segment: Sequence[Dict[str, Any]], tolerance: float
) -> Optional[float]:
    for index, sample in enumerate(segment):
        remaining = segment[index:]
        if all(
            abs(
                float(value["effective_target_m_s"])
                - float(value["measured_speed_m_s"])
            ) <= tolerance
            for value in remaining
        ):
            return float(sample["elapsed_s"]) - float(segment[0]["elapsed_s"])
    return None


def best_completed_record(
    records: Sequence[Dict[str, Any]], key: Tuple[Any, ...]
) -> Optional[Dict[str, Any]]:
    matches = [
        record for record in records
        if record.get("lap_completed") and config_from_record(record).key() == key
    ]
    return max(matches, key=lambda record: record.get("score", -math.inf)) \
        if matches else None


def pid_lap_improves(
    pid_laps: Sequence[Dict[str, Any]],
    off_lap: Optional[Dict[str, Any]],
) -> bool:
    if off_lap is None or len(pid_laps) < 2:
        return False
    if not all(record.get("lap_completed") for record in pid_laps):
        return False
    if any(int(record.get("steering_saturation_count") or 0) for record in pid_laps):
        return False
    on = aggregate_records(pid_laps)
    return (
        float(on.get("max_path_error") or math.inf)
        <= float(off_lap.get("max_path_error") or 0.0) * 1.05 + 0.002
        and float(on.get("score") or -math.inf)
        >= float(off_lap.get("score") or -math.inf)
        and float(on.get("mean_abs_measured_speed_error") or math.inf)
        < float(off_lap.get("mean_abs_measured_speed_error") or math.inf)
    )


def final_validation_passes(
    candidate_runs: Sequence[Dict[str, Any]],
    baseline_runs: Sequence[Dict[str, Any]],
) -> bool:
    if len(candidate_runs) < 3 or not baseline_runs:
        return False
    if not all(record.get("lap_completed") for record in candidate_runs):
        return False
    if any(
        record.get("path_error_stop")
        or record.get("target_behind")
        or int(record.get("steering_saturation_count") or 0)
        for record in candidate_runs
    ):
        return False
    candidate = aggregate_records(candidate_runs)
    baseline = aggregate_records(baseline_runs)
    candidate_worst_error = max(float(record["max_path_error"]) for record in candidate_runs)
    baseline_worst_error = max(float(record["max_path_error"]) for record in baseline_runs)
    error_not_worse = candidate_worst_error <= baseline_worst_error * 1.05 + 0.002
    stability_improvement = (
        float(candidate["max_abs_steering"] or math.inf)
        <= float(baseline["max_abs_steering"] or 0.0) - 0.005
        and float(candidate["p95_steering_rate"] or math.inf)
        <= float(baseline["p95_steering_rate"] or 0.0) * 0.95
    )
    laps = [float(record["lap_time"]) for record in candidate_runs]
    errors = [float(record["mean_path_error"]) for record in candidate_runs]
    repeatable = (
        population_stddev(laps) / max(mean(laps) or 1.0, 1.0) <= 0.08
        and population_stddev(errors) <= 0.010
    )
    return (error_not_worse or stability_improvement) and repeatable


def ranked_configs(
    records: Sequence[Dict[str, Any]],
    baseline: Optional[Dict[str, Any]],
) -> List[Tuple[TrialConfig, Dict[str, Any]]]:
    grouped: Dict[Tuple[Any, ...], List[Dict[str, Any]]] = {}
    configs: Dict[Tuple[Any, ...], TrialConfig] = {}
    for record in records:
        if record.get("experiment_type") != "lap":
            continue
        config = config_from_record(record)
        configs[config.key()] = config
        grouped.setdefault(config.key(), []).append(record)
    ranking: List[Tuple[TrialConfig, Dict[str, Any]]] = []
    for key, group in grouped.items():
        aggregate = aggregate_records(group)
        aggregate["success_rate"] = aggregate["completed_laps"] / len(group)
        if baseline is not None:
            representative = {
                **group[0],
                **{
                    metric: aggregate.get(metric)
                    for metric in NUMERIC_METRICS
                },
                "lap_completed": aggregate["completed_laps"] == len(group),
                "steering_saturation_count": aggregate[
                    "steering_saturation_count"
                ],
            }
            aggregate["balanced_score"] = score_lap(representative, baseline)
        else:
            aggregate["balanced_score"] = -10000.0
        ranking.append((configs[key], aggregate))
    ranking.sort(
        key=lambda value: (
            value[1]["success_rate"],
            value[1]["balanced_score"],
        ),
        reverse=True,
    )
    return ranking


def format_config(config: TrialConfig) -> str:
    return (
        f"minLA={config.min_lookahead:.3f} maxLA={config.max_lookahead:.3f} "
        f"curvRef={config.curvature_reference:.2f} "
        f"preview={config.preview_distance:.2f} "
        f"maxLatAcc={config.max_lateral_acceleration:.2f} "
        f"PID={'ON' if config.pid_enabled else 'OFF'} "
        f"kp={config.kp:.3f} ki={config.ki:.3f} kd={config.kd:.3f}"
    )


def format_metrics(metrics: Dict[str, Any]) -> str:
    return (
        f"laps={metrics.get('completed_laps', 0)}/{metrics.get('runs', 0)} "
        f"lap_time={_fmt(metrics.get('lap_time'))} s "
        f"mean_error={_fmt(metrics.get('mean_path_error'))} m "
        f"p95_error={_fmt(metrics.get('p95_path_error'))} m "
        f"max_error={_fmt(metrics.get('max_path_error'))} m "
        f"max_steer={_fmt_degrees(metrics.get('max_abs_steering'))} deg "
        f"p95_rate={_fmt(metrics.get('p95_steering_rate'))} rad/s "
        f"saturation={metrics.get('steering_saturation_count', 0)}"
    )


def improvement_summary(
    baseline: Dict[str, Any], selected: Dict[str, Any]
) -> str:
    pieces = []
    for metric in (
        "mean_path_error",
        "p95_path_error",
        "max_path_error",
        "max_abs_steering",
        "p95_steering_rate",
        "lap_time",
    ):
        before = baseline.get(metric)
        after = selected.get(metric)
        if not isinstance(before, (float, int)) or not isinstance(
            after, (float, int)
        ) or abs(before) <= 1.0e-12:
            continue
        improvement = (before - after) / abs(before) * 100.0
        pieces.append(f"{metric}={improvement:+.2f}%")
    return "improvement (positive is better): " + ", ".join(pieces)


def pid_summary(
    step_records: Sequence[Dict[str, Any]], selected: TrialConfig
) -> str:
    off = [
        record for record in step_records
        if record.get("success") and not record.get("pid_enabled")
    ]
    on = [
        record for record in step_records
        if record.get("success") and record.get("pid_enabled")
    ]
    off_score = min(
        (float(record["step_score"]) for record in off), default=math.inf
    )
    on_score = min(
        (float(record["step_score"]) for record in on), default=math.inf
    )
    return (
        f"PID_OFF_step_score={_fmt(off_score if math.isfinite(off_score) else None)} "
        f"best_PID_ON_step_score={_fmt(on_score if math.isfinite(on_score) else None)} "
        f"final={'PID ON' if selected.pid_enabled else 'PID OFF'} "
        f"({format_config(selected)})"
    )


def ranking_table(
    ranking: Sequence[Tuple[TrialConfig, Dict[str, Any]]]
) -> str:
    header = (
        "rank success score  lap_s meanErr maxErr maxSteer p95Rate config"
    )
    rows = [header]
    for rank, (config, metrics) in enumerate(ranking, start=1):
        rows.append(
            f"{rank:>4} {metrics['success_rate']:>6.1%} "
            f"{metrics['balanced_score']:>6.2f} "
            f"{_fmt(metrics.get('lap_time')):>6} "
            f"{_fmt(metrics.get('mean_path_error')):>7} "
            f"{_fmt(metrics.get('max_path_error')):>6} "
            f"{_fmt_degrees(metrics.get('max_abs_steering')):>8} "
            f"{_fmt(metrics.get('p95_steering_rate')):>7} "
            f"{config.short_name()}"
        )
    return "\n".join(rows)


def _fmt(value: Any) -> str:
    if not isinstance(value, (float, int)) or not math.isfinite(float(value)):
        return "n/a"
    return f"{float(value):.4f}"


def _fmt_degrees(value: Any) -> str:
    if not isinstance(value, (float, int)) or not math.isfinite(float(value)):
        return "n/a"
    return f"{math.degrees(float(value)):.3f}"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Unattended PhysiCar SIM ControllerCore autotuning"
    )
    parser.add_argument("--path", type=Path, default=DEFAULT_PATH)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--sim-api", default=DEFAULT_SIM_API)
    parser.add_argument("--total-budget-min", type=float, default=120.0)
    parser.add_argument("--experiment-budget-min", type=float, default=110.0)
    parser.add_argument("--lap-timeout-s", type=float, default=80.0)
    parser.add_argument("--control-rate-hz", type=float, default=20.0)
    parser.add_argument("--api-timeout-s", type=float, default=0.8)
    parser.add_argument("--reset-settle-s", type=float, default=2.0)
    parser.add_argument("--safe-stop-repetitions", type=int, default=12)
    parser.add_argument("--safe-stop-interval-s", type=float, default=0.05)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args(argv)
    if args.total_budget_min <= 0.0:
        parser.error("--total-budget-min must be positive")
    if not 0.0 < args.experiment_budget_min <= args.total_budget_min:
        parser.error("experiment budget must be positive and <= total budget")
    if args.control_rate_hz < 5.0 or args.control_rate_hz > 50.0:
        parser.error("--control-rate-hz must be within 5..50 Hz")
    if args.lap_timeout_s <= 20.0:
        parser.error("--lap-timeout-s must be greater than 20 seconds")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    tuner: Optional[Autotuner] = None
    exit_code = 1
    try:
        tuner = Autotuner(args)
        tuner.run()
        exit_code = 0
    except RunAbort as error:
        if tuner is not None:
            tuner.termination_reason = "TIME_BUDGET_EXHAUSTED"
            tuner.termination_detail = str(error)
            tuner.logger.warning("time budget termination: %s", error)
    except SafetyAbort as error:
        if tuner is not None:
            tuner.termination_reason = "SAFETY_ABORT"
            tuner.termination_detail = str(error)
            tuner.logger.error("SAFETY_ABORT: %s", error)
    except KeyboardInterrupt:
        if tuner is not None:
            tuner.termination_reason = "SAFETY_ABORT"
            tuner.termination_detail = "KeyboardInterrupt"
            tuner.logger.error("SAFETY_ABORT: KeyboardInterrupt")
    except Exception as error:
        if tuner is not None:
            tuner.termination_reason = "SAFETY_ABORT"
            tuner.termination_detail = f"unhandled exception: {error}"
            tuner.logger.error("unhandled traceback:\n%s", traceback.format_exc())
    finally:
        if tuner is not None:
            try:
                tuner.safe_stop("top-level finally")
                tuner.write_final_outputs()
            except Exception:
                tuner.logger.error(
                    "finalization traceback:\n%s", traceback.format_exc()
                )
                exit_code = 1
            finally:
                tuner.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
