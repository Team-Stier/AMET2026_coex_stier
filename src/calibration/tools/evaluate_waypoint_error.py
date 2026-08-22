#!/usr/bin/env python3
"""Plot raw and calibrated odometry distance to the simulator waypoint route."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


MAP_FROM_ODOM = (1.3955951074258404, 3.4053781750000933, -1.5687430396611617)


def load_waypoints(repository: Path, revision: str) -> np.ndarray:
    payload = subprocess.check_output(
        ["git", "show", f"{revision}:src/control/config/sim_mapping_waypoints.json"],
        cwd=repository,
        text=True,
    )
    return np.asarray(json.loads(payload)["waypoints"], dtype=np.float64)


def transform_to_map(x: float, y: float) -> np.ndarray:
    tx, ty, yaw = MAP_FROM_ODOM
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [tx + cosine * x - sine * y, ty + sine * x + cosine * y],
        dtype=np.float64,
    )


def distance_to_closed_polyline(point: np.ndarray, waypoints: np.ndarray) -> float:
    starts = waypoints
    vectors = np.roll(waypoints, -1, axis=0) - starts
    lengths_sq = np.sum(vectors * vectors, axis=1)
    fractions = np.sum((point - starts) * vectors, axis=1) / np.maximum(
        lengths_sq, 1.0e-12
    )
    fractions = np.clip(fractions, 0.0, 1.0)
    projections = starts + fractions[:, None] * vectors
    return float(np.min(np.linalg.norm(projections - point, axis=1)))


class ErrorCollector(Node):
    def __init__(self, waypoints: np.ndarray, idle_timeout_sec: float) -> None:
        super().__init__("waypoint_error_collector")
        self.waypoints = waypoints
        self.idle_timeout_sec = idle_timeout_sec
        self.raw: dict[tuple[int, int], Odometry] = {}
        self.corrected: dict[tuple[int, int], Odometry] = {}
        self.rows: list[tuple[float, float, float]] = []
        self.last_sample_wall_time = time.monotonic()
        self.finished = False
        self.create_subscription(Odometry, "/odom", self.on_raw, 100)
        self.create_subscription(Odometry, "/odom/calibride", self.on_corrected, 100)
        self.create_timer(1.0, self.check_finished)

    @staticmethod
    def key(message: Odometry) -> tuple[int, int]:
        return message.header.stamp.sec, message.header.stamp.nanosec

    def on_raw(self, message: Odometry) -> None:
        key = self.key(message)
        self.raw[key] = message
        self.try_pair(key)

    def on_corrected(self, message: Odometry) -> None:
        key = self.key(message)
        self.corrected[key] = message
        self.try_pair(key)

    def try_pair(self, key: tuple[int, int]) -> None:
        raw = self.raw.get(key)
        corrected = self.corrected.get(key)
        if raw is None or corrected is None:
            return
        raw_position = raw.pose.pose.position
        corrected_position = corrected.pose.pose.position
        raw_map = transform_to_map(raw_position.x, raw_position.y)
        corrected_map = transform_to_map(corrected_position.x, corrected_position.y)
        stamp = key[0] + key[1] * 1.0e-9
        self.rows.append(
            (
                stamp,
                distance_to_closed_polyline(raw_map, self.waypoints),
                distance_to_closed_polyline(corrected_map, self.waypoints),
            )
        )
        del self.raw[key]
        del self.corrected[key]
        self.last_sample_wall_time = time.monotonic()

    def check_finished(self) -> None:
        if self.rows and time.monotonic() - self.last_sample_wall_time > self.idle_timeout_sec:
            self.finished = True


def metrics(values: np.ndarray) -> dict[str, float]:
    return {
        "mean_m": float(np.mean(values)),
        "rmse_m": float(np.sqrt(np.mean(values * values))),
        "p95_m": float(np.percentile(values, 95.0)),
        "max_m": float(np.max(values)),
    }


def save_outputs(rows: np.ndarray, output: Path) -> None:
    rows = rows[np.argsort(rows[:, 0])]
    time_sec = rows[:, 0] - rows[0, 0]
    raw_error = rows[:, 1]
    corrected_error = rows[:, 2]
    raw_metrics = metrics(raw_error)
    corrected_metrics = metrics(corrected_error)
    improvement = 100.0 * (raw_metrics["rmse_m"] - corrected_metrics["rmse_m"]) / max(
        raw_metrics["rmse_m"], 1.0e-12
    )

    figure, axis = plt.subplots(figsize=(16, 7), constrained_layout=True)
    axis.plot(time_sec, raw_error, color="#e53935", linewidth=0.8, alpha=0.72, label="Raw odom")
    axis.plot(
        time_sec,
        corrected_error,
        color="#00b8d4",
        linewidth=0.9,
        alpha=0.85,
        label="EKF corrected odom",
    )
    axis.set_title("Distance to 669-waypoint reference over time")
    axis.set_xlabel("Time from bag start [s]")
    axis.set_ylabel("Nearest route-segment distance [m]")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper right")
    summary = (
        f"RAW    RMSE {raw_metrics['rmse_m']:.3f} m | mean {raw_metrics['mean_m']:.3f} m | "
        f"P95 {raw_metrics['p95_m']:.3f} m | max {raw_metrics['max_m']:.3f} m\n"
        f"EKF    RMSE {corrected_metrics['rmse_m']:.3f} m | mean {corrected_metrics['mean_m']:.3f} m | "
        f"P95 {corrected_metrics['p95_m']:.3f} m | max {corrected_metrics['max_m']:.3f} m\n"
        f"RMSE change: {improvement:+.1f}%"
    )
    axis.text(
        0.012,
        0.985,
        summary,
        transform=axis.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        family="monospace",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.88},
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
    output.with_suffix(".json").write_text(
        json.dumps(
            {
                "reference": "669 simulator route waypoints",
                "metric": "nearest closed waypoint-route segment distance",
                "sample_count": int(len(rows)),
                "duration_sec": float(time_sec[-1]),
                "raw": raw_metrics,
                "ekf_corrected": corrected_metrics,
                "rmse_improvement_percent": improvement,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", type=Path, default=Path("/ws"))
    parser.add_argument("--waypoint-revision", default="3a4074b")
    parser.add_argument("--idle-timeout-sec", type=float, default=5.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/ws/src/calibration/docs/ekf_waypoint_error_over_time.png"),
    )
    args = parser.parse_args()
    waypoints = load_waypoints(args.repository, args.waypoint_revision)
    rclpy.init()
    node = ErrorCollector(waypoints, args.idle_timeout_sec)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        rows = np.asarray(node.rows, dtype=np.float64)
        node.destroy_node()
        rclpy.shutdown()
    if len(rows) < 2:
        raise RuntimeError(f"not enough paired odometry samples: {len(rows)}")
    save_outputs(rows, args.output)
    print(f"saved {len(rows)} paired samples to {args.output}")


if __name__ == "__main__":
    main()
