#!/usr/bin/env python3
"""Collect and plot every calibration stage from a ROS 2 bag replay."""

from __future__ import annotations

import csv
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import time

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy

from calibration.calibration_node import CalibrationNode
from calibration.pose_geometry import (
    normalize_angle,
    transform_from_local_correction,
    transform_pose_2d,
)


MAP_FROM_ODOM = (1.3955951074258404, 3.4053781750000933, -1.5687430396611617)
SNAPSHOT_TIMES_SEC = (0.0, 100.0, 200.0, 300.0, 400.0)


def load_waypoints(repository: Path) -> np.ndarray:
    payload = subprocess.check_output(
        ["git", "show", "3a4074b:src/control/config/sim_mapping_waypoints.json"],
        cwd=repository,
        text=True,
    )
    return np.asarray(json.loads(payload)["waypoints"], dtype=np.float64)


def transform_to_map(point: np.ndarray) -> np.ndarray:
    tx, ty, yaw = MAP_FROM_ODOM
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return point @ rotation.T + np.asarray([tx, ty])


def polyline_distances(points: np.ndarray, waypoints: np.ndarray) -> np.ndarray:
    starts = waypoints
    vectors = np.roll(waypoints, -1, axis=0) - starts
    lengths_sq = np.sum(vectors * vectors, axis=1)
    result = []
    for point in points:
        fractions = np.sum((point - starts) * vectors, axis=1) / np.maximum(
            lengths_sq, 1.0e-12
        )
        projections = starts + np.clip(fractions, 0.0, 1.0)[:, None] * vectors
        result.append(float(np.min(np.linalg.norm(projections - point, axis=1))))
    return np.asarray(result)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class StageAnalyzer(CalibrationNode):
    def __init__(self, repository: Path, output_dir: Path) -> None:
        super().__init__()
        self.repository = repository
        self.output_dir = output_dir
        self.waypoints = load_waypoints(repository)
        self.detection_rows: list[dict[str, float]] = []
        self.match_rows: list[dict[str, float]] = []
        self.pose_rows: list[dict[str, float]] = []
        self.detection_snapshots: list[dict[str, object]] = []
        self.match_snapshots: list[dict[str, object]] = []
        self.first_stamp: float | None = None
        self.last_sample_wall = time.monotonic()
        self.finished = False
        self.initial_yaw_hold_sec = float(
            os.environ.get("CALIBRATION_INITIAL_YAW_HOLD_SEC", "0.0")
        )
        self.lateral_only_ekf = copy.deepcopy(self.correction_ekf)
        self.lateral_only_last_odom_time_sec: float | None = None
        self.create_timer(1.0, self._check_finished)

    def relative_time(self, stamp) -> float:
        value = stamp_seconds(stamp)
        if self.first_stamp is None:
            self.first_stamp = value
        return value - self.first_stamp

    @staticmethod
    def _snapshot_due(existing, elapsed: float) -> bool:
        index = len(existing)
        return index < len(SNAPSHOT_TIMES_SEC) and elapsed >= SNAPSHOT_TIMES_SEC[index]

    def on_image(self, message) -> None:
        previous_image = self.latest_source_image
        super().on_image(message)
        if self.latest_source_image is previous_image:
            return
        elapsed = self.relative_time(message.header.stamp)
        detection = self.latest_detection
        self.detection_rows.append(
            {
                "time_sec": elapsed,
                "valid": float(detection is not None),
                "confidence": float(detection.confidence if detection else 0.0),
                "point_count": float(detection.point_count if detection else 0),
                "span_m": float(detection.span_m if detection else 0.0),
            }
        )
        if self._snapshot_due(self.detection_snapshots, elapsed):
            mask = (
                detection.mask
                if detection is not None
                else self.detector.create_mask(self.latest_bev)
            )
            overlay = self.detector.draw_overlay(self.latest_bev, detection)
            self.detection_snapshots.append(
                {
                    "time_sec": elapsed,
                    "source": self.latest_source_image.copy(),
                    "bev": self.latest_bev.copy(),
                    "mask": mask.copy(),
                    "overlay": overlay.copy(),
                }
            )

    def on_odometry(self, message) -> None:
        previous_lane_stamp = self.last_processed_lane_stamp
        raw_pose = self._pose_2d(message.pose.pose)
        odom_time_sec = stamp_seconds(message.header.stamp)
        lateral_only_dt = (
            0.0
            if self.lateral_only_last_odom_time_sec is None
            else max(0.0, odom_time_sec - self.lateral_only_last_odom_time_sec)
        )
        self.lateral_only_last_odom_time_sec = odom_time_sec
        self.lateral_only_ekf.predict(
            raw_pose,
            lateral_only_dt,
            pose_covariance=message.pose.covariance,
            twist_covariance=message.twist.covariance,
        )
        super().on_odometry(message)
        elapsed = self.relative_time(message.header.stamp)
        correction_stamp = self.latest_correction_stamp
        correction_applied = (
            correction_stamp is not None
            and correction_stamp.sec == message.header.stamp.sec
            and correction_stamp.nanosec == message.header.stamp.nanosec
        )
        if correction_applied and self.latest_correction is not None:
            correction = self.latest_correction
            experiment_yaw = (
                0.0
                if elapsed <= self.initial_yaw_hold_sec
                else correction.yaw_rad
            )
            measurement_transform = transform_from_local_correction(
                raw_pose, correction.lateral_m, experiment_yaw
            )
            measured_pose = transform_pose_2d(raw_pose, measurement_transform)
            self.lateral_only_ekf.correct(
                measured_pose,
                rms_error_m=correction.rms_error_m,
                match_count=correction.match_count,
            )
        self.lateral_only_ekf.advance_output(lateral_only_dt)
        output_pose = self.correction_ekf.output_pose or raw_pose
        lateral_only_pose = self.lateral_only_ekf.output_pose or raw_pose
        raw_map = transform_to_map(np.asarray([[raw_pose[0], raw_pose[1]]]))[0]
        corrected_map = transform_to_map(
            np.asarray([[output_pose[0], output_pose[1]]])
        )[0]
        lateral_only_map = transform_to_map(
            np.asarray([[lateral_only_pose[0], lateral_only_pose[1]]])
        )[0]
        raw_error = polyline_distances(raw_map[None, :], self.waypoints)[0]
        corrected_error = polyline_distances(corrected_map[None, :], self.waypoints)[0]
        lateral_only_error = polyline_distances(
            lateral_only_map[None, :], self.waypoints
        )[0]
        covariance = self.correction_ekf.output_covariance()
        lag = (
            self.correction_ekf.state - self.correction_ekf.output_state
            if self.correction_ekf.state is not None
            and self.correction_ekf.output_state is not None
            else np.zeros(3)
        )
        lag[2] = normalize_angle(float(lag[2]))
        self.pose_rows.append(
            {
                "time_sec": elapsed,
                "raw_x": raw_pose[0],
                "raw_y": raw_pose[1],
                "raw_yaw": raw_pose[2],
                "output_x": output_pose[0],
                "output_y": output_pose[1],
                "output_yaw": output_pose[2],
                "raw_waypoint_error_m": raw_error,
                "corrected_waypoint_error_m": corrected_error,
                "lateral_only_x": lateral_only_pose[0],
                "lateral_only_y": lateral_only_pose[1],
                "lateral_only_yaw": lateral_only_pose[2],
                "lateral_only_waypoint_error_m": lateral_only_error,
                "p_x": covariance[0, 0],
                "p_y": covariance[1, 1],
                "p_yaw": covariance[2, 2],
                "lag_position_m": float(np.linalg.norm(lag[:2])),
                "lag_yaw_rad": abs(float(lag[2])),
            }
        )
        self.last_sample_wall = time.monotonic()

        if self.last_processed_lane_stamp == previous_lane_stamp:
            return
        diagnostics = self.corrector.last_diagnostics
        correction_stamp = self.latest_correction_stamp
        success = (
            correction_stamp is not None
            and correction_stamp.sec == message.header.stamp.sec
            and correction_stamp.nanosec == message.header.stamp.nanosec
        )
        correction = self.latest_correction if success else None
        measured = self.latest_measurement_pose if success else None
        keep_count = int(np.count_nonzero(diagnostics.keep)) if diagnostics else 0
        total_count = len(diagnostics.keep) if diagnostics else 0
        kept_distances = (
            diagnostics.distances_m[diagnostics.keep]
            if diagnostics is not None and keep_count
            else np.empty(0)
        )
        kept_angles = (
            diagnostics.tangent_angle_rad[diagnostics.keep]
            if diagnostics is not None and keep_count
            else np.empty(0)
        )
        row = {
            "time_sec": elapsed,
            "success": float(correction is not None),
            "candidate_count": float(total_count),
            "match_count": float(keep_count),
            "keep_ratio": float(keep_count / max(1, total_count)),
            "mean_match_distance_m": float(np.mean(kept_distances)) if len(kept_distances) else math.nan,
            "max_match_distance_m": float(np.max(kept_distances)) if len(kept_distances) else math.nan,
            "mean_tangent_angle_deg": float(np.degrees(np.mean(kept_angles))) if len(kept_angles) else math.nan,
            "lateral_m": float(correction.lateral_m) if correction else math.nan,
            "yaw_correction_rad": float(correction.yaw_rad) if correction else math.nan,
            "rms_error_m": float(correction.rms_error_m) if correction else math.nan,
            "measured_x": float(measured[0]) if correction and measured else math.nan,
            "measured_y": float(measured[1]) if correction and measured else math.nan,
            "measured_yaw": float(measured[2]) if correction and measured else math.nan,
            "innovation_x": float(self.correction_ekf.last_innovation[0]),
            "innovation_y": float(self.correction_ekf.last_innovation[1]),
            "innovation_yaw": float(self.correction_ekf.last_innovation[2]),
            "gain_x": float(self.correction_ekf.last_gain_diagonal[0]),
            "gain_y": float(self.correction_ekf.last_gain_diagonal[1]),
            "gain_yaw": float(self.correction_ekf.last_gain_diagonal[2]),
        }
        self.match_rows.append(row)
        if diagnostics is not None and self._snapshot_due(self.match_snapshots, elapsed):
            self.match_snapshots.append(
                {
                    "time_sec": elapsed,
                    "diagnostics": diagnostics,
                    "success": correction is not None,
                }
            )

    def _check_finished(self) -> None:
        if self.pose_rows and time.monotonic() - self.last_sample_wall > 5.0:
            self.finished = True


def save_csv(path: Path, rows: list[dict[str, float]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_detection(node: StageAnalyzer) -> None:
    rows = node.detection_rows
    t = np.asarray([row["time_sec"] for row in rows])
    figure, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True, constrained_layout=True)
    axes[0].plot(t, [row["confidence"] for row in rows], color="#ff9800", lw=0.8)
    axes[0].axhline(0.20, color="red", ls="--", label="minimum 0.20")
    axes[0].set_ylabel("Confidence")
    axes[0].legend()
    axes[1].plot(t, [row["point_count"] for row in rows], color="#673ab7", lw=0.8)
    axes[1].axhline(12, color="red", ls="--", label="minimum 12")
    axes[1].set_ylabel("Skeleton points")
    axes[1].legend()
    axes[2].plot(t, [row["span_m"] for row in rows], color="#009688", lw=0.8)
    axes[2].axhline(0.30, color="red", ls="--", label="minimum 0.30 m")
    axes[2].set_ylabel("Lane span [m]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Stage 1: yellow-lane detection quality")
    figure.savefig(node.output_dir / "01_detection_quality.png", dpi=160)
    plt.close(figure)


def plot_detection_samples(node: StageAnalyzer) -> None:
    snapshots = node.detection_snapshots
    figure, axes = plt.subplots(len(snapshots), 4, figsize=(16, 3.2 * len(snapshots)), constrained_layout=True)
    if len(snapshots) == 1:
        axes = axes[None, :]
    titles = ("Camera", "BEV", "Yellow mask", "Skeleton overlay")
    for row_axes, snapshot in zip(axes, snapshots):
        images = (snapshot["source"], snapshot["bev"], snapshot["mask"], snapshot["overlay"])
        for axis, title, image in zip(row_axes, titles, images):
            axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if image.ndim == 3 else image, cmap="gray")
            axis.set_title(f"{title}  t={snapshot['time_sec']:.0f}s")
            axis.axis("off")
    figure.suptitle("Stage 1 samples: camera to BEV yellow skeleton")
    figure.savefig(node.output_dir / "02_detection_samples.png", dpi=150)
    plt.close(figure)


def plot_matching_snapshots(node: StageAnalyzer) -> None:
    snapshots = node.match_snapshots
    figure, axes = plt.subplots(1, len(snapshots), figsize=(5 * len(snapshots), 5), constrained_layout=True)
    if len(snapshots) == 1:
        axes = [axes]
    for axis, snapshot in zip(axes, snapshots):
        diagnostics = snapshot["diagnostics"]
        keep = diagnostics.keep
        axis.scatter(diagnostics.matched_points[:, 0], diagnostics.matched_points[:, 1], s=7, c="0.75", label="nearest map")
        axis.scatter(diagnostics.observed_odom[~keep, 0], diagnostics.observed_odom[~keep, 1], s=9, c="red", label="rejected")
        axis.scatter(diagnostics.observed_odom[keep, 0], diagnostics.observed_odom[keep, 1], s=10, c="#00bcd4", label="kept")
        for observed, matched, valid in zip(diagnostics.observed_odom, diagnostics.matched_points, keep):
            axis.plot([observed[0], matched[0]], [observed[1], matched[1]], color="#4caf50" if valid else "#ef9a9a", lw=0.35)
        step = max(1, len(diagnostics.observed_odom) // 12)
        sampled = slice(None, None, step)
        axis.quiver(
            diagnostics.observed_odom[sampled, 0], diagnostics.observed_odom[sampled, 1],
            diagnostics.observed_directions[sampled, 0], diagnostics.observed_directions[sampled, 1],
            color="#3f51b5", scale=12,
        )
        axis.set_title(f"t={snapshot['time_sec']:.0f}s, success={snapshot['success']}")
        axis.set_aspect("equal")
        axis.grid(alpha=0.2)
    axes[0].legend(fontsize=8)
    figure.suptitle("Stages 2-3: local directions and nearest Lane Map associations")
    figure.savefig(node.output_dir / "03_local_direction_matching.png", dpi=160)
    plt.close(figure)


def plot_matching_metrics(node: StageAnalyzer) -> None:
    rows = node.match_rows
    t = np.asarray([row["time_sec"] for row in rows])
    figure, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True, constrained_layout=True)
    axes[0].plot(t, [row["match_count"] for row in rows], lw=0.8, label="kept matches")
    axes[0].plot(t, [row["candidate_count"] for row in rows], lw=0.5, alpha=0.5, label="candidates")
    axes[0].axhline(8, color="red", ls="--")
    axes[0].set_ylabel("Point count")
    axes[0].legend()
    axes[1].plot(t, [row["mean_match_distance_m"] for row in rows], label="mean distance", lw=0.8)
    axes[1].plot(t, [row["max_match_distance_m"] for row in rows], label="max distance", lw=0.6)
    axes[1].axhline(0.35, color="red", ls="--")
    axes[1].set_ylabel("Distance [m]")
    axes[1].legend()
    axes[2].plot(t, [row["mean_tangent_angle_deg"] for row in rows], color="#9c27b0", lw=0.8)
    axes[2].axhline(math.degrees(0.44), color="red", ls="--")
    axes[2].set_ylabel("Tangent angle [deg]")
    axes[3].plot(t, [row["rms_error_m"] for row in rows], label="point-to-line RMS", lw=0.8)
    axes[3].plot(t, np.abs([row["lateral_m"] for row in rows]), label="|lateral correction|", lw=0.7)
    axes[3].set_ylabel("Correction/error [m]")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Stages 3-4: matching gates and geometric correction")
    figure.savefig(node.output_dir / "04_matching_quality.png", dpi=160)
    plt.close(figure)


def plot_waypoint_map_difference(node: StageAnalyzer) -> dict[str, float]:
    lane_points = node.reference_lane.points
    offsets = node.waypoints[:, None, :] - lane_points[None, :, :]
    distances = np.min(np.linalg.norm(offsets, axis=2), axis=1)
    figure, axes = plt.subplots(1, 2, figsize=(16, 6), constrained_layout=True)
    axes[0].scatter(lane_points[:, 0], lane_points[:, 1], s=2, c="#ff9800", label="Lane Map skeleton")
    axes[0].plot(node.waypoints[:, 0], node.waypoints[:, 1], c="#4caf50", lw=1.2, label="669 waypoints")
    axes[0].set_aspect("equal")
    axes[0].legend()
    axes[0].grid(alpha=0.2)
    axes[1].plot(np.arange(len(distances)), distances, lw=0.9)
    axes[1].set_xlabel("Waypoint index")
    axes[1].set_ylabel("Nearest Lane Map point distance [m]")
    axes[1].grid(alpha=0.25)
    figure.suptitle("Stage 5: structural difference between waypoint route and Lane Map")
    figure.savefig(node.output_dir / "05_waypoint_lane_map_difference.png", dpi=160)
    plt.close(figure)
    return {
        "mean_m": float(np.mean(distances)),
        "rmse_m": float(np.sqrt(np.mean(distances * distances))),
        "p95_m": float(np.percentile(distances, 95)),
        "max_m": float(np.max(distances)),
    }


def plot_pose_measurements(node: StageAnalyzer) -> None:
    rows = [row for row in node.match_rows if row["success"] > 0.5]
    t = np.asarray([row["time_sec"] for row in rows])
    figure, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True, constrained_layout=True)
    axes[0].plot(t, [row["lateral_m"] for row in rows], lw=0.8)
    axes[0].axhline(0.20, color="red", ls="--"); axes[0].axhline(-0.20, color="red", ls="--")
    axes[0].set_ylabel("Lateral measurement [m]")
    axes[1].plot(t, np.degrees([row["yaw_correction_rad"] for row in rows]), lw=0.8)
    axes[1].set_ylabel("Yaw measurement [deg]")
    position_innovation = np.hypot([row["innovation_x"] for row in rows], [row["innovation_y"] for row in rows])
    axes[2].plot(t, position_innovation, label="pose innovation", lw=0.8)
    axes[2].set_ylabel("Innovation position [m]")
    axes[2].set_xlabel("Time [s]")
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Stage 6: lateral/yaw correction converted to EKF pose measurement")
    figure.savefig(node.output_dir / "06_pose_measurement.png", dpi=160)
    plt.close(figure)


def plot_ekf(node: StageAnalyzer) -> None:
    matches = [row for row in node.match_rows if row["success"] > 0.5]
    poses = node.pose_rows
    tm = np.asarray([row["time_sec"] for row in matches])
    tp = np.asarray([row["time_sec"] for row in poses])
    figure, axes = plt.subplots(4, 1, figsize=(16, 13), sharex=True, constrained_layout=True)
    axes[0].plot(tm, [row["gain_x"] for row in matches], label="Kx", lw=0.7)
    axes[0].plot(tm, [row["gain_y"] for row in matches], label="Ky", lw=0.7)
    axes[0].plot(tm, [row["gain_yaw"] for row in matches], label="Kyaw", lw=0.7)
    axes[0].set_ylabel("Kalman gain"); axes[0].legend()
    axes[1].plot(tp, [row["p_x"] for row in poses], label="Pxx", lw=0.7)
    axes[1].plot(tp, [row["p_y"] for row in poses], label="Pyy", lw=0.7)
    axes[1].plot(tp, [row["p_yaw"] for row in poses], label="Pyaw", lw=0.7)
    axes[1].set_yscale("log"); axes[1].set_ylabel("Output variance"); axes[1].legend()
    axes[2].plot(tp, [row["lag_position_m"] for row in poses], label="position lag", lw=0.8)
    axes[2].plot(tp, [row["lag_yaw_rad"] for row in poses], label="yaw lag [rad]", lw=0.8)
    axes[2].set_ylabel("EKF target-output lag"); axes[2].legend()
    axes[3].plot(tp, [row["raw_waypoint_error_m"] for row in poses], color="red", label="RAW", lw=0.7)
    axes[3].plot(tp, [row["corrected_waypoint_error_m"] for row in poses], color="#00bcd4", label="EKF", lw=0.7)
    axes[3].set_ylabel("Waypoint distance [m]"); axes[3].set_xlabel("Time [s]"); axes[3].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Stage 7: EKF gain, uncertainty, rate-limit lag, and final error")
    figure.savefig(node.output_dir / "07_ekf_diagnostics.png", dpi=160)
    plt.close(figure)


def plot_final_effect(node: StageAnalyzer) -> None:
    poses = node.pose_rows
    t = np.asarray([row["time_sec"] for row in poses])
    raw = np.asarray([row["raw_waypoint_error_m"] for row in poses])
    corrected = np.asarray([row["corrected_waypoint_error_m"] for row in poses])
    improvement = raw - corrected
    figure, axes = plt.subplots(2, 1, figsize=(16, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t, raw, color="red", label="RAW odom", lw=0.7)
    axes[0].plot(t, corrected, color="#00bcd4", label="EKF corrected", lw=0.7)
    axes[0].set_ylabel("Waypoint distance [m]")
    axes[0].legend()
    axes[1].fill_between(t, 0.0, improvement, where=improvement >= 0.0,
                         color="#2ca02c", alpha=0.65, label="improved")
    axes[1].fill_between(t, 0.0, improvement, where=improvement < 0.0,
                         color="#d62728", alpha=0.65, label="worsened")
    axes[1].axhline(0.0, color="black", lw=0.8)
    axes[1].set_ylabel("RAW - EKF [m]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Stage 8: final waypoint-distance effect (positive means improvement)")
    figure.savefig(node.output_dir / "08_final_waypoint_effect.png", dpi=160)
    plt.close(figure)


def plot_initial_lateral_only_experiment(node: StageAnalyzer) -> None:
    if node.initial_yaw_hold_sec <= 0.0:
        return
    rows = [
        row for row in node.pose_rows
        if row["time_sec"] <= node.initial_yaw_hold_sec
    ]
    t = np.asarray([row["time_sec"] for row in rows])
    raw = np.asarray([row["raw_waypoint_error_m"] for row in rows])
    baseline = np.asarray([row["corrected_waypoint_error_m"] for row in rows])
    lateral_only = np.asarray(
        [row["lateral_only_waypoint_error_m"] for row in rows]
    )
    raw_yaw = np.unwrap([row["raw_yaw"] for row in rows])
    baseline_yaw = np.unwrap([row["output_yaw"] for row in rows])
    lateral_only_yaw = np.unwrap([row["lateral_only_yaw"] for row in rows])
    figure, axes = plt.subplots(2, 1, figsize=(16, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t, raw, label="RAW", color="red", lw=0.9)
    axes[0].plot(t, baseline, label="baseline yaw+lateral", color="#00bcd4", lw=0.9)
    axes[0].plot(t, lateral_only, label="lateral-only", color="#2ca02c", lw=0.9)
    axes[0].set_ylabel("Waypoint distance [m]")
    axes[0].legend()
    axes[1].plot(t, np.degrees(baseline_yaw - raw_yaw), label="baseline yaw - RAW", lw=0.9)
    axes[1].plot(t, np.degrees(lateral_only_yaw - raw_yaw), label="lateral-only yaw - RAW", lw=0.9)
    axes[1].axhline(0.0, color="black", lw=0.8)
    axes[1].set_ylabel("Output yaw difference [deg]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle(
        f"Initial {node.initial_yaw_hold_sec:.0f}s isolation: disable lane yaw correction"
    )
    figure.savefig(node.output_dir / "09_initial_lateral_only_experiment.png", dpi=160)
    plt.close(figure)

    def metrics(values: np.ndarray) -> dict[str, float]:
        return {
            "mean_m": float(np.mean(values)),
            "rmse_m": float(np.sqrt(np.mean(values * values))),
            "p95_m": float(np.percentile(values, 95)),
            "max_m": float(np.max(values)),
        }
    comparison = {
        "duration_sec": node.initial_yaw_hold_sec,
        "sample_count": len(rows),
        "raw": metrics(raw),
        "baseline_yaw_lateral": metrics(baseline),
        "lateral_only": metrics(lateral_only),
    }
    (node.output_dir / "09_initial_lateral_only_experiment.json").write_text(
        json.dumps(comparison, indent=2) + "\n", encoding="utf-8"
    )


def plot_final_effect(node: StageAnalyzer) -> None:
    poses = node.pose_rows
    t = np.asarray([row["time_sec"] for row in poses])
    raw = np.asarray([row["raw_waypoint_error_m"] for row in poses])
    corrected = np.asarray([row["corrected_waypoint_error_m"] for row in poses])
    improvement = raw - corrected
    figure, axes = plt.subplots(2, 1, figsize=(16, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t, raw, color="red", label="RAW odom", lw=0.7)
    axes[0].plot(t, corrected, color="#00bcd4", label="EKF corrected", lw=0.7)
    axes[0].set_ylabel("Waypoint distance [m]")
    axes[0].legend()
    axes[1].fill_between(t, 0.0, improvement, where=improvement >= 0.0,
                         color="#2ca02c", alpha=0.65, label="improved")
    axes[1].fill_between(t, 0.0, improvement, where=improvement < 0.0,
                         color="#d62728", alpha=0.65, label="worsened")
    axes[1].axhline(0.0, color="black", lw=0.8)
    axes[1].set_ylabel("RAW - EKF [m]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Stage 8: final waypoint-distance effect (positive means improvement)")
    figure.savefig(node.output_dir / "08_final_waypoint_effect.png", dpi=160)
    plt.close(figure)


def summarize(node: StageAnalyzer, map_difference: dict[str, float]) -> dict[str, object]:
    detections = node.detection_rows
    matches = node.match_rows
    poses = node.pose_rows
    raw = np.asarray([row["raw_waypoint_error_m"] for row in poses])
    corrected = np.asarray([row["corrected_waypoint_error_m"] for row in poses])
    successful = [row for row in matches if row["success"] > 0.5]
    successful_lateral = np.abs([row["lateral_m"] for row in successful])
    successful_yaw = np.abs([row["yaw_correction_rad"] for row in successful])
    successful_rms = np.asarray([row["rms_error_m"] for row in successful])
    improvement = raw - corrected
    large_error_improvement = improvement[raw > 0.20]
    successful_lateral = np.abs([row["lateral_m"] for row in successful])
    successful_yaw = np.abs([row["yaw_correction_rad"] for row in successful])
    successful_rms = np.asarray([row["rms_error_m"] for row in successful])
    improvement = raw - corrected
    metric = lambda values: {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }
    return {
        "detection": {
            "processed_frames": len(detections),
            "valid_frames": int(sum(row["valid"] for row in detections)),
            "valid_ratio": float(np.mean([row["valid"] for row in detections])),
            "mean_confidence_valid": float(np.mean([row["confidence"] for row in detections if row["valid"]])) if any(row["valid"] for row in detections) else 0.0,
        },
        "matching": {
            "attempts": len(matches),
            "successes": len(successful),
            "success_ratio": len(successful) / max(1, len(matches)),
            "mean_match_count": float(np.mean([row["match_count"] for row in successful])) if successful else 0.0,
            "mean_rms_m": float(np.mean([row["rms_error_m"] for row in successful])) if successful else math.nan,
            "mean_abs_lateral_m": float(np.mean(np.abs([row["lateral_m"] for row in successful]))) if successful else math.nan,
            "mean_abs_yaw_deg": float(np.degrees(np.mean(np.abs([row["yaw_correction_rad"] for row in successful])))) if successful else math.nan,
            "lateral_saturation_count": int(np.sum(successful_lateral >= 0.195)),
            "yaw_saturation_count": int(np.sum(successful_yaw >= 0.118)),
            "rms_over_0_10m_count": int(np.sum(successful_rms > 0.10)),
            "rms_over_0_20m_count": int(np.sum(successful_rms > 0.20)),
            "lateral_saturation_count": int(np.sum(successful_lateral >= 0.195)),
            "yaw_saturation_count": int(np.sum(successful_yaw >= 0.118)),
            "rms_over_0_10m_count": int(np.sum(successful_rms > 0.10)),
            "rms_over_0_20m_count": int(np.sum(successful_rms > 0.20)),
        },
        "waypoint_lane_map": map_difference,
        "waypoint_error_raw": metric(raw),
        "waypoint_error_corrected": metric(corrected),
        "final_effect": {
            "improved_samples": int(np.sum(improvement > 0.0)),
            "worsened_samples": int(np.sum(improvement < 0.0)),
            "mean_improvement_m": float(np.mean(improvement)),
            "mean_improvement_when_raw_over_0_20m": (
                float(np.mean(large_error_improvement))
                if len(large_error_improvement)
                else None
            ),
            "mean_improvement_when_raw_at_most_0_10m": float(np.mean(improvement[raw <= 0.10])),
        },
        "final_effect": {
            "improved_samples": int(np.sum(improvement > 0.0)),
            "worsened_samples": int(np.sum(improvement < 0.0)),
            "mean_improvement_m": float(np.mean(improvement)),
            "mean_improvement_when_raw_over_0_20m": float(np.mean(improvement[raw > 0.20])),
            "mean_improvement_when_raw_at_most_0_10m": float(np.mean(improvement[raw <= 0.10])),
        },
        "sample_count": len(poses),
    }


def main() -> None:
    repository = Path("/ws")
    output_dir = Path(
        os.environ.get(
            "CALIBRATION_STAGE_OUTPUT_DIR",
            str(repository / "src/calibration/docs/calibration_stage_analysis"),
        )
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = StageAnalyzer(repository, output_dir)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    save_csv(output_dir / "detection_metrics.csv", node.detection_rows)
    save_csv(output_dir / "matching_ekf_metrics.csv", node.match_rows)
    save_csv(output_dir / "pose_waypoint_metrics.csv", node.pose_rows)
    plot_detection(node)
    plot_detection_samples(node)
    plot_matching_snapshots(node)
    plot_matching_metrics(node)
    map_difference = plot_waypoint_map_difference(node)
    plot_pose_measurements(node)
    plot_ekf(node)
    plot_final_effect(node)
    plot_initial_lateral_only_experiment(node)
    plot_final_effect(node)
    summary = summarize(node, map_difference)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
