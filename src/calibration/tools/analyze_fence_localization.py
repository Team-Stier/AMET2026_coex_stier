#!/usr/bin/env python3
"""Replay a diagnostic MCAP and graph four-line fence localization accuracy."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from calibration.correction_ekf import PoseCorrectionEkf
from calibration.fence_corrector import FenceOdomCorrector, FenceReference
from calibration.pose_geometry import (
    map_from_odom_pose,
    normalize_angle,
    transform_pose_2d,
)


TOPICS = {
    "/odom",
    "/odom/laser",
    "/scan_filtered",
    "/sim/ground_truth/tf",
}
COLORS = {
    "truth": "#20d420",
    "odom": "#ff3030",
    "laser": "#ff9e00",
    "fence": "#00cfe8",
}


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_yaw(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def odometry_pose(message) -> tuple[float, float, float]:
    return (
        float(message.pose.pose.position.x),
        float(message.pose.pose.position.y),
        quaternion_yaw(message.pose.pose.orientation),
    )


def open_bag(path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = TOPICS - set(topic_types)
    if missing:
        raise ValueError(f"bag is missing required topics: {sorted(missing)}")
    message_types = {
        topic: get_message(topic_types[topic]) for topic in TOPICS
    }
    return reader, message_types


def load_bag(path: Path):
    reader, message_types = open_bag(path)
    odometry = []
    laser_odometry = []
    scans = []
    truth = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in TOPICS:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == "/odom":
            odometry.append(message)
        elif topic == "/odom/laser":
            laser_odometry.append(message)
        elif topic == "/scan_filtered":
            ranges = np.asarray(message.ranges, dtype=np.float64)
            indices = np.arange(0, len(ranges), 2)
            selected = ranges[indices]
            valid = (
                np.isfinite(selected)
                & (selected >= float(message.range_min))
                & (selected < float(message.range_max))
            )
            angles = (
                float(message.angle_min)
                + indices[valid] * float(message.angle_increment)
            )
            distance = selected[valid]
            # Recorded static TF: base_link <- lidar_link translation x=-0.027m.
            points = np.column_stack(
                (-0.027 + distance * np.cos(angles), distance * np.sin(angles))
            )
            scans.append((stamp_ns(message.header.stamp), points))
        else:
            if not message.transforms:
                continue
            transform = message.transforms[0]
            truth.append(
                (
                    stamp_ns(transform.header.stamp),
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    quaternion_yaw(transform.transform.rotation),
                )
            )
    if not odometry or not laser_odometry or not scans or not truth:
        raise ValueError("one or more required bag streams contain no messages")
    return odometry, laser_odometry, scans, np.asarray(truth, dtype=np.float64)


def nearest_index(timestamps: np.ndarray, target: int) -> int:
    index = int(np.searchsorted(timestamps, target))
    candidates = (max(0, index - 1), min(len(timestamps) - 1, index))
    return min(candidates, key=lambda item: abs(int(timestamps[item]) - target))


def transform_matrix(transform) -> np.ndarray:
    x_m, y_m, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cosine, -sine, 0.0, x_m],
            [sine, cosine, 0.0, y_m],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def metrics(values) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(array)),
        "rmse": float(np.sqrt(np.mean(array * array))),
        "median": float(np.median(array)),
        "p95": float(np.quantile(array, 0.95)),
        "max": float(np.max(array)),
    }


def analyze(path: Path, fixed_start_pose):
    odometry, laser_odometry, scans, truth = load_bag(path)
    truth_ns = truth[:, 0].astype(np.int64)
    raw_initial = odometry_pose(odometry[0])
    map_from_odom = map_from_odom_pose(fixed_start_pose, raw_initial)
    reference_odom = FenceReference.rectangle(0.0, 12.0, 0.0, 7.0).transformed(
        np.linalg.inv(transform_matrix(map_from_odom))
    )
    matcher = FenceOdomCorrector(
        maximum_match_distance_m=0.25,
        segment_endpoint_margin_m=0.10,
        minimum_matches=80,
        minimum_segments=3,
        minimum_matches_per_segment=10,
        maximum_position_correction_m=0.35,
        maximum_yaw_correction_rad=0.15,
        huber_delta_m=0.03,
    )
    ekf = PoseCorrectionEkf(
        process_position_variance_per_sec=0.0025,
        process_yaw_variance_per_sec=0.0016,
        minimum_measurement_position_variance=0.0025,
        measurement_yaw_variance=0.0076,
        maximum_output_position_rate_m_s=0.08,
        maximum_output_yaw_rate_rad_s=0.08,
    )

    scan_index = 0
    latest_scan = None
    latest_scan_stamp = None
    processed_scan_stamp = None
    previous_time = None
    accepted = 0
    attempted = 0
    match_rms = []
    match_counts = []
    rows = []
    start_ns = stamp_ns(odometry[0].header.stamp)
    for message in odometry:
        current_ns = stamp_ns(message.header.stamp)
        while scan_index < len(scans) and scans[scan_index][0] <= current_ns:
            latest_scan_stamp, latest_scan = scans[scan_index]
            scan_index += 1
        raw_pose = odometry_pose(message)
        time_sec = current_ns * 1.0e-9
        dt_sec = 0.0 if previous_time is None else max(0.0, time_sec - previous_time)
        previous_time = time_sec
        ekf.predict(
            raw_pose,
            dt_sec,
            pose_covariance=message.pose.covariance,
            twist_covariance=message.twist.covariance,
        )
        if (
            latest_scan is not None
            and latest_scan_stamp != processed_scan_stamp
            and 0 <= current_ns - latest_scan_stamp <= 150_000_000
        ):
            attempted += 1
            processed_scan_stamp = latest_scan_stamp
            correction = matcher.estimate(
                latest_scan, reference_odom, tuple(ekf.state)
            )
            if correction is not None and correction.rms_error_m <= 0.08:
                accepted += 1
                match_rms.append(correction.rms_error_m)
                match_counts.append(correction.match_count)
                ekf.correct(
                    correction.measured_pose,
                    rms_error_m=correction.rms_error_m,
                    match_count=correction.match_count,
                )
        ekf.advance_output(dt_sec)
        raw_map = transform_pose_2d(raw_pose, map_from_odom)
        corrected_map = transform_pose_2d(ekf.output_pose, map_from_odom)
        truth_index = nearest_index(truth_ns, current_ns)
        truth_pose = tuple(truth[truth_index, 1:])
        rows.append(
            {
                "time_sec": (current_ns - start_ns) * 1.0e-9,
                "raw_x": raw_map[0],
                "raw_y": raw_map[1],
                "raw_yaw": raw_map[2],
                "fence_x": corrected_map[0],
                "fence_y": corrected_map[1],
                "fence_yaw": corrected_map[2],
                "truth_x": truth_pose[0],
                "truth_y": truth_pose[1],
                "truth_yaw": truth_pose[2],
                "raw_position_error_m": math.hypot(
                    raw_map[0] - truth_pose[0], raw_map[1] - truth_pose[1]
                ),
                "fence_position_error_m": math.hypot(
                    corrected_map[0] - truth_pose[0],
                    corrected_map[1] - truth_pose[1],
                ),
                "raw_yaw_error_deg": math.degrees(
                    abs(normalize_angle(raw_map[2] - truth_pose[2]))
                ),
                "fence_yaw_error_deg": math.degrees(
                    abs(normalize_angle(corrected_map[2] - truth_pose[2]))
                ),
            }
        )

    laser_initial = odometry_pose(laser_odometry[0])
    map_from_laser = map_from_odom_pose(fixed_start_pose, laser_initial)
    laser_rows = []
    for message in laser_odometry:
        current_ns = stamp_ns(message.header.stamp)
        pose_map = transform_pose_2d(odometry_pose(message), map_from_laser)
        truth_pose = tuple(truth[nearest_index(truth_ns, current_ns), 1:])
        laser_rows.append(
            {
                "time_sec": (current_ns - start_ns) * 1.0e-9,
                "laser_x": pose_map[0],
                "laser_y": pose_map[1],
                "laser_yaw": pose_map[2],
                "truth_x": truth_pose[0],
                "truth_y": truth_pose[1],
                "truth_yaw": truth_pose[2],
                "laser_position_error_m": math.hypot(
                    pose_map[0] - truth_pose[0], pose_map[1] - truth_pose[1]
                ),
                "laser_yaw_error_deg": math.degrees(
                    abs(normalize_angle(pose_map[2] - truth_pose[2]))
                ),
            }
        )

    summary = {
        "bag": str(path),
        "fixed_start_pose_map": list(fixed_start_pose),
        "map_from_odom": list(map_from_odom),
        "map_from_laser_odom": list(map_from_laser),
        "odom_samples": len(rows),
        "laser_odom_samples": len(laser_rows),
        "truth_samples": len(truth),
        "scan_attempts": attempted,
        "scan_corrections_accepted": accepted,
        "scan_acceptance_fraction": accepted / max(1, attempted),
        "match_rms_m": metrics(match_rms),
        "match_count": metrics(match_counts),
        "raw_position_error_m": metrics(
            [row["raw_position_error_m"] for row in rows]
        ),
        "laser_position_error_m": metrics(
            [row["laser_position_error_m"] for row in laser_rows]
        ),
        "fence_position_error_m": metrics(
            [row["fence_position_error_m"] for row in rows]
        ),
        "raw_yaw_error_deg": metrics(
            [row["raw_yaw_error_deg"] for row in rows]
        ),
        "laser_yaw_error_deg": metrics(
            [row["laser_yaw_error_deg"] for row in laser_rows]
        ),
        "fence_yaw_error_deg": metrics(
            [row["fence_yaw_error_deg"] for row in rows]
        ),
    }
    return rows, laser_rows, truth, summary


def write_csv(path: Path, rows) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_results(output: Path, rows, laser_rows, truth, summary) -> None:
    raw = np.asarray([[row["raw_x"], row["raw_y"]] for row in rows])
    fence = np.asarray([[row["fence_x"], row["fence_y"]] for row in rows])
    laser = np.asarray([[row["laser_x"], row["laser_y"]] for row in laser_rows])

    figure, axis = plt.subplots(figsize=(12, 7))
    axis.plot(truth[:, 1], truth[:, 2], color=COLORS["truth"], lw=2.2, label="TRUTH")
    axis.plot(raw[:, 0], raw[:, 1], color=COLORS["odom"], lw=1.2, label="/odom")
    axis.plot(laser[:, 0], laser[:, 1], color=COLORS["laser"], lw=1.2, label="/odom/laser")
    axis.plot(fence[:, 0], fence[:, 1], color=COLORS["fence"], lw=1.8, label="fence corrected")
    axis.plot([0, 12, 12, 0, 0], [0, 0, 7, 7, 0], color="black", lw=2.0, label="fence map")
    axis.set_aspect("equal")
    axis.set_xlim(-0.4, 12.4)
    axis.set_ylim(-0.4, 7.4)
    axis.set_xlabel("sim_world/map x [m]")
    axis.set_ylabel("sim_world/map y [m]")
    axis.set_title("Two-lap trajectory comparison from a fixed start pose")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=5, loc="upper center")
    figure.tight_layout()
    figure.savefig(output / "01_trajectory_comparison.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(13, 5))
    axis.plot([row["time_sec"] for row in rows], [row["raw_position_error_m"] for row in rows], color=COLORS["odom"], lw=1.0, label="/odom")
    axis.plot([row["time_sec"] for row in laser_rows], [row["laser_position_error_m"] for row in laser_rows], color=COLORS["laser"], lw=1.0, label="/odom/laser")
    axis.plot([row["time_sec"] for row in rows], [row["fence_position_error_m"] for row in rows], color=COLORS["fence"], lw=1.2, label="fence corrected")
    axis.set_xlabel("bag elapsed time [s]")
    axis.set_ylabel("position error to truth [m]")
    axis.set_title("Position error over time")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output / "02_position_error_over_time.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(13, 5))
    axis.plot([row["time_sec"] for row in rows], [row["raw_yaw_error_deg"] for row in rows], color=COLORS["odom"], lw=1.0, label="/odom")
    axis.plot([row["time_sec"] for row in laser_rows], [row["laser_yaw_error_deg"] for row in laser_rows], color=COLORS["laser"], lw=1.0, label="/odom/laser")
    axis.plot([row["time_sec"] for row in rows], [row["fence_yaw_error_deg"] for row in rows], color=COLORS["fence"], lw=1.2, label="fence corrected")
    axis.set_xlabel("bag elapsed time [s]")
    axis.set_ylabel("absolute yaw error to truth [deg]")
    axis.set_title("Yaw error over time")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output / "03_yaw_error_over_time.png", dpi=180)
    plt.close(figure)

    plt.style.use("dark_background")
    figure, axis = plt.subplots(figsize=(14, 8))
    axis.plot([0, 12, 12, 0, 0], [0, 0, 7, 7, 0], color="#b8b8b8", lw=3.0, label="four-line fence")
    axis.plot(truth[:, 1], truth[:, 2], color=COLORS["truth"], lw=2.0, label="TRUTH")
    axis.plot(raw[:, 0], raw[:, 1], color=COLORS["odom"], lw=1.3, label="/odom")
    axis.plot(laser[:, 0], laser[:, 1], color=COLORS["laser"], lw=1.3, label="/odom/laser")
    axis.plot(fence[:, 0], fence[:, 1], color=COLORS["fence"], lw=2.2, label="FENCE CORRECTED")
    axis.set_aspect("equal")
    axis.set_xlim(-0.5, 12.5)
    axis.set_ylim(-0.5, 7.5)
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.set_title("RViz-style final retained paths after replay")
    axis.grid(True, color="#666666", alpha=0.35)
    axis.legend(ncol=5, loc="upper center")
    figure.tight_layout()
    figure.savefig(output / "04_rviz_style_final_paths.png", dpi=180, facecolor=figure.get_facecolor())
    plt.close(figure)


def write_analysis(output: Path, summary) -> None:
    position = {
        name: summary[f"{name}_position_error_m"]
        for name in ("raw", "laser", "fence")
    }
    yaw = {
        name: summary[f"{name}_yaw_error_deg"]
        for name in ("raw", "laser", "fence")
    }
    markdown = f"""# 네 직선 LiDAR 펜스 보정 성능 분석

## 조건

- 고정 시작 pose: `map (1.4 m, 3.4 m, -pi/2 rad)`
- 보정 입력: `/odom`, `/scan_filtered`, 고정 펜스 `x=0, x=12, y=0, y=7`
- 비교 전용: `/odom/laser`, `/sim/ground_truth/tf`
- simulator truth는 최초 정렬과 fence matcher 입력에 사용하지 않았다.

## 궤적

![궤적 비교](01_trajectory_comparison.png)

## 위치 오차

![위치 오차](02_position_error_over_time.png)

| 출력 | RMSE | Median | P95 | Max |
|---|---:|---:|---:|---:|
| `/odom` | {position['raw']['rmse']:.4f} m | {position['raw']['median']:.4f} m | {position['raw']['p95']:.4f} m | {position['raw']['max']:.4f} m |
| `/odom/laser` | {position['laser']['rmse']:.4f} m | {position['laser']['median']:.4f} m | {position['laser']['p95']:.4f} m | {position['laser']['max']:.4f} m |
| fence corrected | {position['fence']['rmse']:.4f} m | {position['fence']['median']:.4f} m | {position['fence']['p95']:.4f} m | {position['fence']['max']:.4f} m |

## Yaw 오차

![Yaw 오차](03_yaw_error_over_time.png)

| 출력 | RMSE | Median | P95 | Max |
|---|---:|---:|---:|---:|
| `/odom` | {yaw['raw']['rmse']:.3f} deg | {yaw['raw']['median']:.3f} deg | {yaw['raw']['p95']:.3f} deg | {yaw['raw']['max']:.3f} deg |
| `/odom/laser` | {yaw['laser']['rmse']:.3f} deg | {yaw['laser']['median']:.3f} deg | {yaw['laser']['p95']:.3f} deg | {yaw['laser']['max']:.3f} deg |
| fence corrected | {yaw['fence']['rmse']:.3f} deg | {yaw['fence']['median']:.3f} deg | {yaw['fence']['p95']:.3f} deg | {yaw['fence']['max']:.3f} deg |

## RViz 색상과 종료 화면

![RViz 형식 최종 경로](04_rviz_style_final_paths.png)

- 초록: simulator truth
- 빨강: `/odom`
- 주황: `/odom/laser`
- 청록: fence corrected `/odom/calibride`
- 회색: `12 m x 7 m` 외곽 펜스

실제 RViz 창 캡처는 `05_rviz_replay_final.png`로 별도 저장한다.

## 매칭 상태

- scan 보정 채택: {summary['scan_corrections_accepted']} / {summary['scan_attempts']} ({100.0 * summary['scan_acceptance_fraction']:.1f}%)
- point-to-line RMS median: {summary['match_rms_m']['median']:.4f} m
- point-to-line RMS P95: {summary['match_rms_m']['p95']:.4f} m
"""
    (output / "ANALYSIS.md").write_text(markdown, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    fixed_start = (1.4, 3.4, -math.pi / 2.0)
    rows, laser_rows, truth, summary = analyze(args.bag, fixed_start)
    write_csv(args.output / "pose_errors.csv", rows)
    write_csv(args.output / "laser_pose_errors.csv", laser_rows)
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    plot_results(args.output, rows, laser_rows, truth, summary)
    write_analysis(args.output, summary)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
