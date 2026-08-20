from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class CorrectionResult:
    lateral_m: float
    yaw_rad: float
    rms_error_m: float
    match_count: int


def load_centerline_csv(path: str) -> np.ndarray:
    """Load an ordered centerline stored as two CSV columns: x,y."""
    points = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                points.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                # A single x,y header is allowed.
                if points:
                    raise ValueError(f"invalid centerline row: {row}")
    if len(points) < 3:
        raise ValueError("centerline CSV must contain at least three x,y points")
    return np.asarray(points, dtype=np.float64)


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((points, np.zeros(len(points)), np.ones(len(points))))
    return (transform @ homogeneous.T).T[:, :2]


class LaneOdomCorrector:
    """Estimate bounded lateral/yaw corrections with point-to-line matching."""

    def __init__(
        self,
        maximum_match_distance_m: float = 0.35,
        minimum_matches: int = 8,
        maximum_lateral_correction_m: float = 0.20,
        maximum_yaw_correction_rad: float = 0.12,
        smoothing_alpha: float = 0.25,
    ) -> None:
        self.maximum_match_distance_m = maximum_match_distance_m
        self.minimum_matches = minimum_matches
        self.maximum_lateral_correction_m = maximum_lateral_correction_m
        self.maximum_yaw_correction_rad = maximum_yaw_correction_rad
        self.smoothing_alpha = smoothing_alpha
        self._filtered_lateral = 0.0
        self._filtered_yaw = 0.0

    def estimate(
        self,
        observed_base_points: np.ndarray,
        reference_odom_points: np.ndarray,
        odom_x: float,
        odom_y: float,
        odom_yaw: float,
    ) -> CorrectionResult | None:
        if len(observed_base_points) < self.minimum_matches or len(reference_odom_points) < 3:
            return None

        cosine, sine = np.cos(odom_yaw), np.sin(odom_yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        observed = observed_base_points @ rotation.T + np.array([odom_x, odom_y])

        segment_start = reference_odom_points[:-1]
        segment_vector = reference_odom_points[1:] - segment_start
        segment_length_sq = np.sum(segment_vector * segment_vector, axis=1)
        valid_segments = segment_length_sq > 1e-8
        segment_start = segment_start[valid_segments]
        segment_vector = segment_vector[valid_segments]
        segment_length_sq = segment_length_sq[valid_segments]
        if not len(segment_start):
            return None

        offsets = observed[:, None, :] - segment_start[None, :, :]
        fractions = np.sum(offsets * segment_vector[None, :, :], axis=2)
        fractions /= segment_length_sq[None, :]
        fractions = np.clip(fractions, 0.0, 1.0)
        projections = segment_start[None, :, :] + fractions[:, :, None] * segment_vector[None, :, :]
        distances_sq = np.sum((observed[:, None, :] - projections) ** 2, axis=2)
        nearest = np.argmin(distances_sq, axis=1)
        rows = np.arange(len(observed))
        matched = projections[rows, nearest]
        distances = np.sqrt(distances_sq[rows, nearest])
        keep = distances <= self.maximum_match_distance_m
        if np.count_nonzero(keep) < self.minimum_matches:
            return None

        observed = observed[keep]
        matched = matched[keep]
        tangents = segment_vector[nearest[keep]]
        tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)
        normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
        residual = np.sum(normals * (observed - matched), axis=1)

        vehicle_lateral = np.array([-sine, cosine])
        relative = observed - np.array([odom_x, odom_y])
        rotated_relative = np.column_stack((-relative[:, 1], relative[:, 0]))
        jacobian = np.column_stack(
            (normals @ vehicle_lateral, np.sum(normals * rotated_relative, axis=1))
        )
        regularization = np.diag([1e-3, 5e-3])
        solution = -np.linalg.solve(
            jacobian.T @ jacobian + regularization,
            jacobian.T @ residual,
        )
        lateral = float(np.clip(solution[0], -self.maximum_lateral_correction_m, self.maximum_lateral_correction_m))
        yaw = float(np.clip(solution[1], -self.maximum_yaw_correction_rad, self.maximum_yaw_correction_rad))
        alpha = self.smoothing_alpha
        self._filtered_lateral = (1.0 - alpha) * self._filtered_lateral + alpha * lateral
        self._filtered_yaw = (1.0 - alpha) * self._filtered_yaw + alpha * yaw
        return CorrectionResult(
            lateral_m=self._filtered_lateral,
            yaw_rad=self._filtered_yaw,
            rms_error_m=float(np.sqrt(np.mean(residual * residual))),
            match_count=int(np.count_nonzero(keep)),
        )
