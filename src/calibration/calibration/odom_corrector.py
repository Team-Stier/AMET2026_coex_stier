from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from calibration.lane_map import LaneReference, polyline_reference


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


def fit_local_line_directions(
    points: np.ndarray,
    neighborhood_radius_m: float,
    minimum_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Fit an independent first-order line around every observed lane point.

    The returned direction sign is arbitrary. ``valid`` identifies points whose
    local neighborhood contained enough samples for a meaningful fit.
    """
    points = np.asarray(points, dtype=np.float64)
    directions = np.zeros_like(points)
    valid = np.zeros(len(points), dtype=bool)
    if len(points) == 0:
        return directions, valid

    offsets = points[:, None, :] - points[None, :, :]
    distances_sq = np.sum(offsets * offsets, axis=2)
    radius_sq = neighborhood_radius_m * neighborhood_radius_m
    for index in range(len(points)):
        neighbors = points[distances_sq[index] <= radius_sq]
        if len(neighbors) < minimum_points:
            continue
        centered = neighbors - np.mean(neighbors, axis=0)
        _, singular_values, axes = np.linalg.svd(centered, full_matrices=False)
        if singular_values[0] <= 1.0e-9:
            continue
        directions[index] = axes[0]
        valid[index] = True
    return directions, valid


class LaneOdomCorrector:
    """Estimate bounded lateral/yaw corrections with geometric map matching."""

    def __init__(
        self,
        maximum_match_distance_m: float = 0.35,
        minimum_matches: int = 8,
        maximum_lateral_correction_m: float = 0.20,
        maximum_yaw_correction_rad: float = 0.12,
        smoothing_alpha: float = 0.25,
        local_fit_radius_m: float = 0.20,
        minimum_local_fit_points: int = 3,
        maximum_tangent_angle_difference_rad: float = 0.44,
    ) -> None:
        self.maximum_match_distance_m = maximum_match_distance_m
        self.minimum_matches = minimum_matches
        self.maximum_lateral_correction_m = maximum_lateral_correction_m
        self.maximum_yaw_correction_rad = maximum_yaw_correction_rad
        self.smoothing_alpha = smoothing_alpha
        self.local_fit_radius_m = local_fit_radius_m
        self.minimum_local_fit_points = minimum_local_fit_points
        self.maximum_tangent_angle_difference_rad = (
            maximum_tangent_angle_difference_rad
        )
        self._filtered_lateral = 0.0
        self._filtered_yaw = 0.0

    def reset(self) -> None:
        self._filtered_lateral = 0.0
        self._filtered_yaw = 0.0

    def estimate(
        self,
        observed_base_points: np.ndarray,
        reference_odom: LaneReference | np.ndarray,
        odom_x: float,
        odom_y: float,
        odom_yaw: float,
    ) -> CorrectionResult | None:
        if isinstance(reference_odom, np.ndarray):
            reference = polyline_reference(reference_odom)
        else:
            reference = reference_odom
        if len(observed_base_points) < self.minimum_matches or len(reference.points) < 3:
            return None

        local_directions, valid_local_fit = fit_local_line_directions(
            observed_base_points,
            self.local_fit_radius_m,
            self.minimum_local_fit_points,
        )
        if int(np.count_nonzero(valid_local_fit)) < self.minimum_matches:
            return None

        cosine, sine = np.cos(odom_yaw), np.sin(odom_yaw)
        vehicle_lateral = np.array([-sine, cosine])
        translation = np.array([odom_x, odom_y])
        lateral = 0.0
        yaw = 0.0
        residual = np.empty(0, dtype=np.float64)
        match_count = 0
        for _ in range(4):
            corrected_yaw = odom_yaw + yaw
            corrected_cosine = np.cos(corrected_yaw)
            corrected_sine = np.sin(corrected_yaw)
            rotation = np.array(
                [
                    [corrected_cosine, -corrected_sine],
                    [corrected_sine, corrected_cosine],
                ]
            )
            relative = observed_base_points @ rotation.T
            observed = relative + translation + vehicle_lateral * lateral

            offsets = observed[:, None, :] - reference.points[None, :, :]
            distances_sq = np.sum(offsets * offsets, axis=2)
            nearest = np.argmin(distances_sq, axis=1)
            rows = np.arange(len(observed))
            distances = np.sqrt(distances_sq[rows, nearest])
            observed_directions = local_directions @ rotation.T
            matched_tangents = reference.tangents[nearest]
            # A line has no forward/backward sign, hence the absolute dot product.
            tangent_alignment = np.abs(
                np.sum(observed_directions * matched_tangents, axis=1)
            )
            minimum_alignment = np.cos(self.maximum_tangent_angle_difference_rad)
            keep = (
                (distances <= self.maximum_match_distance_m)
                & valid_local_fit
                & (tangent_alignment >= minimum_alignment)
            )
            match_count = int(np.count_nonzero(keep))
            if match_count < self.minimum_matches:
                return None

            observed_kept = observed[keep]
            matched = reference.points[nearest[keep]]
            tangents = reference.tangents[nearest[keep]]
            normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
            residual = np.sum(normals * (observed_kept - matched), axis=1)
            rotated_relative = np.column_stack((-relative[keep, 1], relative[keep, 0]))
            jacobian = np.column_stack(
                (
                    normals @ vehicle_lateral,
                    np.sum(normals * rotated_relative, axis=1),
                )
            )

            huber_limit = max(0.02, min(0.10, 2.0 * np.median(np.abs(residual))))
            weights = np.minimum(1.0, huber_limit / np.maximum(np.abs(residual), 1.0e-9))
            weighted_jacobian = jacobian * weights[:, None]
            regularization = np.diag([1e-3, 5e-3])
            try:
                increment = -np.linalg.solve(
                    jacobian.T @ weighted_jacobian + regularization,
                    weighted_jacobian.T @ residual,
                )
            except np.linalg.LinAlgError:
                return None
            lateral = float(
                np.clip(
                    lateral + increment[0],
                    -self.maximum_lateral_correction_m,
                    self.maximum_lateral_correction_m,
                )
            )
            yaw = float(
                np.clip(
                    yaw + increment[1],
                    -self.maximum_yaw_correction_rad,
                    self.maximum_yaw_correction_rad,
                )
            )
            if float(np.linalg.norm(increment)) < 1.0e-5:
                break

        alpha = self.smoothing_alpha
        self._filtered_lateral = (1.0 - alpha) * self._filtered_lateral + alpha * lateral
        self._filtered_yaw = (1.0 - alpha) * self._filtered_yaw + alpha * yaw
        return CorrectionResult(
            lateral_m=self._filtered_lateral,
            yaw_rad=self._filtered_yaw,
            rms_error_m=float(np.sqrt(np.mean(residual * residual))),
            match_count=match_count,
        )
