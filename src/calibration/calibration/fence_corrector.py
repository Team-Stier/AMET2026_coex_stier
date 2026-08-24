from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from calibration.pose_geometry import normalize_angle


@dataclass(frozen=True)
class FenceReference:
    """Finite fence segments expressed in one planar reference frame."""

    starts: np.ndarray
    ends: np.ndarray
    tangents: np.ndarray
    normals: np.ndarray
    lengths: np.ndarray

    @classmethod
    def rectangle(
        cls,
        minimum_x_m: float,
        maximum_x_m: float,
        minimum_y_m: float,
        maximum_y_m: float,
    ) -> "FenceReference":
        if maximum_x_m <= minimum_x_m or maximum_y_m <= minimum_y_m:
            raise ValueError("fence rectangle maximum bounds must exceed minimum bounds")
        starts = np.asarray(
            [
                [minimum_x_m, minimum_y_m],
                [maximum_x_m, minimum_y_m],
                [maximum_x_m, maximum_y_m],
                [minimum_x_m, maximum_y_m],
            ],
            dtype=np.float64,
        )
        ends = np.roll(starts, -1, axis=0)
        return cls.from_segments(starts, ends)

    @classmethod
    def from_segments(cls, starts: np.ndarray, ends: np.ndarray) -> "FenceReference":
        starts = np.asarray(starts, dtype=np.float64)
        ends = np.asarray(ends, dtype=np.float64)
        if starts.shape != ends.shape or starts.ndim != 2 or starts.shape[1] != 2:
            raise ValueError("fence segment endpoints must have shape (N, 2)")
        vectors = ends - starts
        lengths = np.linalg.norm(vectors, axis=1)
        if len(lengths) < 2 or np.any(lengths <= 1.0e-9):
            raise ValueError("fence reference requires at least two non-zero segments")
        tangents = vectors / lengths[:, None]
        normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
        return cls(starts, ends, tangents, normals, lengths)

    def transformed(self, transform: np.ndarray) -> "FenceReference":
        transform = np.asarray(transform, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("fence transform must be a 4x4 matrix")

        def apply(points: np.ndarray) -> np.ndarray:
            homogeneous = np.column_stack(
                (points, np.zeros(len(points)), np.ones(len(points)))
            )
            return (transform @ homogeneous.T).T[:, :2]

        return FenceReference.from_segments(apply(self.starts), apply(self.ends))


@dataclass(frozen=True)
class FenceCorrectionResult:
    measured_pose: tuple[float, float, float]
    delta_x_m: float
    delta_y_m: float
    delta_yaw_rad: float
    rms_error_m: float
    match_count: int
    segment_match_counts: tuple[int, ...]
    normal_matrix_condition: float


@dataclass(frozen=True)
class FenceMatchDiagnostics:
    observed_points: np.ndarray
    segment_indices: np.ndarray
    keep: np.ndarray
    residual_m: np.ndarray


class FenceOdomCorrector:
    """Estimate an SE(2) pose from 2D LiDAR returns against finite fence lines."""

    def __init__(
        self,
        *,
        maximum_match_distance_m: float = 0.25,
        segment_endpoint_margin_m: float = 0.10,
        minimum_matches: int = 80,
        minimum_segments: int = 3,
        minimum_matches_per_segment: int = 10,
        maximum_position_correction_m: float = 0.35,
        maximum_yaw_correction_rad: float = 0.15,
        huber_delta_m: float = 0.03,
        maximum_iterations: int = 8,
        maximum_normal_matrix_condition: float = 1.0e7,
    ) -> None:
        self.maximum_match_distance_m = float(maximum_match_distance_m)
        self.segment_endpoint_margin_m = float(segment_endpoint_margin_m)
        self.minimum_matches = int(minimum_matches)
        self.minimum_segments = int(minimum_segments)
        self.minimum_matches_per_segment = int(minimum_matches_per_segment)
        self.maximum_position_correction_m = float(maximum_position_correction_m)
        self.maximum_yaw_correction_rad = float(maximum_yaw_correction_rad)
        self.huber_delta_m = float(huber_delta_m)
        self.maximum_iterations = int(maximum_iterations)
        self.maximum_normal_matrix_condition = float(maximum_normal_matrix_condition)
        self.last_diagnostics: FenceMatchDiagnostics | None = None

    @staticmethod
    def _transform_points(points: np.ndarray, pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        cosine = math.cos(float(pose[2]))
        sine = math.sin(float(pose[2]))
        rotation = np.asarray([[cosine, -sine], [sine, cosine]])
        rotated = points @ rotation.T
        return rotated + pose[:2], rotated

    def _associate(
        self,
        observed: np.ndarray,
        reference: FenceReference,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        offsets = observed[:, None, :] - reference.starts[None, :, :]
        along = np.sum(offsets * reference.tangents[None, :, :], axis=2)
        residual = np.sum(offsets * reference.normals[None, :, :], axis=2)
        within_segment = (
            (along >= -self.segment_endpoint_margin_m)
            & (along <= reference.lengths[None, :] + self.segment_endpoint_margin_m)
        )
        distance = np.where(within_segment, np.abs(residual), np.inf)
        nearest = np.argmin(distance, axis=1)
        rows = np.arange(len(observed))
        best_distance = distance[rows, nearest]
        keep = np.isfinite(best_distance) & (
            best_distance <= self.maximum_match_distance_m
        )
        return nearest, keep, residual[rows, nearest]

    def _is_observable(self, counts: np.ndarray) -> bool:
        active = counts >= self.minimum_matches_per_segment
        return int(np.count_nonzero(active)) >= self.minimum_segments

    def estimate(
        self,
        observed_base_points: np.ndarray,
        reference: FenceReference,
        initial_pose: tuple[float, float, float],
    ) -> FenceCorrectionResult | None:
        self.last_diagnostics = None
        points = np.asarray(observed_base_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("observed fence points must have shape (N, 2)")
        points = points[np.all(np.isfinite(points), axis=1)]
        if len(points) < self.minimum_matches:
            return None

        initial = np.asarray(initial_pose, dtype=np.float64)
        pose = initial.copy()
        condition = math.inf
        for _ in range(self.maximum_iterations):
            observed, rotated = self._transform_points(points, pose)
            nearest, keep, all_residual = self._associate(observed, reference)
            counts = np.bincount(
                nearest[keep], minlength=len(reference.starts)
            )
            if int(np.count_nonzero(keep)) < self.minimum_matches or not self._is_observable(counts):
                self.last_diagnostics = FenceMatchDiagnostics(
                    observed, nearest, keep, all_residual[keep]
                )
                return None

            segment_indices = nearest[keep]
            normals = reference.normals[segment_indices]
            residual = all_residual[keep]
            rotated_derivative = np.column_stack(
                (-rotated[keep, 1], rotated[keep, 0])
            )
            jacobian = np.column_stack(
                (
                    normals[:, 0],
                    normals[:, 1],
                    np.sum(normals * rotated_derivative, axis=1),
                )
            )

            robust_weights = np.minimum(
                1.0,
                self.huber_delta_m / np.maximum(np.abs(residual), 1.0e-9),
            )
            active_count = max(1, int(np.count_nonzero(counts)))
            balance = np.asarray(
                [len(residual) / (active_count * counts[index]) for index in segment_indices]
            )
            weights = robust_weights * balance
            normal_matrix = jacobian.T @ (jacobian * weights[:, None])
            condition = float(np.linalg.cond(normal_matrix))
            if not np.isfinite(condition) or condition > self.maximum_normal_matrix_condition:
                return None
            try:
                increment = -np.linalg.solve(
                    normal_matrix + np.diag([1.0e-6, 1.0e-6, 1.0e-5]),
                    jacobian.T @ (weights * residual),
                )
            except np.linalg.LinAlgError:
                return None

            candidate = pose + increment
            candidate[2] = normalize_angle(float(candidate[2]))
            total_position = float(np.linalg.norm(candidate[:2] - initial[:2]))
            total_yaw = normalize_angle(float(candidate[2] - initial[2]))
            if (
                total_position > self.maximum_position_correction_m
                or abs(total_yaw) > self.maximum_yaw_correction_rad
            ):
                return None
            pose = candidate
            if float(np.linalg.norm(increment)) < 1.0e-6:
                break

        observed, _ = self._transform_points(points, pose)
        nearest, keep, all_residual = self._associate(observed, reference)
        counts = np.bincount(nearest[keep], minlength=len(reference.starts))
        if int(np.count_nonzero(keep)) < self.minimum_matches or not self._is_observable(counts):
            return None
        residual = all_residual[keep]
        self.last_diagnostics = FenceMatchDiagnostics(
            observed, nearest, keep, residual.copy()
        )
        delta = pose - initial
        delta[2] = normalize_angle(float(delta[2]))
        return FenceCorrectionResult(
            measured_pose=(float(pose[0]), float(pose[1]), float(pose[2])),
            delta_x_m=float(delta[0]),
            delta_y_m=float(delta[1]),
            delta_yaw_rad=float(delta[2]),
            rms_error_m=float(np.sqrt(np.mean(residual * residual))),
            match_count=int(np.count_nonzero(keep)),
            segment_match_counts=tuple(int(value) for value in counts),
            normal_matrix_condition=condition,
        )
