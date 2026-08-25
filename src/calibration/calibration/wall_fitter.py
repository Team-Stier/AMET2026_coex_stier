from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
import math

import numpy as np


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def lidar_pose_from_ego(
    ego_pose: tuple[float, float, float], lidar_offset_x_m: float
) -> tuple[float, float, float]:
    x, y, yaw = ego_pose
    return (
        x + math.cos(yaw) * lidar_offset_x_m,
        y + math.sin(yaw) * lidar_offset_x_m,
        yaw,
    )


def ego_pose_from_lidar(
    lidar_pose: tuple[float, float, float], lidar_offset_x_m: float
) -> tuple[float, float, float]:
    """Convert a LiDAR-origin pose using the base-to-LiDAR longitudinal offset."""
    x, y, yaw = lidar_pose
    return (
        x - math.cos(yaw) * lidar_offset_x_m,
        y - math.sin(yaw) * lidar_offset_x_m,
        yaw,
    )


@dataclass(frozen=True)
class FitResult:
    pose: tuple[float, float, float]
    rms_error_m: float
    match_count: int
    wall_match_counts: tuple[int, int, int, int]


class RectangleWallFitter:
    """Fit LiDAR points to four finite, axis-aligned map walls."""

    def __init__(
        self,
        bounds: tuple[float, float, float, float],
        *,
        maximum_match_distance_m: float = 0.30,
        minimum_matches: int = 50,
        minimum_walls: int = 3,
        minimum_matches_per_wall: int = 8,
        maximum_position_step_m: float = 0.45,
        maximum_yaw_step_rad: float = 0.35,
        huber_delta_m: float = 0.04,
        maximum_iterations: int = 6,
    ) -> None:
        minimum_x, maximum_x, minimum_y, maximum_y = map(float, bounds)
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError("wall bounds must be finite")
        if maximum_x <= minimum_x or maximum_y <= minimum_y:
            raise ValueError("wall maximum bounds must exceed minimum bounds")
        if maximum_match_distance_m <= 0.0 or huber_delta_m <= 0.0:
            raise ValueError("wall distance thresholds must be positive")
        if minimum_matches < 3 or minimum_matches_per_wall < 1:
            raise ValueError("wall match thresholds must be positive")
        if minimum_walls < 2 or minimum_walls > 4:
            raise ValueError("minimum_walls must be between 2 and 4")
        if maximum_position_step_m <= 0.0 or maximum_yaw_step_rad <= 0.0:
            raise ValueError("pose step limits must be positive")
        if maximum_iterations < 1:
            raise ValueError("maximum_iterations must be positive")

        self.maximum_match_distance_m = float(maximum_match_distance_m)
        self.minimum_matches = int(minimum_matches)
        self.minimum_walls = int(minimum_walls)
        self.minimum_matches_per_wall = int(minimum_matches_per_wall)
        self.maximum_position_step_m = float(maximum_position_step_m)
        self.maximum_yaw_step_rad = float(maximum_yaw_step_rad)
        self.huber_delta_m = float(huber_delta_m)
        self.maximum_iterations = int(maximum_iterations)

        self._starts = np.asarray(
            [
                [minimum_x, minimum_y],
                [maximum_x, minimum_y],
                [maximum_x, maximum_y],
                [minimum_x, maximum_y],
            ],
            dtype=np.float64,
        )
        vectors = np.roll(self._starts, -1, axis=0) - self._starts
        self._lengths = np.linalg.norm(vectors, axis=1)
        self._tangents = vectors / self._lengths[:, None]
        self._normals = np.column_stack(
            (-self._tangents[:, 1], self._tangents[:, 0])
        )

    @staticmethod
    def _transform(
        points: np.ndarray, pose: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        cosine, sine = math.cos(float(pose[2])), math.sin(float(pose[2]))
        rotation = np.asarray(((cosine, -sine), (sine, cosine)))
        rotated = points @ rotation.T
        return rotated + pose[:2], rotated

    def _associate(
        self, observed: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        offsets = observed[:, None, :] - self._starts[None, :, :]
        along = np.sum(offsets * self._tangents[None, :, :], axis=2)
        residuals = np.sum(offsets * self._normals[None, :, :], axis=2)
        within_wall = (along >= -0.10) & (
            along <= self._lengths[None, :] + 0.10
        )
        distances = np.where(within_wall, np.abs(residuals), np.inf)
        nearest = np.argmin(distances, axis=1)
        rows = np.arange(len(observed))
        keep = distances[rows, nearest] <= self.maximum_match_distance_m
        return nearest, keep, residuals[rows, nearest]

    def _enough_walls(self, counts: np.ndarray) -> bool:
        qualified = counts >= self.minimum_matches_per_wall
        if int(np.count_nonzero(qualified)) < self.minimum_walls:
            return False
        if self.minimum_walls != 2:
            return True

        # Opposite rectangle walls are parallel and cannot constrain motion
        # along the walls reliably. A two-wall fit is observable only when at
        # least one horizontal and one vertical wall are both represented.
        horizontal_visible = bool(qualified[0] or qualified[2])
        vertical_visible = bool(qualified[1] or qualified[3])
        return horizontal_visible and vertical_visible

    def fit(
        self,
        lidar_points: np.ndarray,
        initial_pose: tuple[float, float, float],
    ) -> FitResult | None:
        points = np.asarray(lidar_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("lidar_points must have shape (N, 2)")
        points = points[np.all(np.isfinite(points), axis=1)]
        if len(points) < self.minimum_matches:
            return None

        initial = np.asarray(initial_pose, dtype=np.float64)
        if initial.shape != (3,) or not np.all(np.isfinite(initial)):
            raise ValueError("initial_pose must contain three finite values")
        pose = initial.copy()

        for _ in range(self.maximum_iterations):
            observed, rotated = self._transform(points, pose)
            nearest, keep, all_residuals = self._associate(observed)
            counts = np.bincount(nearest[keep], minlength=4)
            match_count = int(np.count_nonzero(keep))
            if match_count < self.minimum_matches or not self._enough_walls(counts):
                return None

            wall_indices = nearest[keep]
            normals = self._normals[wall_indices]
            residuals = all_residuals[keep]
            rotated_derivative = np.column_stack(
                (-rotated[keep, 1], rotated[keep, 0])
            )
            jacobian = np.column_stack(
                (
                    normals,
                    np.sum(normals * rotated_derivative, axis=1),
                )
            )

            robust_weights = np.minimum(
                1.0,
                self.huber_delta_m / np.maximum(np.abs(residuals), 1.0e-9),
            )
            active_walls = max(1, int(np.count_nonzero(counts)))
            wall_weights = match_count / (active_walls * counts[wall_indices])
            weights = robust_weights * wall_weights
            normal_matrix = jacobian.T @ (jacobian * weights[:, None])
            if not np.all(np.isfinite(normal_matrix)):
                return None
            try:
                increment = -np.linalg.solve(
                    normal_matrix + np.diag((1.0e-6, 1.0e-6, 1.0e-5)),
                    jacobian.T @ (weights * residuals),
                )
            except np.linalg.LinAlgError:
                return None

            candidate = pose + increment
            candidate[2] = normalize_angle(float(candidate[2]))
            if (
                np.linalg.norm(candidate[:2] - initial[:2])
                > self.maximum_position_step_m
                or abs(normalize_angle(float(candidate[2] - initial[2])))
                > self.maximum_yaw_step_rad
            ):
                return None
            pose = candidate
            if float(np.linalg.norm(increment)) < 1.0e-6:
                break

        observed, _ = self._transform(points, pose)
        nearest, keep, all_residuals = self._associate(observed)
        counts = np.bincount(nearest[keep], minlength=4)
        match_count = int(np.count_nonzero(keep))
        if match_count < self.minimum_matches or not self._enough_walls(counts):
            return None
        residuals = all_residuals[keep]
        return FitResult(
            pose=(float(pose[0]), float(pose[1]), float(pose[2])),
            rms_error_m=float(np.sqrt(np.mean(residuals * residuals))),
            match_count=match_count,
            wall_match_counts=tuple(int(value) for value in counts),
        )

    def fit_first(
        self,
        lidar_points: np.ndarray,
        initial_poses: Iterable[tuple[float, float, float]],
        maximum_rms_error_m: float,
    ) -> FitResult | None:
        """Return the first acceptable fit, preserving seed priority."""
        for initial_pose in initial_poses:
            result = self.fit(lidar_points, initial_pose)
            if result is not None and result.rms_error_m <= maximum_rms_error_m:
                return result
        return None
