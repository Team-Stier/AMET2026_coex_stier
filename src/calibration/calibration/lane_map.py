from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from calibration.lane_detector import skeletonize_mask


@dataclass(frozen=True)
class LaneReference:
    """Unordered metric lane points with a local unit tangent per point."""

    points: np.ndarray
    tangents: np.ndarray

    def __post_init__(self) -> None:
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError("lane reference points must have shape (N, 2)")
        if self.tangents.shape != self.points.shape:
            raise ValueError("lane reference tangents must match point shape")
        if len(self.points) < 3:
            raise ValueError("lane reference needs at least three points")


def polyline_reference(points: np.ndarray) -> LaneReference:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError("polyline must have shape (N, 2) with at least three points")
    tangents = np.empty_like(points)
    tangents[0] = points[1] - points[0]
    tangents[-1] = points[-1] - points[-2]
    tangents[1:-1] = points[2:] - points[:-2]
    norms = np.linalg.norm(tangents, axis=1)
    if np.any(norms <= 1.0e-9):
        raise ValueError("polyline contains points without a local direction")
    tangents /= norms[:, None]
    return LaneReference(points=points, tangents=tangents)


def estimate_point_tangents(
    points: np.ndarray,
    neighborhood_radius_m: float,
) -> np.ndarray:
    if neighborhood_radius_m <= 0.0:
        raise ValueError("tangent neighborhood radius must be positive")
    offsets = points[:, None, :] - points[None, :, :]
    distances_sq = np.sum(offsets * offsets, axis=2)
    radius_sq = neighborhood_radius_m * neighborhood_radius_m
    tangents = np.empty_like(points)
    for index in range(len(points)):
        neighbors = points[distances_sq[index] <= radius_sq]
        if len(neighbors) < 3:
            nearest = np.argsort(distances_sq[index])[: min(5, len(points))]
            neighbors = points[nearest]
        centered = neighbors - np.mean(neighbors, axis=0)
        _, _, axes = np.linalg.svd(centered, full_matrices=False)
        tangent = axes[0]
        norm = float(np.linalg.norm(tangent))
        tangents[index] = tangent / max(norm, 1.0e-12)
    return tangents


def load_lane_reference_image(
    path: str,
    *,
    resolution_m: float,
    origin_x_m: float = 0.0,
    origin_y_m: float = 0.0,
    hsv_lower: tuple[int, int, int] = (14, 135, 145),
    hsv_upper: tuple[int, int, int] = (32, 255, 255),
    point_spacing_m: float = 0.02,
    tangent_radius_m: float = 0.10,
) -> LaneReference:
    if resolution_m <= 0.0 or point_spacing_m <= 0.0:
        raise ValueError("map resolution and point spacing must be positive")
    image = cv2.imread(str(Path(path)), cv2.IMREAD_COLOR)
    if image is None:
        raise OSError(f"could not read lane reference image: {path}")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray(hsv_lower, dtype=np.uint8),
        np.asarray(hsv_upper, dtype=np.uint8),
    )
    skeleton = skeletonize_mask(mask)
    rows, columns = np.nonzero(skeleton)
    if len(rows) < 3:
        raise ValueError(f"lane reference image contains too few yellow pixels: {path}")

    height = image.shape[0]
    x_m = origin_x_m + columns.astype(np.float64) * resolution_m
    y_m = origin_y_m + (height - 1 - rows).astype(np.float64) * resolution_m
    points = np.column_stack((x_m, y_m))
    cells = np.floor(points / point_spacing_m).astype(np.int64)
    _, indices = np.unique(cells, axis=0, return_index=True)
    points = points[np.sort(indices)]
    tangents = estimate_point_tangents(points, tangent_radius_m)
    return LaneReference(points=points, tangents=tangents)


def transform_reference(reference: LaneReference, transform: np.ndarray) -> LaneReference:
    if transform.shape != (4, 4):
        raise ValueError("reference transform must be 4x4")
    homogeneous = np.column_stack(
        (reference.points, np.zeros(len(reference.points)), np.ones(len(reference.points)))
    )
    points = (transform @ homogeneous.T).T[:, :2]
    rotation = transform[:2, :2]
    tangents = reference.tangents @ rotation.T
    norms = np.linalg.norm(tangents, axis=1)
    tangents /= np.maximum(norms[:, None], 1.0e-12)
    return LaneReference(points=points, tangents=tangents)
