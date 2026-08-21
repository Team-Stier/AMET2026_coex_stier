from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from calibration.bev import BevGeometry


@dataclass(frozen=True)
class LaneDetection:
    """Observed yellow lane geometry in ``base_footprint`` coordinates."""

    points_m: np.ndarray
    confidence: float
    point_count: int
    span_m: float
    mask: np.ndarray
    skeleton: np.ndarray


def skeletonize_mask(mask: np.ndarray) -> np.ndarray:
    """Return a one-pixel morphological skeleton without contrib modules."""

    if mask.ndim != 2:
        raise ValueError("skeleton input must be a single-channel mask")
    remaining = np.where(mask > 0, 255, 0).astype(np.uint8)
    skeleton = np.zeros_like(remaining)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while cv2.countNonZero(remaining):
        opened = cv2.morphologyEx(remaining, cv2.MORPH_OPEN, kernel)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(remaining, opened))
        remaining = cv2.erode(remaining, kernel)
    return skeleton


def principal_span(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    centered = points - np.mean(points, axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    projection = centered @ axes[0]
    return float(np.ptp(projection))


class YellowLaneDetector:
    """Extract map-matchable yellow lane points without a polynomial model."""

    def __init__(
        self,
        geometry: BevGeometry,
        hsv_lower: tuple[int, int, int] = (15, 30, 30),
        hsv_upper: tuple[int, int, int] = (31, 220, 220),
        minimum_points: int = 12,
        minimum_span_m: float = 0.3,
        minimum_component_area_px: int = 4,
        point_spacing_m: float = 0.04,
    ):
        self.geometry = geometry
        self.hsv_lower = np.asarray(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.asarray(hsv_upper, dtype=np.uint8)
        self.minimum_points = minimum_points
        self.minimum_span_m = minimum_span_m
        self.minimum_component_area_px = minimum_component_area_px
        self.point_spacing_m = point_spacing_m

    def create_mask(self, bev_image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # The projected dashed line is often only one or two pixels wide. Opening
        # would erase it; component geometry below rejects compact color noise.
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _linear_components(self, mask: np.ndarray) -> tuple[np.ndarray, int]:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        result = np.zeros_like(mask)
        retained_pixels = 0
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.minimum_component_area_px:
                continue
            result[labels == label] = 255
            retained_pixels += area
        return result, retained_pixels

    def _downsample(self, points_m: np.ndarray) -> np.ndarray:
        if self.point_spacing_m <= 0.0 or not len(points_m):
            return points_m
        cells = np.floor(points_m / self.point_spacing_m).astype(np.int64)
        _, indices = np.unique(cells, axis=0, return_index=True)
        return points_m[np.sort(indices)]

    def detect(self, bev_image: np.ndarray) -> LaneDetection | None:
        raw_mask = self.create_mask(bev_image)
        raw_count = cv2.countNonZero(raw_mask)
        if raw_count < self.minimum_points:
            return None

        mask, retained_pixels = self._linear_components(raw_mask)
        skeleton = skeletonize_mask(mask)
        rows, columns = np.nonzero(skeleton)
        if rows.size < self.minimum_points:
            return None

        x_m, y_m = self.geometry.pixel_to_ground(rows, columns)
        points_m = np.column_stack((x_m, y_m))
        points_m = points_m[np.isfinite(points_m).all(axis=1)]
        points_m = self._downsample(points_m)
        if len(points_m) < self.minimum_points:
            return None

        span_m = principal_span(points_m)
        if span_m < self.minimum_span_m:
            return None
        span_score = min(1.0, span_m / self.minimum_span_m)
        count_score = min(1.0, len(points_m) / max(40.0, self.minimum_points))
        retained_ratio = retained_pixels / max(1, raw_count)
        confidence = float(span_score * count_score * retained_ratio)

        return LaneDetection(
            points_m=points_m,
            confidence=confidence,
            point_count=len(points_m),
            span_m=span_m,
            mask=mask,
            skeleton=skeleton,
        )

    def draw_overlay(
        self, bev_image: np.ndarray, detection: LaneDetection | None
    ) -> np.ndarray:
        overlay = bev_image.copy()
        if detection is None:
            return overlay
        overlay[detection.skeleton > 0] = (255, 0, 0)
        return overlay
