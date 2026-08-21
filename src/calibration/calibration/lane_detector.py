from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from calibration.bev import BevGeometry


@dataclass(frozen=True)
class LaneDetection:
    coefficients: np.ndarray
    confidence: float
    inlier_count: int
    point_count: int
    x_min_m: float
    x_max_m: float
    mask: np.ndarray

    def evaluate(self, x_m: np.ndarray) -> np.ndarray:
        return np.polyval(self.coefficients, x_m)


class YellowLaneDetector:
    def __init__(
        self,
        geometry: BevGeometry,
        hsv_lower: tuple[int, int, int] = (8, 80, 80),
        hsv_upper: tuple[int, int, int] = (40, 255, 255),
        minimum_points: int = 30,
        minimum_span_m: float = 0.4,
        maximum_residual_m: float = 0.08,
    ):
        self.geometry = geometry
        self.hsv_lower = np.asarray(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.asarray(hsv_upper, dtype=np.uint8)
        self.minimum_points = minimum_points
        self.minimum_span_m = minimum_span_m
        self.maximum_residual_m = maximum_residual_m

    def create_mask(self, bev_image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3))
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, vertical_kernel)
        return cv2.dilate(mask, horizontal_kernel)

    def detect(self, bev_image: np.ndarray) -> LaneDetection | None:
        mask = self.create_mask(bev_image)
        rows, columns = np.nonzero(mask)
        if rows.size < self.minimum_points:
            return None

        x_m, y_m = self.geometry.pixel_to_ground(rows, columns)
        finite = np.isfinite(x_m) & np.isfinite(y_m)
        x_m, y_m = x_m[finite], y_m[finite]
        if x_m.size < self.minimum_points:
            return None

        keep = np.ones(x_m.size, dtype=bool)
        coefficients = None
        for _ in range(4):
            if np.count_nonzero(keep) < self.minimum_points:
                return None
            if np.ptp(x_m[keep]) < self.minimum_span_m:
                return None
            coefficients = np.polyfit(x_m[keep], y_m[keep], deg=2)
            residuals = np.abs(y_m - np.polyval(coefficients, x_m))
            median = np.median(residuals[keep])
            mad = np.median(np.abs(residuals[keep] - median))
            threshold = min(
                self.maximum_residual_m,
                max(0.02, median + 3.0 * 1.4826 * mad),
            )
            next_keep = residuals <= threshold
            if np.array_equal(next_keep, keep):
                break
            keep = next_keep

        if coefficients is None or np.count_nonzero(keep) < self.minimum_points:
            return None

        x_min_m = float(np.min(x_m[keep]))
        x_max_m = float(np.max(x_m[keep]))
        span_score = min(1.0, (x_max_m - x_min_m) / self.minimum_span_m)
        count_score = min(1.0, np.count_nonzero(keep) / 200.0)
        inlier_ratio = np.count_nonzero(keep) / x_m.size
        confidence = float(span_score * count_score * inlier_ratio)

        return LaneDetection(
            coefficients=coefficients,
            confidence=confidence,
            inlier_count=int(np.count_nonzero(keep)),
            point_count=int(x_m.size),
            x_min_m=x_min_m,
            x_max_m=x_max_m,
            mask=mask,
        )

    def draw_overlay(
        self, bev_image: np.ndarray, detection: LaneDetection | None
    ) -> np.ndarray:
        overlay = bev_image.copy()
        if detection is None:
            return overlay
        x_values = np.linspace(detection.x_min_m, detection.x_max_m, 80)
        y_values = detection.evaluate(x_values)
        rows, columns = self.geometry.ground_to_pixel(x_values, y_values)
        valid = (
            (rows >= 0)
            & (rows < self.geometry.height)
            & (columns >= 0)
            & (columns < self.geometry.width)
        )
        points = np.column_stack((columns[valid], rows[valid])).round().astype(np.int32)
        if points.shape[0] >= 2:
            cv2.polylines(overlay, [points], False, (255, 0, 0), 2, cv2.LINE_AA)
        return overlay
