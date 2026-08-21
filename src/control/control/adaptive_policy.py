"""Curvature-based adaptive lookahead and speed-cap policy."""

import math
from typing import Sequence

from .models import (
    AdaptiveControlConfig,
    AdaptiveControlResult,
    VehicleState,
)
from .path_metrics import PointInput, preview_curvature

CURVATURE_EPSILON_INV_M = 1.0e-9


class AdaptiveControlPolicy:
    """Compute deterministic adaptive commands without retaining frame state."""

    def __init__(self, config: AdaptiveControlConfig) -> None:
        self.config = config

    def compute(
        self,
        state: VehicleState,
        path: Sequence[PointInput],
        closed_loop: bool = False,
    ) -> AdaptiveControlResult:
        curvature = preview_curvature(
            state,
            path,
            self.config.preview_distance_m,
            closed_loop=closed_loop,
        )
        normalized = _clamp(
            curvature / self.config.curvature_reference_inv_m, 0.0, 1.0
        )
        lookahead = self.config.max_lookahead_m - normalized * (
            self.config.max_lookahead_m - self.config.min_lookahead_m
        )
        speed_limit = curvature_speed_limit(curvature, self.config)
        return AdaptiveControlResult(
            curvature_inv_m=curvature,
            lookahead_distance_m=lookahead,
            speed_limit_m_s=speed_limit,
        )


def curvature_speed_limit(
    curvature_inv_m: float, config: AdaptiveControlConfig
) -> float:
    """Bound speed using the configured maximum lateral acceleration."""

    if curvature_inv_m <= CURVATURE_EPSILON_INV_M:
        return config.max_speed_limit_m_s
    curve_speed = math.sqrt(
        config.max_lateral_acceleration_m_s2
        / max(curvature_inv_m, CURVATURE_EPSILON_INV_M)
    )
    return _clamp(
        curve_speed,
        config.min_speed_limit_m_s,
        config.max_speed_limit_m_s,
    )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
