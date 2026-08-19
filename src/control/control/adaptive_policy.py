"""Curvature-based adaptive lookahead and speed-cap policy."""

from typing import Sequence

from .models import (
    AdaptiveControlConfig,
    AdaptiveControlResult,
    VehicleState,
)
from .path_metrics import PointInput, preview_curvature


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
        speed_limit = self.config.max_speed_limit_m_s - normalized * (
            self.config.max_speed_limit_m_s
            - self.config.min_speed_limit_m_s
        )
        return AdaptiveControlResult(
            curvature_inv_m=curvature,
            lookahead_distance_m=lookahead,
            speed_limit_m_s=speed_limit,
        )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
