"""ROS-independent Pure Pursuit lateral controller."""

import math
from typing import List, Optional, Sequence, Tuple, Union

from .geometry import as_path_point, distance_xy, normalize_angle
from .models import PathPoint, PurePursuitConfig, PurePursuitResult, VehicleState

PointInput = Union[PathPoint, Tuple[float, float]]
DUPLICATE_ENDPOINT_TOLERANCE_M = 1.0e-9


class PurePursuit:
    """Compute steering with a fresh global nearest-point search every update."""

    def __init__(self, config: PurePursuitConfig) -> None:
        self.config = config

    @staticmethod
    def _coerce_path(path: Sequence[PointInput]) -> List[PathPoint]:
        if not path:
            raise ValueError("path must contain at least one point")
        points = []  # type: List[PathPoint]
        for value in path:
            if isinstance(value, PathPoint):
                points.append(value)
            else:
                points.append(as_path_point(value))
        return points

    def compute(
        self,
        state: VehicleState,
        path: Sequence[PointInput],
        lookahead_distance_override_m: Optional[float] = None,
    ) -> PurePursuitResult:
        """Return a steering command and target-selection debug information.

        The nearest point is searched over the complete path on every call. From
        that point, the first forward waypoint at least one lookahead distance
        from the vehicle is selected. The final waypoint is a safe fallback.
        """

        lookahead_distance = (
            self.config.lookahead_distance_m
            if lookahead_distance_override_m is None
            else lookahead_distance_override_m
        )
        if lookahead_distance <= 0.0:
            raise ValueError("lookahead distance override must be positive")

        points = self._coerce_path(path)
        reference_x = state.x + (
            self.config.reference_point_offset_m * math.cos(state.yaw)
        )
        reference_y = state.y + (
            self.config.reference_point_offset_m * math.sin(state.yaw)
        )
        distances = [
            distance_xy(reference_x, reference_y, p.x, p.y)
            for p in points
        ]

        if self.config.closed_loop:
            logical_count = len(points)
            if (
                logical_count > 1
                and distance_xy(
                    points[0].x,
                    points[0].y,
                    points[-1].x,
                    points[-1].y,
                ) <= DUPLICATE_ENDPOINT_TOLERANCE_M
            ):
                logical_count -= 1

            nearest_index = min(
                range(logical_count), key=distances.__getitem__
            )
            target_index = nearest_index
            for offset in range(logical_count):
                index = (nearest_index + offset) % logical_count
                if distances[index] >= lookahead_distance:
                    target_index = index
                    break
        else:
            nearest_index = min(range(len(points)), key=distances.__getitem__)
            target_index = len(points) - 1
            for index in range(nearest_index, len(points)):
                if distances[index] >= lookahead_distance:
                    target_index = index
                    break

        target = points[target_index]
        target_distance = distances[target_index]
        target_heading = math.atan2(
            target.y - reference_y, target.x - reference_x
        )
        alpha = normalize_angle(target_heading - state.yaw)

        # Use the actual target distance near the end of a finite path. A tiny
        # floor prevents division by zero when the vehicle reaches the endpoint.
        effective_distance = max(target_distance, 1.0e-9)
        curvature_term = (
            2.0 * self.config.wheelbase_m * math.sin(alpha) / effective_distance
        )
        steering = math.atan(curvature_term)
        steering = max(
            -self.config.max_steering_rad,
            min(self.config.max_steering_rad, steering),
        )

        return PurePursuitResult(
            steering_rad=steering,
            target_point=target,
            target_index=target_index,
            nearest_point=points[nearest_index],
            nearest_index=nearest_index,
            alpha_rad=alpha,
            target_distance_m=target_distance,
            lookahead_distance_m=lookahead_distance,
        )
