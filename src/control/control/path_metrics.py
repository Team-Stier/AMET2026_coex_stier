"""Stateless path geometry metrics for adaptive vehicle control."""

import math
from bisect import bisect_right
from typing import List, Sequence, Tuple, Union

from .geometry import as_path_point, distance_xy
from .models import PathPoint, VehicleState

PointInput = Union[PathPoint, Tuple[float, float]]
DUPLICATE_ENDPOINT_TOLERANCE_M = 1.0e-9
MIN_SEGMENT_LENGTH_M = 1.0e-9
MIN_TWICE_TRIANGLE_AREA_M2 = 1.0e-12
ROBUST_HIGH_CURVATURE_COUNT = 3
WIDE_CURVATURE_HALF_SPAN_M = 0.40
PERSISTENT_TURN_SUPPORT_M = 0.75
MAX_STRAIGHT_GAP_M = 0.50
CONSISTENT_TURN_RATIO = 0.70


def coerce_path(path: Sequence[PointInput]) -> List[PathPoint]:
    """Return path inputs as points, rejecting an empty path."""

    if not path:
        raise ValueError("path must contain at least one point")
    return [
        value if isinstance(value, PathPoint) else as_path_point(value)
        for value in path
    ]


def logical_path_count(points: Sequence[PathPoint], closed_loop: bool) -> int:
    """Exclude a duplicate closing endpoint from a closed path."""

    count = len(points)
    if (
        closed_loop
        and count > 1
        and distance_xy(
            points[0].x, points[0].y, points[-1].x, points[-1].y
        ) <= DUPLICATE_ENDPOINT_TOLERANCE_M
    ):
        count -= 1
    return count


def nearest_waypoint_index(
    state: VehicleState, points: Sequence[PathPoint], logical_count: int
) -> int:
    """Find the nearest logical waypoint with a fresh full search."""

    if logical_count <= 0 or logical_count > len(points):
        raise ValueError("logical_count must select at least one path point")
    return min(
        range(logical_count),
        key=lambda index: distance_xy(
            state.x, state.y, points[index].x, points[index].y
        ),
    )


def discrete_curvature(
    first: PathPoint, middle: PathPoint, last: PathPoint
) -> float:
    """Return unsigned three-point curvature in inverse metres."""

    return abs(_signed_discrete_curvature(first, middle, last))


def _signed_discrete_curvature(
    first: PathPoint, middle: PathPoint, last: PathPoint
) -> float:
    """Return signed three-point curvature in inverse metres."""

    ab = distance_xy(first.x, first.y, middle.x, middle.y)
    bc = distance_xy(middle.x, middle.y, last.x, last.y)
    ca = distance_xy(last.x, last.y, first.x, first.y)
    if min(ab, bc, ca) <= MIN_SEGMENT_LENGTH_M:
        return 0.0

    twice_area = (
        (middle.x - first.x) * (last.y - first.y)
        - (middle.y - first.y) * (last.x - first.x)
    )
    if abs(twice_area) <= MIN_TWICE_TRIANGLE_AREA_M2:
        return 0.0

    denominator = ab * bc * ca
    if denominator <= MIN_SEGMENT_LENGTH_M ** 3:
        return 0.0
    curvature = 2.0 * twice_area / denominator
    return curvature if math.isfinite(curvature) else 0.0


def _high_curvature_mean(curvatures: Sequence[float]) -> float:
    high_values = sorted(curvatures, reverse=True)[:ROBUST_HIGH_CURVATURE_COUNT]
    return sum(high_values) / len(high_values) if high_values else 0.0


def _interpolate_point(
    points: Sequence[PathPoint], distances: Sequence[float], target: float
) -> PathPoint:
    index = max(0, bisect_right(distances, target) - 1)
    while (
        index + 1 < len(points)
        and distances[index + 1] - distances[index] <= MIN_SEGMENT_LENGTH_M
    ):
        index += 1
    if index + 1 >= len(points):
        return points[-1]

    span = distances[index + 1] - distances[index]
    ratio = (target - distances[index]) / span
    return PathPoint(
        points[index].x + ratio * (points[index + 1].x - points[index].x),
        points[index].y + ratio * (points[index + 1].y - points[index].y),
    )


def _wide_preview_curvature(
    points: Sequence[PathPoint],
    preview_indices: Sequence[int],
) -> float:
    support = [points[index] for index in preview_indices]
    distances = [0.0]
    for previous, following in zip(support, support[1:]):
        distances.append(
            distances[-1]
            + distance_xy(
                previous.x,
                previous.y,
                following.x,
                following.y,
            )
        )

    curvatures = []
    for center in distances:
        if (
            center < WIDE_CURVATURE_HALF_SPAN_M
            or center + WIDE_CURVATURE_HALF_SPAN_M > distances[-1]
        ):
            continue
        curvatures.append(
            discrete_curvature(
                _interpolate_point(
                    support, distances, center - WIDE_CURVATURE_HALF_SPAN_M
                ),
                _interpolate_point(support, distances, center),
                _interpolate_point(
                    support, distances, center + WIDE_CURVATURE_HALF_SPAN_M
                ),
            )
        )
    return _high_curvature_mean(curvatures)


def _persistent_turn_curvature(
    samples: Sequence[Tuple[float, float]],
) -> float:
    best = 0.0
    sign = 0
    support = 0.0
    straight_gap = 0.0
    curvatures: List[float] = []

    def close_run() -> float:
        if support < PERSISTENT_TURN_SUPPORT_M:
            return 0.0
        return _high_curvature_mean(curvatures)

    for curvature, sample_support in samples:
        current_sign = 1 if curvature > 0.0 else -1 if curvature < 0.0 else 0
        if current_sign == 0:
            straight_gap += sample_support
            if straight_gap > MAX_STRAIGHT_GAP_M:
                best = max(best, close_run())
                sign = 0
                support = 0.0
                curvatures = []
            continue

        if sign and current_sign != sign:
            best = max(best, close_run())
            support = 0.0
            curvatures = []
        sign = current_sign
        support += sample_support
        straight_gap = 0.0
        curvatures.append(abs(curvature))

    return max(best, close_run())


def _turn_coherence(samples: Sequence[Tuple[float, float]]) -> float:
    absolute_turn = sum(abs(curvature) * support for curvature, support in samples)
    if absolute_turn <= MIN_TWICE_TRIANGLE_AREA_M2:
        return 0.0
    net_turn = sum(curvature * support for curvature, support in samples)
    return abs(net_turn) / absolute_turn


def preview_curvature(
    state: VehicleState,
    path: Sequence[PointInput],
    preview_distance_m: float,
    closed_loop: bool = False,
) -> float:
    """Summarize curvature ahead using the mean of up to three largest values."""

    if preview_distance_m <= 0.0:
        raise ValueError("preview_distance_m must be positive")

    points = coerce_path(path)
    count = logical_path_count(points, closed_loop)
    nearest = nearest_waypoint_index(state, points, count)
    if count < 3:
        return 0.0

    preview_indices = [nearest]
    travelled = 0.0
    current = nearest
    maximum_steps = count - 1 if closed_loop else count - nearest - 1
    for _ in range(maximum_steps):
        following = (current + 1) % count
        segment_length = distance_xy(
            points[current].x,
            points[current].y,
            points[following].x,
            points[following].y,
        )
        if travelled + segment_length > preview_distance_m:
            break
        travelled += segment_length
        preview_indices.append(following)
        current = following

    samples = []
    for index in preview_indices:
        if not closed_loop and (index == 0 or index == count - 1):
            continue
        previous = (index - 1) % count
        following = (index + 1) % count
        curvature = _signed_discrete_curvature(
            points[previous], points[index], points[following]
        )
        support = 0.5 * (
            distance_xy(
                points[previous].x,
                points[previous].y,
                points[index].x,
                points[index].y,
            )
            + distance_xy(
                points[index].x,
                points[index].y,
                points[following].x,
                points[following].y,
            )
        )
        samples.append((curvature, support))

    if not samples:
        return 0.0
    raw_curvature = _high_curvature_mean(
        [abs(curvature) for curvature, _ in samples]
    )
    if _turn_coherence(samples) >= CONSISTENT_TURN_RATIO:
        return raw_curvature
    wide_curvature = _wide_preview_curvature(
        points, preview_indices
    )
    persistent_curvature = _persistent_turn_curvature(samples)
    if wide_curvature == 0.0 and persistent_curvature == 0.0:
        return raw_curvature
    return max(wide_curvature, persistent_curvature)
