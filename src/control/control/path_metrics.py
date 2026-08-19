"""Stateless path geometry metrics for adaptive vehicle control."""

import math
from typing import List, Sequence, Tuple, Union

from .geometry import as_path_point, distance_xy
from .models import PathPoint, VehicleState

PointInput = Union[PathPoint, Tuple[float, float]]
DUPLICATE_ENDPOINT_TOLERANCE_M = 1.0e-9
MIN_SEGMENT_LENGTH_M = 1.0e-9
MIN_TWICE_TRIANGLE_AREA_M2 = 1.0e-12
ROBUST_HIGH_CURVATURE_COUNT = 3


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

    ab = distance_xy(first.x, first.y, middle.x, middle.y)
    bc = distance_xy(middle.x, middle.y, last.x, last.y)
    ca = distance_xy(last.x, last.y, first.x, first.y)
    if min(ab, bc, ca) <= MIN_SEGMENT_LENGTH_M:
        return 0.0

    twice_area = abs(
        (middle.x - first.x) * (last.y - first.y)
        - (middle.y - first.y) * (last.x - first.x)
    )
    if twice_area <= MIN_TWICE_TRIANGLE_AREA_M2:
        return 0.0

    denominator = ab * bc * ca
    if denominator <= MIN_SEGMENT_LENGTH_M ** 3:
        return 0.0
    curvature = 2.0 * twice_area / denominator
    return curvature if math.isfinite(curvature) else 0.0


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

    curvatures = []
    for index in preview_indices:
        if not closed_loop and (index == 0 or index == count - 1):
            continue
        previous = (index - 1) % count
        following = (index + 1) % count
        curvatures.append(
            discrete_curvature(points[previous], points[index], points[following])
        )

    if not curvatures:
        return 0.0
    high_values = sorted(curvatures, reverse=True)[:ROBUST_HIGH_CURVATURE_COUNT]
    return sum(high_values) / len(high_values)
