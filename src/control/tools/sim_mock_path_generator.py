#!/usr/bin/env python3
"""Generate and validate a SIM-only curvature-bounded mock route.

This tool only reads route geometry and writes a JSON result under /tmp. It does
not import ROS, publish commands, or move the vehicle.
"""

import argparse
import json
import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple
from urllib.request import urlopen

Point = Tuple[float, float]

DEFAULT_ROUTE_URL = "http://localhost/sim/api/route"
DEFAULT_OUTPUT_PATH = "/tmp/amet_merged_fillet_route.json"
RDP_EPSILONS_M = (0.03, 0.05, 0.08, 0.10, 0.12, 0.15)
TARGET_MAX_CURVATURE_INV_M = 1.8
FILLET_RADIUS_M = 1.0 / TARGET_MAX_CURVATURE_INV_M
SAMPLE_SPACING_M = 0.05
PASS_MAX_CURVATURE_INV_M = 1.85
PASS_MIN_BOUNDARY_CLEARANCE_M = 0.12
DUPLICATE_ENDPOINT_TOLERANCE_M = 1.0e-9
GEOMETRY_TOLERANCE = 1.0e-9
MAX_MERGE_ITERATIONS = 1000
MAX_VIRTUAL_CORNER_DISTANCE_FACTOR = 3.0
MAX_OUTSIDE_DIAGNOSTICS = 20


@dataclass(frozen=True)
class Fillet:
    tangent_in: Point
    tangent_out: Point
    center: Optional[Point]
    turn_sign: int
    tangent_distance_m: float


@dataclass(frozen=True)
class CandidateMetrics:
    epsilon_m: float
    vertices_before_merge: int
    vertices_after_merge: int
    merge_count: int
    point_count: int
    max_curvature_inv_m: float
    p99_curvature_inv_m: float
    old_outside_count: int
    cell_outside_count: int
    min_boundary_clearance_m: float
    mean_centerline_deviation_m: float
    max_centerline_deviation_m: float

    @property
    def passed(self) -> bool:
        return (
            self.max_curvature_inv_m <= PASS_MAX_CURVATURE_INV_M
            and self.cell_outside_count == 0
            and self.min_boundary_clearance_m
            >= PASS_MIN_BOUNDARY_CLEARANCE_M
        )


def distance(first: Point, second: Point) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


def add(first: Point, second: Point) -> Point:
    return first[0] + second[0], first[1] + second[1]


def subtract(first: Point, second: Point) -> Point:
    return first[0] - second[0], first[1] - second[1]


def scale(vector: Point, factor: float) -> Point:
    return vector[0] * factor, vector[1] * factor


def cross(first: Point, second: Point) -> float:
    return first[0] * second[1] - first[1] * second[0]


def dot(first: Point, second: Point) -> float:
    return first[0] * second[0] + first[1] * second[1]


def normalize(vector: Point) -> Optional[Point]:
    length = math.hypot(vector[0], vector[1])
    if length <= GEOMETRY_TOLERANCE:
        return None
    return vector[0] / length, vector[1] / length


def coerce_point(value: object) -> Point:
    if isinstance(value, dict):
        if "x" in value and "y" in value:
            return float(value["x"]), float(value["y"])
        position = value.get("position")
        if isinstance(position, dict) and "x" in position and "y" in position:
            return float(position["x"]), float(position["y"])
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return float(value[0]), float(value[1])
    raise ValueError("route point must be [x, y] or an object containing x and y")


def logical_closed_path(values: Sequence[object]) -> List[Point]:
    points = [coerce_point(value) for value in values]
    if len(points) > 1 and distance(points[0], points[-1]) <= DUPLICATE_ENDPOINT_TOLERANCE_M:
        points.pop()
    if len(points) < 3:
        raise ValueError("closed route must contain at least three logical points")
    return points


def read_route(url: str, timeout_s: float) -> Dict[str, object]:
    with urlopen(url, timeout=timeout_s) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("route API response must be a JSON object")
    missing = [key for key in ("world", "waypoints", "inner", "outer") if key not in payload]
    if missing:
        raise ValueError("route API response is missing keys: " + ", ".join(missing))
    return payload


def point_segment_distance(point: Point, start: Point, end: Point) -> float:
    segment = subtract(end, start)
    length_squared = dot(segment, segment)
    if length_squared <= GEOMETRY_TOLERANCE ** 2:
        return distance(point, start)
    projection = dot(subtract(point, start), segment) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = add(start, scale(segment, projection))
    return distance(point, closest)


def point_polyline_distance(point: Point, path: Sequence[Point], closed: bool) -> float:
    segment_count = len(path) if closed else len(path) - 1
    return min(
        point_segment_distance(point, path[index], path[(index + 1) % len(path)])
        for index in range(segment_count)
    )


def perpendicular_distance(point: Point, start: Point, end: Point) -> float:
    return point_segment_distance(point, start, end)


def rdp_open(points: Sequence[Point], epsilon_m: float) -> List[Point]:
    if len(points) <= 2:
        return list(points)
    maximum_distance = -1.0
    split_index = 0
    for index in range(1, len(points) - 1):
        candidate_distance = perpendicular_distance(points[index], points[0], points[-1])
        if candidate_distance > maximum_distance:
            maximum_distance = candidate_distance
            split_index = index
    if maximum_distance <= epsilon_m:
        return [points[0], points[-1]]
    first = rdp_open(points[: split_index + 1], epsilon_m)
    second = rdp_open(points[split_index:], epsilon_m)
    return first[:-1] + second


def rdp_closed(points: Sequence[Point], epsilon_m: float) -> List[Point]:
    """Simplify a ring by splitting it at a deterministic farthest-point pair."""

    anchor = min(range(len(points)), key=lambda index: (points[index][0], points[index][1]))
    opposite = max(range(len(points)), key=lambda index: distance(points[anchor], points[index]))
    if anchor > opposite:
        anchor, opposite = opposite, anchor
    first_chain = list(points[anchor : opposite + 1])
    second_chain = list(points[opposite:]) + list(points[: anchor + 1])
    first_result = rdp_open(first_chain, epsilon_m)
    second_result = rdp_open(second_chain, epsilon_m)
    simplified = first_result[:-1] + second_result[:-1]
    if len(simplified) < 3:
        raise ValueError("RDP simplification produced fewer than three vertices")
    return simplified


def fillet_for_corner(previous: Point, corner: Point, following: Point, radius_m: float) -> Fillet:
    toward_previous = normalize(subtract(previous, corner))
    toward_following = normalize(subtract(following, corner))
    if toward_previous is None or toward_following is None:
        return Fillet(corner, corner, None, 0, 0.0)
    cosine = max(-1.0, min(1.0, dot(toward_previous, toward_following)))
    angle = math.acos(cosine)
    if angle <= 1.0e-6 or math.pi - angle <= 1.0e-6:
        return Fillet(corner, corner, None, 0, 0.0)
    tangent_distance = radius_m / math.tan(angle / 2.0)
    bisector = normalize(add(toward_previous, toward_following))
    sine_half = math.sin(angle / 2.0)
    if bisector is None or sine_half <= GEOMETRY_TOLERANCE:
        return Fillet(corner, corner, None, 0, 0.0)
    tangent_in = add(corner, scale(toward_previous, tangent_distance))
    tangent_out = add(corner, scale(toward_following, tangent_distance))
    center = add(corner, scale(bisector, radius_m / sine_half))
    incoming = subtract(corner, previous)
    outgoing = subtract(following, corner)
    turn_cross = cross(incoming, outgoing)
    turn_sign = 1 if turn_cross > 0.0 else -1
    return Fillet(tangent_in, tangent_out, center, turn_sign, tangent_distance)


def compute_fillets(vertices: Sequence[Point], radius_m: float) -> List[Fillet]:
    return [
        fillet_for_corner(
            vertices[(index - 1) % len(vertices)],
            vertices[index],
            vertices[(index + 1) % len(vertices)],
            radius_m,
        )
        for index in range(len(vertices))
    ]


def infinite_line_intersection(a: Point, b: Point, c: Point, d: Point) -> Optional[Point]:
    first_direction = subtract(b, a)
    second_direction = subtract(d, c)
    denominator = cross(first_direction, second_direction)
    if abs(denominator) <= GEOMETRY_TOLERANCE:
        return None
    parameter = cross(subtract(c, a), second_direction) / denominator
    intersection = add(a, scale(first_direction, parameter))
    if not all(math.isfinite(value) for value in intersection):
        return None
    return intersection


def merged_corner(vertices: Sequence[Point], first_index: int, radius_m: float) -> Point:
    count = len(vertices)
    second_index = (first_index + 1) % count
    previous = vertices[(first_index - 1) % count]
    first = vertices[first_index]
    second = vertices[second_index]
    following = vertices[(second_index + 1) % count]
    midpoint = scale(add(first, second), 0.5)
    intersection = infinite_line_intersection(previous, first, second, following)
    if intersection is None:
        return midpoint
    local_scale = max(
        distance(previous, first),
        distance(first, second),
        distance(second, following),
        radius_m,
    )
    if distance(intersection, midpoint) > MAX_VIRTUAL_CORNER_DISTANCE_FACTOR * local_scale:
        return midpoint
    return intersection


def merge_overlapping_corners(vertices: Sequence[Point], radius_m: float) -> Tuple[List[Point], int]:
    merged = list(vertices)
    merge_count = 0
    for _ in range(MAX_MERGE_ITERATIONS):
        if len(merged) < 3:
            raise ValueError("corner merging left fewer than three vertices")
        fillets = compute_fillets(merged, radius_m)
        overlap_index = None
        for index in range(len(merged)):
            following = (index + 1) % len(merged)
            available = distance(merged[index], merged[following])
            required = (
                fillets[index].tangent_distance_m
                + fillets[following].tangent_distance_m
            )
            if required > available + GEOMETRY_TOLERANCE:
                overlap_index = index
                break
        if overlap_index is None:
            return merged, merge_count

        virtual_corner = merged_corner(merged, overlap_index, radius_m)
        following = (overlap_index + 1) % len(merged)
        if following == 0:
            merged = [virtual_corner] + merged[1:overlap_index]
        else:
            merged = (
                merged[:overlap_index]
                + [virtual_corner]
                + merged[following + 1 :]
            )
        merge_count += 1
    raise RuntimeError("corner merge did not converge")


def append_line_samples(samples: List[Point], start: Point, end: Point, spacing_m: float) -> None:
    length = distance(start, end)
    steps = max(1, int(math.ceil(length / spacing_m)))
    for step in range(1, steps + 1):
        fraction = step / steps
        samples.append(
            (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
            )
        )


def append_arc_samples(samples: List[Point], fillet: Fillet, radius_m: float, spacing_m: float) -> None:
    if fillet.center is None or fillet.turn_sign == 0:
        if distance(samples[-1], fillet.tangent_out) > GEOMETRY_TOLERANCE:
            samples.append(fillet.tangent_out)
        return
    start_angle = math.atan2(
        fillet.tangent_in[1] - fillet.center[1],
        fillet.tangent_in[0] - fillet.center[0],
    )
    end_angle = math.atan2(
        fillet.tangent_out[1] - fillet.center[1],
        fillet.tangent_out[0] - fillet.center[0],
    )
    if fillet.turn_sign > 0:
        while end_angle <= start_angle:
            end_angle += 2.0 * math.pi
    else:
        while end_angle >= start_angle:
            end_angle -= 2.0 * math.pi
    sweep = end_angle - start_angle
    steps = max(1, int(math.ceil(abs(sweep) * radius_m / spacing_m)))
    for step in range(1, steps + 1):
        angle = start_angle + sweep * step / steps
        samples.append(
            (
                fillet.center[0] + radius_m * math.cos(angle),
                fillet.center[1] + radius_m * math.sin(angle),
            )
        )


def sample_fillet_path(vertices: Sequence[Point], radius_m: float, spacing_m: float) -> List[Point]:
    fillets = compute_fillets(vertices, radius_m)
    samples = [fillets[0].tangent_out]
    for index in range(len(vertices)):
        following = (index + 1) % len(vertices)
        append_line_samples(
            samples,
            fillets[index].tangent_out,
            fillets[following].tangent_in,
            spacing_m,
        )
        append_arc_samples(samples, fillets[following], radius_m, spacing_m)
    if distance(samples[0], samples[-1]) <= 1.0e-7:
        samples.pop()
    return remove_consecutive_duplicates(samples)


def remove_consecutive_duplicates(points: Sequence[Point]) -> List[Point]:
    cleaned = []
    for point in points:
        if not cleaned or distance(cleaned[-1], point) > GEOMETRY_TOLERANCE:
            cleaned.append(point)
    if len(cleaned) > 1 and distance(cleaned[0], cleaned[-1]) <= GEOMETRY_TOLERANCE:
        cleaned.pop()
    return cleaned


def discrete_curvature(first: Point, middle: Point, last: Point) -> float:
    ab = distance(first, middle)
    bc = distance(middle, last)
    ca = distance(last, first)
    denominator = ab * bc * ca
    if min(ab, bc, ca) <= GEOMETRY_TOLERANCE or denominator <= GEOMETRY_TOLERANCE ** 3:
        return 0.0
    twice_area = abs(cross(subtract(middle, first), subtract(last, first)))
    curvature = 2.0 * twice_area / denominator
    return curvature if math.isfinite(curvature) else 0.0


def path_curvatures(path: Sequence[Point]) -> List[float]:
    return [
        discrete_curvature(path[(index - 1) % len(path)], path[index], path[(index + 1) % len(path)])
        for index in range(len(path))
    ]


def percentile(values: Sequence[float], probability: float) -> float:
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(math.ceil(probability * len(ordered))) - 1))
    return ordered[index]


def point_on_segment(point: Point, start: Point, end: Point, tolerance: float) -> bool:
    return point_segment_distance(point, start, end) <= tolerance


def point_in_polygon(point: Point, polygon: Sequence[Point], tolerance: float = 1.0e-9) -> bool:
    """Legacy global ray-casting result, retained only for comparison."""

    inside = False
    for index in range(len(polygon)):
        start = polygon[index]
        end = polygon[(index + 1) % len(polygon)]
        if point_on_segment(point, start, end, tolerance):
            return True
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing_x = (
                (end[0] - start[0]) * (point[1] - start[1])
                / (end[1] - start[1])
                + start[0]
            )
            if point[0] < crossing_x:
                inside = not inside
    return inside


def point_in_triangle(
    point: Point, first: Point, second: Point, third: Point, tolerance: float = 1.0e-9
) -> bool:
    if abs(cross(subtract(second, first), subtract(third, first))) <= tolerance:
        return any(
            point_on_segment(point, start, end, tolerance)
            for start, end in (
                (first, second),
                (second, third),
                (third, first),
            )
        )
    c1 = cross(subtract(second, first), subtract(point, first))
    c2 = cross(subtract(third, second), subtract(point, second))
    c3 = cross(subtract(first, third), subtract(point, third))
    has_negative = c1 < -tolerance or c2 < -tolerance or c3 < -tolerance
    has_positive = c1 > tolerance or c2 > tolerance or c3 > tolerance
    return not (has_negative and has_positive)


def point_in_track_cells(point: Point, inner: Sequence[Point], outer: Sequence[Point]) -> bool:
    for index in range(len(inner)):
        following = (index + 1) % len(inner)
        if point_in_triangle(point, inner[index], inner[following], outer[following]):
            return True
        if point_in_triangle(point, inner[index], outer[following], outer[index]):
            return True
    return False


def old_corridor_contains(point: Point, inner: Sequence[Point], outer: Sequence[Point]) -> bool:
    """Classify using one global polygon made from both boundary rings."""

    corridor_polygon = list(outer) + list(reversed(inner))
    return point_in_polygon(point, corridor_polygon)


def closest_waypoint_index(point: Point, waypoints: Sequence[Point]) -> int:
    return min(range(len(waypoints)), key=lambda index: distance(point, waypoints[index]))


def evaluate_candidate(
    epsilon_m: float,
    vertices_before: int,
    vertices_after: int,
    merge_count: int,
    path: Sequence[Point],
    centerline: Sequence[Point],
    inner: Sequence[Point],
    outer: Sequence[Point],
) -> Tuple[CandidateMetrics, List[int]]:
    curvatures = path_curvatures(path)
    center_deviations = [
        point_polyline_distance(point, centerline, closed=True) for point in path
    ]
    inner_distances = [point_polyline_distance(point, inner, closed=True) for point in path]
    outer_distances = [point_polyline_distance(point, outer, closed=True) for point in path]
    old_outside = [
        index for index, point in enumerate(path) if not old_corridor_contains(point, inner, outer)
    ]
    cell_outside = [
        index for index, point in enumerate(path) if not point_in_track_cells(point, inner, outer)
    ]
    metrics = CandidateMetrics(
        epsilon_m=epsilon_m,
        vertices_before_merge=vertices_before,
        vertices_after_merge=vertices_after,
        merge_count=merge_count,
        point_count=len(path),
        max_curvature_inv_m=max(curvatures),
        p99_curvature_inv_m=percentile(curvatures, 0.99),
        old_outside_count=len(old_outside),
        cell_outside_count=len(cell_outside),
        min_boundary_clearance_m=min(min(inner_distances), min(outer_distances)),
        mean_centerline_deviation_m=sum(center_deviations) / len(center_deviations),
        max_centerline_deviation_m=max(center_deviations),
    )
    return metrics, cell_outside


def print_metrics(metrics: CandidateMetrics) -> None:
    print(f"epsilon: {metrics.epsilon_m:.3f} m")
    print(f"vertices before merge: {metrics.vertices_before_merge}")
    print(f"vertices after merge: {metrics.vertices_after_merge}")
    print(f"merge count: {metrics.merge_count}")
    print(f"path point count: {metrics.point_count}")
    print(f"max curvature: {metrics.max_curvature_inv_m:.6f} 1/m")
    print(f"p99 curvature: {metrics.p99_curvature_inv_m:.6f} 1/m")
    print(f"oldOutside: {metrics.old_outside_count}")
    print(f"cellOutside: {metrics.cell_outside_count}")
    print(f"minimum boundary clearance: {metrics.min_boundary_clearance_m:.6f} m")
    print(f"mean centerline deviation: {metrics.mean_centerline_deviation_m:.6f} m")
    print(f"max centerline deviation: {metrics.max_centerline_deviation_m:.6f} m")
    print(f"result: {'PASS' if metrics.passed else 'FAIL'}")


def print_outside_diagnostics(
    outside_indices: Sequence[int],
    path: Sequence[Point],
    centerline: Sequence[Point],
    inner: Sequence[Point],
    outer: Sequence[Point],
) -> None:
    if not outside_indices:
        return
    print("cellOutside diagnostics:")
    for path_index in outside_indices[:MAX_OUTSIDE_DIAGNOSTICS]:
        point = path[path_index]
        center_distance = point_polyline_distance(point, centerline, closed=True)
        inner_distance = point_polyline_distance(point, inner, closed=True)
        outer_distance = point_polyline_distance(point, outer, closed=True)
        closest_index = closest_waypoint_index(point, centerline)
        print(
            f"  path_index={path_index} x={point[0]:.6f} y={point[1]:.6f} "
            f"center_distance={center_distance:.6f} "
            f"inner_distance={inner_distance:.6f} "
            f"outer_distance={outer_distance:.6f} "
            f"closest_centerline_index={closest_index}"
        )
    if len(outside_indices) > MAX_OUTSIDE_DIAGNOSTICS:
        print(
            f"  ... {len(outside_indices) - MAX_OUTSIDE_DIAGNOSTICS} "
            "additional points omitted"
        )


def save_candidate(
    output_path: str,
    path: Sequence[Point],
    metrics: CandidateMetrics,
    source_url: str,
    world: object,
) -> None:
    closed_path = list(path) + [path[0]]
    payload = {
        "waypoints": [[point[0], point[1]] for point in closed_path],
        "metadata": {
            "sim_only": True,
            "source_url": source_url,
            "world": world,
            "rdp_epsilon_m": metrics.epsilon_m,
            "fillet_radius_m": FILLET_RADIUS_M,
            "sample_spacing_m": SAMPLE_SPACING_M,
            "target_max_curvature_inv_m": TARGET_MAX_CURVATURE_INV_M,
            "measured_max_curvature_inv_m": metrics.max_curvature_inv_m,
            "p99_curvature_inv_m": metrics.p99_curvature_inv_m,
            "old_outside_count": metrics.old_outside_count,
            "cell_outside_count": metrics.cell_outside_count,
            "min_boundary_clearance_m": metrics.min_boundary_clearance_m,
            "mean_centerline_deviation_m": metrics.mean_centerline_deviation_m,
            "max_centerline_deviation_m": metrics.max_centerline_deviation_m,
            "vertices_before_merge": metrics.vertices_before_merge,
            "vertices_after_merge": metrics.vertices_after_merge,
            "merge_count": metrics.merge_count,
        },
    }
    with open(output_path, "w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)
        output_file.write("\n")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a SIM-only merged-fillet mock route."
    )
    parser.add_argument("--url", default=DEFAULT_ROUTE_URL, help="SIM route API URL")
    parser.add_argument(
        "--timeout", type=float, default=5.0, help="HTTP read timeout in seconds"
    )
    parser.add_argument(
        "--output", default=DEFAULT_OUTPUT_PATH, help="first-PASS JSON output path"
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        route = read_route(args.url, args.timeout)
        centerline = logical_closed_path(route["waypoints"])
        inner = logical_closed_path(route["inner"])
        outer = logical_closed_path(route["outer"])
        if not (len(centerline) == len(inner) == len(outer)):
            raise ValueError(
                "waypoints, inner, and outer must have equal logical point counts"
            )

        print(f"world: {route['world']}")
        print(f"logical route points: {len(centerline)}")
        print(f"fillet radius: {FILLET_RADIUS_M:.6f} m")
        print(f"sampling spacing: {SAMPLE_SPACING_M:.3f} m")

        first_pass = None
        for epsilon in RDP_EPSILONS_M:
            print("\n" + "=" * 72)
            simplified = rdp_closed(centerline, epsilon)
            merged, merge_count = merge_overlapping_corners(
                simplified, FILLET_RADIUS_M
            )
            mock_path = sample_fillet_path(
                merged, FILLET_RADIUS_M, SAMPLE_SPACING_M
            )
            metrics, cell_outside = evaluate_candidate(
                epsilon,
                len(simplified),
                len(merged),
                merge_count,
                mock_path,
                centerline,
                inner,
                outer,
            )
            print_metrics(metrics)
            print_outside_diagnostics(
                cell_outside, mock_path, centerline, inner, outer
            )
            if metrics.passed and first_pass is None:
                first_pass = (list(mock_path), metrics)

        if first_pass is None:
            print("\nNo PASS candidate; output file was not written.")
            return 2

        save_candidate(
            args.output,
            first_pass[0],
            first_pass[1],
            args.url,
            route["world"],
        )
        print(
            f"\nSaved first PASS candidate (epsilon={first_pass[1].epsilon_m:.3f}) "
            f"to {args.output}"
        )
        return 0
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
