#!/usr/bin/env python3
"""Rectify a PhysiCar top-view screenshot into metric simulator coordinates.

The screenshot homography is solved from the orange cones visible in the image
and the same cones' live simulator poses.  Yellow and white track pixels are
then warped into a north-up ``sim_world`` raster with a fixed metre-per-pixel
resolution.  No scale is applied to ROS TF: TF already uses SI metres.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

import cv2
import numpy as np

from extract_sim_map_lines import extract_line_masks


def api_json(url: str, timeout_sec: float) -> dict[str, object]:
    try:
        with urlopen(url, timeout=timeout_sec) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (HTTPError, URLError, TimeoutError, OSError) as error:
        raise RuntimeError(f"simulator API request failed: {error}") from error
    if not isinstance(payload, dict):
        raise RuntimeError(f"simulator API returned a non-object: {url}")
    return payload


def live_cone_positions(api_base: str, timeout_sec: float) -> tuple[str, list[str], np.ndarray]:
    payload = api_json(f"{api_base.rstrip('/')}/objects", timeout_sec)
    world = str(payload.get("world", ""))
    objects = payload.get("objects")
    if not isinstance(objects, list):
        raise RuntimeError("simulator /objects response has no object list")

    cones: list[tuple[str, float, float]] = []
    for item in objects:
        if not isinstance(item, dict) or not str(item.get("name", "")).startswith("cone"):
            continue
        pose = item.get("current")
        if not isinstance(pose, dict):
            continue
        try:
            cones.append((str(item["name"]), float(pose["x"]), float(pose["y"])))
        except (KeyError, TypeError, ValueError):
            continue
    cones.sort(key=lambda item: item[0])
    if len(cones) < 4:
        raise RuntimeError(f"need at least four live cones, found {len(cones)}")
    names = [item[0] for item in cones]
    positions = np.asarray([[item[1], item[2]] for item in cones], dtype=np.float32)
    return world, names, positions


def saved_cone_positions(path: Path) -> tuple[str, list[str], np.ndarray]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    world = str(payload.get("simulator_world", ""))
    items = payload.get("cones")
    if not isinstance(items, list):
        raise ValueError(f"cone reference has no cones list: {path}")
    cones: list[tuple[str, float, float]] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        try:
            cones.append((str(item["name"]), float(item["x_m"]), float(item["y_m"])))
        except (KeyError, TypeError, ValueError):
            continue
    cones.sort(key=lambda item: item[0])
    if len(cones) < 4:
        raise ValueError(f"cone reference needs at least four valid cones: {path}")
    return (
        world,
        [item[0] for item in cones],
        np.asarray([[item[1], item[2]] for item in cones], dtype=np.float32),
    )


def detected_cone_contacts(image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return orange mask and cone/ground contact pixels."""

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(
        hsv,
        np.array([0, 100, 50], dtype=np.uint8),
        np.array([13, 255, 255], dtype=np.uint8),
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(orange, connectivity=8)
    components: list[tuple[int, float, float]] = []
    minimum_area = max(5, int(round(image.shape[0] * image.shape[1] * 0.00004)))
    maximum_area = int(round(image.shape[0] * image.shape[1] * 0.01))
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if not minimum_area <= area <= maximum_area:
            continue
        rows, columns = np.nonzero(labels == label)
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        bottom_band = rows >= rows.max() - max(1, int(round(height * 0.2)))
        contact_u = float(np.median(columns[bottom_band]))
        contact_v = float(rows.max())
        components.append((area, contact_u, contact_v))
    components.sort(reverse=True)
    contacts = np.asarray([[item[1], item[2]] for item in components], dtype=np.float32)
    return orange, contacts


def solve_world_to_image(
    world_points: np.ndarray,
    image_points: np.ndarray,
) -> tuple[np.ndarray, tuple[int, ...], float, np.ndarray]:
    """Match equal-size point sets and solve a projective homography."""

    if len(world_points) != len(image_points):
        raise ValueError("world and image cone counts differ")
    if len(world_points) > 8:
        raise ValueError("refusing factorial matching for more than eight cones")

    best: tuple[float, tuple[int, ...], np.ndarray, np.ndarray] | None = None
    for permutation in itertools.permutations(range(len(image_points))):
        targets = image_points[list(permutation)]
        homography, _ = cv2.findHomography(world_points, targets, method=0)
        if homography is None or not np.isfinite(homography).all():
            continue
        projected = cv2.perspectiveTransform(world_points[None, :, :], homography)[0]
        errors = np.linalg.norm(projected - targets, axis=1)
        rmse = float(np.sqrt(np.mean(np.square(errors))))
        if best is None or rmse < best[0]:
            best = rmse, permutation, homography, errors
    if best is None:
        raise RuntimeError("could not solve screenshot homography")
    return best[2], best[1], best[0], best[3]


def world_to_raster_matrix(
    bounds: tuple[float, float, float, float], resolution_m: float
) -> tuple[np.ndarray, int, int]:
    min_x, max_x, min_y, max_y = bounds
    width = int(round((max_x - min_x) / resolution_m))
    height = int(round((max_y - min_y) / resolution_m))
    matrix = np.array(
        [
            [1.0 / resolution_m, 0.0, -min_x / resolution_m],
            [0.0, -1.0 / resolution_m, max_y / resolution_m - 1.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return matrix, width, height


def write_map_yaml(path: Path, image_name: str, resolution_m: float, origin: tuple[float, float]) -> None:
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: trinary",
                f"resolution: {resolution_m:.6f}",
                f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.0]",
                "negate: 1",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="top-view simulator screenshot")
    parser.add_argument("--output-dir", type=Path, default=Path("docs"))
    parser.add_argument("--api-base", default="http://localhost/sim/api")
    parser.add_argument("--api-timeout-sec", type=float, default=2.0)
    parser.add_argument(
        "--cone-reference",
        type=Path,
        help="saved capture-time cone poses; use this if the SIM has since reset",
    )
    parser.add_argument("--resolution-m", type=float, default=0.01)
    parser.add_argument("--min-x", type=float, default=0.0)
    parser.add_argument("--max-x", type=float, default=12.0)
    parser.add_argument("--min-y", type=float, default=0.0)
    parser.add_argument("--max-y", type=float, default=7.0)
    parser.add_argument("--maximum-reprojection-rmse-px", type=float, default=3.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.resolution_m <= 0.0:
        raise SystemExit("--resolution-m must be positive")
    bounds = (args.min_x, args.max_x, args.min_y, args.max_y)
    if not args.min_x < args.max_x or not args.min_y < args.max_y:
        raise SystemExit("world bounds must have positive width and height")

    image = cv2.imread(str(args.input), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"could not read input image: {args.input}")
    if args.cone_reference is not None:
        world_name, cone_names, cone_world = saved_cone_positions(args.cone_reference)
        cone_position_source = str(args.cone_reference)
    else:
        world_name, cone_names, cone_world = live_cone_positions(
            args.api_base, args.api_timeout_sec
        )
        cone_position_source = f"{args.api_base.rstrip('/')}/objects"
    orange_mask, cone_image = detected_cone_contacts(image)
    if len(cone_image) != len(cone_world):
        raise SystemExit(
            f"detected {len(cone_image)} orange components but simulator reports "
            f"{len(cone_world)} cones"
        )

    world_to_image, permutation, rmse, errors = solve_world_to_image(
        cone_world, cone_image
    )
    if rmse > args.maximum_reprojection_rmse_px:
        raise SystemExit(
            f"cone homography RMSE {rmse:.3f} px exceeds "
            f"{args.maximum_reprojection_rmse_px:.3f} px; screenshot and live SIM "
            "object poses probably do not match"
        )

    yellow, white = extract_line_masks(image)
    combined = cv2.bitwise_or(yellow, white)
    source_color = cv2.bitwise_and(image, image, mask=combined)
    world_to_raster, width, height = world_to_raster_matrix(bounds, args.resolution_m)
    image_to_raster = world_to_raster @ np.linalg.inv(world_to_image)
    output_size = (width, height)
    rectified_color = cv2.warpPerspective(
        source_color,
        image_to_raster,
        output_size,
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    rectified_binary = cv2.warpPerspective(
        combined,
        image_to_raster,
        output_size,
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )

    debug = image.copy()
    matched_image = cone_image[list(permutation)]
    projected = cv2.perspectiveTransform(cone_world[None, :, :], world_to_image)[0]
    for name, measured, estimate in zip(cone_names, matched_image, projected):
        measured_pixel = tuple(np.rint(measured).astype(int))
        estimate_pixel = tuple(np.rint(estimate).astype(int))
        cv2.circle(debug, measured_pixel, 6, (255, 0, 255), 2)
        cv2.circle(debug, estimate_pixel, 2, (0, 255, 0), -1)
        cv2.putText(
            debug,
            name,
            (measured_pixel[0] + 7, measured_pixel[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "color": args.output_dir / "sim_lane_map_world_color.png",
        "binary": args.output_dir / "sim_lane_map_world_binary.png",
        "pgm": args.output_dir / "sim_lane_map.pgm",
        "yaml": args.output_dir / "sim_lane_map.yaml",
        "debug": args.output_dir / "sim_lane_map_calibration_debug.png",
        "metadata": args.output_dir / "sim_lane_map_calibration.json",
    }
    for key, value in (
        ("color", rectified_color),
        ("binary", rectified_binary),
        ("pgm", rectified_binary),
        ("debug", debug),
    ):
        if not cv2.imwrite(str(outputs[key]), value):
            raise OSError(f"failed to write {outputs[key]}")
    write_map_yaml(
        outputs["yaml"], outputs["pgm"].name, args.resolution_m, (args.min_x, args.min_y)
    )

    correspondences = []
    for index, name in enumerate(cone_names):
        correspondences.append(
            {
                "name": name,
                "sim_world_xy_m": cone_world[index].astype(float).tolist(),
                "image_uv_px": matched_image[index].astype(float).tolist(),
                "reprojection_error_px": float(errors[index]),
            }
        )
    metadata = {
        "version": 1,
        "source_image": str(args.input),
        "simulator_world": world_name,
        "cone_position_source": cone_position_source,
        "reference_frame": "sim_world",
        "world_bounds_m": {
            "min_x": args.min_x,
            "max_x": args.max_x,
            "min_y": args.min_y,
            "max_y": args.max_y,
        },
        "output_size_px": {"width": width, "height": height},
        "resolution_m_per_px": args.resolution_m,
        "tf_scale": 1.0,
        "tf_note": "ROS TF translations remain SI metres; only the screenshot raster is rectified.",
        "axis_note": "image right is +sim_world x; image up is +sim_world y",
        "world_to_source_image_homography": world_to_image.tolist(),
        "source_image_to_world_homography": np.linalg.inv(world_to_image).tolist(),
        "source_image_to_output_raster_homography": image_to_raster.tolist(),
        "cone_reprojection_rmse_px": rmse,
        "cone_correspondences": correspondences,
        "map_yaml_note": (
            "negate=1 makes white lane pixels occupied if loaded by map_server; "
            "this is a visual lane reference, not a LiDAR occupancy map"
        ),
    }
    outputs["metadata"].write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print(f"world: {world_name}")
    print(f"cone homography RMSE: {rmse:.3f} px")
    print(f"metric raster: {width}x{height} at {args.resolution_m:.4f} m/px")
    for path in outputs.values():
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
