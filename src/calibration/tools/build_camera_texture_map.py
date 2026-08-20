#!/usr/bin/env python3
"""Project recorded camera ground pixels into a slam_toolbox occupancy map.

This creates visualization/reference layers.  It does not replace the original
Nav2-compatible ``map.pgm`` because RGB camera observations are not occupancy
probabilities.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import cv2
import numpy as np
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


TransformKey = Tuple[str, str]
TransformSample = Tuple[int, np.ndarray]

CAMERA_TOPIC = "/camera/image_raw/compressed"
TF_TOPICS = {"/tf", "/tf_static"}
CAMERA_FRAME = "camera_optical_frame"

# PhysiCar simulator camera values from model.sdf.  Gazebo's distortion order
# is converted to OpenCV's k1, k2, p1, p2, k3 order below.
HORIZONTAL_FOV_RAD = 1.7453
DISTORTION = np.array([-0.045, -0.0001, -0.0003, -0.0001, 0.001])

TF_CHAIN: Tuple[TransformKey, ...] = (
    ("map", "odom"),
    ("odom", "base_footprint"),
    ("base_footprint", "base_link"),
    ("base_link", "camera_pan_link"),
    ("camera_pan_link", "camera_tilt_link"),
    ("camera_tilt_link", "camera_link"),
    ("camera_link", CAMERA_FRAME),
)


def stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion in TF")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_matrix(transform: object) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_matrix(
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )
    matrix[:3, 3] = (
        float(translation.x),
        float(translation.y),
        float(translation.z),
    )
    return matrix


def open_reader(bag: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def topic_types(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    return {entry.name: entry.type for entry in reader.get_all_topics_and_types()}


def collect_tf(bag: Path) -> Dict[TransformKey, List[TransformSample]]:
    reader = open_reader(bag)
    types = topic_types(reader)
    missing = TF_TOPICS.difference(types)
    if missing:
        raise RuntimeError(f"bag is missing TF topics: {sorted(missing)}")
    classes = {topic: get_message(types[topic]) for topic in TF_TOPICS}
    wanted = set(TF_CHAIN)
    samples: Dict[TransformKey, List[TransformSample]] = defaultdict(list)

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in TF_TOPICS:
            continue
        message = deserialize_message(data, classes[topic])
        for item in message.transforms:
            key = (item.header.frame_id, item.child_frame_id)
            if key not in wanted:
                continue
            timestamp = stamp_ns(item.header.stamp)
            samples[key].append((timestamp, transform_matrix(item.transform)))

    missing_pairs = wanted.difference(samples)
    if missing_pairs:
        formatted = [f"{parent}->{child}" for parent, child in sorted(missing_pairs)]
        raise RuntimeError(f"bag is missing required transforms: {formatted}")
    for values in samples.values():
        values.sort(key=lambda value: value[0])
    return dict(samples)


class TransformLookup:
    def __init__(self, samples: Dict[TransformKey, List[TransformSample]]) -> None:
        self.samples = samples
        self.times = {
            key: np.array([value[0] for value in values], dtype=np.int64)
            for key, values in samples.items()
        }

    def nearest(self, key: TransformKey, timestamp: int) -> Tuple[np.ndarray, int]:
        values = self.samples[key]
        times = self.times[key]
        if len(values) == 1 or int(times[0]) == 0:
            return values[0][1], 0
        index = int(np.searchsorted(times, timestamp))
        if index <= 0:
            selected = 0
        elif index >= len(times):
            selected = len(times) - 1
        else:
            before = index - 1
            selected = before if timestamp - int(times[before]) <= int(times[index]) - timestamp else index
        age = abs(timestamp - int(times[selected]))
        return values[selected][1], age

    def map_to_camera(self, timestamp: int) -> Tuple[np.ndarray, int]:
        result = np.eye(4, dtype=np.float64)
        maximum_age = 0
        for key in TF_CHAIN:
            transform, age = self.nearest(key, timestamp)
            result = result @ transform
            maximum_age = max(maximum_age, age)
        return result, maximum_age


def load_map(map_yaml: Path) -> Tuple[np.ndarray, float, float, float, Path]:
    payload = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = Path(payload["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = np.array(Image.open(image_path).convert("L"), dtype=np.uint8)
    resolution = float(payload["resolution"])
    origin = payload["origin"]
    return image, resolution, float(origin[0]), float(origin[1]), image_path


def camera_rays(
    width: int,
    height: int,
    pixel_stride: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    focal_length = width / (2.0 * math.tan(HORIZONTAL_FOV_RAD / 2.0))
    camera_matrix = np.array(
        [
            [focal_length, 0.0, (width - 1) / 2.0],
            [0.0, focal_length, (height - 1) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    # Start just below the optical horizon.  Rays above it cannot intersect
    # the flat ground plane in front of the vehicle.
    u_values = np.arange(0, width, pixel_stride, dtype=np.int32)
    v_values = np.arange(height // 2, height, pixel_stride, dtype=np.int32)
    uu, vv = np.meshgrid(u_values, v_values)
    pixels = np.column_stack((uu.ravel(), vv.ravel())).astype(np.float64)
    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2),
        camera_matrix,
        DISTORTION,
    ).reshape(-1, 2)
    rays = np.column_stack((normalized, np.ones(len(normalized)))).T
    return rays, uu.ravel(), vv.ravel()


def save_outputs(
    output_dir: Path,
    occupancy: np.ndarray,
    color_sum: np.ndarray,
    view_count: np.ndarray,
    white_count: np.ndarray,
    yellow_count: np.ndarray,
    metadata: Dict[str, object],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    base_rgb = np.repeat(occupancy[:, :, None], 3, axis=2)
    observed = view_count > 0
    mean_color = np.zeros_like(base_rgb)
    mean_color[observed] = np.clip(
        color_sum[observed] / view_count[observed, None], 0, 255
    ).astype(np.uint8)

    texture = base_rgb.copy()
    texture[observed] = mean_color[observed]

    yellow_ratio = np.divide(
        yellow_count,
        np.maximum(view_count, 1),
        dtype=np.float64,
    )
    white_ratio = np.divide(
        white_count,
        np.maximum(view_count, 1),
        dtype=np.float64,
    )
    yellow = ((yellow_count >= 2) & (yellow_ratio >= 0.06)).astype(np.uint8)
    white = ((white_count >= 2) & (white_ratio >= 0.10)).astype(np.uint8)
    kernel = np.ones((2, 2), dtype=np.uint8)
    yellow = cv2.dilate(yellow, kernel, iterations=1).astype(bool)
    white = cv2.dilate(white, kernel, iterations=1).astype(bool)

    lane_layer = np.full_like(base_rgb, 64)
    lane_layer[white] = (255, 255, 255)
    lane_layer[yellow] = (255, 190, 0)

    fused = base_rgb.copy()
    # Camera color is a reference layer.  Apply it only to LiDAR-confirmed free
    # space so projections cannot paint through walls into unknown space.
    known_free = occupancy >= 250
    camera_free = observed & known_free
    fused[camera_free] = mean_color[camera_free]
    lane_allowed = known_free
    fused[white & lane_allowed] = (255, 255, 255)
    fused[yellow & lane_allowed] = (255, 190, 0)
    fused[occupancy < 100] = (0, 0, 0)

    Image.fromarray(texture, mode="RGB").save(output_dir / "camera_ground_texture.png")
    Image.fromarray(lane_layer, mode="RGB").save(output_dir / "camera_lane_layer.png")
    Image.fromarray(fused, mode="RGB").save(output_dir / "lidar_camera_fused.png")
    (output_dir / "camera_fusion_metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def build(args: argparse.Namespace) -> Dict[str, object]:
    occupancy, resolution, origin_x, origin_y, image_path = load_map(args.map_yaml)
    height, width = occupancy.shape
    transforms = TransformLookup(collect_tf(args.bag))

    reader = open_reader(args.bag)
    types = topic_types(reader)
    if CAMERA_TOPIC not in types:
        raise RuntimeError(f"bag is missing {CAMERA_TOPIC}")
    camera_class = get_message(types[CAMERA_TOPIC])

    color_sum = np.zeros((height, width, 3), dtype=np.float64)
    view_count = np.zeros((height, width), dtype=np.int64)
    white_count = np.zeros((height, width), dtype=np.int64)
    yellow_count = np.zeros((height, width), dtype=np.int64)
    ray_cache: Tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
    camera_messages = 0
    processed_frames = 0
    projected_pixels = 0
    maximum_tf_age_ns = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != CAMERA_TOPIC:
            continue
        camera_messages += 1
        if (camera_messages - 1) % args.frame_step:
            continue
        message = deserialize_message(data, camera_class)
        timestamp = stamp_ns(message.header.stamp)
        if message.header.frame_id != CAMERA_FRAME:
            raise RuntimeError(
                f"unexpected camera frame {message.header.frame_id!r}; expected {CAMERA_FRAME!r}"
            )
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"failed to decode camera frame {camera_messages}")
        frame_height, frame_width = frame.shape[:2]
        if ray_cache is None:
            ray_cache = camera_rays(frame_width, frame_height, args.pixel_stride)
        rays_camera, pixel_u, pixel_v = ray_cache

        map_to_camera, tf_age = transforms.map_to_camera(timestamp)
        maximum_tf_age_ns = max(maximum_tf_age_ns, tf_age)
        rotation = map_to_camera[:3, :3]
        camera_position = map_to_camera[:3, 3]
        rays_map = rotation @ rays_camera
        downward = rays_map[2] < -1.0e-3
        scale = np.full(rays_map.shape[1], np.nan, dtype=np.float64)
        scale[downward] = -camera_position[2] / rays_map[2, downward]
        points_x = camera_position[0] + rays_map[0] * scale
        points_y = camera_position[1] + rays_map[1] * scale
        horizontal_range = np.hypot(
            points_x - camera_position[0],
            points_y - camera_position[1],
        )
        finite_projection = downward & np.isfinite(scale)
        columns = np.full(rays_map.shape[1], -1, dtype=np.int64)
        map_y = np.full(rays_map.shape[1], -1, dtype=np.int64)
        columns[finite_projection] = np.floor(
            (points_x[finite_projection] - origin_x) / resolution
        ).astype(np.int64)
        map_y[finite_projection] = np.floor(
            (points_y[finite_projection] - origin_y) / resolution
        ).astype(np.int64)
        rows = height - 1 - map_y
        valid = (
            finite_projection
            & (scale > 0.0)
            & (horizontal_range >= args.min_ground_range_m)
            & (horizontal_range <= args.max_ground_range_m)
            & (columns >= 0)
            & (columns < width)
            & (rows >= 0)
            & (rows < height)
        )
        if not np.any(valid):
            continue

        sampled_bgr = frame[pixel_v, pixel_u]
        sampled_rgb = sampled_bgr[:, ::-1]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[pixel_v, pixel_u]
        hue, saturation, value = hsv[:, 0], hsv[:, 1], hsv[:, 2]
        is_yellow = (hue >= 8) & (hue <= 38) & (saturation >= 90) & (value >= 100)
        is_white = (saturation <= 55) & (value >= 180)

        valid_rows = rows[valid]
        valid_columns = columns[valid]
        np.add.at(color_sum, (valid_rows, valid_columns), sampled_rgb[valid])
        np.add.at(view_count, (valid_rows, valid_columns), 1)
        selected_yellow = valid & is_yellow
        selected_white = valid & is_white
        np.add.at(yellow_count, (rows[selected_yellow], columns[selected_yellow]), 1)
        np.add.at(white_count, (rows[selected_white], columns[selected_white]), 1)
        projected_pixels += int(np.count_nonzero(valid))
        processed_frames += 1
        if processed_frames % 100 == 0:
            print(
                f"processed {processed_frames} camera frames; "
                f"covered {np.count_nonzero(view_count)} map cells",
                flush=True,
            )

    observed_cells = int(np.count_nonzero(view_count))
    metadata: Dict[str, object] = {
        "source_bag": str(args.bag.resolve()),
        "source_map_yaml": str(args.map_yaml.resolve()),
        "source_occupancy_image": str(image_path.resolve()),
        "camera_topic": CAMERA_TOPIC,
        "camera_messages": camera_messages,
        "processed_camera_frames": processed_frames,
        "frame_step": args.frame_step,
        "pixel_stride": args.pixel_stride,
        "projected_ground_pixels": projected_pixels,
        "observed_map_cells": observed_cells,
        "map_cells": int(width * height),
        "map_coverage_fraction": observed_cells / float(width * height),
        "map_resolution_m": resolution,
        "map_width_px": width,
        "map_height_px": height,
        "max_tf_nearest_sample_age_ms": maximum_tf_age_ns / 1.0e6,
        "ground_projection_range_m": [
            args.min_ground_range_m,
            args.max_ground_range_m,
        ],
        "camera_model": {
            "width": 480,
            "height": 360,
            "horizontal_fov_rad": HORIZONTAL_FOV_RAD,
            "distortion_opencv_k1_k2_p1_p2_k3": DISTORTION.tolist(),
        },
        "tf_chain": [f"{parent}->{child}" for parent, child in TF_CHAIN],
        "navigation_warning": (
            "lidar_camera_fused.png is a visualization/reference layer, not a "
            "Nav2 occupancy map; keep using the original map.yaml and map.pgm."
        ),
    }
    save_outputs(
        args.output_dir,
        occupancy,
        color_sum,
        view_count,
        white_count,
        yellow_count,
        metadata,
    )
    return metadata


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frame-step", type=int, default=5)
    parser.add_argument("--pixel-stride", type=int, default=3)
    parser.add_argument("--min-ground-range-m", type=float, default=0.15)
    parser.add_argument("--max-ground-range-m", type=float, default=3.0)
    args = parser.parse_args(argv)
    if args.output_dir is None:
        args.output_dir = args.map_yaml.parent / "camera_fusion"
    if args.frame_step < 1 or args.pixel_stride < 1:
        parser.error("--frame-step and --pixel-stride must be positive")
    if not 0.0 <= args.min_ground_range_m < args.max_ground_range_m:
        parser.error("ground projection range is invalid")
    if not args.bag.is_dir():
        parser.error(f"bag directory does not exist: {args.bag}")
    if not args.map_yaml.is_file():
        parser.error(f"map yaml does not exist: {args.map_yaml}")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    metadata = build(args)
    print(
        f"saved camera fusion under {args.output_dir}; "
        f"frames={metadata['processed_camera_frames']} "
        f"coverage={float(metadata['map_coverage_fraction']) * 100.0:.1f}%"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
