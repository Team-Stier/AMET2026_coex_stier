#!/usr/bin/env python3
"""Accumulate only yellow and white camera lane pixels in SLAM map coordinates.

The generated PNG is a measured visual lane layer, not an occupancy map.  It
never uses route waypoints or visualization images as geometric input.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


TransformKey = Tuple[str, str]

CAMERA_TOPIC = "/camera/image_raw/compressed"
CAMERA_FRAME = "camera_optical_frame"
SIM_POSE_TOPIC = "/mapping/sim_pose"
SIM_POSE_KEY: TransformKey = ("sim_world", "base_footprint")
TF_TOPICS = {"/tf", "/tf_static"}

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


@dataclass(frozen=True)
class TransformSample:
    timestamp_ns: int
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


def stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion in TF")
    return quaternion / norm


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(first + fraction * (second - first))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) / sine * first
        + math.sin(fraction * angle) / sine * second
    )


def sample_matrix(sample: TransformSample) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_matrix(sample.quaternion_xyzw)
    matrix[:3, 3] = sample.translation
    return matrix


def transform_sample(item: object) -> TransformSample:
    translation = item.transform.translation
    rotation = item.transform.rotation
    return TransformSample(
        timestamp_ns=stamp_ns(item.header.stamp),
        translation=np.array(
            [float(translation.x), float(translation.y), float(translation.z)],
            dtype=np.float64,
        ),
        quaternion_xyzw=normalize_quaternion(
            np.array(
                [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)],
                dtype=np.float64,
            )
        ),
    )


def pose_sample(message: object) -> TransformSample:
    position = message.pose.position
    orientation = message.pose.orientation
    return TransformSample(
        timestamp_ns=stamp_ns(message.header.stamp),
        translation=np.array(
            [float(position.x), float(position.y), float(position.z)],
            dtype=np.float64,
        ),
        quaternion_xyzw=normalize_quaternion(
            np.array(
                [
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                ],
                dtype=np.float64,
            )
        ),
    )


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
            if key in wanted:
                samples[key].append(transform_sample(item))
    missing_pairs = wanted.difference(samples)
    if missing_pairs:
        formatted = [f"{parent}->{child}" for parent, child in sorted(missing_pairs)]
        raise RuntimeError(f"bag is missing required transforms: {formatted}")
    for values in samples.values():
        values.sort(key=lambda value: value.timestamp_ns)
    return dict(samples)


def collect_sim_pose(bag: Path) -> List[TransformSample]:
    reader = open_reader(bag)
    types = topic_types(reader)
    if SIM_POSE_TOPIC not in types:
        raise RuntimeError(
            f"bag is missing {SIM_POSE_TOPIC}; record it with the SIM mapping driver"
        )
    message_class = get_message(types[SIM_POSE_TOPIC])
    samples: List[TransformSample] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != SIM_POSE_TOPIC:
            continue
        message = deserialize_message(data, message_class)
        if message.header.frame_id != SIM_POSE_KEY[0]:
            raise RuntimeError(
                f"unexpected SIM pose frame {message.header.frame_id!r}; "
                f"expected {SIM_POSE_KEY[0]!r}"
            )
        samples.append(pose_sample(message))
    if not samples:
        raise RuntimeError(f"bag contains no messages on {SIM_POSE_TOPIC}")
    samples.sort(key=lambda value: value.timestamp_ns)
    return samples


class TransformLookup:
    """Interpolate dynamic TF while optionally holding final map alignment."""

    def __init__(
        self,
        samples: Dict[TransformKey, List[TransformSample]],
        fixed_final_keys: Sequence[TransformKey] = (),
    ) -> None:
        self.samples = samples
        self.fixed_final_keys = set(fixed_final_keys)
        self.times = {
            key: np.array([sample.timestamp_ns for sample in values], dtype=np.int64)
            for key, values in samples.items()
        }

    def interpolate(self, key: TransformKey, timestamp_ns: int) -> Tuple[np.ndarray, int]:
        values = self.samples[key]
        if key in self.fixed_final_keys:
            return sample_matrix(values[-1]), 0
        times = self.times[key]
        if len(values) == 1 or int(times[0]) == 0:
            return sample_matrix(values[0]), 0
        index = int(np.searchsorted(times, timestamp_ns))
        if index <= 0:
            return sample_matrix(values[0]), int(times[0]) - timestamp_ns
        if index >= len(values):
            return sample_matrix(values[-1]), timestamp_ns - int(times[-1])
        before = values[index - 1]
        after = values[index]
        interval = after.timestamp_ns - before.timestamp_ns
        if interval <= 0:
            return sample_matrix(before), abs(timestamp_ns - before.timestamp_ns)
        fraction = (timestamp_ns - before.timestamp_ns) / float(interval)
        sample = TransformSample(
            timestamp_ns=timestamp_ns,
            translation=(
                (1.0 - fraction) * before.translation + fraction * after.translation
            ),
            quaternion_xyzw=slerp(
                before.quaternion_xyzw,
                after.quaternion_xyzw,
                fraction,
            ),
        )
        nearest_age = min(
            timestamp_ns - before.timestamp_ns,
            after.timestamp_ns - timestamp_ns,
        )
        return sample_matrix(sample), nearest_age

    def map_from_camera(self, timestamp_ns: int) -> Tuple[np.ndarray, int]:
        result = np.eye(4, dtype=np.float64)
        maximum_support_age = 0
        for key in TF_CHAIN:
            transform, support_age = self.interpolate(key, timestamp_ns)
            result = result @ transform
            maximum_support_age = max(maximum_support_age, support_age)
        return result, maximum_support_age


def align_sim_world_to_final_map(
    samples: Dict[TransformKey, List[TransformSample]],
    sim_pose_samples: List[TransformSample],
) -> np.ndarray:
    lookup = TransformLookup(samples)
    last_sim_pose = sim_pose_samples[-1]
    final_map_from_odom = sample_matrix(samples[("map", "odom")][-1])
    odom_from_base, _ = lookup.interpolate(
        ("odom", "base_footprint"), last_sim_pose.timestamp_ns
    )
    map_from_base = final_map_from_odom @ odom_from_base
    world_from_base = sample_matrix(last_sim_pose)
    return map_from_base @ np.linalg.inv(world_from_base)


def map_from_sim_camera(
    timestamp_ns: int,
    transforms: TransformLookup,
    sim_poses: TransformLookup,
    map_from_world: np.ndarray,
) -> Tuple[np.ndarray, int]:
    world_from_base, maximum_support_age = sim_poses.interpolate(
        SIM_POSE_KEY, timestamp_ns
    )
    result = map_from_world @ world_from_base
    for key in TF_CHAIN[2:]:
        transform, support_age = transforms.interpolate(key, timestamp_ns)
        result = result @ transform
        maximum_support_age = max(maximum_support_age, support_age)
    return result, maximum_support_age


def load_map(map_yaml: Path) -> Tuple[np.ndarray, float, float, float, Path]:
    payload = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = Path(payload["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    image = np.array(Image.open(image_path).convert("L"), dtype=np.uint8)
    resolution = float(payload["resolution"])
    origin = payload["origin"]
    return image, resolution, float(origin[0]), float(origin[1]), image_path


def output_grid(
    occupancy: np.ndarray,
    source_resolution: float,
    lane_resolution: float,
) -> np.ndarray:
    source_height, source_width = occupancy.shape
    width = int(math.ceil(source_width * source_resolution / lane_resolution))
    height = int(math.ceil(source_height * source_resolution / lane_resolution))
    return cv2.resize(occupancy, (width, height), interpolation=cv2.INTER_NEAREST)


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
    u_values = np.arange(0, width, pixel_stride, dtype=np.int32)
    v_values = np.arange(height // 2, height, pixel_stride, dtype=np.int32)
    uu, vv = np.meshgrid(u_values, v_values)
    pixels = np.column_stack((uu.ravel(), vv.ravel())).astype(np.float64)
    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2), camera_matrix, DISTORTION
    ).reshape(-1, 2)
    rays = np.column_stack((normalized, np.ones(len(normalized)))).T
    return rays, uu.ravel(), vv.ravel()


def lane_masks(frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array([8, 80, 80]), np.array([40, 255, 255]))
    white = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([179, 55, 255]))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kernel)
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, kernel)
    return yellow > 0, white > 0


def remove_small_components(mask: np.ndarray, minimum_cells: int) -> np.ndarray:
    count, labels, statistics, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    output = np.zeros_like(mask, dtype=bool)
    for label in range(1, count):
        if int(statistics[label, cv2.CC_STAT_AREA]) >= minimum_cells:
            output |= labels == label
    return output


def save_outputs(
    output_dir: Path,
    occupancy: np.ndarray,
    view_count: np.ndarray,
    white_count: np.ndarray,
    yellow_count: np.ndarray,
    args: argparse.Namespace,
    metadata: Dict[str, object],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    yellow_ratio = np.divide(
        yellow_count, np.maximum(view_count, 1), dtype=np.float64
    )
    white_ratio = np.divide(
        white_count, np.maximum(view_count, 1), dtype=np.float64
    )
    known_free = occupancy >= 250
    yellow = (
        (yellow_count >= args.minimum_hits)
        & (yellow_ratio >= args.minimum_yellow_ratio)
        & known_free
    )
    white = (
        (white_count >= args.minimum_hits)
        & (white_ratio >= args.minimum_white_ratio)
        & known_free
    )
    yellow = remove_small_components(yellow, args.minimum_component_cells)
    white = remove_small_components(white, args.minimum_component_cells)
    conflict = yellow & white
    yellow[conflict & (white_ratio > yellow_ratio)] = False
    white[conflict & (yellow_ratio >= white_ratio)] = False

    lane_rgba = np.zeros((*occupancy.shape, 4), dtype=np.uint8)
    lane_rgba[white] = (255, 255, 255, 255)
    lane_rgba[yellow] = (255, 190, 0, 255)

    overlay_gray = np.full_like(occupancy, 140)
    overlay_gray[occupancy >= 250] = 70
    overlay_gray[occupancy < 100] = 0
    overlay = np.repeat(overlay_gray[:, :, None], 3, axis=2)
    overlay[white] = (255, 255, 255)
    overlay[yellow] = (255, 190, 0)

    Image.fromarray(lane_rgba, mode="RGBA").save(output_dir / "lane_layer.png")
    Image.fromarray(overlay, mode="RGB").save(output_dir / "lane_overlay.png")
    np.savez_compressed(
        output_dir / "lane_votes.npz",
        view_count=view_count,
        yellow_count=yellow_count,
        white_count=white_count,
        yellow_ratio=yellow_ratio,
        white_ratio=white_ratio,
    )
    metadata.update(
        {
            "yellow_lane_cells": int(np.count_nonzero(yellow)),
            "white_lane_cells": int(np.count_nonzero(white)),
            "minimum_hits": args.minimum_hits,
            "minimum_yellow_ratio": args.minimum_yellow_ratio,
            "minimum_white_ratio": args.minimum_white_ratio,
            "minimum_component_cells": args.minimum_component_cells,
        }
    )
    (output_dir / "lane_map_metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def build(args: argparse.Namespace) -> Dict[str, object]:
    source_map, source_resolution, origin_x, origin_y, image_path = load_map(
        args.map_yaml
    )
    occupancy = output_grid(source_map, source_resolution, args.lane_resolution_m)
    height, width = occupancy.shape
    transform_samples = collect_tf(args.bag)
    fixed_final_keys: Sequence[TransformKey] = ()
    if args.map_to_odom_mode == "final":
        fixed_final_keys = (("map", "odom"),)
    transforms = TransformLookup(
        transform_samples, fixed_final_keys=fixed_final_keys
    )
    sim_poses = None
    map_from_world = None
    sim_pose_samples = None
    if args.pose_source == "sim_ground_truth":
        sim_pose_samples = collect_sim_pose(args.bag)
        sim_poses = TransformLookup({SIM_POSE_KEY: sim_pose_samples})
        map_from_world = align_sim_world_to_final_map(
            transform_samples, sim_pose_samples
        )

    reader = open_reader(args.bag)
    types = topic_types(reader)
    if CAMERA_TOPIC not in types:
        raise RuntimeError(f"bag is missing {CAMERA_TOPIC}")
    camera_class = get_message(types[CAMERA_TOPIC])

    view_count = np.zeros((height, width), dtype=np.uint32)
    white_count = np.zeros((height, width), dtype=np.uint32)
    yellow_count = np.zeros((height, width), dtype=np.uint32)
    ray_cache: Tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
    camera_messages = 0
    processed_frames = 0
    rejected_tf_frames = 0
    projected_pixels = 0
    maximum_tf_support_age_ns = 0
    accepted_tf_ages_ns: List[int] = []
    maximum_tf_age_ns = int(args.max_tf_age_ms * 1.0e6)

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != CAMERA_TOPIC:
            continue
        camera_messages += 1
        if (camera_messages - 1) % args.frame_step:
            continue
        message = deserialize_message(data, camera_class)
        if message.header.frame_id != CAMERA_FRAME:
            raise RuntimeError(
                f"unexpected camera frame {message.header.frame_id!r}; "
                f"expected {CAMERA_FRAME!r}"
            )
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"failed to decode camera frame {camera_messages}")
        frame_height, frame_width = frame.shape[:2]
        if ray_cache is None:
            ray_cache = camera_rays(frame_width, frame_height, args.pixel_stride)
        rays_camera, pixel_u, pixel_v = ray_cache

        timestamp_ns = stamp_ns(message.header.stamp)
        if args.pose_source == "sim_ground_truth":
            assert sim_poses is not None and map_from_world is not None
            map_from_camera, tf_support_age = map_from_sim_camera(
                timestamp_ns, transforms, sim_poses, map_from_world
            )
        else:
            map_from_camera, tf_support_age = transforms.map_from_camera(timestamp_ns)
        maximum_tf_support_age_ns = max(maximum_tf_support_age_ns, tf_support_age)
        if tf_support_age > maximum_tf_age_ns:
            rejected_tf_frames += 1
            continue
        accepted_tf_ages_ns.append(tf_support_age)

        rotation = map_from_camera[:3, :3]
        camera_position = map_from_camera[:3, 3]
        rays_map = rotation @ rays_camera
        downward = rays_map[2] < -1.0e-3
        scale = np.full(rays_map.shape[1], np.nan, dtype=np.float64)
        scale[downward] = -camera_position[2] / rays_map[2, downward]
        points_x = camera_position[0] + rays_map[0] * scale
        points_y = camera_position[1] + rays_map[1] * scale
        horizontal_range = np.hypot(
            points_x - camera_position[0], points_y - camera_position[1]
        )
        finite_projection = downward & np.isfinite(scale)
        columns = np.full(rays_map.shape[1], -1, dtype=np.int64)
        map_y = np.full(rays_map.shape[1], -1, dtype=np.int64)
        columns[finite_projection] = np.floor(
            (points_x[finite_projection] - origin_x) / args.lane_resolution_m
        ).astype(np.int64)
        map_y[finite_projection] = np.floor(
            (points_y[finite_projection] - origin_y) / args.lane_resolution_m
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

        yellow_mask, white_mask = lane_masks(frame)
        sampled_yellow = yellow_mask[pixel_v, pixel_u]
        sampled_white = white_mask[pixel_v, pixel_u]
        valid_rows = rows[valid]
        valid_columns = columns[valid]
        np.add.at(view_count, (valid_rows, valid_columns), 1)
        selected_yellow = valid & sampled_yellow
        selected_white = valid & sampled_white
        np.add.at(
            yellow_count,
            (rows[selected_yellow], columns[selected_yellow]),
            1,
        )
        np.add.at(
            white_count,
            (rows[selected_white], columns[selected_white]),
            1,
        )
        projected_pixels += int(np.count_nonzero(valid))
        processed_frames += 1
        if processed_frames % 200 == 0:
            print(
                f"processed {processed_frames} camera frames; "
                f"observed {np.count_nonzero(view_count)} lane-map cells",
                flush=True,
            )

    observed_cells = int(np.count_nonzero(view_count))
    metadata: Dict[str, object] = {
        "source_bag": str(args.bag.resolve()),
        "source_map_yaml": str(args.map_yaml.resolve()),
        "source_occupancy_image": str(image_path.resolve()),
        "pose_source": args.pose_source,
        "sim_pose_topic": (
            SIM_POSE_TOPIC if args.pose_source == "sim_ground_truth" else None
        ),
        "sim_pose_messages": (
            len(sim_pose_samples) if sim_pose_samples is not None else 0
        ),
        "map_from_sim_world": (
            map_from_world.tolist() if map_from_world is not None else None
        ),
        "map_to_odom_mode": args.map_to_odom_mode,
        "camera_topic": CAMERA_TOPIC,
        "camera_messages": camera_messages,
        "processed_camera_frames": processed_frames,
        "rejected_tf_frames": rejected_tf_frames,
        "frame_step": args.frame_step,
        "pixel_stride": args.pixel_stride,
        "projected_ground_pixels": projected_pixels,
        "observed_lane_map_cells": observed_cells,
        "lane_map_cells": int(width * height),
        "lane_map_coverage_fraction": observed_cells / float(width * height),
        "source_map_resolution_m": source_resolution,
        "lane_resolution_m": args.lane_resolution_m,
        "lane_map_width_px": width,
        "lane_map_height_px": height,
        "max_tf_support_age_limit_ms": args.max_tf_age_ms,
        "max_tf_support_age_observed_ms": maximum_tf_support_age_ns / 1.0e6,
        "max_tf_support_age_accepted_ms": (
            max(accepted_tf_ages_ns, default=0) / 1.0e6
        ),
        "mean_tf_support_age_accepted_ms": (
            float(np.mean(accepted_tf_ages_ns)) / 1.0e6
            if accepted_tf_ages_ns
            else 0.0
        ),
        "ground_projection_range_m": [
            args.min_ground_range_m,
            args.max_ground_range_m,
        ],
        "tf_chain": [f"{parent}->{child}" for parent, child in TF_CHAIN],
        "geometry_note": (
            "Only measured yellow/white image pixels are accumulated. Route "
            "waypoints and visualization images are not geometric inputs. "
            + (
                "SIM ground-truth poses are rigidly aligned to the final map."
                if args.pose_source == "sim_ground_truth"
                else "Vehicle and camera transforms come from recorded TF."
            )
        ),
        "navigation_warning": (
            "lane_layer.png is a measured visual layer, not a Nav2 occupancy map; "
            "keep using the original map.yaml and map.pgm for occupancy."
        ),
    }
    save_outputs(
        args.output_dir,
        occupancy,
        view_count,
        white_count,
        yellow_count,
        args,
        metadata,
    )
    return metadata


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frame-step", type=int, default=2)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--lane-resolution-m", type=float, default=0.025)
    parser.add_argument("--min-ground-range-m", type=float, default=0.15)
    parser.add_argument("--max-ground-range-m", type=float, default=1.8)
    parser.add_argument("--max-tf-age-ms", type=float, default=75.0)
    parser.add_argument(
        "--pose-source",
        choices=("slam_tf", "sim_ground_truth"),
        default="slam_tf",
        help="select recorded SLAM TF or the SIM-only stable pose topic",
    )
    parser.add_argument(
        "--map-to-odom-mode",
        choices=("final", "interpolated"),
        default="interpolated",
        help=(
            "use the final SLAM map alignment for a saved map, or replay the "
            "time-varying alignment for diagnostics"
        ),
    )
    parser.add_argument("--minimum-hits", type=int, default=3)
    parser.add_argument("--minimum-yellow-ratio", type=float, default=0.06)
    parser.add_argument("--minimum-white-ratio", type=float, default=0.10)
    parser.add_argument("--minimum-component-cells", type=int, default=4)
    args = parser.parse_args(argv)
    if args.output_dir is None:
        args.output_dir = args.map_yaml.parent / "camera_lane_accumulation"
    if args.frame_step < 1 or args.pixel_stride < 1:
        parser.error("--frame-step and --pixel-stride must be positive")
    if args.lane_resolution_m <= 0.0:
        parser.error("--lane-resolution-m must be positive")
    if not 0.0 <= args.min_ground_range_m < args.max_ground_range_m:
        parser.error("ground projection range is invalid")
    if args.max_tf_age_ms < 0.0:
        parser.error("--max-tf-age-ms cannot be negative")
    if args.minimum_hits < 1 or args.minimum_component_cells < 1:
        parser.error("hit and component thresholds must be positive")
    if not 0.0 <= args.minimum_yellow_ratio <= 1.0:
        parser.error("--minimum-yellow-ratio must be between zero and one")
    if not 0.0 <= args.minimum_white_ratio <= 1.0:
        parser.error("--minimum-white-ratio must be between zero and one")
    if not args.bag.is_dir():
        parser.error(f"bag directory does not exist: {args.bag}")
    if not args.map_yaml.is_file():
        parser.error(f"map yaml does not exist: {args.map_yaml}")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    metadata = build(args)
    print(
        f"saved lane-only map under {args.output_dir}; "
        f"frames={metadata['processed_camera_frames']} "
        f"yellow_cells={metadata['yellow_lane_cells']} "
        f"white_cells={metadata['white_lane_cells']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
