#!/usr/bin/env python3
"""Build a training-ready YOLO dataset from data/raw/sim.

The source images are copied, never modified. Traffic-light state is derived
from the image pixels and the manually reviewed source folders; the simulator
state API is not used.
"""

from __future__ import annotations

import csv
import hashlib
import math
import re
import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


DATASET_ROOT = Path(__file__).resolve().parent
RAW_ROOT = DATASET_ROOT.parent / "raw" / "sim"

CLASS_IDS = {"red": 0, "yellow": 1, "green": 2}
CLASS_COLORS = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
}
DETECTION_THRESHOLDS = {"red": 20.0, "yellow": 20.0, "green": 18.0}
VERTICAL_POSITION = {
    "red": (0.23, 0.18),
    "yellow": (0.50, 0.20),
    "green": (0.68, 0.27),
}

# These six transition frames were visually reviewed and contain no visible
# traffic light. Yellow road markings can otherwise resemble an amber lamp.
MANUAL_EMPTY_TRANSITION_PATHS = {
    "brightness_high/run_05/frame_007_20260820T154031_649.jpg",
    "brightness_high/run_05/frame_032_20260820T154034_809.jpg",
    "brightness_medium/run_05/frame_008_20260820T154130_124.jpg",
    "brightness_low/run_03/frame_008_20260820T154213_985.jpg",
    "brightness_low/run_04/frame_032_20260820T154225_037.jpg",
    "brightness_low/run_05/frame_008_20260820T154229_956.jpg",
}


@dataclass
class Detection:
    class_name: str
    bbox: tuple[int, int, int, int]
    color_score: float
    housing_contrast: float


@dataclass
class Record:
    source: Path
    source_relative: Path
    source_hash: str
    split: str
    output_name: str
    detection: Detection | None
    decision_source: str


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def split_for_path(relative_path: Path) -> str:
    for part in relative_path.parts:
        match = re.fullmatch(r"(?:transition_)?run_(\d+)", part)
        if not match:
            continue
        run_number = int(match.group(1))
        if run_number <= 3:
            return "train"
        if run_number == 4:
            return "val"
        if run_number == 5:
            return "test"
    raise ValueError(f"Run number not found in {relative_path}")


def output_name_for_path(relative_path: Path) -> str:
    return "__".join(relative_path.parts)


def color_mask(hsv: np.ndarray, class_name: str) -> np.ndarray:
    # High saturation separates illuminated lamps from the pale simulator
    # grass. Housing validation below rejects yellow lane markings.
    if class_name == "green":
        return cv2.inRange(hsv, (35, 120, 20), (95, 255, 255))
    if class_name == "yellow":
        return cv2.inRange(hsv, (17, 120, 20), (40, 255, 255))
    lower_red = cv2.inRange(hsv, (0, 120, 20), (12, 255, 255))
    upper_red = cv2.inRange(hsv, (168, 120, 20), (179, 255, 255))
    return cv2.bitwise_or(lower_red, upper_red)


def component_candidates(
    image: np.ndarray, class_name: str
) -> list[tuple[float, tuple[int, int, int, int, int]]]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = color_mask(hsv, class_name)
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    expected_position, position_sigma = VERTICAL_POSITION[class_name]
    candidates: list[tuple[float, tuple[int, int, int, int, int]]] = []

    for x, y, width, height, area in stats[1:count]:
        if area < 3 or width < 2 or height < 2:
            continue
        if width > 100 or height > 100 or not 165 <= y <= 300:
            continue
        aspect = width / height
        fill = area / (width * height)
        if not 0.3 <= aspect <= 2.5 or fill < 0.25:
            continue

        diameter = max(width, height)
        center_y = y + height / 2
        relative_position = (center_y - 180) / (2.16 * diameter)
        position_score = math.exp(
            -0.5
            * ((relative_position - expected_position) / position_sigma) ** 2
        )
        candidates.append(
            (area * position_score, (x, y, width, height, area))
        )

    return candidates


def profile_edge(
    profile: np.ndarray,
    predicted: float,
    radius: float,
    left_edge: bool,
) -> tuple[int, float]:
    width = profile.shape[0]
    lower = max(1, int(math.floor(predicted - radius)))
    upper = min(width - 2, int(math.ceil(predicted + radius)))
    indices = list(range(lower, upper + 1))
    if not indices:
        return max(0, min(width - 1, int(round(predicted)))), 0.0

    if left_edge:
        contrasts = [profile[index - 1] - profile[index + 1] for index in indices]
    else:
        contrasts = [profile[index + 1] - profile[index - 1] for index in indices]
    best_offset = int(np.argmax(contrasts))
    return indices[best_offset], float(contrasts[best_offset])


def estimate_housing(
    image: np.ndarray,
    component: tuple[int, int, int, int, int],
) -> tuple[tuple[int, int, int, int], float]:
    x, y, width, height, _ = component
    image_height, image_width = image.shape[:2]
    diameter = max(width, height)
    center_x = x + width / 2
    top = 180

    value = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 2].astype(float)
    profile_bottom = min(
        image_height,
        top + max(8, int(round(2.25 * diameter))),
    )
    horizontal_profile = np.median(value[top:profile_bottom, :], axis=0)
    horizontal_profile = np.convolve(
        horizontal_profile, np.ones(3) / 3, mode="same"
    )

    expected_width = max(4.0, 1.35 * diameter)
    edge_radius = max(2.0, 0.28 * diameter)
    left_edge, left_contrast = profile_edge(
        horizontal_profile,
        center_x - expected_width / 2,
        edge_radius,
        left_edge=True,
    )
    right_edge, right_contrast = profile_edge(
        horizontal_profile,
        center_x + expected_width / 2,
        edge_radius,
        left_edge=False,
    )

    left = max(0, min(left_edge, x))
    right = min(image_width, max(right_edge + 1, x + width))
    if x <= 1:
        left = 0
    if x + width >= image_width - 1:
        right = image_width
    if right - left < 3:
        left = max(0, int(round(center_x - expected_width / 2)))
        right = min(image_width, int(round(center_x + expected_width / 2)))

    visible_width = max(1, right - left)
    vertical_profile = np.median(value[:, left:right], axis=1)
    vertical_profile = np.convolve(
        vertical_profile, np.ones(3) / 3, mode="same"
    )
    expected_bottom = top + 1.62 * visible_width
    lower = max(y + height, int(expected_bottom - 0.35 * visible_width), top + 4)
    upper = min(
        image_height - 2,
        int(expected_bottom + 0.35 * visible_width),
    )
    indices = list(range(lower, upper + 1))
    if indices:
        bottom_contrasts = [
            vertical_profile[index + 1] - vertical_profile[index - 1]
            for index in indices
        ]
        bottom = indices[int(np.argmax(bottom_contrasts))] + 2
    else:
        bottom = min(image_height, int(round(expected_bottom)))
    bottom = min(image_height, max(y + height, bottom, top + 3))

    near_image_edge = x < 2 * diameter or x + width > image_width - 2 * diameter
    if near_image_edge:
        housing_contrast = max(left_contrast, right_contrast)
    else:
        housing_contrast = min(left_contrast, right_contrast)
    return (left, top, right, bottom), float(housing_contrast)


def find_signal(image: np.ndarray, class_name: str) -> Detection | None:
    ranked: list[tuple[float, float, tuple[int, int, int, int]]] = []
    for color_score, component in component_candidates(image, class_name):
        bbox, housing_contrast = estimate_housing(image, component)
        if housing_contrast < 5.0:
            continue
        combined_score = color_score * (1.0 + housing_contrast / 20.0)
        ranked.append((combined_score, color_score, bbox))

    if not ranked:
        return None
    _, color_score, bbox = max(ranked)
    if color_score < DETECTION_THRESHOLDS[class_name]:
        return None

    # Recalculate the chosen housing contrast for audit output.
    chosen_contrast = 0.0
    for candidate_score, component in component_candidates(image, class_name):
        candidate_bbox, contrast = estimate_housing(image, component)
        if candidate_bbox == bbox and candidate_score == color_score:
            chosen_contrast = contrast
            break
    return Detection(class_name, bbox, color_score, chosen_contrast)


def build_hash_class_map(files: list[Path]) -> dict[str, str | None]:
    mapping: dict[str, str | None] = {}
    for class_name in ("green", "yellow", "red"):
        for path in files:
            if path.relative_to(RAW_ROOT).parts[0] != class_name:
                continue
            digest = file_sha256(path)
            previous = mapping.get(digest)
            if previous is not None and previous != class_name:
                raise ValueError(f"Conflicting color folders for hash {digest}")
            mapping[digest] = class_name

    # An unclear-only image remains a negative sample. If the exact same pixels
    # also occur in a reviewed color folder, keep the visual class so identical
    # images cannot receive contradictory training targets.
    for path in files:
        if path.relative_to(RAW_ROOT).parts[0] == "unclear":
            mapping.setdefault(file_sha256(path), None)
    return mapping


def decide_record(
    path: Path,
    digest: str,
    hash_class_map: dict[str, str | None],
) -> tuple[Detection | None, str]:
    relative = path.relative_to(RAW_ROOT)
    top_level = relative.parts[0]
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f"Could not decode {path}")
    if image.shape[:2] != (360, 480):
        raise ValueError(f"Unexpected image size {image.shape[:2]} for {path}")

    if top_level in CLASS_IDS:
        detection = find_signal(image, top_level)
        source = f"{top_level}_folder"
        if detection is None:
            source = f"no_visible_signal_in_{top_level}_folder"
        return detection, source

    if top_level == "unclear":
        duplicate_class = hash_class_map.get(digest)
        if duplicate_class is not None:
            detection = find_signal(image, duplicate_class)
            if detection is not None:
                return detection, f"unclear_duplicate_matches_{duplicate_class}"
        return None, "unclear_folder_empty"

    transition_relative = Path(*relative.parts[1:]).as_posix()
    if transition_relative in MANUAL_EMPTY_TRANSITION_PATHS:
        return None, "transition_visual_review_empty"

    if digest in hash_class_map:
        class_name = hash_class_map[digest]
        if class_name is None:
            return None, "transition_hash_matches_unclear"
        detection = find_signal(image, class_name)
        source = f"transition_hash_matches_{class_name}"
        if detection is None:
            source = f"transition_hash_{class_name}_but_no_visible_signal"
        return detection, source

    detections = {
        class_name: find_signal(image, class_name)
        for class_name in CLASS_IDS
    }
    available = [detection for detection in detections.values() if detection]
    if not available:
        return None, "transition_visual_review_empty"
    detection = max(
        available,
        key=lambda item: item.color_score / DETECTION_THRESHOLDS[item.class_name],
    )
    return detection, "transition_visual_review"


def yolo_values(
    bbox: tuple[int, int, int, int],
) -> tuple[float, float, float, float]:
    left, top, right, bottom = bbox
    width = right - left
    height = bottom - top
    return (
        (left + right) / 2 / 480,
        (top + bottom) / 2 / 360,
        width / 480,
        height / 360,
    )


def ensure_clean_output() -> None:
    for directory_name in ("images", "labels", "review"):
        directory = DATASET_ROOT / directory_name
        if directory.exists() and any(directory.rglob("*")):
            raise RuntimeError(f"Refusing to overwrite non-empty {directory}")
    if (DATASET_ROOT / "manifest.csv").exists():
        raise RuntimeError("Refusing to overwrite existing manifest.csv")


def write_contact_sheets(records: list[Record]) -> None:
    review_root = DATASET_ROOT / "review" / "contact_sheets"
    review_root.mkdir(parents=True, exist_ok=True)
    page_size = 50
    columns = 5
    tile_width, tile_height, header_height = 240, 180, 22

    for page_number, start in enumerate(range(0, len(records), page_size), start=1):
        page_records = records[start : start + page_size]
        tiles: list[np.ndarray] = []
        for record in page_records:
            image = cv2.imread(str(record.source))
            if record.detection:
                left, top, right, bottom = record.detection.bbox
                color = CLASS_COLORS[record.detection.class_name]
                cv2.rectangle(image, (left, top), (right - 1, bottom - 1), color, 2)
                state = record.detection.class_name
            else:
                state = "empty"
            resized = cv2.resize(
                image,
                (tile_width, tile_height),
                interpolation=cv2.INTER_AREA,
            )
            tile = cv2.copyMakeBorder(
                resized,
                header_height,
                0,
                0,
                0,
                cv2.BORDER_CONSTANT,
                value=(255, 255, 255),
            )
            title = f"{start + len(tiles) + 1:04d} {state} {record.source_relative.parent.name}"
            cv2.putText(
                tile,
                title,
                (4, 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
            tiles.append(tile)

        rows = math.ceil(len(tiles) / columns)
        blank = np.full_like(tiles[0], 255)
        tiles.extend([blank] * (rows * columns - len(tiles)))
        sheet = np.vstack(
            [
                np.hstack(tiles[row * columns : (row + 1) * columns])
                for row in range(rows)
            ]
        )
        cv2.imwrite(
            str(review_root / f"page_{page_number:03d}.jpg"),
            sheet,
            [cv2.IMWRITE_JPEG_QUALITY, 94],
        )


def main() -> None:
    ensure_clean_output()
    files = sorted(RAW_ROOT.rglob("*.jpg"))
    if len(files) != 2019:
        raise RuntimeError(f"Expected 2019 source images, found {len(files)}")
    hash_class_map = build_hash_class_map(files)

    records: list[Record] = []
    hash_splits: dict[str, set[str]] = defaultdict(set)
    for path in files:
        relative = path.relative_to(RAW_ROOT)
        digest = file_sha256(path)
        split = split_for_path(relative)
        detection, decision_source = decide_record(path, digest, hash_class_map)
        records.append(
            Record(
                source=path,
                source_relative=relative,
                source_hash=digest,
                split=split,
                output_name=output_name_for_path(relative),
                detection=detection,
                decision_source=decision_source,
            )
        )
        hash_splits[digest].add(split)

    cross_split_hashes = {
        digest: splits for digest, splits in hash_splits.items() if len(splits) > 1
    }
    if cross_split_hashes:
        raise RuntimeError(f"Exact duplicate leakage detected: {cross_split_hashes}")

    # Exact duplicate pixels must never receive conflicting annotations.
    hash_annotations: dict[str, set[tuple[str | None, tuple[int, int, int, int] | None]]] = defaultdict(set)
    for record in records:
        if record.detection:
            value = (record.detection.class_name, record.detection.bbox)
        else:
            value = (None, None)
        hash_annotations[record.source_hash].add(value)
    conflicts = {
        digest: values
        for digest, values in hash_annotations.items()
        if len(values) > 1
    }
    if conflicts:
        raise RuntimeError(f"Conflicting annotations for identical images: {conflicts}")

    for split in ("train", "val", "test"):
        (DATASET_ROOT / "images" / split).mkdir(parents=True, exist_ok=True)
        (DATASET_ROOT / "labels" / split).mkdir(parents=True, exist_ok=True)

    manifest_path = DATASET_ROOT / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "dataset_image",
                "split",
                "source_relative_path",
                "source_sha256",
                "annotation_class",
                "class_id",
                "x_min",
                "y_min",
                "x_max",
                "y_max",
                "yolo_x_center",
                "yolo_y_center",
                "yolo_width",
                "yolo_height",
                "decision_source",
            ]
        )

        for record in records:
            image_target = DATASET_ROOT / "images" / record.split / record.output_name
            label_target = (
                DATASET_ROOT
                / "labels"
                / record.split
                / f"{Path(record.output_name).stem}.txt"
            )
            shutil.copy2(record.source, image_target)

            manifest_values: list[str | int | float] = [
                record.output_name,
                record.split,
                record.source_relative.as_posix(),
                record.source_hash,
            ]
            if record.detection:
                class_name = record.detection.class_name
                class_id = CLASS_IDS[class_name]
                left, top, right, bottom = record.detection.bbox
                normalized = yolo_values(record.detection.bbox)
                label_target.write_text(
                    f"{class_id} " + " ".join(f"{value:.6f}" for value in normalized) + "\n",
                    encoding="utf-8",
                )
                manifest_values.extend(
                    [
                        class_name,
                        class_id,
                        left,
                        top,
                        right,
                        bottom,
                        *[f"{value:.6f}" for value in normalized],
                    ]
                )
            else:
                label_target.write_text("", encoding="utf-8")
                manifest_values.extend(["", "", "", "", "", "", "", "", "", ""])
            manifest_values.append(record.decision_source)
            writer.writerow(manifest_values)

    (DATASET_ROOT / "data.yaml").write_text(
        """train: images/train
val: images/val
test: images/test

names:
  0: red
  1: yellow
  2: green
""",
        encoding="utf-8",
    )

    split_counts = Counter(record.split for record in records)
    class_counts = Counter(
        record.detection.class_name if record.detection else "empty"
        for record in records
    )
    (DATASET_ROOT / "README.md").write_text(
        f"""# Traffic-light YOLO dataset

This dataset contains all {len(records)} JPEG images from `data/raw/sim`.
The source images were copied and were not modified.

## Split policy

- `train`: run_01 through run_03 ({split_counts['train']} images)
- `val`: run_04 ({split_counts['val']} images)
- `test`: run_05 ({split_counts['test']} images)

Sequential frames stay in the same split. Exact duplicate hashes were checked
and do not cross split boundaries.

## Classes

- `0`: red ({class_counts['red']} labels)
- `1`: yellow ({class_counts['yellow']} labels)
- `2`: green ({class_counts['green']} labels)
- empty/background: {class_counts['empty']} images with zero-byte label files

Bounding boxes cover the full visible traffic-light housing. Images that are
visually unclear are represented by empty label files. One file placed in the
`unclear` source folder is an exact duplicate of a clearly illuminated green
frame; it is labeled green to prevent contradictory targets for identical
pixels. `manifest.csv` records the source path, hash, split, class, pixel box,
normalized YOLO values, and the decision source for every image.

The `review/contact_sheets` directory is for quality assurance only and is not
used by YOLO training.
""",
        encoding="utf-8",
    )

    write_contact_sheets(records)
    print(f"images={len(records)}")
    print(f"split_counts={dict(split_counts)}")
    print(f"class_counts={dict(class_counts)}")


if __name__ == "__main__":
    main()
