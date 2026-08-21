#!/usr/bin/env python3
"""Keep only yellow and white track pixels from a simulator screenshot."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def remove_small_components(mask: np.ndarray, minimum_area: int) -> np.ndarray:
    if minimum_area <= 1:
        return mask
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    result = np.zeros_like(mask)
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= minimum_area:
            result[labels == label] = 255
    return result


def extract_line_masks(
    image: np.ndarray,
    *,
    yellow_h_min: int = 14,
    yellow_h_max: int = 32,
    yellow_s_min: int = 135,
    yellow_v_min: int = 145,
    white_s_max: int = 55,
    white_v_min: int = 205,
    white_min_area: int = 40,
    yellow_min_area: int = 2,
) -> tuple[np.ndarray, np.ndarray]:
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("input must be a non-empty BGR color image")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(
        hsv,
        np.array([yellow_h_min, yellow_s_min, yellow_v_min], dtype=np.uint8),
        np.array([yellow_h_max, 255, 255], dtype=np.uint8),
    )
    white = cv2.inRange(
        hsv,
        np.array([0, 0, white_v_min], dtype=np.uint8),
        np.array([179, white_s_max, 255], dtype=np.uint8),
    )
    return (
        remove_small_components(yellow, yellow_min_area),
        remove_small_components(white, white_min_area),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument(
        "--color-output", type=Path, default=Path("docs/sim_map_lines_color.png")
    )
    parser.add_argument(
        "--binary-output", type=Path, default=Path("docs/sim_map_lines_binary.png")
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image = cv2.imread(str(args.input), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"could not read input image: {args.input}")
    yellow, white = extract_line_masks(image)
    combined = cv2.bitwise_or(yellow, white)
    color = cv2.bitwise_and(image, image, mask=combined)
    args.color_output.parent.mkdir(parents=True, exist_ok=True)
    args.binary_output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.color_output), color):
        raise OSError(f"failed to write {args.color_output}")
    if not cv2.imwrite(str(args.binary_output), combined):
        raise OSError(f"failed to write {args.binary_output}")
    print(args.color_output)
    print(args.binary_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
