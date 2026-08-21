#!/usr/bin/env python3
"""Keep yellow and white track lines from a simulator screenshot.

The color output preserves the source pixels on a black background.  A second
binary output writes both line colors as white, which is convenient for later
map processing.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def bounded_int(minimum: int, maximum: int):
    """Return an argparse converter for a bounded integer."""

    def convert(value: str) -> int:
        number = int(value)
        if not minimum <= number <= maximum:
            raise argparse.ArgumentTypeError(
                f"expected an integer in [{minimum}, {maximum}], got {number}"
            )
        return number

    return convert


def remove_small_components(mask: np.ndarray, minimum_area: int) -> np.ndarray:
    """Remove connected foreground components smaller than ``minimum_area``."""

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
    """Return cleaned yellow and white masks for a BGR screenshot."""

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

    yellow = remove_small_components(yellow, yellow_min_area)
    white = remove_small_components(white, white_min_area)
    return yellow, white


def write_outputs(
    image: np.ndarray,
    yellow_mask: np.ndarray,
    white_mask: np.ndarray,
    color_path: Path,
    binary_path: Path,
) -> None:
    """Write a source-color result and a white-on-black binary result."""

    combined = cv2.bitwise_or(yellow_mask, white_mask)
    color = cv2.bitwise_and(image, image, mask=combined)

    color_path.parent.mkdir(parents=True, exist_ok=True)
    binary_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(color_path), color):
        raise OSError(f"failed to write {color_path}")
    if not cv2.imwrite(str(binary_path), combined):
        raise OSError(f"failed to write {binary_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="simulator screenshot")
    parser.add_argument(
        "--color-output",
        type=Path,
        default=Path("sim_map_lines_color.png"),
        help="yellow/white source pixels on black (default: %(default)s)",
    )
    parser.add_argument(
        "--binary-output",
        type=Path,
        default=Path("sim_map_lines_binary.png"),
        help="both line colors as white on black (default: %(default)s)",
    )

    hsv = parser.add_argument_group("HSV thresholds (OpenCV ranges: H 0-179, S/V 0-255)")
    hsv.add_argument("--yellow-h-min", type=bounded_int(0, 179), default=14)
    hsv.add_argument("--yellow-h-max", type=bounded_int(0, 179), default=32)
    hsv.add_argument("--yellow-s-min", type=bounded_int(0, 255), default=135)
    hsv.add_argument("--yellow-v-min", type=bounded_int(0, 255), default=145)
    hsv.add_argument("--white-s-max", type=bounded_int(0, 255), default=55)
    hsv.add_argument("--white-v-min", type=bounded_int(0, 255), default=205)
    hsv.add_argument("--white-min-area", type=bounded_int(1, 1_000_000), default=40)
    hsv.add_argument("--yellow-min-area", type=bounded_int(1, 1_000_000), default=2)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image = cv2.imread(str(args.input), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"could not read input image: {args.input}")
    if args.yellow_h_min > args.yellow_h_max:
        raise SystemExit("--yellow-h-min must be <= --yellow-h-max")

    yellow, white = extract_line_masks(
        image,
        yellow_h_min=args.yellow_h_min,
        yellow_h_max=args.yellow_h_max,
        yellow_s_min=args.yellow_s_min,
        yellow_v_min=args.yellow_v_min,
        white_s_max=args.white_s_max,
        white_v_min=args.white_v_min,
        white_min_area=args.white_min_area,
        yellow_min_area=args.yellow_min_area,
    )
    write_outputs(
        image,
        yellow,
        white,
        args.color_output,
        args.binary_output,
    )

    total = image.shape[0] * image.shape[1]
    yellow_count = int(cv2.countNonZero(yellow))
    white_count = int(cv2.countNonZero(white))
    print(f"input: {args.input} ({image.shape[1]}x{image.shape[0]})")
    print(f"yellow pixels: {yellow_count} ({yellow_count / total:.3%})")
    print(f"white pixels: {white_count} ({white_count / total:.3%})")
    print(f"color output: {args.color_output}")
    print(f"binary output: {args.binary_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
