"""Geometry helpers shared by ROS-independent controller modules."""

import math
from typing import Tuple

from .models import PathPoint


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
    """Return Euclidean distance between two planar positions."""

    return math.hypot(x2 - x1, y2 - y1)


def as_path_point(value: Tuple[float, float]) -> PathPoint:
    """Convert a simple (x, y) pair to a PathPoint."""

    if len(value) != 2:
        raise ValueError("each path point must contain exactly x and y")
    return PathPoint(float(value[0]), float(value[1]))
