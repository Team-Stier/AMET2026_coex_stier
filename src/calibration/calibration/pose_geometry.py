from __future__ import annotations

import math


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def map_from_odom_pose(
    map_base: tuple[float, float, float],
    odom_base: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return the 2D map->odom transform from simultaneous base poses."""

    map_x, map_y, map_yaw = map_base
    odom_x, odom_y, odom_yaw = odom_base
    yaw = normalize_angle(map_yaw - odom_yaw)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x = map_x - (cosine * odom_x - sine * odom_y)
    y = map_y - (sine * odom_x + cosine * odom_y)
    return x, y, yaw


def transform_pose_2d(
    pose: tuple[float, float, float],
    transform: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, yaw = pose
    tx, ty, transform_yaw = transform
    cosine = math.cos(transform_yaw)
    sine = math.sin(transform_yaw)
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
        normalize_angle(transform_yaw + yaw),
    )


def transform_from_local_correction(
    raw_pose: tuple[float, float, float],
    lateral_m: float,
    yaw_rad: float,
) -> tuple[float, float, float]:
    """Convert a local lateral/yaw correction into a persistent SE(2) transform."""

    x, y, raw_yaw = raw_pose
    corrected_pose = (
        x - math.sin(raw_yaw) * lateral_m,
        y + math.cos(raw_yaw) * lateral_m,
        normalize_angle(raw_yaw + yaw_rad),
    )
    return map_from_odom_pose(corrected_pose, raw_pose)
