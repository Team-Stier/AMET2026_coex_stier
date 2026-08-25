import math

import numpy as np
import pytest

from calibration.wall_fitter import (
    RectangleWallFitter,
    ego_pose_from_lidar,
    lidar_pose_from_ego,
    normalize_angle,
)


def rectangle_points() -> np.ndarray:
    horizontal = np.linspace(0.1, 11.9, 120)
    vertical = np.linspace(0.1, 6.9, 70)
    return np.vstack(
        (
            np.column_stack((horizontal, np.zeros_like(horizontal))),
            np.column_stack((np.full_like(vertical, 12.0), vertical)),
            np.column_stack((horizontal, np.full_like(horizontal, 7.0))),
            np.column_stack((np.zeros_like(vertical), vertical)),
        )
    )


def in_lidar_frame(world_points: np.ndarray, pose) -> np.ndarray:
    x, y, yaw = pose
    rotation = np.asarray(
        ((math.cos(yaw), -math.sin(yaw)), (math.sin(yaw), math.cos(yaw)))
    )
    return (world_points - np.asarray((x, y))) @ rotation


def test_recovers_pose_while_ignoring_interior_returns():
    truth = (1.52, 3.31, -1.48)
    walls = rectangle_points()
    interior_returns = np.column_stack(
        (np.linspace(3.0, 9.0, 80), 3.5 + 0.2 * np.sin(np.arange(80)))
    )
    points = in_lidar_frame(np.vstack((walls, interior_returns)), truth)
    fitter = RectangleWallFitter((0.0, 12.0, 0.0, 7.0))

    result = fitter.fit(points, (1.4, 3.4, -math.pi / 2.0))

    assert result is not None
    assert result.pose == pytest.approx(truth, abs=1.0e-5)
    assert result.match_count == len(walls)
    assert result.rms_error_m < 1.0e-5


def test_initial_pose_selects_the_rectangle_symmetry():
    truth = (2.0, 2.5, 0.3)
    symmetric = (10.0, 4.5, normalize_angle(0.3 + math.pi))
    points = in_lidar_frame(rectangle_points(), truth)
    fitter = RectangleWallFitter((0.0, 12.0, 0.0, 7.0))

    truth_result = fitter.fit(points, truth)
    symmetric_result = fitter.fit(points, symmetric)

    assert truth_result is not None
    assert symmetric_result is not None
    assert truth_result.pose == pytest.approx(truth, abs=1.0e-8)
    assert symmetric_result.pose == pytest.approx(symmetric, abs=1.0e-8)


def test_first_seed_selects_the_requested_symmetric_branch():
    truth = (2.0, 2.5, 0.3)
    symmetric = (10.0, 4.5, normalize_angle(0.3 + math.pi))
    points = in_lidar_frame(rectangle_points(), truth)
    fitter = RectangleWallFitter((0.0, 12.0, 0.0, 7.0))

    prior_first = fitter.fit_first(points, (truth, symmetric), 0.01)
    previous_first = fitter.fit_first(points, (symmetric, truth), 0.01)

    assert prior_first is not None
    assert previous_first is not None
    assert prior_first.pose == pytest.approx(truth, abs=1.0e-8)
    assert previous_first.pose == pytest.approx(symmetric, abs=1.0e-8)


def test_converts_lidar_origin_to_ego_with_rotated_offset():
    expected_ego = (1.0, 2.027, math.pi / 2.0)
    ego = ego_pose_from_lidar((1.0, 2.0, math.pi / 2.0), -0.027)

    assert ego == pytest.approx(expected_ego)
    assert lidar_pose_from_ego(ego, -0.027) == pytest.approx(
        (1.0, 2.0, math.pi / 2.0)
    )


def test_rejects_a_single_visible_wall():
    world_points = np.column_stack((np.linspace(0.1, 11.9, 120), np.zeros(120)))
    fitter = RectangleWallFitter((0.0, 12.0, 0.0, 7.0))

    points = in_lidar_frame(world_points, (6.0, 3.5, 0.0))

    assert fitter.fit(points, (6.0, 3.5, 0.0)) is None


def test_recovers_pose_from_two_perpendicular_walls():
    truth = (1.52, 3.31, -1.48)
    horizontal = np.linspace(0.1, 11.9, 120)
    vertical = np.linspace(0.1, 6.9, 70)
    adjacent_walls = np.vstack(
        (
            np.column_stack((horizontal, np.zeros_like(horizontal))),
            np.column_stack((np.zeros_like(vertical), vertical)),
        )
    )
    fitter = RectangleWallFitter(
        (0.0, 12.0, 0.0, 7.0), minimum_walls=2
    )

    result = fitter.fit(
        in_lidar_frame(adjacent_walls, truth),
        (1.4, 3.4, -math.pi / 2.0),
    )

    assert result is not None
    assert np.linalg.norm(np.asarray(result.pose[:2]) - truth[:2]) < 0.08
    assert abs(normalize_angle(result.pose[2] - truth[2])) < 0.02
    assert result.rms_error_m < 0.08


def test_rejects_two_parallel_walls():
    horizontal = np.linspace(0.1, 11.9, 120)
    parallel_walls = np.vstack(
        (
            np.column_stack((horizontal, np.zeros_like(horizontal))),
            np.column_stack((horizontal, np.full_like(horizontal, 7.0))),
        )
    )
    fitter = RectangleWallFitter(
        (0.0, 12.0, 0.0, 7.0), minimum_walls=2
    )

    points = in_lidar_frame(parallel_walls, (6.0, 3.5, 0.0))

    assert fitter.fit(points, (6.0, 3.5, 0.0)) is None
