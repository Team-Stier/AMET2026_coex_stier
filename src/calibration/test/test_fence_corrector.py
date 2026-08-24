import math

import numpy as np
import pytest

from calibration.fence_corrector import FenceOdomCorrector, FenceReference


def rectangle_points() -> np.ndarray:
    x = np.linspace(0.15, 11.85, 160)
    y = np.linspace(0.15, 6.85, 100)
    return np.vstack(
        (
            np.column_stack((x, np.zeros_like(x))),
            np.column_stack((np.full_like(y, 12.0), y)),
            np.column_stack((x, np.full_like(x, 7.0))),
            np.column_stack((np.zeros_like(y), y)),
        )
    )


def points_in_base(world_points: np.ndarray, pose) -> np.ndarray:
    x, y, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    return (world_points - np.asarray([x, y])) @ rotation


def test_recovers_full_pose_error_from_four_fences():
    truth = (6.0, 3.5, 0.40)
    initial = (6.12, 3.42, 0.46)
    observed = points_in_base(rectangle_points(), truth)
    corrector = FenceOdomCorrector()

    result = corrector.estimate(
        observed, FenceReference.rectangle(0.0, 12.0, 0.0, 7.0), initial
    )

    assert result is not None
    assert result.measured_pose == pytest.approx(truth, abs=1.0e-5)
    assert result.rms_error_m < 1.0e-5
    assert all(count >= 90 for count in result.segment_match_counts)


def test_rejects_interior_cones_and_keeps_fence_solution():
    truth = (4.0, 2.0, -0.7)
    fence = rectangle_points()
    angles = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False)
    cones = np.column_stack((6.0 + 0.15 * np.cos(angles), 3.5 + 0.15 * np.sin(angles)))
    observed = points_in_base(np.vstack((fence, cones)), truth)
    corrector = FenceOdomCorrector(maximum_match_distance_m=0.20)

    result = corrector.estimate(
        observed,
        FenceReference.rectangle(0.0, 12.0, 0.0, 7.0),
        (3.92, 2.06, -0.66),
    )

    assert result is not None
    assert result.measured_pose == pytest.approx(truth, abs=1.0e-5)
    assert result.match_count == len(fence)


def test_rejects_pose_when_only_one_fence_is_observed():
    world = np.column_stack((np.linspace(0.2, 11.8, 200), np.zeros(200)))
    observed = points_in_base(world, (6.0, 3.5, 0.0))
    corrector = FenceOdomCorrector(minimum_segments=3)

    assert (
        corrector.estimate(
            observed,
            FenceReference.rectangle(0.0, 12.0, 0.0, 7.0),
            (6.0, 3.5, 0.0),
        )
        is None
    )


def test_transformed_reference_preserves_rectangle_dimensions():
    reference = FenceReference.rectangle(0.0, 12.0, 0.0, 7.0)
    yaw = 0.8
    transform = np.asarray(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0, 2.0],
            [math.sin(yaw), math.cos(yaw), 0.0, -1.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    transformed = reference.transformed(transform)

    np.testing.assert_allclose(transformed.lengths, [12.0, 7.0, 12.0, 7.0])
