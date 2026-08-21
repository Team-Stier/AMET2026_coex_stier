import numpy as np

from calibration.lane_map import LaneReference
from calibration.odom_corrector import (
    LaneOdomCorrector,
    fit_local_line_directions,
    transform_points,
)


def test_estimates_lateral_and_yaw_error_against_centerline():
    x = np.linspace(0.2, 3.0, 80)
    reference = np.column_stack((x, np.zeros_like(x)))
    observed_base = reference.copy()
    corrector = LaneOdomCorrector(smoothing_alpha=1.0)

    result = corrector.estimate(
        observed_base, reference, odom_x=0.0, odom_y=0.10, odom_yaw=0.05
    )

    assert result is not None
    np.testing.assert_allclose(result.lateral_m, -0.10, atol=0.015)
    np.testing.assert_allclose(result.yaw_rad, -0.05, atol=0.015)


def test_rejects_reference_that_is_too_far_away():
    x = np.linspace(0.2, 3.0, 30)
    observed = np.column_stack((x, np.zeros_like(x)))
    reference = np.column_stack((x, np.full_like(x, 2.0)))
    corrector = LaneOdomCorrector()

    assert corrector.estimate(observed, reference, 0.0, 0.0, 0.0) is None


def test_transforms_map_points_into_odom_frame():
    points = np.array([[0.0, 0.0], [1.0, 0.0]])
    transform = np.array(
        [[0.0, -1.0, 0.0, 2.0], [1.0, 0.0, 0.0, 3.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    np.testing.assert_allclose(transform_points(points, transform), [[2.0, 3.0], [2.0, 4.0]])


def test_fits_different_directions_for_short_local_sections():
    horizontal = np.column_stack((np.linspace(0.0, 0.4, 9), np.zeros(9)))
    vertical = np.column_stack((np.full(9, 1.0), np.linspace(0.0, 0.4, 9)))
    points = np.vstack((horizontal, vertical))

    directions, valid = fit_local_line_directions(points, 0.16, 3)

    assert np.all(valid)
    assert np.median(np.abs(directions[:9, 0])) > 0.99
    assert np.median(np.abs(directions[9:, 1])) > 0.99


def test_rejects_nearby_points_when_local_line_direction_disagrees_with_map():
    observed = np.column_stack((np.full(20, 0.1), np.linspace(-0.3, 0.3, 20)))
    reference_points = np.column_stack((np.linspace(-0.5, 0.5, 50), np.zeros(50)))
    reference = LaneReference(
        points=reference_points,
        tangents=np.tile(np.array([[1.0, 0.0]]), (len(reference_points), 1)),
    )
    corrector = LaneOdomCorrector(
        minimum_matches=8,
        smoothing_alpha=1.0,
        maximum_tangent_angle_difference_rad=0.25,
    )

    assert corrector.estimate(observed, reference, 0.0, 0.0, 0.0) is None
