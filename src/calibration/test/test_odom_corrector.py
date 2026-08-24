import numpy as np

from calibration.lane_map import LaneReference
from calibration.odom_corrector import (
    CorrectionQualityGate,
    CorrectionResult,
    LaneOdomCorrector,
    fit_local_line_directions,
    is_meaningful_correction,
    transform_points,
)


def test_quality_gate_requires_motion_and_consistent_measurements():
    gate = CorrectionQualityGate(
        minimum_speed_m_s=0.10,
        maximum_rms_error_m=0.08,
        maximum_abs_lateral_m=0.19,
        maximum_abs_yaw_rad=0.11,
        maximum_lateral_jump_m=0.05,
        maximum_yaw_jump_rad=0.04,
        required_consistent_measurements=3,
    )
    correction = CorrectionResult(0.04, 0.01, 0.02, 20)

    assert not gate.accept(correction, 0.02)
    assert not gate.accept(correction, 0.50)
    assert not gate.accept(CorrectionResult(0.045, 0.012, 0.02, 20), 0.50)
    assert gate.accept(CorrectionResult(0.043, 0.011, 0.02, 20), 0.50)


def test_quality_gate_resets_after_outlier_or_saturation():
    gate = CorrectionQualityGate(
        minimum_speed_m_s=0.10,
        maximum_rms_error_m=0.08,
        maximum_abs_lateral_m=0.19,
        maximum_abs_yaw_rad=0.11,
        maximum_lateral_jump_m=0.05,
        maximum_yaw_jump_rad=0.04,
        required_consistent_measurements=2,
    )
    valid = CorrectionResult(0.04, 0.01, 0.02, 20)

    assert not gate.accept(valid, 0.50)
    assert not gate.accept(CorrectionResult(0.20, 0.01, 0.02, 20), 0.50)
    assert not gate.accept(valid, 0.50)
    assert gate.accept(valid, 0.50)


def test_zero_lateral_holds_persistent_correction_target():
    zero = CorrectionResult(0.0, 0.0, 0.01, 30)
    meaningful = CorrectionResult(0.011, 0.0, 0.01, 30)

    assert not is_meaningful_correction(
        zero, minimum_lateral_m=0.01, use_yaw=False
    )
    assert is_meaningful_correction(
        meaningful, minimum_lateral_m=0.01, use_yaw=False
    )


def test_quality_gate_can_disable_absolute_lateral_limit():
    gate = CorrectionQualityGate(
        minimum_speed_m_s=0.10,
        maximum_rms_error_m=0.08,
        maximum_abs_lateral_m=-1.0,
        maximum_abs_yaw_rad=0.11,
        maximum_lateral_jump_m=10.0,
        maximum_yaw_jump_rad=0.04,
        required_consistent_measurements=1,
    )

    assert gate.accept(CorrectionResult(2.0, 0.01, 0.02, 20), 0.50)


def test_lateral_estimate_is_not_clipped_when_limit_is_disabled():
    x = np.linspace(0.2, 3.0, 80)
    reference = np.column_stack((x, np.zeros_like(x)))
    observed_base = reference.copy()
    corrector = LaneOdomCorrector(
        maximum_match_distance_m=1.0,
        maximum_lateral_correction_m=-1.0,
        smoothing_alpha=1.0,
    )

    result = corrector.estimate(
        observed_base, reference, odom_x=0.0, odom_y=0.45, odom_yaw=0.0
    )

    assert result is not None
    np.testing.assert_allclose(result.lateral_m, -0.45, atol=0.02)


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
