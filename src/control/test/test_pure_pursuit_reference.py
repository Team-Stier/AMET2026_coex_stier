import math

import pytest

from control.models import PurePursuitConfig, VehicleState
from control.pure_pursuit import PurePursuit


def make_pursuit(reference_point_offset_m=0.0):
    return PurePursuit(
        PurePursuitConfig(
            wheelbase_m=0.18,
            lookahead_distance_m=0.45,
            max_steering_rad=0.3491,
            reference_point_offset_m=reference_point_offset_m,
        )
    )


def test_zero_reference_offset_preserves_existing_result():
    state = VehicleState(0.1, -0.2, 0.3, 0.5)
    path = [(0.0, 0.0), (0.5, 0.2), (1.0, 0.6)]

    implicit_default = PurePursuit(
        PurePursuitConfig(0.18, 0.45, 0.3491, False)
    ).compute(state, path)
    explicit_zero = make_pursuit(0.0).compute(state, path)

    assert explicit_zero == implicit_default


@pytest.mark.parametrize(
    ("offset_m", "target_distance_m", "alpha_rad", "steering_rad"),
    [
        (0.20, 1.2806248475, 0.8960553846, 0.2160849723),
        (0.0, 1.4142135624, 0.7853981634, 0.1780929382),
        (-0.20, 1.5620499352, 0.6947382762, 0.1464841784),
    ],
)
def test_signed_reference_offset_changes_pure_pursuit_geometry(
    offset_m, target_distance_m, alpha_rad, steering_rad
):
    result = make_pursuit(offset_m).compute(
        VehicleState(0.0, 0.0, 0.0, 0.5),
        [(0.0, 0.0), (1.0, 1.0)],
    )

    assert result.target_index == 1
    assert result.target_distance_m == pytest.approx(target_distance_m)
    assert result.alpha_rad == pytest.approx(alpha_rad)
    assert result.steering_rad == pytest.approx(steering_rad)


def test_reference_offset_rotates_with_vehicle_yaw():
    state = VehicleState(1.0, 2.0, math.pi / 2.0, 0.5)
    path = [(1.0, 2.0), (0.5, 3.0), (0.0, 4.0)]

    offset_result = make_pursuit(0.2).compute(state, path)
    translated_result = make_pursuit().compute(
        VehicleState(1.0, 2.2, math.pi / 2.0, 0.5), path
    )

    assert offset_result.target_index == translated_result.target_index
    assert offset_result.target_distance_m == pytest.approx(
        translated_result.target_distance_m
    )
    assert offset_result.alpha_rad == pytest.approx(
        translated_result.alpha_rad
    )
    assert offset_result.steering_rad == pytest.approx(
        translated_result.steering_rad
    )


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_reference_offset_must_be_finite(value):
    with pytest.raises(ValueError, match="reference_point_offset_m"):
        make_pursuit(value)
