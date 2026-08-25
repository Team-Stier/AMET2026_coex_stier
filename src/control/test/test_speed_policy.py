import math

import pytest

from control.speed_policy import policy_for_max_speed


def test_speed_policy_anchors_interpolation_bounds_and_determinism():
    for cap in (1.5, 2.0, 2.5):
        policy = policy_for_max_speed(cap)
        assert policy["source_lower_knot_m_s"] == cap
        assert policy["source_upper_knot_m_s"] == cap
        assert policy["interpolation_ratio"] == 0.0

    middle = policy_for_max_speed(1.75)
    assert middle["source_lower_knot_m_s"] == 1.5
    assert middle["source_upper_knot_m_s"] == 2.0
    assert middle["interpolation_ratio"] == 0.5
    assert middle["optimized_curve_speed_max_m_s"] == 1.75

    for cap in (1.5, 1.75, 2.0, 2.25, 2.5):
        policy = policy_for_max_speed(cap)
        speeds = (
            policy["optimized_curve_speed_min_m_s"],
            policy["optimized_curve_speed_max_m_s"],
            policy["optimized_straight_speed_min_m_s"],
            policy["user_selected_max_speed_m_s"],
        )
        assert speeds == tuple(sorted(speeds))
        assert all(math.isfinite(value) and value <= cap for value in speeds)
        assert all(
            math.isfinite(policy[key])
            for key in (
                "straight_curvature_threshold_inv_m",
                "curve_entry_preview_m",
                "max_lateral_acceleration_m_s2",
                "acceleration_limit_m_s2",
                "deceleration_limit_m_s2",
            )
        )
        assert all(math.isfinite(value) for value in policy["lookahead_parameters"].values())
        assert policy == policy_for_max_speed(cap)

    for cap in (1.49, 2.51, math.inf, math.nan):
        with pytest.raises(ValueError):
            policy_for_max_speed(cap)
