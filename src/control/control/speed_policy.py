"""Deterministic speed-policy anchors measured during simulator tuning."""

import json
import math


_KNOTS = {
    cap: {
        "user_selected_max_speed_m_s": cap,
        "optimized_curve_speed_max_m_s": cap,
        "optimized_curve_speed_min_m_s": 0.30,
        "optimized_straight_speed_min_m_s": cap,
        "straight_curvature_threshold_inv_m": 0.10,
        "curve_entry_preview_m": 1.0,
        "lookahead_parameters": {
            "adaptive_min_lookahead_m": 0.45,
            "adaptive_max_lookahead_m": 1.50,
            "curvature_reference_inv_m": 2.0,
            "speed_lookahead_time_s": 0.55,
            "speed_min_lookahead_m": 0.45,
            "speed_max_lookahead_m": 1.50,
        },
        "max_lateral_acceleration_m_s2": 0.8,
        "acceleration_limit_m_s2": 2.5,
        "deceleration_limit_m_s2": 2.5,
        "validation_status": (
            "SCREENED_CANDIDATE_01" if cap == 2.5 else "CALCULATED_NOT_RUN"
        ),
    }
    for cap in (1.5, 2.0, 2.5)
}


def policy_for_max_speed(user_selected_max_speed_m_s: float) -> dict:
    """Return an anchor policy or linear interpolation for a cap in [1.5, 2.5]."""

    cap = float(user_selected_max_speed_m_s)
    if not math.isfinite(cap) or not 1.5 <= cap <= 2.5:
        raise ValueError("user_selected_max_speed_m_s must be within [1.5, 2.5]")

    anchors = sorted(_KNOTS)
    lower = max(value for value in anchors if value <= cap)
    upper = min(value for value in anchors if value >= cap)
    ratio = 0.0 if lower == upper else (cap - lower) / (upper - lower)
    policy = _interpolate(_KNOTS[lower], _KNOTS[upper], ratio)
    policy.update(
        user_selected_max_speed_m_s=cap,
        source_lower_knot_m_s=lower,
        source_upper_knot_m_s=upper,
        interpolation_ratio=ratio,
        validation_status=(
            _KNOTS[lower]["validation_status"]
            if lower == upper else "CALCULATED_NOT_RUN"
        ),
    )
    return policy


def _interpolate(lower: dict, upper: dict, ratio: float) -> dict:
    result = {}
    for key, value in lower.items():
        if key == "validation_status":
            continue
        other = upper[key]
        if isinstance(value, dict):
            result[key] = _interpolate(value, other, ratio)
        else:
            result[key] = value + (other - value) * ratio
    return result


if __name__ == "__main__":
    print(json.dumps([policy_for_max_speed(cap) for cap in sorted(_KNOTS)], indent=2))
