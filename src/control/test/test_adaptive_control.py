import math
import unittest

from control.adaptive_policy import (
    AdaptiveControlPolicy,
    curvature_speed_limit,
)
from control.controller_core import ControllerCore, speed_lookahead_distance_m
from control.models import (
    AdaptiveControlConfig,
    ControllerConfig,
    PIDConfig,
    PathPoint,
    PurePursuitConfig,
    SpeedLookaheadConfig,
    VehicleState,
)
from control.path_metrics import discrete_curvature, preview_curvature
from control.pid import PIDController
from control.pure_pursuit import PurePursuit


def adaptive_config(**overrides):
    values = {
        "enabled": True,
        "preview_distance_m": 1.0,
        "min_lookahead_m": 0.25,
        "max_lookahead_m": 0.45,
        "curvature_reference_inv_m": 2.0,
        "max_lateral_acceleration_m_s2": 0.8,
        "min_speed_limit_m_s": 0.30,
        "max_speed_limit_m_s": 0.80,
    }
    values.update(overrides)
    return AdaptiveControlConfig(**values)


def speed_lookahead_config(**overrides):
    values = {
        "enabled": True,
        "lookahead_time_sec": 0.55,
        "min_lookahead_m": 0.45,
        "max_lookahead_m": 1.50,
    }
    values.update(overrides)
    return SpeedLookaheadConfig(**values)


def make_core(
    adaptive=None,
    closed_loop=False,
    pid_enabled=False,
    speed_lookahead=None,
):
    pursuit = PurePursuit(PurePursuitConfig(
        wheelbase_m=0.18,
        lookahead_distance_m=0.45,
        max_steering_rad=0.3491,
        closed_loop=closed_loop,
    ))
    pid = PIDController(PIDConfig(
        kp=0.20,
        ki=0.05,
        kd=0.0,
        output_min=-0.20,
        output_max=0.20,
        integral_min=-0.50,
        integral_max=0.50,
    ))
    return ControllerCore(
        pursuit,
        pid,
        ControllerConfig(
            longitudinal_pid_enabled=pid_enabled,
            max_speed_m_s=3.0,
            speed_lookahead=speed_lookahead or SpeedLookaheadConfig(),
            adaptive_control=adaptive or AdaptiveControlConfig(),
        ),
    )


class SpeedLookaheadTests(unittest.TestCase):
    def test_configured_operating_points(self):
        config = speed_lookahead_config()

        self.assertAlmostEqual(
            speed_lookahead_distance_m(0.55, config), 0.45
        )
        self.assertAlmostEqual(
            speed_lookahead_distance_m(1.5, config), 0.825
        )
        self.assertAlmostEqual(
            speed_lookahead_distance_m(3.0, config), 1.50
        )

    def test_speed_magnitude_and_bounds_are_applied(self):
        config = speed_lookahead_config()

        self.assertAlmostEqual(speed_lookahead_distance_m(0.0, config), 0.45)
        self.assertAlmostEqual(speed_lookahead_distance_m(-1.5, config), 0.825)
        self.assertAlmostEqual(speed_lookahead_distance_m(10.0, config), 1.50)

    def test_invalid_speed_is_rejected(self):
        with self.assertRaises(ValueError):
            speed_lookahead_distance_m(math.nan, speed_lookahead_config())

    def test_invalid_bounds_are_rejected(self):
        with self.assertRaises(ValueError):
            speed_lookahead_config(
                min_lookahead_m=1.0,
                max_lookahead_m=0.5,
            )


class DiscreteCurvatureTests(unittest.TestCase):
    def test_collinear_points_have_zero_curvature(self):
        curvature = discrete_curvature(
            PathPoint(0.0, 0.0),
            PathPoint(1.0, 0.0),
            PathPoint(2.0, 0.0),
        )
        self.assertEqual(curvature, 0.0)

    def test_points_on_known_circle_have_expected_curvature(self):
        radius = 2.0
        points = [
            PathPoint(radius * math.cos(angle), radius * math.sin(angle))
            for angle in (-0.2, 0.0, 0.2)
        ]
        self.assertAlmostEqual(discrete_curvature(*points), 1.0 / radius)

    def test_duplicate_and_tiny_segments_are_finite_and_zero(self):
        cases = [
            (
                PathPoint(0.0, 0.0),
                PathPoint(0.0, 0.0),
                PathPoint(1.0, 0.0),
            ),
            (
                PathPoint(0.0, 0.0),
                PathPoint(1.0e-12, 0.0),
                PathPoint(2.0e-12, 1.0e-12),
            ),
        ]
        for points in cases:
            with self.subTest(points=points):
                curvature = discrete_curvature(*points)
                self.assertEqual(curvature, 0.0)
                self.assertTrue(math.isfinite(curvature))


class AdaptivePolicyTests(unittest.TestCase):
    straight_path = [
        (0.0, 0.0),
        (0.25, 0.0),
        (0.5, 0.0),
        (0.75, 0.0),
    ]
    sharp_path = [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.3, 0.6)]
    state = VehicleState(0.0, 0.0, 0.0, 0.5)

    def test_straight_path_selects_maximum_values(self):
        result = AdaptiveControlPolicy(adaptive_config()).compute(
            self.state, self.straight_path
        )
        self.assertEqual(result.curvature_inv_m, 0.0)
        self.assertAlmostEqual(result.lookahead_distance_m, 0.45)
        self.assertAlmostEqual(result.speed_limit_m_s, 0.80)

    def test_sharp_curve_reduces_lookahead_and_speed_limit(self):
        result = AdaptiveControlPolicy(adaptive_config()).compute(
            self.state, self.sharp_path
        )
        self.assertGreater(result.curvature_inv_m, 0.0)
        self.assertLess(result.lookahead_distance_m, 0.45)
        self.assertLess(result.speed_limit_m_s, 0.80)

    def test_policy_outputs_stay_within_configured_bounds(self):
        for curvature_reference in (0.01, 1.0, 100.0):
            with self.subTest(curvature_reference=curvature_reference):
                config = adaptive_config(
                    curvature_reference_inv_m=curvature_reference
                )
                result = AdaptiveControlPolicy(config).compute(
                    self.state, self.sharp_path
                )
                self.assertLessEqual(
                    config.min_lookahead_m, result.lookahead_distance_m
                )
                self.assertLessEqual(
                    result.lookahead_distance_m, config.max_lookahead_m
                )
                self.assertLessEqual(
                    config.min_speed_limit_m_s, result.speed_limit_m_s
                )
                self.assertLessEqual(
                    result.speed_limit_m_s, config.max_speed_limit_m_s
                )

    def test_lateral_acceleration_speed_law(self):
        config = adaptive_config()
        expected_speeds = {
            0.0: 0.80,
            1.0: 0.80,
            2.0: math.sqrt(0.8 / 2.0),
            5.0: math.sqrt(0.8 / 5.0),
            8.0: math.sqrt(0.8 / 8.0),
            100.0: 0.30,
        }
        for curvature, expected in expected_speeds.items():
            with self.subTest(curvature=curvature):
                self.assertAlmostEqual(
                    curvature_speed_limit(curvature, config), expected
                )

    def test_zero_minimum_speed_is_valid(self):
        config = adaptive_config(min_speed_limit_m_s=0.0)
        self.assertEqual(config.min_speed_limit_m_s, 0.0)

    def test_negative_minimum_speed_is_invalid(self):
        with self.assertRaises(ValueError):
            adaptive_config(min_speed_limit_m_s=-0.01)


class ControllerIntegrationTests(unittest.TestCase):
    def test_disabled_speed_lookahead_uses_fixed_distance(self):
        path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
        result = make_core().update(
            VehicleState(0.0, 0.0, 0.0, 3.0), path, 1.0, 0.02
        )

        self.assertAlmostEqual(
            result.pure_pursuit.lookahead_distance_m, 0.45
        )

    def test_vehicle_speed_changes_pure_pursuit_lookahead(self):
        path = [(index * 0.1, 0.0) for index in range(21)]
        core = make_core(speed_lookahead=speed_lookahead_config())

        low_speed = core.update(
            VehicleState(0.0, 0.0, 0.0, 0.55), path, 0.55, 0.02
        )
        high_speed = core.update(
            VehicleState(0.0, 0.0, 0.0, 1.5), path, 1.5, 0.02
        )

        self.assertAlmostEqual(
            low_speed.pure_pursuit.lookahead_distance_m, 0.45
        )
        self.assertAlmostEqual(
            high_speed.pure_pursuit.lookahead_distance_m, 0.825
        )
        self.assertGreater(
            high_speed.pure_pursuit.target_index,
            low_speed.pure_pursuit.target_index,
        )

    def test_curvature_adaptive_caps_speed_based_lookahead(self):
        speed_config = speed_lookahead_config()
        adaptive = adaptive_config(
            min_lookahead_m=0.25,
            max_lookahead_m=1.50,
            curvature_reference_inv_m=0.01,
        )
        core = make_core(adaptive=adaptive, speed_lookahead=speed_config)
        straight = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]
        sharp = [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.3, 0.6)]
        state = VehicleState(0.0, 0.0, 0.0, 1.5)

        straight_result = core.update(state, straight, 1.5, 0.02)
        sharp_result = core.update(state, sharp, 1.5, 0.02)

        self.assertAlmostEqual(
            straight_result.pure_pursuit.lookahead_distance_m, 0.825
        )
        self.assertAlmostEqual(
            sharp_result.pure_pursuit.lookahead_distance_m, 0.25
        )

    def test_point_five_target_is_reduced_by_point_three_speed_limit(self):
        result = make_core(
            adaptive_config(max_lateral_acceleration_m_s2=0.01)
        ).update(
            VehicleState(0.0, 0.0, 0.0, 0.0),
            [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.3, 0.6)],
            target_speed=0.5,
            dt=0.02,
        )
        self.assertEqual(result.adaptive.speed_limit_m_s, 0.3)
        self.assertEqual(result.speed_command_m_s, 0.3)

    def test_point_four_target_is_not_raised_on_straight(self):
        result = make_core(adaptive_config()).update(
            VehicleState(0.0, 0.0, 0.0, 0.0),
            [(0.0, 0.0), (0.4, 0.0), (0.8, 0.0)],
            target_speed=0.4,
            dt=0.02,
        )
        self.assertEqual(result.adaptive.speed_limit_m_s, 0.8)
        self.assertEqual(result.speed_command_m_s, 0.4)

    def test_pid_receives_curvature_limited_target(self):
        result = make_core(
            adaptive_config(max_lateral_acceleration_m_s2=0.01),
            pid_enabled=True,
        ).update(
            VehicleState(0.0, 0.0, 0.0, 0.2),
            [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.3, 0.6)],
            target_speed=0.8,
            dt=0.02,
        )
        self.assertEqual(result.adaptive.speed_limit_m_s, 0.3)
        self.assertAlmostEqual(result.pid.error, 0.1)

    def test_adaptive_off_matches_legacy_controller_result(self):
        state = VehicleState(0.0, 0.1, 0.0, 0.2)
        path = [(0.0, 0.0), (0.4, 0.0), (0.8, 0.2)]
        default_result = make_core().update(state, path, 0.6, 0.02)
        disabled_result = make_core(
            AdaptiveControlConfig(enabled=False)
        ).update(state, path, 0.6, 0.02)
        self.assertEqual(default_result, disabled_result)
        self.assertIsNone(default_result.adaptive)

    def test_adaptive_on_passes_reduced_lookahead_to_pure_pursuit(self):
        config = adaptive_config(curvature_reference_inv_m=0.01)
        state = VehicleState(0.0, 0.0, 0.0, 0.5)
        path = [(0.0, 0.0), (0.3, 0.0), (0.3, 0.3), (0.3, 0.6)]
        adaptive_result = make_core(config).update(state, path, 0.5, 0.02)
        legacy_result = make_core().update(state, path, 0.5, 0.02)
        self.assertAlmostEqual(
            adaptive_result.adaptive.lookahead_distance_m, 0.25
        )
        self.assertEqual(adaptive_result.pure_pursuit.target_index, 1)
        self.assertEqual(legacy_result.pure_pursuit.target_index, 3)


class PreviewCurvatureTests(unittest.TestCase):
    def test_closed_loop_preview_wraps_across_duplicate_endpoint(self):
        path = [
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]
        curvature = preview_curvature(
            VehicleState(0.0, 1.0, 0.0, 0.5),
            path,
            preview_distance_m=1.1,
            closed_loop=True,
        )
        self.assertGreater(curvature, 0.0)

    def test_open_path_preview_does_not_wrap_to_start(self):
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        curvature = preview_curvature(
            VehicleState(0.0, 1.0, 0.0, 0.5),
            path,
            preview_distance_m=10.0,
            closed_loop=False,
        )
        self.assertEqual(curvature, 0.0)

    def test_short_paths_are_safe(self):
        for path in ([(0.0, 0.0)], [(0.0, 0.0), (0.1, 0.0)]):
            with self.subTest(path=path):
                curvature = preview_curvature(
                    VehicleState(0.0, 0.0, 0.0, 0.0),
                    path,
                    preview_distance_m=1.0,
                    closed_loop=True,
                )
                self.assertEqual(curvature, 0.0)


if __name__ == "__main__":
    unittest.main()
