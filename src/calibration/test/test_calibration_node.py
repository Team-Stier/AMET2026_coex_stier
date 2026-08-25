import math
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import LaserScan

from calibration.calibration_node import (
    CalibrationNode,
    fit_with_yaw_recovery,
    propagate_pose_with_relative_motion,
)
from calibration.wall_fitter import FitResult


class CalibrationState(SimpleNamespace):
    _fresh_prior = CalibrationNode._fresh_prior
    _fresh_twist = CalibrationNode._fresh_twist
    _fallback_pose = CalibrationNode._fallback_pose


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SilentLogger:
    def warning(self, *args, **kwargs):
        pass


class ScriptedRecoveryFitter:
    minimum_matches_per_wall = 12

    def __init__(self, results_by_seed_yaw=None, first_result=None):
        self.results_by_seed_yaw = results_by_seed_yaw or {}
        self.first_result = first_result
        self.fit_calls = []
        self.fit_first_calls = []

    def fit(self, _points, seed):
        self.fit_calls.append(seed)
        return self.results_by_seed_yaw.get(round(seed[2], 2))

    def fit_first(self, _points, initial_poses, maximum_rms_error_m):
        self.fit_first_calls.append((initial_poses, maximum_rms_error_m))
        return self.first_result


def fit_result(pose, rms=0.02, wall_counts=(20, 20, 0, 0)):
    return FitResult(
        pose=pose,
        rms_error_m=rms,
        match_count=sum(wall_counts),
        wall_match_counts=wall_counts,
    )


def test_tracking_reset_clears_pose_prior():
    state = CalibrationState(
        _initial_pose=(1.4, 3.427, -1.57),
        _pose=(8.0, 5.0, 1.0),
        _tracking_initialized=True,
        _latest_prior_pose=(7.9, 5.1, 1.1),
        _latest_prior_stamp_ns=123,
        _latest_prior_twist=Odometry().twist,
        _latest_prior_twist_stamp_ns=123,
        _tracking_pose_anchor=(8.0, 5.0, 1.0),
        _tracking_prior_anchor=(7.9, 5.1, 1.1),
    )

    CalibrationNode._reset_tracking(state)

    assert state._pose == state._initial_pose
    assert state._tracking_initialized is False
    assert state._latest_prior_pose is None
    assert state._latest_prior_stamp_ns is None
    assert state._latest_prior_twist is None
    assert state._latest_prior_twist_stamp_ns is None
    assert state._tracking_pose_anchor is None
    assert state._tracking_prior_anchor is None


def test_prior_twist_is_copied_for_calibrated_output():
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = 1
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = 1.25
    message.twist.twist.angular.z = 0.4
    message.twist.covariance[0] = 0.03
    state = SimpleNamespace(_lidar_offset_x_m=-0.027)

    CalibrationNode._on_prior(state, message)
    message.twist.twist.linear.x = 9.0

    assert state._latest_prior_twist.twist.linear.x == pytest.approx(1.25)
    assert state._latest_prior_twist.twist.angular.z == pytest.approx(0.4)
    assert state._latest_prior_twist.covariance[0] == pytest.approx(0.03)
    assert state._latest_prior_twist_stamp_ns == 1_000_000_000


def test_invalid_prior_twist_is_not_forwarded():
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = 1
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = math.nan
    state = SimpleNamespace(_lidar_offset_x_m=-0.027)

    CalibrationNode._on_prior(state, message)

    assert state._latest_prior_pose is not None
    assert state._latest_prior_twist is None
    assert state._latest_prior_twist_stamp_ns is None


def test_nonfinite_prior_twist_covariance_is_sanitized():
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = 1
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = 0.8
    message.twist.covariance[0] = math.nan
    state = SimpleNamespace(_lidar_offset_x_m=-0.027)

    CalibrationNode._on_prior(state, message)

    assert state._latest_prior_twist.twist.linear.x == pytest.approx(0.8)
    assert state._latest_prior_twist.covariance[0] == pytest.approx(1.0e6)
    assert all(
        math.isfinite(value) for value in state._latest_prior_twist.covariance
    )


def test_calibrated_odometry_forwards_only_fresh_twist():
    prior = Odometry()
    prior.twist.twist.linear.x = 1.4
    prior.twist.twist.angular.z = 0.3
    prior.twist.covariance[0] = 0.02
    scan = LaserScan()
    scan.header.stamp.sec = 1
    state = CalibrationState(
        _pose=(1.4, 3.421607, -math.pi / 2.0),
        _lidar_offset_x_m=-0.027,
        _lidar_frame="lidar_link",
        _fallback_position_variance_m2=0.04,
        _fallback_yaw_variance_rad2=0.03,
        _latest_prior_twist=prior.twist,
        _latest_prior_twist_stamp_ns=1_000_000_000,
        _maximum_prior_age_ns=250_000_000,
    )

    fresh = CalibrationNode._odometry(state, scan, 0.02)
    state._latest_prior_twist_stamp_ns = 0
    stale = CalibrationNode._odometry(state, scan, 0.02)

    assert fresh.twist.twist.linear.x == pytest.approx(1.4)
    assert fresh.twist.twist.angular.z == pytest.approx(0.3)
    assert fresh.twist.covariance[0] == pytest.approx(0.02)
    assert stale.twist.twist.linear.x == pytest.approx(0.0)
    assert stale.twist.covariance[0] == pytest.approx(1.0e6)


def test_first_fit_uses_only_fixed_initial_pose_even_with_fresh_prior():
    state = CalibrationState(
        _initial_pose=(1.4, 3.427, -1.57),
        _pose=(1.4, 3.427, -1.57),
        _tracking_initialized=False,
        _latest_prior_pose=(8.0, 5.0, 1.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=None,
        _tracking_prior_anchor=None,
    )

    initial_poses = CalibrationNode._fit_initial_poses(state, 1_100)

    assert initial_poses == (state._initial_pose,)


def test_tracking_uses_relative_prior_motion_without_absolute_prior_position():
    state = CalibrationState(
        _initial_pose=(1.4, 3.427, -1.57),
        _pose=(1.5, 3.3, 0.0),
        _tracking_initialized=True,
        _latest_prior_pose=(100.0, 201.0, math.pi / 2.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.5, 3.3, 0.0),
        _tracking_prior_anchor=(100.0, 200.0, math.pi / 2.0),
    )

    initial_poses = CalibrationNode._fit_initial_poses(state, 1_100)

    assert initial_poses[0] == pytest.approx((2.5, 3.3, 0.0))
    assert initial_poses[1] == state._pose


def test_tracking_drops_stale_prior():
    state = CalibrationState(
        _initial_pose=(1.4, 3.427, -1.57),
        _pose=(1.5, 3.3, -1.55),
        _tracking_initialized=True,
        _latest_prior_pose=(8.0, 5.0, 1.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.5, 3.3, -1.55),
        _tracking_prior_anchor=(7.9, 5.1, 1.1),
    )

    initial_poses = CalibrationNode._fit_initial_poses(state, 1_251)

    assert initial_poses == (state._pose,)


def test_relative_motion_rotates_into_calibrated_frame():
    predicted = propagate_pose_with_relative_motion(
        calibrated_anchor=(1.0, 2.0, math.pi / 2.0),
        prior_anchor=(100.0, 200.0, 0.0),
        current_prior=(101.0, 200.0, math.pi / 4.0),
    )

    assert predicted[0] == pytest.approx(1.0)
    assert predicted[1] == pytest.approx(3.0)
    assert predicted[2] == pytest.approx(3.0 * math.pi / 4.0)


def test_yaw_recovery_prefers_smaller_correction_within_rms_tie():
    fitter = ScriptedRecoveryFitter(
        {
            -0.2: fit_result((0.04, 0.0, 0.08), rms=0.020),
            0.2: fit_result((0.01, 0.0, 0.03), rms=0.023),
        }
    )

    result = fit_with_yaw_recovery(
        fitter,
        np.zeros((50, 2)),
        (0.0, 0.0, 0.0),
        (-0.2, 0.2),
        maximum_rms_error_m=0.08,
        maximum_position_correction_m=0.25,
        maximum_yaw_correction_rad=0.15,
        rms_tie_tolerance_m=0.005,
    )

    assert result is not None
    assert result.pose == pytest.approx((0.01, 0.0, 0.03))


@pytest.mark.parametrize(
    "unsafe_pose",
    [
        (0.0, 0.0, 0.16),
        (0.26, 0.0, 0.0),
        (0.0, 0.0, math.pi),
    ],
)
def test_yaw_recovery_rechecks_limits_from_unshifted_reference(unsafe_pose):
    fitter = ScriptedRecoveryFitter({0.2: fit_result(unsafe_pose)})

    result = fit_with_yaw_recovery(
        fitter,
        np.zeros((50, 2)),
        (0.0, 0.0, 0.0),
        (0.2,),
        maximum_rms_error_m=0.08,
        maximum_position_correction_m=0.25,
        maximum_yaw_correction_rad=0.15,
        rms_tie_tolerance_m=0.005,
    )

    assert result is None


def test_regular_two_wall_fit_keeps_priority_over_yaw_recovery():
    regular_result = fit_result((1.1, 2.0, 0.02))
    primary = ScriptedRecoveryFitter(first_result=None)
    two_wall = ScriptedRecoveryFitter(first_result=regular_result)
    state = SimpleNamespace(
        _fit_initial_poses=lambda _stamp: ((1.0, 2.0, 0.0),),
        _fitter=primary,
        _two_wall_fitter=two_wall,
        _maximum_rms_error_m=0.10,
        _two_wall_maximum_rms_error_m=0.08,
        _yaw_recovery_enabled=True,
        _fallback_pose=lambda _stamp: (1.0, 2.0, 0.0),
        _yaw_recovery_offsets_rad=(-0.2, -0.1, 0.1, 0.2),
        _two_wall_maximum_position_step_m=0.25,
        _two_wall_maximum_yaw_step_rad=0.15,
        _yaw_recovery_rms_tie_tolerance_m=0.005,
    )

    result = CalibrationNode._fit(state, np.zeros((50, 2)), 1_000)

    assert result is regular_result
    assert two_wall.fit_calls == []


def test_yaw_recovery_uses_only_fresh_predicted_pose():
    recovered_result = fit_result((1.02, 2.0, 0.05))
    primary = ScriptedRecoveryFitter(first_result=None)
    two_wall = ScriptedRecoveryFitter(
        results_by_seed_yaw={0.2: recovered_result}, first_result=None
    )
    predicted_pose = (1.0, 2.0, 0.0)
    state = SimpleNamespace(
        _fit_initial_poses=lambda _stamp: (predicted_pose, (9.0, 9.0, 1.0)),
        _fitter=primary,
        _two_wall_fitter=two_wall,
        _maximum_rms_error_m=0.10,
        _two_wall_maximum_rms_error_m=0.08,
        _yaw_recovery_enabled=True,
        _fallback_pose=lambda _stamp: predicted_pose,
        _yaw_recovery_offsets_rad=(0.2,),
        _two_wall_maximum_position_step_m=0.25,
        _two_wall_maximum_yaw_step_rad=0.15,
        _yaw_recovery_rms_tie_tolerance_m=0.005,
    )

    result = CalibrationNode._fit(state, np.zeros((50, 2)), 1_000)

    assert result is recovered_result
    assert two_wall.fit_calls == [(1.0, 2.0, 0.2)]

    state._fallback_pose = lambda _stamp: None
    two_wall.fit_calls.clear()
    result = CalibrationNode._fit(state, np.zeros((50, 2)), 1_100)

    assert result is None
    assert two_wall.fit_calls == []


def test_fallback_pose_uses_fresh_relative_odom_motion():
    state = CalibrationState(
        _tracking_initialized=True,
        _latest_prior_pose=(101.0, 200.0, math.pi / 4.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.0, 2.0, math.pi / 2.0),
        _tracking_prior_anchor=(100.0, 200.0, 0.0),
    )

    fallback = CalibrationNode._fallback_pose(state, 1_100)

    assert fallback == pytest.approx((1.0, 3.0, 3.0 * math.pi / 4.0))


def test_fallback_pose_rejects_stale_or_uninitialized_odom():
    stale = CalibrationState(
        _tracking_initialized=True,
        _latest_prior_pose=(101.0, 200.0, 0.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.0, 2.0, 0.0),
        _tracking_prior_anchor=(100.0, 200.0, 0.0),
    )
    uninitialized = CalibrationState(
        _tracking_initialized=False,
        _latest_prior_pose=(101.0, 200.0, 0.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=None,
        _tracking_prior_anchor=None,
    )

    assert CalibrationNode._fallback_pose(stale, 1_251) is None
    assert CalibrationNode._fallback_pose(uninitialized, 1_100) is None


def test_failed_fit_publishes_fresh_odom_fallback():
    publisher = RecordingPublisher()
    scan = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=100))
    )
    state = SimpleNamespace(
        _last_stamp_ns=None,
        _pose=(1.0, 2.0, 0.0),
        _scan_points=lambda _: np.zeros((50, 2)),
        _fit=lambda _points, _stamp_ns: None,
        _fallback_pose=lambda _stamp_ns: (1.2, 2.1, 0.1),
        _publisher=publisher,
        _odometry=lambda _scan, rms_error_m: SimpleNamespace(
            pose=state._pose, rms_error_m=rms_error_m
        ),
        _reset_tracking=lambda: None,
        get_logger=lambda: SilentLogger(),
    )

    CalibrationNode._on_scan(state, scan)

    assert state._pose == pytest.approx((1.2, 2.1, 0.1))
    assert len(publisher.messages) == 1
    assert publisher.messages[0].pose == pytest.approx(state._pose)
    assert publisher.messages[0].rms_error_m is None


def test_recovered_fit_without_fresh_odom_clears_old_fallback_anchor():
    state = CalibrationState(
        _pose=(1.2, 2.1, 0.1),
        _tracking_initialized=True,
        _latest_prior_pose=(101.0, 200.0, 0.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.0, 2.0, 0.0),
        _tracking_prior_anchor=(100.0, 200.0, 0.0),
    )

    CalibrationNode._accept_pose(state, (1.3, 2.2, 0.2), 1_251)

    assert state._pose == pytest.approx((1.3, 2.2, 0.2))
    assert state._tracking_initialized is True
    assert state._tracking_pose_anchor is None
    assert state._tracking_prior_anchor is None


def test_accepted_fit_with_fresh_odom_refreshes_fallback_anchor():
    latest_prior = (101.0, 201.0, 0.2)
    accepted_pose = (1.3, 2.2, 0.1)
    state = CalibrationState(
        _pose=(1.2, 2.1, 0.0),
        _tracking_initialized=True,
        _latest_prior_pose=latest_prior,
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.0, 2.0, 0.0),
        _tracking_prior_anchor=(100.0, 200.0, 0.0),
    )

    CalibrationNode._accept_pose(state, accepted_pose, 1_100)

    assert state._pose == accepted_pose
    assert state._tracking_pose_anchor == accepted_pose
    assert state._tracking_prior_anchor == latest_prior


def test_failed_fit_without_fresh_odom_does_not_publish_or_change_pose():
    publisher = RecordingPublisher()
    scan = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=100))
    )
    original_pose = (1.0, 2.0, 0.0)
    state = SimpleNamespace(
        _last_stamp_ns=None,
        _pose=original_pose,
        _scan_points=lambda _: np.zeros((50, 2)),
        _fit=lambda _points, _stamp_ns: None,
        _fallback_pose=lambda _stamp_ns: None,
        _publisher=publisher,
        _odometry=lambda _scan, _rms_error_m: None,
        _reset_tracking=lambda: None,
        get_logger=lambda: SilentLogger(),
    )

    CalibrationNode._on_scan(state, scan)

    assert state._pose == original_pose
    assert publisher.messages == []


def test_recovered_fit_reanchors_next_fallback_to_latest_fit_and_odom():
    recovered_pose = (5.0, 6.0, math.pi / 2.0)
    recovered_prior = (20.0, 30.0, 0.0)
    state = CalibrationState(
        _pose=(2.0, 3.0, 0.0),
        _tracking_initialized=True,
        _latest_prior_pose=recovered_prior,
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.0, 2.0, 0.0),
        _tracking_prior_anchor=(10.0, 30.0, 0.0),
    )
    CalibrationNode._accept_pose(state, recovered_pose, 1_000)
    state._latest_prior_pose = (21.0, 30.0, 0.0)
    state._latest_prior_stamp_ns = 1_100

    fallback = CalibrationNode._fallback_pose(state, 1_100)

    assert fallback == pytest.approx((5.0, 7.0, math.pi / 2.0))


def test_consecutive_fallbacks_use_fixed_last_fit_anchor():
    state = CalibrationState(
        _tracking_initialized=True,
        _latest_prior_pose=(11.0, 20.0, 0.0),
        _latest_prior_stamp_ns=1_000,
        _maximum_prior_age_ns=250,
        _tracking_pose_anchor=(1.0, 2.0, 0.0),
        _tracking_prior_anchor=(10.0, 20.0, 0.0),
    )
    first = CalibrationNode._fallback_pose(state, 1_000)
    state._pose = first
    state._latest_prior_pose = (12.0, 20.0, 0.0)
    state._latest_prior_stamp_ns = 1_100

    second = CalibrationNode._fallback_pose(state, 1_100)

    assert first == pytest.approx((2.0, 2.0, 0.0))
    assert second == pytest.approx((3.0, 2.0, 0.0))


def test_fallback_odometry_uses_conservative_covariance():
    state = CalibrationState(
        _pose=(1.0, 2.0, 0.3),
        _lidar_frame="lidar_link",
        _lidar_offset_x_m=-0.027,
        _fallback_position_variance_m2=0.04,
        _fallback_yaw_variance_rad2=0.03,
        _latest_prior_twist=None,
        _latest_prior_twist_stamp_ns=None,
        _maximum_prior_age_ns=250_000_000,
    )
    scan = LaserScan()
    scan.header.stamp.sec = 1
    scan.header.stamp.nanosec = 100

    message = CalibrationNode._odometry(state, scan, None)

    assert message.header.frame_id == "map"
    assert message.child_frame_id == "lidar_link"
    assert message.pose.covariance[0] == pytest.approx(0.04)
    assert message.pose.covariance[7] == pytest.approx(0.04)
    assert message.pose.covariance[35] == pytest.approx(0.03)
