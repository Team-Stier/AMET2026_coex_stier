import math
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import LaserScan

from calibration.calibration_node import (
    CalibrationNode,
    fit_with_bounded_pose_recovery,
    fit_with_yaw_recovery,
    prior_motion_is_continuous,
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


def empty_prior_state(lidar_offset_x_m=-0.027):
    return CalibrationState(
        _lidar_offset_x_m=lidar_offset_x_m,
        _pose=(0.0, 0.0, 0.0),
        _tracking_initialized=False,
        _latest_prior_pose=None,
        _latest_prior_stamp_ns=None,
        _latest_prior_twist=None,
        _latest_prior_twist_stamp_ns=None,
        _prior_pose_buffer={},
        _prior_twist_buffer={},
        _pending_scans={},
        _tracking_pose_anchor=None,
        _tracking_prior_anchor=None,
        _maximum_prior_age_ns=250_000_000,
        _maximum_prior_gap_ns=1_000_000_000,
        _scan_prior_sync_timeout_ns=60_000_000,
        _scan_prior_buffer_size=32,
        _maximum_prior_linear_speed_m_s=3.0,
        _maximum_prior_yaw_rate_rad_s=3.0,
        _prior_position_jump_tolerance_m=0.10,
        _prior_yaw_jump_tolerance_rad=0.10,
        _pending_recovery_pose=None,
        _pending_recovery_prior=None,
        _pending_recovery_stamp_ns=None,
        _pending_recovery_count=0,
        get_logger=lambda: SilentLogger(),
    )


def make_scan(stamp_ns):
    scan = LaserScan()
    scan.header.stamp.sec = stamp_ns // 1_000_000_000
    scan.header.stamp.nanosec = stamp_ns % 1_000_000_000
    return scan


def scan_dispatch_state():
    state = empty_prior_state(lidar_offset_x_m=0.0)
    state._last_stamp_ns = None
    state._scan_points = lambda _scan: np.zeros((50, 2))
    state._fit = lambda _points, _stamp_ns: fit_result((1.0, 2.0, 0.1))
    state.accepted = []
    state._accept_pose = lambda pose, stamp_ns: state.accepted.append(
        (pose, stamp_ns, CalibrationNode._prior_pose_for_stamp(state, stamp_ns))
    )
    state._publisher = RecordingPublisher()
    state._odometry = lambda _scan, rms_error_m: SimpleNamespace(
        rms_error_m=rms_error_m
    )
    return state


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


class ScriptedPoseRecoveryFitter(ScriptedRecoveryFitter):
    def __init__(self, results_by_seed=None):
        super().__init__()
        self.results_by_seed = results_by_seed or {}

    def fit(self, _points, seed):
        self.fit_calls.append(seed)
        key = tuple(round(value, 2) for value in seed)
        return self.results_by_seed.get(key)


class AlwaysRecoveryFitter(ScriptedRecoveryFitter):
    minimum_matches_per_wall = 8

    def __init__(self, result):
        super().__init__()
        self.result = result

    def fit(self, _points, seed):
        self.fit_calls.append(seed)
        return self.result


def fit_result(pose, rms=0.02, wall_counts=(20, 20, 0, 0)):
    return FitResult(
        pose=pose,
        rms_error_m=rms,
        match_count=sum(wall_counts),
        wall_match_counts=wall_counts,
    )


def test_tracking_reset_clears_pose_prior():
    state = empty_prior_state()
    state._initial_pose = (1.4, 3.427, -1.57)
    state._pose = (8.0, 5.0, 1.0)
    state._tracking_initialized = True
    state._latest_prior_pose = (7.9, 5.1, 1.1)
    state._latest_prior_stamp_ns = 123
    state._latest_prior_twist = Odometry().twist
    state._latest_prior_twist_stamp_ns = 123
    state._prior_pose_buffer[123] = state._latest_prior_pose
    state._prior_twist_buffer[123] = state._latest_prior_twist
    state._pending_scans[456] = (LaserScan(), 0)
    state._tracking_pose_anchor = state._pose
    state._tracking_prior_anchor = state._latest_prior_pose
    state._last_fit_stamp_ns = 123
    state._last_fit_mode = "regular"
    state._last_stamp_ns = 456

    CalibrationNode._reset_tracking(state)

    assert state._pose == state._initial_pose
    assert state._tracking_initialized is False
    assert state._latest_prior_pose is None
    assert state._latest_prior_stamp_ns is None
    assert state._latest_prior_twist is None
    assert state._latest_prior_twist_stamp_ns is None
    assert state._tracking_pose_anchor is None
    assert state._tracking_prior_anchor is None
    assert state._prior_pose_buffer == {}
    assert state._prior_twist_buffer == {}
    assert state._pending_scans == {}
    assert state._last_stamp_ns is None


def test_prior_twist_is_copied_for_calibrated_output():
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = 1
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = 1.25
    message.twist.twist.angular.z = 0.4
    message.twist.covariance[0] = 0.03
    state = empty_prior_state()

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
    state = empty_prior_state()

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
    state = empty_prior_state()

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


def test_prior_and_twist_lookup_require_exact_scan_timestamp():
    state = empty_prior_state(lidar_offset_x_m=0.0)
    pose = (1.0, 2.0, 0.1)
    twist = Odometry().twist
    state._latest_prior_pose = pose
    state._latest_prior_stamp_ns = 1_000
    state._latest_prior_twist = twist
    state._latest_prior_twist_stamp_ns = 1_000
    state._prior_pose_buffer[1_000] = pose
    state._prior_twist_buffer[1_000] = twist

    assert CalibrationNode._fresh_prior(state, 1_000)
    assert CalibrationNode._fresh_twist(state, 1_000)
    assert not CalibrationNode._fresh_prior(state, 999)
    assert not CalibrationNode._fresh_prior(state, 1_001)
    assert not CalibrationNode._fresh_twist(state, 999)
    assert not CalibrationNode._fresh_twist(state, 1_001)


def test_scan_waits_until_same_stamp_prior_arrives():
    stamp_ns = 1_000_000_000
    scan = make_scan(stamp_ns)
    state = scan_dispatch_state()

    CalibrationNode._on_scan(state, scan)

    assert list(state._pending_scans) == [stamp_ns]
    assert state.accepted == []
    assert state._publisher.messages == []

    CalibrationNode._on_prior(state, make_prior(10.0, 20.0, 0.0, stamp_ns))

    assert state._pending_scans == {}
    assert len(state.accepted) == 1
    assert state.accepted[0][1] == stamp_ns
    assert state.accepted[0][2] == pytest.approx((10.0, 20.0, 0.0))
    assert len(state._publisher.messages) == 1


def test_duplicate_scan_timestamp_is_processed_only_once():
    stamp_ns = 1_000_000_000
    scan = make_scan(stamp_ns)
    state = scan_dispatch_state()

    CalibrationNode._on_scan(state, scan)
    CalibrationNode._on_scan(state, scan)

    assert len(state._pending_scans) == 1
    CalibrationNode._drain_pending_scans(state, force=True)
    assert len(state.accepted) == 1
    assert len(state._publisher.messages) == 1


def test_scan_without_exact_prior_cannot_use_previous_pose_for_fallback():
    scan_stamp_ns = 1_100_000_000
    prior_stamp_ns = 1_000_000_000
    scan = make_scan(scan_stamp_ns)
    state = empty_prior_state(lidar_offset_x_m=0.0)
    state._last_stamp_ns = None
    state._tracking_initialized = True
    state._pose = (1.0, 2.0, 0.0)
    state._tracking_pose_anchor = state._pose
    state._tracking_prior_anchor = (10.0, 20.0, 0.0)
    state._latest_prior_pose = (10.1, 20.0, 0.0)
    state._latest_prior_stamp_ns = prior_stamp_ns
    state._prior_pose_buffer[prior_stamp_ns] = state._latest_prior_pose
    state._scan_points = lambda _scan: np.zeros((50, 2))
    state._fit = lambda _points, _stamp_ns: None
    state._publisher = RecordingPublisher()
    state._odometry = lambda _scan, _rms_error_m: object()

    CalibrationNode._on_scan(state, scan)
    CalibrationNode._drain_pending_scans(state, force=True)

    assert state._pose == (1.0, 2.0, 0.0)
    assert state._publisher.messages == []


def test_first_fit_uses_only_fixed_initial_pose_even_with_fresh_prior():
    state = CalibrationState(
        _initial_pose=(1.4, 3.427, -1.57),
        _pose=(1.4, 3.427, -1.57),
        _tracking_initialized=False,
        _latest_prior_pose=(8.0, 5.0, 1.0),
        _latest_prior_stamp_ns=1_100,
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
        _latest_prior_stamp_ns=1_100,
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
        _fallback_recovery_enabled=False,
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
        _fallback_recovery_enabled=False,
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
        _latest_prior_stamp_ns=1_100,
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
        _last_fit_stamp_ns=1_000_000_000,
        _maximum_fallback_duration_ns=500_000_000,
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

    CalibrationNode._process_scan(state, scan)

    assert state._pose == pytest.approx((1.2, 2.1, 0.1))
    assert len(publisher.messages) == 1
    assert publisher.messages[0].pose == pytest.approx(state._pose)
    assert publisher.messages[0].rms_error_m is None


def test_recovered_fit_without_fresh_odom_clears_old_fallback_anchor():
    state = CalibrationState(
        _pose=(1.2, 2.1, 0.1),
        _tracking_initialized=True,
        _latest_prior_pose=(101.0, 200.0, 0.0),
        _latest_prior_stamp_ns=1_100,
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
        _latest_prior_stamp_ns=1_100,
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

    CalibrationNode._process_scan(state, scan)

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


def make_prior(x, y, yaw, stamp_ns):
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def prior_tracking_state():
    prior_pose = (10.0, 20.0, 0.0)
    prior_twist = Odometry().twist
    return CalibrationState(
        _lidar_offset_x_m=0.0,
        _pose=(1.0, 2.0, 0.0),
        _tracking_initialized=True,
        _latest_prior_pose=prior_pose,
        _latest_prior_stamp_ns=1_000_000_000,
        _latest_prior_twist=prior_twist,
        _latest_prior_twist_stamp_ns=1_000_000_000,
        _prior_pose_buffer={1_000_000_000: prior_pose},
        _prior_twist_buffer={1_000_000_000: prior_twist},
        _pending_scans={},
        _tracking_pose_anchor=(1.0, 2.0, 0.0),
        _tracking_prior_anchor=prior_pose,
        _maximum_prior_age_ns=250_000_000,
        _maximum_prior_gap_ns=1_000_000_000,
        _scan_prior_sync_timeout_ns=60_000_000,
        _scan_prior_buffer_size=32,
        _maximum_prior_linear_speed_m_s=3.0,
        _maximum_prior_yaw_rate_rad_s=3.0,
        _prior_position_jump_tolerance_m=0.10,
        _prior_yaw_jump_tolerance_rad=0.10,
        _pending_recovery_pose=None,
        _pending_recovery_prior=None,
        _pending_recovery_stamp_ns=None,
        _pending_recovery_count=0,
        get_logger=lambda: SilentLogger(),
    )


def test_prior_motion_continuity_uses_dt_and_wrapped_yaw():
    assert prior_motion_is_continuous(
        (10.0, 20.0, 3.13),
        (10.2, 20.0, -3.13),
        elapsed_sec=0.1,
        maximum_linear_speed_m_s=3.0,
        maximum_yaw_rate_rad_s=3.0,
        position_tolerance_m=0.10,
        yaw_tolerance_rad=0.10,
    )
    assert not prior_motion_is_continuous(
        (10.0, 20.0, 0.0),
        (11.0, 20.0, 0.5),
        elapsed_sec=0.1,
        maximum_linear_speed_m_s=3.0,
        maximum_yaw_rate_rad_s=3.0,
        position_tolerance_m=0.10,
        yaw_tolerance_rad=0.10,
    )


def test_prior_jump_rebases_without_publishing_jump():
    state = prior_tracking_state()

    CalibrationNode._on_prior(
        state, make_prior(20.0, 20.0, 0.0, 1_100_000_000)
    )

    assert state._latest_prior_pose == pytest.approx((20.0, 20.0, 0.0))
    assert state._tracking_pose_anchor == state._pose
    assert state._tracking_prior_anchor == state._latest_prior_pose
    assert state._latest_prior_twist is None
    assert state._prior_twist_buffer[1_100_000_000] is None
    assert CalibrationNode._fallback_pose(state, 1_100_000_000) == state._pose

    CalibrationNode._on_prior(
        state, make_prior(20.1, 20.0, 0.0, 1_200_000_000)
    )

    assert CalibrationNode._fallback_pose(
        state, 1_200_000_000
    ) == pytest.approx((1.1, 2.0, 0.0))


def test_normal_prior_increment_keeps_last_fit_anchor():
    state = prior_tracking_state()
    pose_anchor = state._tracking_pose_anchor
    prior_anchor = state._tracking_prior_anchor

    CalibrationNode._on_prior(
        state, make_prior(10.2, 20.0, 0.0, 1_100_000_000)
    )

    assert state._tracking_pose_anchor is pose_anchor
    assert state._tracking_prior_anchor is prior_anchor
    assert CalibrationNode._fallback_pose(
        state, 1_100_000_000
    ) == pytest.approx((1.2, 2.0, 0.0))


def test_prior_spike_and_return_are_both_absorbed():
    state = prior_tracking_state()

    CalibrationNode._on_prior(
        state, make_prior(20.0, 20.0, 0.0, 1_100_000_000)
    )
    CalibrationNode._on_prior(
        state, make_prior(10.1, 20.0, 0.0, 1_200_000_000)
    )

    assert CalibrationNode._fallback_pose(state, 1_200_000_000) == state._pose

    CalibrationNode._on_prior(
        state, make_prior(10.2, 20.0, 0.0, 1_300_000_000)
    )
    assert CalibrationNode._fallback_pose(
        state, 1_300_000_000
    ) == pytest.approx((1.1, 2.0, 0.0))


def test_duplicate_or_out_of_order_prior_is_ignored():
    state = prior_tracking_state()
    original_pose = state._latest_prior_pose
    original_twist = state._latest_prior_twist
    original_anchor = state._tracking_prior_anchor

    CalibrationNode._on_prior(
        state, make_prior(30.0, 40.0, 1.0, 1_000_000_000)
    )

    assert state._latest_prior_pose is original_pose
    assert state._latest_prior_twist is original_twist
    assert state._tracking_prior_anchor is original_anchor


def test_long_prior_gap_invalidates_fallback_anchor():
    state = prior_tracking_state()

    CalibrationNode._on_prior(
        state, make_prior(10.5, 20.0, 0.0, 2_100_000_000)
    )

    assert state._latest_prior_pose == pytest.approx((10.5, 20.0, 0.0))
    assert state._tracking_pose_anchor is None
    assert state._tracking_prior_anchor is None
    assert state._latest_prior_twist is None
    assert CalibrationNode._fallback_pose(state, 2_100_000_000) is None


def test_physically_continuous_prior_after_short_gap_keeps_anchor():
    state = prior_tracking_state()

    CalibrationNode._on_prior(
        state, make_prior(10.5, 20.0, 0.0, 1_300_000_000)
    )

    assert state._tracking_pose_anchor == (1.0, 2.0, 0.0)
    assert state._tracking_prior_anchor == (10.0, 20.0, 0.0)
    assert CalibrationNode._fallback_pose(
        state, 1_300_000_000
    ) == pytest.approx((1.5, 2.0, 0.0))


def test_bounded_pose_recovery_finds_xy_and_yaw_correction():
    expected = fit_result(
        (0.35, 0.0, 0.28), rms=0.04, wall_counts=(30, 30, 20, 0)
    )
    fitter = ScriptedPoseRecoveryFitter(
        {(0.2, 0.0, 0.15): expected}
    )

    result = fit_with_bounded_pose_recovery(
        fitter,
        np.zeros((50, 2)),
        (0.0, 0.0, 0.0),
        xy_offsets_m=(-0.2, 0.0, 0.2),
        yaw_offsets_rad=(-0.15, 0.0, 0.15),
        maximum_rms_error_m=0.10,
        maximum_position_correction_m=0.40,
        maximum_yaw_correction_rad=0.30,
        rms_tie_tolerance_m=0.005,
        ambiguity_position_m=0.10,
        ambiguity_yaw_rad=0.10,
    )

    assert result is expected


def test_bounded_pose_recovery_rejects_ambiguous_near_best_solutions():
    fitter = ScriptedPoseRecoveryFitter(
        {
            (-0.2, 0.0, 0.0): fit_result((-0.3, 0.0, 0.0), rms=0.020),
            (0.2, 0.0, 0.0): fit_result((0.3, 0.0, 0.0), rms=0.022),
        }
    )

    result = fit_with_bounded_pose_recovery(
        fitter,
        np.zeros((50, 2)),
        (0.0, 0.0, 0.0),
        xy_offsets_m=(-0.2, 0.0, 0.2),
        yaw_offsets_rad=(0.0,),
        maximum_rms_error_m=0.08,
        maximum_position_correction_m=0.40,
        maximum_yaw_correction_rad=0.30,
        rms_tie_tolerance_m=0.005,
        ambiguity_position_m=0.10,
        ambiguity_yaw_rad=0.10,
    )

    assert result is None


def test_recovery_candidate_requires_three_motion_consistent_frames():
    state = prior_tracking_state()
    state._recovery_confirmation_frames = 3
    state._recovery_consistency_position_m = 0.30
    state._recovery_consistency_yaw_rad = 0.20

    assert not CalibrationNode._confirm_recovery_candidate(
        state, (5.0, 6.0, 0.1), 1_000_000_000
    )
    state._latest_prior_pose = (10.1, 20.0, 0.0)
    state._latest_prior_stamp_ns = 1_100_000_000
    assert not CalibrationNode._confirm_recovery_candidate(
        state, (5.1, 6.0, 0.1), 1_100_000_000
    )
    state._latest_prior_pose = (10.2, 20.0, 0.0)
    state._latest_prior_stamp_ns = 1_200_000_000
    assert CalibrationNode._confirm_recovery_candidate(
        state, (5.2, 6.0, 0.1), 1_200_000_000
    )


def test_same_stamp_cannot_advance_recovery_confirmation():
    state = prior_tracking_state()
    state._recovery_confirmation_frames = 3
    state._recovery_consistency_position_m = 0.30
    state._recovery_consistency_yaw_rad = 0.20

    assert not CalibrationNode._confirm_recovery_candidate(
        state, (5.0, 6.0, 0.1), 1_000_000_000
    )
    assert not CalibrationNode._confirm_recovery_candidate(
        state, (5.0, 6.0, 0.1), 1_000_000_000
    )
    assert state._pending_recovery_count == 1


def test_fit_accepts_bounded_recovery_only_after_three_consistent_frames():
    fitter = AlwaysRecoveryFitter(
        fit_result((5.0, 6.0, 0.28), rms=0.04, wall_counts=(30, 30, 20, 0))
    )
    state = CalibrationState(
        _predicted_pose=(4.65, 6.0, 0.0),
        _fit_initial_poses=lambda _stamp: (state._predicted_pose,),
        _fallback_pose=lambda _stamp: state._predicted_pose,
        _fitter=fitter,
        _two_wall_fitter=None,
        _maximum_rms_error_m=0.10,
        _yaw_recovery_enabled=False,
        _fallback_recovery_enabled=True,
        _fallback_recovery_trigger_frames=1,
        _fallback_recovery_xy_offsets_m=(-0.2, 0.0, 0.2),
        _fallback_recovery_yaw_offsets_rad=(-0.15, 0.0, 0.15),
        _fallback_recovery_maximum_position_correction_m=0.40,
        _fallback_recovery_maximum_yaw_correction_rad=0.30,
        _yaw_recovery_rms_tie_tolerance_m=0.005,
        _recovery_ambiguity_position_m=0.10,
        _recovery_ambiguity_yaw_rad=0.10,
        _recovery_consistency_position_m=0.30,
        _recovery_consistency_yaw_rad=0.20,
        _recovery_confirmation_frames=3,
        _consecutive_fit_failures=0,
        _pending_recovery_pose=None,
        _pending_recovery_prior=None,
        _pending_recovery_stamp_ns=None,
        _pending_recovery_count=0,
        _latest_prior_pose=(10.0, 20.0, 0.0),
        _latest_prior_stamp_ns=1_000_000_000,
        _maximum_prior_age_ns=250_000_000,
    )

    first = CalibrationNode._fit(state, np.zeros((50, 2)), 1_000_000_000)
    state._predicted_pose = (4.75, 6.0, 0.0)
    state._latest_prior_pose = (10.1, 20.0, 0.0)
    state._latest_prior_stamp_ns = 1_100_000_000
    fitter.result = fit_result(
        (5.1, 6.0, 0.28), rms=0.04, wall_counts=(30, 30, 20, 0)
    )
    second = CalibrationNode._fit(state, np.zeros((50, 2)), 1_100_000_000)
    state._predicted_pose = (4.85, 6.0, 0.0)
    state._latest_prior_pose = (10.2, 20.0, 0.0)
    state._latest_prior_stamp_ns = 1_200_000_000
    fitter.result = fit_result(
        (5.2, 6.0, 0.28), rms=0.04, wall_counts=(30, 30, 20, 0)
    )
    third = CalibrationNode._fit(state, np.zeros((50, 2)), 1_200_000_000)

    assert first is None
    assert second is None
    assert third is fitter.result
    assert state._last_fit_mode == "bounded_recovery"
    assert state._consecutive_fit_failures == 0
    assert state._pending_recovery_pose is None


def test_fallback_budget_stops_publishing_but_keeps_recovery_seed_moving():
    publisher = RecordingPublisher()
    scan = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0))
    )
    state = SimpleNamespace(
        _last_stamp_ns=None,
        _last_fit_stamp_ns=1_000_000_000,
        _maximum_fallback_duration_ns=500_000_000,
        _pose=(1.0, 2.0, 0.0),
        _scan_points=lambda _: np.zeros((50, 2)),
        _fit=lambda _points, _stamp_ns: None,
        _fallback_pose=lambda _stamp_ns: (2.0, 2.0, 0.0),
        _publisher=publisher,
        _odometry=lambda _scan, rms_error_m: SimpleNamespace(
            pose=state._pose, rms_error_m=rms_error_m
        ),
        _reset_tracking=lambda: None,
        get_logger=lambda: SilentLogger(),
    )

    CalibrationNode._process_scan(state, scan)

    assert state._pose == pytest.approx((2.0, 2.0, 0.0))
    assert publisher.messages == []
