import math

import pytest
import numpy as np

from calibration.correction_ekf import PoseCorrectionEkf


def ros_covariance(x_variance, y_variance, yaw_variance):
    covariance = [0.0] * 36
    covariance[0] = x_variance
    covariance[7] = y_variance
    covariance[35] = yaw_variance
    return covariance


def test_initializes_planar_covariance_from_odometry_pose_covariance():
    ekf = PoseCorrectionEkf()
    covariance = ros_covariance(0.001, 0.002, 0.5)

    ekf.predict((0.0, 0.0, 0.0), 0.0, pose_covariance=covariance)

    np.testing.assert_allclose(np.diag(ekf.covariance), [0.001, 0.002, 0.5])


def test_integrates_twist_covariance_into_process_noise():
    ekf = PoseCorrectionEkf(
        process_position_variance_per_sec=0.0,
        process_yaw_variance_per_sec=0.0,
    )
    zero_pose_covariance = ros_covariance(1.0e-9, 1.0e-9, 1.0e-9)
    twist_covariance = ros_covariance(0.04, 0.09, 0.16)
    ekf.predict(
        (0.0, 0.0, 0.0), 0.0, pose_covariance=zero_pose_covariance
    )

    ekf.predict(
        (0.0, 0.0, 0.0), 0.5, twist_covariance=twist_covariance
    )

    np.testing.assert_allclose(
        np.diag(ekf.covariance),
        np.asarray([1.0e-9, 1.0e-9, 1.0e-9]) + np.asarray([0.04, 0.09, 0.16]) * 0.25,
        atol=1.0e-10,
    )


def test_output_covariance_accounts_for_rate_limited_lag():
    ekf = PoseCorrectionEkf(maximum_output_position_rate_m_s=0.1)
    ekf.predict((0.0, 0.0, 0.0), 0.0)
    ekf.correct((1.0, 0.0, 0.0), rms_error_m=0.01, match_count=40)

    assert ekf.output_covariance()[0, 0] > ekf.covariance[0, 0]


def test_lane_measurement_is_applied_gradually_without_teleporting():
    ekf = PoseCorrectionEkf(
        maximum_output_position_rate_m_s=0.20,
        maximum_output_yaw_rate_rad_s=0.10,
    )
    ekf.predict((0.0, 0.0, 0.0), 0.0)
    ekf.correct((1.0, 0.0, 0.5), rms_error_m=0.02, match_count=30)

    assert ekf.output_pose == pytest.approx((0.0, 0.0, 0.0))
    ekf.advance_output(0.1)

    assert ekf.output_pose[0] == pytest.approx(0.02)
    assert ekf.output_pose[1] == pytest.approx(0.0)
    assert ekf.output_pose[2] == pytest.approx(0.01)


def test_output_keeps_correction_and_follows_only_odom_motion_without_measurement():
    ekf = PoseCorrectionEkf(
        maximum_output_position_rate_m_s=1.0,
        maximum_output_yaw_rate_rad_s=1.0,
    )
    ekf.predict((0.0, 0.0, 0.0), 0.0)
    ekf.correct((0.2, 0.0, 0.0), rms_error_m=0.01, match_count=40)
    for _ in range(10):
        ekf.advance_output(0.1)
    correction_before_motion = ekf.output_pose[0]

    ekf.predict((1.0, 0.0, 0.0), 0.1)
    ekf.advance_output(0.1)

    assert ekf.output_pose[0] == pytest.approx(1.0 + correction_before_motion)
    assert ekf.output_pose[1] == pytest.approx(0.0)


def test_prediction_rotates_odom_delta_by_corrected_heading():
    ekf = PoseCorrectionEkf(
        maximum_output_position_rate_m_s=10.0,
        maximum_output_yaw_rate_rad_s=10.0,
    )
    ekf.predict((0.0, 0.0, 0.0), 0.0)
    ekf.correct((0.0, 0.0, math.pi / 2.0), rms_error_m=0.001, match_count=100)
    ekf.advance_output(1.0)
    yaw = ekf.output_pose[2]

    ekf.predict((1.0, 0.0, 0.0), 0.1)

    assert ekf.output_pose[0] == pytest.approx(math.cos(yaw), abs=1e-6)
    assert ekf.output_pose[1] == pytest.approx(math.sin(yaw), abs=1e-6)


def test_local_lane_residual_never_reanchors_to_absolute_raw_pose():
    ekf = PoseCorrectionEkf(
        maximum_output_position_rate_m_s=10.0,
        maximum_output_yaw_rate_rad_s=10.0,
    )
    ekf.predict((0.0, 0.0, 0.0), 0.0)
    ekf.correct_local_residual(
        0.20, 0.0, rms_error_m=0.001, match_count=100
    )
    ekf.advance_output(1.0)
    held_y = ekf.output_pose[1]

    ekf.predict((1.0, 0.0, 0.0), 0.1)
    ekf.correct_local_residual(
        0.0, 0.0, rms_error_m=0.001, match_count=100
    )
    ekf.advance_output(1.0)

    assert ekf.output_pose[0] == pytest.approx(1.0, abs=1e-6)
    assert ekf.output_pose[1] == pytest.approx(held_y)
