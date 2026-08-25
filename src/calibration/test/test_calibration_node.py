from types import SimpleNamespace

from calibration.calibration_node import CalibrationNode


def test_tracking_reset_clears_pose_prior():
    state = SimpleNamespace(
        _initial_pose=(1.4, 3.427, -1.57),
        _pose=(8.0, 5.0, 1.0),
        _latest_prior_pose=(7.9, 5.1, 1.1),
        _latest_prior_stamp_ns=123,
    )

    CalibrationNode._reset_tracking(state)

    assert state._pose == state._initial_pose
    assert state._latest_prior_pose is None
    assert state._latest_prior_stamp_ns is None
