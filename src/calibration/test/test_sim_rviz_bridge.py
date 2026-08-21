import math

import pytest

from calibration.pose_geometry import (
    map_from_odom_pose,
    transform_from_local_correction,
    transform_pose_2d,
)


def test_bootstrap_transform_maps_odom_pose_to_world_pose():
    world_pose = (9.4, 5.2, 2.7)
    odom_pose = (1.1, -0.4, -0.2)

    transform = map_from_odom_pose(world_pose, odom_pose)
    recovered = transform_pose_2d(odom_pose, transform)

    assert recovered[0] == pytest.approx(world_pose[0])
    assert recovered[1] == pytest.approx(world_pose[1])
    assert math.atan2(
        math.sin(recovered[2] - world_pose[2]),
        math.cos(recovered[2] - world_pose[2]),
    ) == pytest.approx(0.0)


def test_local_correction_becomes_a_persistent_world_transform():
    raw_at_update = (2.0, 1.0, math.pi / 2.0)
    transform = transform_from_local_correction(
        raw_at_update, lateral_m=0.2, yaw_rad=0.1
    )

    corrected_at_update = transform_pose_2d(raw_at_update, transform)
    corrected_later = transform_pose_2d((2.0, 2.0, math.pi / 2.0), transform)

    assert corrected_at_update[0] == pytest.approx(1.8)
    assert corrected_at_update[1] == pytest.approx(1.0)
    assert corrected_at_update[2] == pytest.approx(math.pi / 2.0 + 0.1)
    assert math.hypot(corrected_later[0] - 2.0, corrected_later[1] - 2.0) > 0.0
