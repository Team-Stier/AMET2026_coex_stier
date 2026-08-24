import math
from types import SimpleNamespace

import numpy as np
import pytest

from calibration.calibration_node import CalibrationNode


def vector(**values):
    defaults = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
    defaults.update(values)
    return SimpleNamespace(**defaults)


def test_transform_matrix_applies_lidar_rotation_and_translation():
    half_yaw = math.pi / 4.0
    transform = SimpleNamespace(
        translation=vector(x=2.0, y=3.0, z=0.5),
        rotation=vector(z=math.sin(half_yaw), w=math.cos(half_yaw)),
    )

    matrix = CalibrationNode._transform_to_matrix(transform)
    transformed = matrix @ np.asarray([1.0, 0.0, 0.0, 1.0])

    np.testing.assert_allclose(transformed, [2.0, 4.0, 0.5, 1.0])


def test_transform_matrix_rejects_zero_quaternion():
    transform = SimpleNamespace(translation=vector(), rotation=vector())

    with pytest.raises(ValueError, match="quaternion"):
        CalibrationNode._transform_to_matrix(transform)
