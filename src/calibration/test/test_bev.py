from types import SimpleNamespace

import numpy as np

from calibration.bev import BevGeometry, CameraModel, transform_to_matrix


def test_camera_matrix_uses_horizontal_fov():
    camera = CameraModel(
        width=480,
        height=360,
        horizontal_fov_rad=np.deg2rad(100.0),
        distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
    )

    expected_focal = 480.0 / (2.0 * np.tan(np.deg2rad(50.0)))
    np.testing.assert_allclose(camera.matrix[0, 0], expected_focal)
    np.testing.assert_allclose(camera.matrix[1, 1], expected_focal)
    np.testing.assert_allclose(camera.matrix[0, 2], 239.5)
    np.testing.assert_allclose(camera.matrix[1, 2], 179.5)


def test_ground_pixel_round_trip():
    geometry = BevGeometry()
    x_m = np.array([0.2, 1.0, 2.8])
    y_m = np.array([1.0, 0.0, -0.8])

    rows, columns = geometry.ground_to_pixel(x_m, y_m)
    recovered_x, recovered_y = geometry.pixel_to_ground(rows, columns)

    np.testing.assert_allclose(recovered_x, x_m)
    np.testing.assert_allclose(recovered_y, y_m)


def test_transform_to_matrix_normalizes_quaternion():
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=2.0),
    )

    matrix = transform_to_matrix(transform)

    np.testing.assert_allclose(matrix, np.array([
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 2.0],
        [0.0, 0.0, 1.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
    ]))
