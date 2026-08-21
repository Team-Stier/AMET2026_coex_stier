import cv2
import numpy as np

from calibration.lane_map import (
    LaneReference,
    load_lane_reference_image,
    transform_reference,
)


def test_loads_yellow_pixels_as_metric_map_points(tmp_path):
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    cv2.line(image, (20, 70), (180, 70), (0, 190, 255), 3)
    path = tmp_path / "lane.png"
    assert cv2.imwrite(str(path), image)

    reference = load_lane_reference_image(
        str(path),
        resolution_m=0.01,
        point_spacing_m=0.02,
        tangent_radius_m=0.08,
    )

    assert len(reference.points) > 50
    np.testing.assert_allclose(np.median(reference.points[:, 1]), 0.29, atol=0.02)
    assert reference.points[:, 0].min() < 0.25
    assert reference.points[:, 0].max() > 1.75
    assert np.median(np.abs(reference.tangents[:, 0])) > 0.95


def test_transforms_reference_points_and_tangents():
    reference = LaneReference(
        points=np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]),
        tangents=np.array([[1.0, 0.0], [1.0, 0.0], [1.0, 0.0]]),
    )
    transform = np.array(
        [
            [0.0, -1.0, 0.0, 2.0],
            [1.0, 0.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    transformed = transform_reference(reference, transform)

    np.testing.assert_allclose(transformed.points, [[2.0, 3.0], [2.0, 4.0], [2.0, 5.0]])
    np.testing.assert_allclose(transformed.tangents, [[0.0, 1.0]] * 3)
