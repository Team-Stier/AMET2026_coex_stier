import cv2
import numpy as np

from calibration.bev import BevGeometry
from calibration.lane_detector import YellowLaneDetector


def test_detects_dashed_yellow_quadratic_lane():
    geometry = BevGeometry(
        x_min_m=0.15,
        x_max_m=3.0,
        y_left_m=1.2,
        y_right_m=1.2,
        resolution_m=0.02,
    )
    image = np.zeros((geometry.height, geometry.width, 3), dtype=np.uint8)
    expected = np.array([0.025, -0.04, 0.12])
    x_values = np.linspace(0.3, 2.8, 140)
    y_values = np.polyval(expected, x_values)
    rows, columns = geometry.ground_to_pixel(x_values, y_values)

    for index, (row, column) in enumerate(zip(rows, columns)):
        if (index // 8) % 2 == 0:
            cv2.circle(
                image,
                (int(round(column)), int(round(row))),
                2,
                (0, 190, 255),
                -1,
            )

    detector = YellowLaneDetector(geometry)
    detection = detector.detect(image)

    assert detection is not None
    assert detection.confidence > 0.35
    assert detection.x_max_m - detection.x_min_m > 2.0
    np.testing.assert_allclose(detection.coefficients, expected, atol=0.03)


def test_rejects_image_without_enough_lane_pixels():
    geometry = BevGeometry()
    image = np.zeros((geometry.height, geometry.width, 3), dtype=np.uint8)
    detector = YellowLaneDetector(geometry)

    assert detector.detect(image) is None


def test_preserves_thin_dashed_yellow_lane():
    geometry = BevGeometry()
    image = np.zeros((geometry.height, geometry.width, 3), dtype=np.uint8)
    x_values = np.linspace(0.3, 2.8, 140)
    rows, columns = geometry.ground_to_pixel(x_values, np.zeros_like(x_values))

    for index, (row, column) in enumerate(zip(rows, columns)):
        if (index // 8) % 2 == 0:
            image[int(round(row)), int(round(column))] = (0, 190, 255)

    detector = YellowLaneDetector(geometry)
    detection = detector.detect(image)

    assert detection is not None
    assert detection.confidence > 0.35
    np.testing.assert_allclose(detection.coefficients, [0.0, 0.0, 0.0], atol=0.02)
