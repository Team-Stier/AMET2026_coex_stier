import cv2
import numpy as np

from calibration.bev import BevGeometry
from calibration.lane_detector import YellowLaneDetector


def test_extracts_dashed_yellow_lane_points_without_polynomial_fitting():
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
                (87, 118, 130),
                -1,
            )

    detector = YellowLaneDetector(geometry)
    detection = detector.detect(image)

    assert detection is not None
    assert detection.confidence > 0.35
    assert detection.span_m > 2.0
    residuals = np.abs(
        detection.points_m[:, 1] - np.polyval(expected, detection.points_m[:, 0])
    )
    assert np.median(residuals) < 0.03
    assert not hasattr(detection, "coefficients")


def test_preserves_a_sharp_corner_that_a_quadratic_would_smooth():
    geometry = BevGeometry()
    image = np.zeros((geometry.height, geometry.width, 3), dtype=np.uint8)
    corner_points = np.array([[0.4, 0.0], [1.5, 0.0], [1.5, 0.8]])
    rows, columns = geometry.ground_to_pixel(
        corner_points[:, 0], corner_points[:, 1]
    )
    pixels = np.column_stack((columns, rows)).round().astype(np.int32)
    cv2.polylines(image, [pixels], False, (87, 118, 130), 5)

    detection = YellowLaneDetector(geometry).detect(image)

    assert detection is not None
    assert np.any(
        (np.abs(detection.points_m[:, 0] - 1.5) < 0.05)
        & (detection.points_m[:, 1] > 0.6)
    )
    assert np.any(
        (detection.points_m[:, 0] < 0.7)
        & (np.abs(detection.points_m[:, 1]) < 0.05)
    )


def test_rejects_round_orange_blob_without_a_lane():
    geometry = BevGeometry()
    image = np.zeros((geometry.height, geometry.width, 3), dtype=np.uint8)
    cv2.circle(image, (geometry.width // 2, geometry.height // 2), 8, (0, 140, 255), -1)

    detector = YellowLaneDetector(geometry)

    assert detector.detect(image) is None


def test_hsv_range_excludes_an_elongated_saturated_orange_cone():
    geometry = BevGeometry()
    image = np.zeros((geometry.height, geometry.width, 3), dtype=np.uint8)
    cv2.line(image, (20, 15), (20, geometry.height - 15), (87, 118, 130), 4)
    cv2.ellipse(image, (90, 55), (9, 28), 0, 0, 360, (13, 61, 220), -1)

    detection = YellowLaneDetector(geometry).detect(image)

    assert detection is not None
    cone_rows, _ = np.nonzero(detection.mask[:, 80:101])
    assert len(cone_rows) == 0


def test_rejects_image_without_enough_lane_pixels():
    geometry = BevGeometry()
    image = np.zeros((geometry.height, geometry.width, 3), dtype=np.uint8)
    detector = YellowLaneDetector(geometry)

    assert detector.detect(image) is None
