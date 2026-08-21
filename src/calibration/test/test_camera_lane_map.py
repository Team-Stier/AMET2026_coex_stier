import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np


SCRIPT = Path(__file__).parents[1] / "tools" / "build_camera_lane_map.py"
SPEC = importlib.util.spec_from_file_location("build_camera_lane_map", SCRIPT)
lane_map = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = lane_map
SPEC.loader.exec_module(lane_map)


def test_interpolates_translation_and_rotation_at_image_timestamp():
    key = ("map", "camera")
    samples = {
        key: [
            lane_map.TransformSample(
                100,
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            lane_map.TransformSample(
                200,
                np.array([2.0, 0.0, 0.0]),
                np.array([0.0, 0.0, np.sin(np.pi / 4.0), np.cos(np.pi / 4.0)]),
            ),
        ]
    }

    matrix, support_age = lane_map.TransformLookup(samples).interpolate(key, 150)

    np.testing.assert_allclose(matrix[:3, 3], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(
        matrix[:3, :3],
        [
            [np.sqrt(0.5), -np.sqrt(0.5), 0.0],
            [np.sqrt(0.5), np.sqrt(0.5), 0.0],
            [0.0, 0.0, 1.0],
        ],
        atol=1e-12,
    )
    assert support_age == 50


def test_can_hold_a_transform_at_its_final_mapping_value():
    key = ("map", "odom")
    samples = {
        key: [
            lane_map.TransformSample(
                100,
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            lane_map.TransformSample(
                200,
                np.array([2.0, 3.0, 0.0]),
                np.array([0.0, 0.0, 0.0, 1.0]),
            ),
        ]
    }

    matrix, support_age = lane_map.TransformLookup(
        samples, fixed_final_keys=(key,)
    ).interpolate(key, 100)

    np.testing.assert_allclose(matrix[:3, 3], [2.0, 3.0, 0.0])
    assert support_age == 0


def test_aligns_sim_world_rigidly_to_the_final_slam_map():
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    samples = {
        ("map", "odom"): [
            lane_map.TransformSample(100, np.array([10.0, 0.0, 0.0]), identity)
        ],
        ("odom", "base_footprint"): [
            lane_map.TransformSample(100, np.array([1.0, 0.0, 0.0]), identity)
        ],
    }
    sim_samples = [
        lane_map.TransformSample(100, np.array([4.0, 0.0, 0.0]), identity)
    ]

    map_from_world = lane_map.align_sim_world_to_final_map(samples, sim_samples)

    np.testing.assert_allclose(map_from_world[:3, 3], [7.0, 0.0, 0.0])
    np.testing.assert_allclose(map_from_world[:3, :3], np.eye(3))


def test_separates_yellow_and_white_pixels():
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    cv2.line(frame, (4, 2), (4, 17), (0, 190, 255), 2)
    cv2.line(frame, (15, 2), (15, 17), (255, 255, 255), 2)

    yellow, white = lane_map.lane_masks(frame)

    assert yellow[:, 3:6].any()
    assert not yellow[:, 14:17].any()
    assert white[:, 14:17].any()
    assert not white[:, 3:6].any()


def test_removes_small_isolated_vote_components():
    mask = np.zeros((10, 10), dtype=bool)
    mask[1, 1] = True
    mask[5:7, 5:7] = True

    filtered = lane_map.remove_small_components(mask, minimum_cells=3)

    assert not filtered[1, 1]
    assert np.count_nonzero(filtered) == 4
