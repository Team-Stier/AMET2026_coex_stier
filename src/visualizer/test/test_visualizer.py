import math

import pytest
from interfaces.msg import SearchTree
from nav_msgs.msg import Odometry

from visualizer.visualizer import (
    GLOBAL_FRAME,
    LOCAL_FRAME,
    load_xy_csv,
    path_message,
    pose_from_odometry,
    search_tree_marker,
    sim_pose_from_state,
    stamp_from_seconds,
)


def test_sim_state_keeps_api_pose_and_time():
    pose, stamp = sim_pose_from_state(
        {
            "time": 12.25,
            "vehicle": {
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "q": [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
            },
        }
    )

    assert (stamp.sec, stamp.nanosec) == (12, 250_000_000)
    assert (pose.position.x, pose.position.y) == (1.0, 2.0)
    assert pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_sim_state_accepts_documented_yaw_fallback():
    pose, _ = sim_pose_from_state(
        {"time": 1.0, "vehicle": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": math.pi}}
    )
    assert pose.orientation.z == pytest.approx(1.0)
    assert pose.orientation.w == pytest.approx(0.0, abs=1e-12)


def test_timestamp_rounding_carries_to_next_second():
    stamp = stamp_from_seconds(1.9999999996)
    assert (stamp.sec, stamp.nanosec) == (2, 0)


def test_pose_odometry_keeps_map_lidar_pose_and_normalizes_orientation():
    message = Odometry()
    message.header.frame_id = GLOBAL_FRAME
    message.child_frame_id = LOCAL_FRAME
    message.pose.pose.position.x = 1.0
    message.pose.pose.position.y = 2.0
    message.pose.pose.orientation.z = 1.0
    message.pose.pose.orientation.w = 1.0

    pose = pose_from_odometry(message)

    assert (pose.position.x, pose.position.y) == (1.0, 2.0)
    assert pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_load_rddf_and_build_global_path(tmp_path):
    csv_path = tmp_path / "centerline.csv"
    csv_path.write_text("x_m,y_m\n1.0,2.0\n3.0,4.0\n", encoding="utf-8")

    message = path_message(load_xy_csv(csv_path))

    assert message.header.frame_id == GLOBAL_FRAME
    assert [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses] == [
        (1.0, 2.0),
        (3.0, 4.0),
    ]


def test_rddf_path_keeps_raw_map_coordinates():
    message = path_message([(1.0, 2.0), (3.0, 4.0)])
    positions = [
        (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
    ]

    assert positions == [(1.0, 2.0), (3.0, 4.0)]


def test_search_tree_becomes_map_line_list():
    tree = SearchTree()
    tree.header.frame_id = "map"
    tree.header.stamp.sec = 4
    tree.x = [0.0, 1.0, 1.0]
    tree.y = [0.0, 0.0, 1.0]
    tree.yaw = [0.0, 0.0, math.pi / 2.0]
    tree.parent_index = [-1, 0, 1]
    tree.final_node_index = 2

    marker = search_tree_marker(tree)

    assert marker.header.frame_id == GLOBAL_FRAME
    assert marker.header.stamp == tree.header.stamp
    assert marker.type == marker.LINE_LIST
    assert [(point.x, point.y) for point in marker.points] == [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0, 0.0),
        (1.0, 1.0),
    ]


def test_search_tree_rejects_invalid_parent():
    tree = SearchTree()
    tree.header.frame_id = GLOBAL_FRAME
    tree.x = [0.0, 1.0]
    tree.y = [0.0, 0.0]
    tree.yaw = [0.0, 0.0]
    tree.parent_index = [-1, 2]
    tree.final_node_index = 1

    with pytest.raises(ValueError, match="parent_index"):
        search_tree_marker(tree)


def test_search_tree_rejects_non_map_frame():
    tree = SearchTree()
    tree.header.frame_id = "lidar_link"

    with pytest.raises(ValueError, match="frame"):
        search_tree_marker(tree)
