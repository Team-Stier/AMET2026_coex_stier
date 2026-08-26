import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from interfaces.msg import Objects, SearchTree
from nav_msgs.msg import Odometry

from visualizer.visualizer import (
    GLOBAL_FRAME,
    LOCAL_FRAME,
    MARKER_COLORS,
    MARKER_TOPICS,
    Visualizer,
    load_xy_csv,
    object_marker,
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


def test_calibration_pose_gets_matching_distinct_ego_marker():
    pose_messages = []
    calibration_messages = []
    node = SimpleNamespace(
        _marker_publishers={
            "pose": SimpleNamespace(publish=pose_messages.append),
            "calibration": SimpleNamespace(
                publish=calibration_messages.append
            ),
        }
    )
    pose = Pose()
    pose.position.x = 1.0
    pose.orientation.w = 1.0
    stamp = Time(sec=3)

    Visualizer._publish_marker(node, "pose", pose, stamp)
    Visualizer._publish_marker(node, "calibration", pose, stamp)

    pose_marker = pose_messages[0]
    calibration_marker = calibration_messages[0]
    assert MARKER_TOPICS["calibration"] == "/rddf/ego_marker_calibration"
    assert calibration_marker.header == pose_marker.header
    assert calibration_marker.ns == pose_marker.ns == "ego"
    assert calibration_marker.pose == pose_marker.pose
    assert calibration_marker.type == pose_marker.type
    assert calibration_marker.action == pose_marker.action
    assert calibration_marker.scale == pose_marker.scale
    assert calibration_marker.lifetime == pose_marker.lifetime
    assert calibration_marker.color.a == pose_marker.color.a
    assert calibration_marker.id != pose_marker.id
    assert (
        calibration_marker.color.r,
        calibration_marker.color.g,
        calibration_marker.color.b,
    ) == MARKER_COLORS["calibration"]
    assert calibration_marker.color != pose_marker.color


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


def test_object_info_becomes_lidar_attached_spheres():
    objects = Objects()
    objects.header.frame_id = LOCAL_FRAME
    objects.header.stamp.sec = 7
    objects.length = 2
    objects.x[0], objects.y[0] = 1.0, 0.25
    objects.x[1], objects.y[1] = 2.0, -0.5

    marker = object_marker(objects, 0.25)

    assert marker.header == objects.header
    assert marker.type == marker.SPHERE_LIST
    assert (marker.scale.x, marker.scale.y, marker.scale.z) == (0.5, 0.5, 0.02)
    assert [(point.x, point.y) for point in marker.points] == [
        (1.0, 0.25),
        (2.0, -0.5),
    ]


def test_object_info_rejects_non_lidar_frame():
    objects = Objects()
    objects.header.frame_id = GLOBAL_FRAME

    with pytest.raises(ValueError, match="frame"):
        object_marker(objects, 0.25)
