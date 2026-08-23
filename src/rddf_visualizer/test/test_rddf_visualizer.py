import math
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped
from interfaces.msg import SearchTree
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan

from rddf_visualizer.rddf_visualizer_node import (
    LASER_ODOM_YAW_OFFSET_DEG,
    ODOM_YAW_OFFSET_DEG,
    alignment_translation,
    attach_path_to_world,
    attach_search_tree_to_world,
    create_ego_marker,
    create_pose,
    create_scan_cloud,
    create_search_tree_final_path_marker,
    create_search_tree_marker,
    load_path,
    project_scan_points,
    quaternion_yaw,
    transform_pose,
)


def test_load_path(tmp_path: Path) -> None:
    csv_path = tmp_path / "path.csv"
    csv_path.write_text("x_m,y_m\n0.0,1.0\n2.0,3.0\n", encoding="utf-8")

    message = load_path(csv_path, "map", Clock().now().to_msg())

    assert message.header.frame_id == "map"
    assert [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses] == [
        (0.0, 1.0),
        (2.0, 3.0),
    ]
    assert all(pose.pose.orientation.w == 1.0 for pose in message.poses)


def test_load_path_rejects_wrong_header(tmp_path: Path) -> None:
    csv_path = tmp_path / "path.csv"
    csv_path.write_text("x,y\n0.0,1.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="expected x_m,y_m header"):
        load_path(csv_path, "map", Clock().now().to_msg())


def test_ego_marker_is_centered_on_rear_axle() -> None:
    odometry = Odometry()
    odometry.header.frame_id = "odom"
    odometry.pose.pose.position.x = 1.0
    odometry.pose.pose.position.y = 2.0
    odometry.pose.pose.orientation.w = 1.0

    header = type(odometry.header)(stamp=odometry.header.stamp, frame_id="map")
    marker = create_ego_marker(header, odometry.pose.pose, (1.0, 0.0, 0.0))

    assert marker.header.frame_id == "map"
    assert marker.pose.position.x == pytest.approx(1.0)
    assert marker.pose.position.y == pytest.approx(2.0)
    assert marker.scale.x == pytest.approx(0.28)
    assert marker.scale.y == pytest.approx(0.20)
    assert marker.scale.z == pytest.approx(0.05)
    assert marker.pose.orientation == odometry.pose.pose.orientation

    assert marker.color.r == pytest.approx(1.0)


def test_search_tree_marker_connects_parents_to_children() -> None:
    search_tree = SearchTree()
    search_tree.header.frame_id = "odom"
    search_tree.x = [0.0, 1.0, 2.0]
    search_tree.y = [0.0, 1.0, 0.0]
    search_tree.yaw = [0.0, 0.1, -0.1]
    search_tree.parent_index = [-1, 0, 1]

    marker = create_search_tree_marker(search_tree)

    assert marker.header.frame_id == "odom"
    assert marker.type == marker.LINE_LIST
    assert marker.pose.orientation.w == pytest.approx(1.0)
    assert [(point.x, point.y) for point in marker.points[:4]] == [
        (0.0, 0.0),
        (1.0, 1.0),
        (1.0, 1.0),
        (2.0, 0.0),
    ]
    assert len(marker.points) == 16


def test_search_tree_final_path_follows_parents_from_goal() -> None:
    search_tree = SearchTree(
        x=[0.0, 1.0, 1.0, 2.0],
        y=[0.0, 0.0, 1.0, 0.0],
        yaw=[0.0, 0.0, 0.0, 0.0],
        parent_index=[-1, 0, 0, 1],
        final_node_index=3,
    )

    marker = create_search_tree_final_path_marker(search_tree)

    assert marker.type == marker.LINE_STRIP
    assert [(point.x, point.y) for point in marker.points] == [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
    ]


def test_search_tree_root_is_attached_to_sim_pose_in_map() -> None:
    search_tree = SearchTree(
        x=[2.0, 3.0],
        y=[1.0, 1.0],
        yaw=[0.5, 0.5],
        parent_index=[-1, 0],
    )
    transformed = attach_search_tree_to_world(
        search_tree,
        {"x": 10.0, "y": 20.0, "yaw": math.pi / 2.0},
        "map",
    )
    marker = create_search_tree_marker(transformed)

    assert marker.header.frame_id == "map"
    yaw_offset = math.pi / 2.0 - 0.5
    assert transformed.x == pytest.approx([10.0, 10.0 + math.cos(yaw_offset)])
    assert transformed.y == pytest.approx([20.0, 20.0 + math.sin(yaw_offset)])
    assert transformed.yaw == pytest.approx([math.pi / 2.0, math.pi / 2.0])
    assert len(marker.points) == 10


def test_search_tree_marker_rejects_malformed_tree() -> None:
    search_tree = SearchTree(
        x=[0.0, 1.0],
        y=[0.0, 1.0],
        yaw=[0.0, 0.0],
        parent_index=[-1, 2],
    )

    with pytest.raises(ValueError, match="invalid parent index"):
        create_search_tree_marker(search_tree)


def test_project_scan_points_uses_pose_without_trails() -> None:
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = math.pi / 2.0
    scan.range_min = 0.1
    scan.range_max = 2.0
    scan.ranges = [1.0, 1.0, float("inf")]

    points = project_scan_points(scan, 1.0, 2.0, 0.0, math.pi / 2.0)

    assert points == pytest.approx(
        [(1.0, 2.973, 0.163), (0.0, 1.973, 0.163)]
    )


def test_direct_projection_uses_map_frame_and_yaw_offsets() -> None:
    assert ODOM_YAW_OFFSET_DEG == pytest.approx(-90.0)
    assert LASER_ODOM_YAW_OFFSET_DEG == pytest.approx(-90.0)

    pose = create_pose(1.0, 2.0, 0.0, math.radians(ODOM_YAW_OFFSET_DEG))
    assert pose.position.x == pytest.approx(1.0)
    assert pose.position.y == pytest.approx(2.0)
    assert quaternion_yaw(pose.orientation) == pytest.approx(-math.pi / 2.0)

    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 1.0
    scan.range_min = 0.1
    scan.range_max = 2.0
    scan.ranges = [1.0]
    cloud = create_scan_cloud(scan, 1.0, 2.0, 0.0, -math.pi / 2.0, "map")
    assert cloud.header.frame_id == "map"


def test_initial_alignment_places_rotated_odometry_on_gt() -> None:
    odometry = Odometry()
    odometry.pose.pose.position.x = 0.02
    odometry.pose.pose.position.y = 0.01
    odometry.pose.pose.orientation.w = 1.0
    yaw_offset = -math.pi / 2.0

    translation = alignment_translation(
        odometry.pose.pose, yaw_offset, target_x=1.4, target_y=3.4
    )
    aligned = transform_pose(odometry.pose.pose, yaw_offset, translation)

    assert aligned.position.x == pytest.approx(1.4)
    assert aligned.position.y == pytest.approx(3.4)
    assert quaternion_yaw(aligned.orientation) == pytest.approx(-math.pi / 2.0)


def test_path_root_is_attached_to_sim_ego_pose() -> None:
    path = NavPath()
    path.header.stamp = Clock().now().to_msg()
    root = PoseStamped()
    root.pose = create_pose(3.0, 1.0, 0.0, 0.5)
    child = PoseStamped()
    child.pose = create_pose(4.0, 1.0, 0.0, 0.5)
    path.poses = [root, child]

    transformed = attach_path_to_world(
        path,
        {"x": 10.0, "y": 20.0, "z": 0.0, "yaw": math.pi / 2.0},
        "map",
    )

    yaw_offset = math.pi / 2.0 - 0.5
    assert transformed.header.frame_id == "map"
    assert transformed.poses[0].pose.position.x == pytest.approx(10.0)
    assert transformed.poses[0].pose.position.y == pytest.approx(20.0)
    assert transformed.poses[1].pose.position.x == pytest.approx(
        10.0 + math.cos(yaw_offset)
    )
    assert transformed.poses[1].pose.position.y == pytest.approx(
        20.0 + math.sin(yaw_offset)
    )
    assert quaternion_yaw(transformed.poses[0].pose.orientation) == pytest.approx(
        math.pi / 2.0
    )
