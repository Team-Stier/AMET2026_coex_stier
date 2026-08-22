import math
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan

from rddf_visualizer.rddf_visualizer_node import (
    create_ego_marker,
    create_map_to_odom_transform,
    load_path,
    project_scan_points,
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

    marker = create_ego_marker(odometry)

    assert marker.header.frame_id == "odom"
    assert marker.pose.position.x == pytest.approx(1.0)
    assert marker.pose.position.y == pytest.approx(2.0)
    assert marker.scale.x == pytest.approx(0.28)
    assert marker.scale.y == pytest.approx(0.20)
    assert marker.scale.z == pytest.approx(0.05)
    assert marker.pose.orientation == odometry.pose.pose.orientation


def test_map_to_odom_transform_aligns_vehicle_poses() -> None:
    odometry = Odometry()
    odometry.header.frame_id = "odom"
    odometry.pose.pose.position.x = 1.0
    odometry.pose.pose.position.y = 2.0
    odometry.pose.pose.orientation.w = 1.0

    transform = create_map_to_odom_transform(
        odometry,
        {"x": 4.0, "y": 6.0, "z": 0.0, "yaw": math.pi / 2.0},
        "map",
    )

    assert transform.header.frame_id == "map"
    assert transform.child_frame_id == "odom"
    assert transform.transform.translation.x == pytest.approx(6.0)
    assert transform.transform.translation.y == pytest.approx(5.0)
    assert transform.transform.rotation.z == pytest.approx(math.sqrt(0.5))
    assert transform.transform.rotation.w == pytest.approx(math.sqrt(0.5))


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
