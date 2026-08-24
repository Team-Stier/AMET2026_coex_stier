import math
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry

from pose_tf.pose_tf_node import convert_odometry, load_origin


def test_load_origin_uses_first_centerline_point(tmp_path: Path) -> None:
    centerline = tmp_path / "centerline.csv"
    centerline.write_text(
        "x_m,y_m\n1.399976,3.402105\n9.0,10.0\n", encoding="utf-8"
    )

    assert load_origin(centerline) == (1.399976, 3.402105)


def test_load_origin_rejects_empty_centerline(tmp_path: Path) -> None:
    centerline = tmp_path / "centerline.csv"
    centerline.write_text("x_m,y_m\n", encoding="utf-8")

    with pytest.raises(ValueError, match="at least one point"):
        load_origin(centerline)


def test_convert_odometry_rotates_pose_and_tf_z_clockwise_90() -> None:
    incoming = Odometry()
    incoming.header.stamp.sec = 12
    incoming.header.stamp.nanosec = 34
    incoming.header.frame_id = "odom"
    incoming.child_frame_id = "sensor"
    incoming.pose.pose.position.x = 2.0
    incoming.pose.pose.position.y = -1.0
    incoming.pose.pose.position.z = 0.2
    incoming.pose.pose.orientation.z = 0.6
    incoming.pose.pose.orientation.w = 0.8
    incoming.pose.covariance[0] = 0.25
    incoming.twist.twist.linear.x = 4.0
    incoming.twist.covariance[0] = 0.5

    converted, transform = convert_odometry(incoming, (1.4, 3.4))

    assert converted.header.stamp == incoming.header.stamp
    assert converted.header.frame_id == "map"
    assert converted.child_frame_id == "lidar_link"
    assert converted.pose.pose.position.x == pytest.approx(0.4)
    assert converted.pose.pose.position.y == pytest.approx(1.4)
    assert converted.pose.pose.position.z == incoming.pose.pose.position.z
    half_sqrt_two = math.sqrt(0.5)
    assert converted.pose.pose.orientation.x == pytest.approx(0.0)
    assert converted.pose.pose.orientation.y == pytest.approx(0.0)
    assert converted.pose.pose.orientation.z == pytest.approx(-0.2 * half_sqrt_two)
    assert converted.pose.pose.orientation.w == pytest.approx(1.4 * half_sqrt_two)
    assert converted.pose.covariance.tolist() == incoming.pose.covariance.tolist()
    assert converted.twist.twist == incoming.twist.twist
    assert converted.twist.covariance.tolist() == incoming.twist.covariance.tolist()
    assert (incoming.pose.pose.position.x, incoming.pose.pose.position.y) == (2.0, -1.0)

    assert transform.header.stamp == converted.header.stamp
    assert transform.header.frame_id == "map"
    assert transform.child_frame_id == "lidar_link"
    assert transform.transform.translation.x == converted.pose.pose.position.x
    assert transform.transform.translation.y == converted.pose.pose.position.y
    assert transform.transform.translation.z == converted.pose.pose.position.z
    assert transform.transform.rotation == converted.pose.pose.orientation
