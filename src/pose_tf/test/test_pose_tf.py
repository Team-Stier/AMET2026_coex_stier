import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from pose_tf.pose_tf_node import (
    CALIBRATED_POSE_TOPIC,
    DEFAULT_LIDAR_OFFSET_X_M,
    PoseTfNode,
    RAW_ODOMETRY_TOPIC,
    RAW_POSE_TOPIC,
    convert_odometry,
    load_origin,
    transform_from_odometry,
    validate_lidar_offset_x_m,
    validate_tf_source,
)


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


@pytest.mark.parametrize("source", [RAW_POSE_TOPIC, CALIBRATED_POSE_TOPIC])
def test_validate_tf_source_accepts_supported_topics(source: str) -> None:
    assert validate_tf_source(source) == source


@pytest.mark.parametrize("source", ["/pose/calibride", RAW_ODOMETRY_TOPIC, 1])
def test_validate_tf_source_rejects_other_values(source: object) -> None:
    with pytest.raises(ValueError, match="tf_source must be one of"):
        validate_tf_source(source)


def test_validate_lidar_offset_accepts_default() -> None:
    assert validate_lidar_offset_x_m(DEFAULT_LIDAR_OFFSET_X_M) == -0.027


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_validate_lidar_offset_rejects_nonfinite(value: float) -> None:
    with pytest.raises(ValueError, match="lidar_offset_x_m must be finite"):
        validate_lidar_offset_x_m(value)


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


def test_transform_from_calibrated_odometry_preserves_map_pose() -> None:
    incoming = Odometry()
    incoming.header.stamp.sec = 56
    incoming.header.stamp.nanosec = 78
    incoming.header.frame_id = "map"
    incoming.child_frame_id = "lidar_link"
    incoming.pose.pose.position.x = 4.2
    incoming.pose.pose.position.y = 1.3
    incoming.pose.pose.position.z = 0.1
    incoming.pose.pose.orientation.z = -0.25
    incoming.pose.pose.orientation.w = 0.968

    transform = transform_from_odometry(incoming)

    assert transform.header == incoming.header
    assert transform.child_frame_id == incoming.child_frame_id
    assert transform.transform.translation.x == incoming.pose.pose.position.x
    assert transform.transform.translation.y == incoming.pose.pose.position.y
    assert transform.transform.translation.z == incoming.pose.pose.position.z
    assert transform.transform.rotation == incoming.pose.pose.orientation


def test_calibrated_callback_applies_lidar_offset_along_yaw() -> None:
    incoming = Odometry()
    incoming.header.frame_id = "map"
    incoming.child_frame_id = "lidar_link"
    incoming.pose.pose.position.x = 4.2
    incoming.pose.pose.position.y = 1.3
    incoming.pose.pose.orientation.z = math.sqrt(0.5)
    incoming.pose.pose.orientation.w = math.sqrt(0.5)
    node = SimpleNamespace(
        _lidar_offset_x_m=DEFAULT_LIDAR_OFFSET_X_M,
        _broadcaster=Mock(),
    )

    PoseTfNode._on_calibrated_odometry(node, incoming)
    transform = node._broadcaster.sendTransform.call_args.args[0]

    assert node._broadcaster.sendTransform.call_count == 1
    assert transform.transform.translation.x == pytest.approx(4.2)
    assert transform.transform.translation.y == pytest.approx(1.273)
    assert transform.transform.rotation == incoming.pose.pose.orientation


@pytest.mark.parametrize(
    ("tf_source", "expected_tf_count"),
    [(RAW_POSE_TOPIC, 1), (CALIBRATED_POSE_TOPIC, 0)],
)
def test_raw_odometry_always_publishes_pose_but_only_selected_tf(
    tf_source: str, expected_tf_count: int
) -> None:
    node = SimpleNamespace(
        _origin=(1.4, 3.4),
        _tf_source=tf_source,
        _publisher=Mock(),
        _broadcaster=Mock(),
    )

    PoseTfNode._on_odometry(node, Odometry())

    assert node._publisher.publish.call_count == 1
    assert node._broadcaster.sendTransform.call_count == expected_tf_count
