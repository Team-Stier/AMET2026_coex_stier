import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from control.control_node import (
    ControlNode,
    DEFAULT_CALIBRATED_POSE_ORIGIN_X_FROM_LIDAR_M,
    reference_offset_from_calibrated_pose,
)


class Recorder:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


@pytest.mark.parametrize(
    ("reference_x_from_lidar_m", "expected_offset_m"),
    [
        (0.027, 0.0),
        (0.0, -0.027),
        (0.10, 0.073),
        (-0.10, -0.127),
    ],
)
def test_reference_offset_uses_lidar_origin(
    reference_x_from_lidar_m, expected_offset_m
):
    assert reference_offset_from_calibrated_pose(
        reference_x_from_lidar_m
    ) == pytest.approx(expected_offset_m)


def test_reference_offset_rejects_nonfinite_value():
    with pytest.raises(ValueError, match="must be finite"):
        reference_offset_from_calibrated_pose(math.nan)


def test_control_node_latches_first_true_gosign():
    rclpy.init()
    node = ControlNode()
    node.speed_pub = Recorder()
    node.steering_pub = Recorder()
    node.camera_pan_pub = Recorder()
    try:
        assert node.reference_point_x_from_lidar_m == pytest.approx(
            DEFAULT_CALIBRATED_POSE_ORIGIN_X_FROM_LIDAR_M
        )
        assert node.calibrated_pose_origin_x_from_lidar_m == pytest.approx(
            DEFAULT_CALIBRATED_POSE_ORIGIN_X_FROM_LIDAR_M
        )
        assert (
            node.controller.pure_pursuit.config.reference_point_offset_m
            == pytest.approx(0.0)
        )

        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        pose.twist.twist.linear.x = 0.7
        node.on_pose(pose)
        assert node.vehicle_state.speed == pytest.approx(0.7)

        path = Path()
        path.header.frame_id = "map"
        path.poses = [PoseStamped(), PoseStamped()]
        path.poses[1].pose.position.x = 1.0

        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.0)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)

        node.on_gosign(Bool(data=True))
        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.55)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
        assert node.camera_pan_pub.values[-1] == pytest.approx(0.0)

        published_speed_count = len(node.speed_pub.values)
        node.on_gosign(Bool(data=False))
        assert len(node.speed_pub.values) == published_speed_count

        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.55)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_node_stops_on_nonfinite_calibrated_speed():
    rclpy.init()
    node = ControlNode()
    node.speed_pub = Recorder()
    node.steering_pub = Recorder()
    node.camera_pan_pub = Recorder()
    try:
        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        pose.twist.twist.linear.x = math.nan

        node.on_pose(pose)

        assert node.vehicle_state is None
        assert node.speed_pub.values[-1] == pytest.approx(0.0)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_node_handles_large_finite_quaternion_without_overflow():
    rclpy.init()
    node = ControlNode()
    node.speed_pub = Recorder()
    node.steering_pub = Recorder()
    node.camera_pan_pub = Recorder()
    try:
        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0e308
        pose.twist.twist.linear.x = 0.5

        node.on_pose(pose)

        assert node.vehicle_state is not None
        assert node.vehicle_state.yaw == pytest.approx(0.0)
        assert node.vehicle_state.speed == pytest.approx(0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_node_rejects_stale_twist_covariance_marker():
    rclpy.init()
    node = ControlNode()
    node.speed_pub = Recorder()
    node.steering_pub = Recorder()
    node.camera_pan_pub = Recorder()
    try:
        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        pose.twist.twist.linear.x = 0.0
        pose.twist.covariance[0] = 1.0e6

        node.on_pose(pose)

        assert node.vehicle_state is None
        assert node.speed_pub.values[-1] == pytest.approx(0.0)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_watchdog_stops_when_calibrated_pose_is_stale():
    rclpy.init()
    node = ControlNode()
    node.speed_pub = Recorder()
    node.steering_pub = Recorder()
    node.camera_pan_pub = Recorder()
    try:
        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        pose.twist.twist.linear.x = 0.5
        node.on_pose(pose)

        path = Path()
        path.header.frame_id = "map"
        path.poses = [PoseStamped(), PoseStamped()]
        path.poses[1].pose.position.x = 1.0
        node.on_gosign(Bool(data=True))
        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.55)

        node.last_pose_time = (
            time.monotonic() - node.pose_timeout_sec - 0.1
        )
        node.on_watchdog()

        assert node.vehicle_state is None
        assert node.speed_pub.values[-1] == pytest.approx(0.0)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_watchdog_stops_when_path_is_stale():
    rclpy.init()
    node = ControlNode()
    node.speed_pub = Recorder()
    node.steering_pub = Recorder()
    node.camera_pan_pub = Recorder()
    try:
        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        node.on_pose(pose)

        path = Path()
        path.header.frame_id = "map"
        path.poses = [PoseStamped(), PoseStamped()]
        path.poses[1].pose.position.x = 1.0
        node.on_gosign(Bool(data=True))
        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.55)

        node.last_path_time = (
            time.monotonic() - node.path_timeout_sec - 0.1
        )
        node.on_watchdog()

        assert node.vehicle_state is not None
        assert node.speed_pub.values[-1] == pytest.approx(0.0)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_node_applies_parameter_overrides():
    rclpy.init()
    node = ControlNode(
        parameter_overrides=[
            Parameter("target_speed_m_s", value=1.2),
            Parameter("max_speed_m_s", value=2.5),
            Parameter("pose_timeout_sec", value=0.7),
            Parameter("path_timeout_sec", value=0.8),
            Parameter("watchdog_period_sec", value=0.2),
            Parameter("maximum_speed_variance_m2_s2", value=0.5),
            Parameter(
                "calibrated_pose.origin_x_from_lidar_m", value=0.03
            ),
            Parameter("speed_lookahead.enabled", value=True),
            Parameter("speed_lookahead.lookahead_time_sec", value=0.6),
            Parameter("speed_lookahead.min_lookahead_m", value=0.3),
            Parameter("speed_lookahead.max_lookahead_m", value=1.1),
            Parameter(
                "camera_pan_command_rad", value=0.1
            ),
            Parameter(
                "pure_pursuit.lookahead_distance_m", value=0.7
            ),
            Parameter(
                "pure_pursuit.reference_point_x_from_lidar_m", value=-0.12
            ),
            Parameter(
                "longitudinal_pid.enabled", value=True
            ),
            Parameter("longitudinal_pid.kp", value=0.4),
            Parameter(
                "adaptive_control.enabled", value=True
            ),
        ]
    )
    try:
        assert node.target_speed == pytest.approx(1.2)
        assert node.camera_pan_command == pytest.approx(0.1)
        assert node.pose_timeout_sec == pytest.approx(0.7)
        assert node.path_timeout_sec == pytest.approx(0.8)
        assert node.watchdog_period_sec == pytest.approx(0.2)
        assert node.maximum_speed_variance_m2_s2 == pytest.approx(0.5)
        assert node.calibrated_pose_origin_x_from_lidar_m == pytest.approx(
            0.03
        )
        assert node.controller.config.speed_lookahead.enabled is True
        assert (
            node.controller.config.speed_lookahead.lookahead_time_sec
            == pytest.approx(0.6)
        )
        assert (
            node.controller.config.speed_lookahead.min_lookahead_m
            == pytest.approx(0.3)
        )
        assert (
            node.controller.config.speed_lookahead.max_lookahead_m
            == pytest.approx(1.1)
        )
        assert node.controller.config.max_speed_m_s == pytest.approx(2.5)
        assert node.controller.config.longitudinal_pid_enabled is True
        assert node.controller.config.adaptive_control.enabled is True
        assert node.controller.pid.config.kp == pytest.approx(0.4)
        assert (
            node.controller.pure_pursuit.config.lookahead_distance_m
            == pytest.approx(0.7)
        )
        assert node.reference_point_x_from_lidar_m == pytest.approx(-0.12)
        assert (
            node.controller.pure_pursuit.config.reference_point_offset_m
            == pytest.approx(-0.12 - 0.03)
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
