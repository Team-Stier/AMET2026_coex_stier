from __future__ import annotations

import copy
import csv
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


MAP_FRAME = "map"
LIDAR_FRAME = "lidar_link"
RAW_ODOMETRY_TOPIC = "/odom/laser"
CALIBRATED_POSE_TOPIC = "/pose/calibration"
TF_SOURCES = (RAW_ODOMETRY_TOPIC, CALIBRATED_POSE_TOPIC)
DEFAULT_LIDAR_OFFSET_X_M = -0.027
HALF_SQRT_TWO = math.sqrt(0.5)


def with_z_clockwise_90(orientation: Quaternion) -> Quaternion:
    # RotZ(-90°) ⊗ q_odom: express the odometry orientation in the SIM map axes.
    return Quaternion(
        x=HALF_SQRT_TWO * (orientation.x + orientation.y),
        y=HALF_SQRT_TWO * (orientation.y - orientation.x),
        z=HALF_SQRT_TWO * (orientation.z - orientation.w),
        w=HALF_SQRT_TWO * (orientation.w + orientation.z),
    )


def load_origin(path: Path) -> tuple[float, float]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {"x_m", "y_m"}.issubset(reader.fieldnames):
            raise ValueError(f"{path} must contain x_m and y_m columns")
        row = next(reader, None)
    if row is None:
        raise ValueError(f"{path} must contain at least one point")
    origin = (float(row["x_m"]), float(row["y_m"]))
    if not all(math.isfinite(value) for value in origin):
        raise ValueError(f"{path} first point must be finite")
    return origin


def validate_tf_source(value: object) -> str:
    if not isinstance(value, str) or value not in TF_SOURCES:
        raise ValueError(
            f"tf_source must be one of {TF_SOURCES}, got {value!r}"
        )
    return value


def validate_lidar_offset_x_m(value: object) -> float:
    offset = float(value)
    if not math.isfinite(offset):
        raise ValueError(f"lidar_offset_x_m must be finite, got {value!r}")
    return offset


def transform_from_odometry(
    message: Odometry, lidar_offset_x_m: float = 0.0
) -> TransformStamped:
    transform = TransformStamped()
    transform.header = copy.deepcopy(message.header)
    transform.child_frame_id = message.child_frame_id
    orientation = message.pose.pose.orientation
    x = message.pose.pose.position.x
    y = message.pose.pose.position.y
    if lidar_offset_x_m:
        yaw = 2.0 * math.atan2(orientation.z, orientation.w)
        x += math.cos(yaw) * lidar_offset_x_m
        y += math.sin(yaw) * lidar_offset_x_m
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = message.pose.pose.position.z
    transform.transform.rotation = copy.deepcopy(orientation)
    return transform


def convert_odometry(
    message: Odometry, origin: tuple[float, float]
) -> tuple[Odometry, TransformStamped]:
    converted = copy.deepcopy(message)
    converted.header.frame_id = MAP_FRAME
    converted.child_frame_id = LIDAR_FRAME
    odom_x = message.pose.pose.position.x
    odom_y = message.pose.pose.position.y
    converted.pose.pose.position.x = origin[0] + odom_y
    converted.pose.pose.position.y = origin[1] - odom_x
    converted.pose.pose.orientation = with_z_clockwise_90(
        converted.pose.pose.orientation
    )
    return converted, transform_from_odometry(converted)


class PoseTfNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_tf_node")
        self.declare_parameter("tf_source", RAW_ODOMETRY_TOPIC)
        self._tf_source = validate_tf_source(
            self.get_parameter("tf_source").value
        )
        self.declare_parameter(
            "lidar_offset_x_m", DEFAULT_LIDAR_OFFSET_X_M
        )
        self._lidar_offset_x_m = validate_lidar_offset_x_m(
            self.get_parameter("lidar_offset_x_m").value
        )
        centerline = (
            Path(get_package_share_directory("pose_tf")) / "rddf" / "centerline.csv"
        )
        self._origin = load_origin(centerline)
        self._publisher = self.create_publisher(
            Odometry, "/pose", qos_profile_sensor_data
        )
        self._broadcaster = TransformBroadcaster(self)
        self._raw_subscription = self.create_subscription(
            Odometry,
            RAW_ODOMETRY_TOPIC,
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self._calibrated_subscription = None
        if self._tf_source == CALIBRATED_POSE_TOPIC:
            self._calibrated_subscription = self.create_subscription(
                Odometry,
                CALIBRATED_POSE_TOPIC,
                self._on_calibrated_odometry,
                qos_profile_sensor_data,
            )
        self.get_logger().info(
            f"map origin loaded from {centerline}: {self._origin[0]}, "
            f"{self._origin[1]}; TF source: {self._tf_source}"
        )

    def _on_odometry(self, message: Odometry) -> None:
        converted, transform = convert_odometry(message, self._origin)
        self._publisher.publish(converted)
        if self._tf_source == RAW_ODOMETRY_TOPIC:
            self._broadcaster.sendTransform(transform)

    def _on_calibrated_odometry(self, message: Odometry) -> None:
        self._broadcaster.sendTransform(
            transform_from_odometry(message, self._lidar_offset_x_m)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseTfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
