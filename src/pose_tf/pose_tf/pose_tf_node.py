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

    transform = TransformStamped()
    transform.header.stamp = copy.deepcopy(converted.header.stamp)
    transform.header.frame_id = MAP_FRAME
    transform.child_frame_id = LIDAR_FRAME
    transform.transform.translation.x = converted.pose.pose.position.x
    transform.transform.translation.y = converted.pose.pose.position.y
    transform.transform.translation.z = converted.pose.pose.position.z
    transform.transform.rotation = copy.deepcopy(converted.pose.pose.orientation)
    return converted, transform


class PoseTfNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_tf_node")
        centerline = (
            Path(get_package_share_directory("pose_tf")) / "rddf" / "centerline.csv"
        )
        self._origin = load_origin(centerline)
        self._publisher = self.create_publisher(
            Odometry, "/pose", qos_profile_sensor_data
        )
        self._broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry, "/odom/laser", self._on_odometry, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"map origin loaded from {centerline}: {self._origin[0]}, {self._origin[1]}"
        )

    def _on_odometry(self, message: Odometry) -> None:
        converted, transform = convert_odometry(message, self._origin)
        self._publisher.publish(converted)
        self._broadcaster.sendTransform(transform)


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
