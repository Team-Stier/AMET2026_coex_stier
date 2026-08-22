from __future__ import annotations

import csv
import json
import math
from pathlib import Path as FilePath
from urllib.request import urlopen

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.transform_broadcaster import TransformBroadcaster
from visualization_msgs.msg import Marker


RDDF_TOPICS = {
    "centerline.csv": "/rddf/centerline",
    "inner_boundary.csv": "/rddf/inner_boundary",
    "outer_boundary.csv": "/rddf/outer_boundary",
}
VEHICLE_LENGTH_M = 0.28
VEHICLE_WIDTH_M = 0.20
MARKER_HEIGHT_M = 0.05
LIDAR_X_M = -0.027
LIDAR_Z_M = 0.163


def load_path(csv_path: FilePath, frame_id: str, stamp) -> Path:
    message = Path()
    message.header.frame_id = frame_id
    message.header.stamp = stamp

    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not {"x_m", "y_m"}.issubset(reader.fieldnames or ()):
            raise ValueError(f"{csv_path}: expected x_m,y_m header")
        for row in reader:
            try:
                x_m, y_m = float(row["x_m"]), float(row["y_m"])
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"{csv_path}: invalid x_m,y_m values on line {reader.line_num}"
                ) from error
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = x_m
            pose.pose.position.y = y_m
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

    if not message.poses:
        raise ValueError(f"{csv_path}: path is empty")
    return message


def create_ego_marker(odometry: Odometry) -> Marker:
    marker = Marker()
    marker.header = odometry.header
    marker.ns = "ego"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    orientation = odometry.pose.pose.orientation
    marker.pose.position.x = odometry.pose.pose.position.x
    marker.pose.position.y = odometry.pose.pose.position.y
    marker.pose.position.z = odometry.pose.pose.position.z + MARKER_HEIGHT_M / 2.0
    marker.pose.orientation = orientation

    marker.scale.x = VEHICLE_LENGTH_M
    marker.scale.y = VEHICLE_WIDTH_M
    marker.scale.z = MARKER_HEIGHT_M
    marker.color.r = 0.1
    marker.color.g = 0.6
    marker.color.b = 1.0
    marker.color.a = 1.0
    return marker


def quaternion_yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


def create_map_to_odom_transform(
    odometry: Odometry, world_pose: dict, frame_id: str
) -> TransformStamped:
    orientation = odometry.pose.pose.orientation
    odom_yaw = quaternion_yaw(orientation)
    yaw = float(world_pose["yaw"]) - odom_yaw
    cosine, sine = math.cos(yaw), math.sin(yaw)
    odom_position = odometry.pose.pose.position

    transform = TransformStamped()
    transform.header.stamp = odometry.header.stamp
    transform.header.frame_id = frame_id
    transform.child_frame_id = odometry.header.frame_id
    transform.transform.translation.x = float(world_pose["x"]) - (
        cosine * odom_position.x - sine * odom_position.y
    )
    transform.transform.translation.y = float(world_pose["y"]) - (
        sine * odom_position.x + cosine * odom_position.y
    )
    transform.transform.translation.z = (
        float(world_pose.get("z", 0.0)) - odom_position.z
    )
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def project_scan_points(
    scan: LaserScan, x: float, y: float, z: float, yaw: float
) -> list[tuple[float, float, float]]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    points = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(
            distance
        ) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        lidar_x = LIDAR_X_M + distance * math.cos(angle)
        lidar_y = distance * math.sin(angle)
        points.append(
            (
                x + cosine * lidar_x - sine * lidar_y,
                y + sine * lidar_x + cosine * lidar_y,
                z + LIDAR_Z_M,
            )
        )
    return points


class RddfVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("rddf_visualizer_node")
        self.declare_parameter("rddf_directory", "rddf")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("enable_sim_gt", False)

        directory = FilePath(str(self.get_parameter("rddf_directory").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.sim_gt_enabled = bool(self.get_parameter("enable_sim_gt").value)
        stamp = self.get_clock().now().to_msg()

        self.tf_broadcaster = None
        self.latest_odometry = None
        self.latest_world_pose = None
        if self.sim_gt_enabled:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.sim_gt_timer = self.create_timer(0.1, self.update_map_to_odom)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        for filename, topic in RDDF_TOPICS.items():
            try:
                path = load_path(directory / filename, self.frame_id, stamp)
            except (OSError, ValueError) as error:
                self.get_logger().fatal(str(error))
                raise
            publisher = self.create_publisher(Path, topic, qos)
            publisher.publish(path)
            self.get_logger().info(
                f"published {len(path.poses)} points from {directory / filename} on {topic}"
            )
        self.ego_marker_pub = self.create_publisher(Marker, "/rddf/ego_marker", qos)
        self.scan_sim_pub = self.create_publisher(PointCloud2, "/rddf/scan_sim", 1)
        self.scan_odom_pub = self.create_publisher(PointCloud2, "/rddf/scan_odom", 1)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odometry, qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

    def on_odometry(self, odometry: Odometry) -> None:
        self.latest_odometry = odometry
        self.ego_marker_pub.publish(create_ego_marker(odometry))

    def on_scan(self, scan: LaserScan) -> None:
        if self.latest_odometry is None:
            return

        pose = self.latest_odometry.pose.pose
        odom_points = project_scan_points(
            scan,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            quaternion_yaw(pose.orientation),
        )
        odom_header = Header(
            stamp=scan.header.stamp,
            frame_id=self.latest_odometry.header.frame_id or "odom",
        )
        self.scan_odom_pub.publish(
            point_cloud2.create_cloud_xyz32(odom_header, odom_points)
        )

        if self.latest_world_pose is None:
            return

        world = self.latest_world_pose
        sim_points = project_scan_points(
            scan,
            float(world["x"]),
            float(world["y"]),
            float(world.get("z", 0.0)),
            float(world["yaw"]),
        )
        sim_header = Header(stamp=scan.header.stamp, frame_id=self.frame_id)
        self.scan_sim_pub.publish(
            point_cloud2.create_cloud_xyz32(sim_header, sim_points)
        )

    def update_map_to_odom(self) -> None:
        if self.latest_odometry is None:
            return
        try:
            with urlopen("http://localhost/sim/api/pose", timeout=0.1) as response:
                world_pose = json.load(response)
            self.latest_world_pose = world_pose
            transform = create_map_to_odom_transform(
                self.latest_odometry, world_pose, self.frame_id
            )
            self.tf_broadcaster.sendTransform(transform)
            self.alignment_error_logged = False
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(
                f"SIM ground truth disabled; continuing odom-only: {error}"
            )
            self.sim_gt_timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RddfVisualizerNode()
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
