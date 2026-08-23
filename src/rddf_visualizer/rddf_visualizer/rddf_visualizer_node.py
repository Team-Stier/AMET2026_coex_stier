from __future__ import annotations

import csv
import json
import math
from pathlib import Path as FilePath
from urllib.request import urlopen

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from interfaces.msg import SearchTree
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
ODOM_YAW_OFFSET_DEG = -90.0
LASER_ODOM_YAW_OFFSET_DEG = -90.0


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


def create_ego_marker(
    header: Header, pose: Pose, color: tuple[float, float, float]
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = "ego"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z + MARKER_HEIGHT_M / 2.0
    marker.pose.orientation = pose.orientation

    marker.scale.x = VEHICLE_LENGTH_M
    marker.scale.y = VEHICLE_WIDTH_M
    marker.scale.z = MARKER_HEIGHT_M
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    return marker


def validate_search_tree(search_tree: SearchTree) -> int:
    lengths = {
        len(search_tree.x),
        len(search_tree.y),
        len(search_tree.yaw),
        len(search_tree.parent_index),
    }
    if len(lengths) != 1:
        raise ValueError("SearchTree arrays must have equal lengths")

    node_count = len(search_tree.x)
    for index, (x, y, yaw, parent_index) in enumerate(
        zip(search_tree.x, search_tree.y, search_tree.yaw, search_tree.parent_index)
    ):
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError(f"SearchTree node {index} contains a non-finite pose")
        if index == 0:
            if parent_index != -1:
                raise ValueError("SearchTree root parent must be -1")
            continue
        if not 0 <= parent_index < node_count:
            raise ValueError(f"SearchTree node {index} has an invalid parent index")
    return node_count


def create_search_tree_marker(search_tree: SearchTree) -> Marker:
    node_count = validate_search_tree(search_tree)

    marker = Marker()
    marker.header = search_tree.header
    marker.ns = "search_tree"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.03
    marker.color.r = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    for index in range(1, node_count):
        x = search_tree.x[index]
        y = search_tree.y[index]
        parent_index = search_tree.parent_index[index]
        marker.points.extend(
            (
                Point(
                    x=float(search_tree.x[parent_index]),
                    y=float(search_tree.y[parent_index]),
                    z=0.08,
                ),
                Point(x=float(x), y=float(y), z=0.08),
            )
        )

    node_half_size = 0.10
    for x, y in zip(search_tree.x, search_tree.y):
        marker.points.extend(
            (
                Point(x=float(x) - node_half_size, y=float(y), z=0.08),
                Point(x=float(x) + node_half_size, y=float(y), z=0.08),
                Point(x=float(x), y=float(y) - node_half_size, z=0.08),
                Point(x=float(x), y=float(y) + node_half_size, z=0.08),
            )
        )
    return marker


def create_search_tree_final_path_marker(search_tree: SearchTree) -> Marker:
    node_count = validate_search_tree(search_tree)
    marker = Marker()
    marker.header = search_tree.header
    marker.ns = "search_tree_final_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.07
    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.a = 1.0

    index = search_tree.final_node_index
    if node_count == 0 and index == -1:
        return marker
    if not 0 <= index < node_count:
        raise ValueError("SearchTree final_node_index is invalid")

    path_indices = []
    for _ in range(node_count):
        path_indices.append(index)
        index = search_tree.parent_index[index]
        if index == -1:
            break
    else:
        raise ValueError("SearchTree final path contains a parent cycle")

    for index in reversed(path_indices):
        marker.points.append(
            Point(
                x=float(search_tree.x[index]),
                y=float(search_tree.y[index]),
                z=0.10,
            )
        )
    return marker


def quaternion_yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


def create_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def transform_pose(
    pose: Pose, yaw_offset: float, translation: tuple[float, float] = (0.0, 0.0)
) -> Pose:
    cosine, sine = math.cos(yaw_offset), math.sin(yaw_offset)
    return create_pose(
        cosine * pose.position.x - sine * pose.position.y + translation[0],
        sine * pose.position.x + cosine * pose.position.y + translation[1],
        pose.position.z,
        quaternion_yaw(pose.orientation) + yaw_offset,
    )


def alignment_translation(
    pose: Pose, yaw_offset: float, target_x: float, target_y: float
) -> tuple[float, float]:
    rotated = transform_pose(pose, yaw_offset)
    return target_x - rotated.position.x, target_y - rotated.position.y


def attach_search_tree_to_world(
    search_tree: SearchTree, world_pose: dict, frame_id: str
) -> SearchTree:
    lengths = {
        len(search_tree.x),
        len(search_tree.y),
        len(search_tree.yaw),
        len(search_tree.parent_index),
    }
    if len(lengths) != 1:
        raise ValueError("SearchTree arrays must have equal lengths")

    transformed = SearchTree()
    transformed.header = Header(stamp=search_tree.header.stamp, frame_id=frame_id)
    transformed.parent_index = list(search_tree.parent_index)
    transformed.final_node_index = search_tree.final_node_index
    if not search_tree.x:
        return transformed

    root_x = float(search_tree.x[0])
    root_y = float(search_tree.y[0])
    root_yaw = float(search_tree.yaw[0])
    world_x = float(world_pose["x"])
    world_y = float(world_pose["y"])
    world_yaw = float(world_pose["yaw"])
    yaw_offset = world_yaw - root_yaw
    cosine, sine = math.cos(yaw_offset), math.sin(yaw_offset)

    for x, y, yaw in zip(search_tree.x, search_tree.y, search_tree.yaw):
        delta_x = float(x) - root_x
        delta_y = float(y) - root_y
        transformed.x.append(world_x + cosine * delta_x - sine * delta_y)
        transformed.y.append(world_y + sine * delta_x + cosine * delta_y)
        transformed.yaw.append(world_yaw + float(yaw) - root_yaw)
    return transformed


def attach_path_to_world(path: Path, world_pose: dict, frame_id: str) -> Path:
    transformed = Path()
    transformed.header = Header(stamp=path.header.stamp, frame_id=frame_id)
    if not path.poses:
        return transformed

    root = path.poses[0].pose
    root_yaw = quaternion_yaw(root.orientation)
    world_x = float(world_pose["x"])
    world_y = float(world_pose["y"])
    world_z = float(world_pose.get("z", 0.0))
    world_yaw = float(world_pose["yaw"])
    yaw_offset = world_yaw - root_yaw
    cosine, sine = math.cos(yaw_offset), math.sin(yaw_offset)

    for stamped in path.poses:
        pose = stamped.pose
        delta_x = pose.position.x - root.position.x
        delta_y = pose.position.y - root.position.y
        map_pose = create_pose(
            world_x + cosine * delta_x - sine * delta_y,
            world_y + sine * delta_x + cosine * delta_y,
            world_z + pose.position.z - root.position.z,
            world_yaw + quaternion_yaw(pose.orientation) - root_yaw,
        )
        transformed.poses.append(
            PoseStamped(header=transformed.header, pose=map_pose)
        )
    return transformed


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


def create_scan_cloud(
    scan: LaserScan, x: float, y: float, z: float, yaw: float, frame_id: str
) -> PointCloud2:
    points = project_scan_points(scan, x, y, z, yaw)
    header = Header(stamp=scan.header.stamp, frame_id=frame_id)
    return point_cloud2.create_cloud_xyz32(header, points)


class RddfVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("rddf_visualizer_node")
        self.declare_parameter("rddf_directory", "rddf")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("enable_sim_gt", False)
        self.declare_parameter("odom_yaw_offset_deg", ODOM_YAW_OFFSET_DEG)
        self.declare_parameter(
            "laser_odom_yaw_offset_deg", LASER_ODOM_YAW_OFFSET_DEG
        )

        directory = FilePath(str(self.get_parameter("rddf_directory").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.sim_gt_enabled = bool(self.get_parameter("enable_sim_gt").value)
        self.odom_yaw_offset = math.radians(
            float(self.get_parameter("odom_yaw_offset_deg").value)
        )
        self.laser_odom_yaw_offset = math.radians(
            float(self.get_parameter("laser_odom_yaw_offset_deg").value)
        )
        stamp = self.get_clock().now().to_msg()

        self.latest_odometry = None
        self.latest_laser_odometry = None
        self.latest_world_pose = None
        self.odom_alignment = None
        self.laser_odom_alignment = None

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        search_tree_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        self.ego_sim_pub = self.create_publisher(Marker, "/rddf/ego_marker_sim", qos)
        self.ego_odom_pub = self.create_publisher(Marker, "/rddf/ego_marker_odom", qos)
        self.ego_odom_laser_pub = self.create_publisher(
            Marker, "/rddf/ego_marker_odom_laser", qos
        )
        self.scan_sim_pub = self.create_publisher(PointCloud2, "/rddf/scan_sim", 1)
        self.scan_odom_pub = self.create_publisher(PointCloud2, "/rddf/scan_odom", 1)
        self.scan_odom_laser_pub = self.create_publisher(
            PointCloud2, "/rddf/scan_odom_laser", 1
        )
        self.path_sim_pub = self.create_publisher(Path, "/rddf/path_sim", 10)
        self.search_tree_marker_pub = self.create_publisher(
            Marker, "/path_planning/debug/search_tree_marker", qos
        )
        self.search_tree_final_path_marker_pub = self.create_publisher(
            Marker, "/path_planning/debug/search_tree_final_path_marker", qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odometry, qos_profile_sensor_data
        )
        self.odom_laser_sub = self.create_subscription(
            Odometry, "/odom/laser", self.on_laser_odometry, qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.path_sub = self.create_subscription(Path, "/path", self.on_path, 10)
        self.search_tree_sub = self.create_subscription(
            SearchTree,
            "/path_planning/debug/search_tree",
            self.on_search_tree,
            search_tree_qos,
        )

    def on_odometry(self, odometry: Odometry) -> None:
        self.latest_odometry = odometry
        if self.sim_gt_enabled:
            self.update_sim_gt(odometry.header.stamp)

        pose = odometry.pose.pose
        if self.odom_alignment is None and self.latest_world_pose is not None:
            self.odom_alignment = alignment_translation(
                pose,
                self.odom_yaw_offset,
                float(self.latest_world_pose["x"]),
                float(self.latest_world_pose["y"]),
            )
        map_pose = transform_pose(
            pose, self.odom_yaw_offset, self.odom_alignment or (0.0, 0.0)
        )
        self.ego_odom_pub.publish(
            create_ego_marker(
                Header(stamp=odometry.header.stamp, frame_id=self.frame_id),
                map_pose,
                (1.0, 0.0, 0.0),
            )
        )

    def on_laser_odometry(self, odometry: Odometry) -> None:
        self.latest_laser_odometry = odometry
        pose = odometry.pose.pose
        if self.laser_odom_alignment is None and self.latest_world_pose is not None:
            self.laser_odom_alignment = alignment_translation(
                pose,
                self.laser_odom_yaw_offset,
                float(self.latest_world_pose["x"]),
                float(self.latest_world_pose["y"]),
            )
        map_pose = transform_pose(
            pose,
            self.laser_odom_yaw_offset,
            self.laser_odom_alignment or (0.0, 0.0),
        )
        self.ego_odom_laser_pub.publish(
            create_ego_marker(
                Header(stamp=odometry.header.stamp, frame_id=self.frame_id),
                map_pose,
                (0.0, 1.0, 0.0),
            )
        )

    def on_path(self, path: Path) -> None:
        if self.sim_gt_enabled and self.latest_world_pose is not None:
            self.path_sim_pub.publish(
                attach_path_to_world(path, self.latest_world_pose, self.frame_id)
            )

    def on_search_tree(self, search_tree: SearchTree) -> None:
        try:
            if self.sim_gt_enabled and self.latest_world_pose is not None:
                search_tree = attach_search_tree_to_world(
                    search_tree, self.latest_world_pose, self.frame_id
                )
            marker = create_search_tree_marker(search_tree)
            final_path_marker = create_search_tree_final_path_marker(search_tree)
        except ValueError as error:
            self.get_logger().warning(f"ignored invalid SearchTree: {error}")
            return
        self.search_tree_marker_pub.publish(marker)
        self.search_tree_final_path_marker_pub.publish(final_path_marker)

    def on_scan(self, scan: LaserScan) -> None:
        if self.latest_odometry is not None:
            map_pose = transform_pose(
                self.latest_odometry.pose.pose,
                self.odom_yaw_offset,
                self.odom_alignment or (0.0, 0.0),
            )
            self.scan_odom_pub.publish(
                create_scan_cloud(
                    scan,
                    map_pose.position.x,
                    map_pose.position.y,
                    map_pose.position.z,
                    quaternion_yaw(map_pose.orientation),
                    self.frame_id,
                )
            )
        if self.latest_laser_odometry is not None:
            map_pose = transform_pose(
                self.latest_laser_odometry.pose.pose,
                self.laser_odom_yaw_offset,
                self.laser_odom_alignment or (0.0, 0.0),
            )
            self.scan_odom_laser_pub.publish(
                create_scan_cloud(
                    scan,
                    map_pose.position.x,
                    map_pose.position.y,
                    map_pose.position.z,
                    quaternion_yaw(map_pose.orientation),
                    self.frame_id,
                )
            )
        if self.sim_gt_enabled and self.latest_world_pose is not None:
            world_pose = self.latest_world_pose
            self.scan_sim_pub.publish(
                create_scan_cloud(
                    scan,
                    float(world_pose["x"]),
                    float(world_pose["y"]),
                    float(world_pose.get("z", 0.0)),
                    float(world_pose["yaw"]),
                    self.frame_id,
                )
            )

    def update_sim_gt(self, stamp) -> None:
        try:
            with urlopen("http://localhost/sim/api/pose", timeout=0.1) as response:
                world_pose = json.load(response)
            self.latest_world_pose = world_pose
            sim_pose = create_pose(
                float(world_pose["x"]),
                float(world_pose["y"]),
                float(world_pose.get("z", 0.0)),
                float(world_pose["yaw"]),
            )
            self.ego_sim_pub.publish(
                create_ego_marker(
                    Header(stamp=stamp, frame_id=self.frame_id),
                    sim_pose,
                    (0.0, 1.0, 1.0),
                )
            )
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(
                f"SIM ground truth disabled; continuing odom-only: {error}"
            )
            self.sim_gt_enabled = False


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
