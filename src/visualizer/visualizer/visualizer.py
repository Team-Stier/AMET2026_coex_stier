from __future__ import annotations

import copy
import csv
import json
import math
import time
from pathlib import Path as FilePath
from urllib.request import urlopen

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped
from interfaces.msg import Objects, SearchTree
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from visualization_msgs.msg import Marker


GLOBAL_FRAME = "map"
LOCAL_FRAME = "lidar_link"
MARKER_TOPICS = {
    "sim": "/rddf/ego_marker_sim",
    "pose": "/rddf/ego_marker_pose",
    "calibration": "/rddf/ego_marker_calibration",
}
MARKER_COLORS = {
    "sim": (0.0, 0.85, 1.0),
    "pose": (0.1, 1.0, 0.1),
    "calibration": (1.0, 0.25, 0.85),
}
SEARCH_TREE_INPUT_TOPIC = "/path_planning/debug/search_tree"
SEARCH_TREE_MARKER_TOPIC = "/visualizer/path_planning/search_tree"
GLOBAL_PATH_INPUT_TOPIC = "/path_planning/debug/global_path"
GLOBAL_PATH_TOPIC = "/visualizer/path_planning/global_path"
OBJECT_MARKER_TOPIC = "/visualizer/object_info"


def stamp_from_seconds(seconds: object) -> Time:
    value = float(seconds)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("timestamp must be a finite non-negative number")
    sec = math.floor(value)
    nanosec = round((value - sec) * 1_000_000_000)
    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0
    return Time(sec=sec, nanosec=nanosec)


def stamp_is_zero(stamp: Time) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


def normalized_quaternion(values: object) -> tuple[float, float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError("quaternion must contain four values")
    quaternion = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError("quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ValueError("quaternion has zero norm")
    return tuple(value / norm for value in quaternion)


def sim_pose_from_state(payload: object) -> tuple[Pose, Time]:
    if not isinstance(payload, dict) or not isinstance(payload.get("vehicle"), dict):
        raise ValueError("SIM state has no vehicle pose")
    vehicle = payload["vehicle"]
    coordinates = tuple(float(vehicle[name]) for name in ("x", "y", "z"))
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError("SIM vehicle position contains a non-finite value")
    if "q" in vehicle:
        qx, qy, qz, qw = normalized_quaternion(vehicle["q"])
    else:
        yaw = float(vehicle["yaw"])
        if not math.isfinite(yaw):
            raise ValueError("SIM vehicle yaw is not finite")
        qx, qy, qz, qw = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = coordinates
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose, stamp_from_seconds(payload["time"])


def pose_from_odometry(message: Odometry) -> Pose:
    position = message.pose.pose.position
    coordinates = (position.x, position.y, position.z)
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError("pose position contains a non-finite value")
    orientation = message.pose.pose.orientation
    qx, qy, qz, qw = normalized_quaternion(
        (orientation.x, orientation.y, orientation.z, orientation.w)
    )

    pose = copy.deepcopy(message.pose.pose)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def load_xy_csv(path: FilePath) -> list[tuple[float, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {"x_m", "y_m"}.issubset(reader.fieldnames):
            raise ValueError(f"{path} must contain x_m and y_m columns")
        points = [(float(row["x_m"]), float(row["y_m"])) for row in reader]
    if len(points) < 2 or not all(
        math.isfinite(coordinate) for point in points for coordinate in point
    ):
        raise ValueError(f"{path} must contain at least two finite points")
    return points


def path_message(points: list[tuple[float, float]]) -> Path:
    message = Path()
    message.header.frame_id = GLOBAL_FRAME
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = GLOBAL_FRAME
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def search_tree_marker(message: SearchTree) -> Marker:
    if message.header.frame_id != GLOBAL_FRAME:
        raise ValueError(f"SearchTree frame must be {GLOBAL_FRAME!r}")
    count = len(message.x)
    if not (count == len(message.y) == len(message.yaw) == len(message.parent_index)):
        raise ValueError("SearchTree arrays must have equal lengths")
    if count and not 0 <= message.final_node_index < count:
        raise ValueError("SearchTree final_node_index is out of range")
    values = (*message.x, *message.y, *message.yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("SearchTree contains a non-finite pose")

    marker = Marker()
    marker.header = copy.deepcopy(message.header)
    marker.header.frame_id = GLOBAL_FRAME
    marker.ns = "path_planning_search_tree"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD if count else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01
    marker.color.r = 0.35
    marker.color.g = 0.55
    marker.color.b = 1.0
    marker.color.a = 0.45
    for child, parent in enumerate(message.parent_index):
        if child == 0 and parent == -1:
            continue
        if parent < 0 or parent >= child:
            raise ValueError(f"SearchTree parent_index[{child}] is invalid")
        marker.points.append(
            Point(x=float(message.x[parent]), y=float(message.y[parent]))
        )
        marker.points.append(
            Point(x=float(message.x[child]), y=float(message.y[child]))
        )
    return marker


def object_marker(message: Objects, obstacle_inflation_radius_m: float) -> Marker:
    if message.header.frame_id != LOCAL_FRAME:
        raise ValueError(f"Objects frame must be {LOCAL_FRAME!r}")
    count = int(message.length)
    if count < 0 or count > len(message.x) or count > len(message.y):
        raise ValueError("Objects length is out of range")
    if not all(
        math.isfinite(value)
        for index in range(count)
        for value in (message.x[index], message.y[index])
    ):
        raise ValueError("Objects contains a non-finite position")

    marker = Marker()
    marker.header = copy.deepcopy(message.header)
    marker.ns = "object_info"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD if count else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = 2.0 * obstacle_inflation_radius_m
    marker.scale.z = 0.02
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.1
    marker.color.a = 0.9
    marker.points = [
        Point(x=float(message.x[index]), y=float(message.y[index]))
        for index in range(count)
    ]
    return marker


class Visualizer(Node):
    def __init__(self) -> None:
        super().__init__("visualizer")

        package_share = FilePath(get_package_share_directory("visualizer"))
        self.declare_parameter("rddf_directory", str(package_share / "rddf"))
        self.declare_parameter("enable_sim_gt", True)
        self.declare_parameter("sim_state_url", "http://localhost/sim/api/state")
        self.declare_parameter("sim_poll_rate_hz", 20.0)
        self.declare_parameter("sim_timeout_sec", 0.05)
        self.declare_parameter("sim_retry_sec", 2.0)
        self._obstacle_inflation_radius_m = float(
            self.declare_parameter("obstacle_inflation_radius_m", 0.25).value
        )
        if (
            not math.isfinite(self._obstacle_inflation_radius_m)
            or self._obstacle_inflation_radius_m <= 0.0
        ):
            raise ValueError("obstacle_inflation_radius_m must be positive")
        self._warned: set[tuple[str, ...]] = set()

        marker_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._marker_publishers = {
            source: self.create_publisher(Marker, topic, marker_qos)
            for source, topic in MARKER_TOPICS.items()
        }
        self._rddf_publishers = []
        self._publish_rddf()

        self.create_subscription(
            Odometry,
            "/pose",
            self._on_pose,
            qos_profile_sensor_data,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            Odometry,
            "/pose/calibration",
            self._on_calibration_pose,
            qos_profile_sensor_data,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        debug_group = MutuallyExclusiveCallbackGroup()
        debug_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self._global_path_publisher = self.create_publisher(
            Path, GLOBAL_PATH_TOPIC, debug_qos
        )
        self.create_subscription(
            Path,
            GLOBAL_PATH_INPUT_TOPIC,
            self._global_path_publisher.publish,
            debug_qos,
            callback_group=debug_group,
        )
        self._search_tree_publisher = self.create_publisher(
            Marker, SEARCH_TREE_MARKER_TOPIC, debug_qos
        )
        self.create_subscription(
            SearchTree,
            SEARCH_TREE_INPUT_TOPIC,
            self._relay_search_tree,
            debug_qos,
            callback_group=debug_group,
        )
        self._object_marker_publisher = self.create_publisher(
            Marker, OBJECT_MARKER_TOPIC, debug_qos
        )
        self.create_subscription(
            Objects,
            "/object_info",
            self._relay_objects,
            qos_profile_sensor_data,
            callback_group=debug_group,
        )

        if self.get_parameter("enable_sim_gt").value:
            rate = float(self.get_parameter("sim_poll_rate_hz").value)
            timeout = float(self.get_parameter("sim_timeout_sec").value)
            retry = float(self.get_parameter("sim_retry_sec").value)
            if not math.isfinite(rate) or rate <= 0.0:
                raise ValueError("sim_poll_rate_hz must be positive")
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError("sim_timeout_sec must be positive")
            if not math.isfinite(retry) or retry <= 0.0:
                raise ValueError("sim_retry_sec must be positive")
            self._sim_state_url = str(self.get_parameter("sim_state_url").value)
            self._sim_timeout_sec = timeout
            self._sim_retry_sec = retry
            self._next_sim_attempt = 0.0
            self._sim_online: bool | None = None
            self.create_timer(
                1.0 / rate,
                self._poll_sim_state,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

    def _warn_once(self, key: tuple[str, ...], message: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(message)

    def _publish_rddf(self) -> None:
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        directory = FilePath(str(self.get_parameter("rddf_directory").value))
        for name in ("inner_boundary", "centerline", "outer_boundary"):
            publisher = self.create_publisher(Path, f"/rddf/{name}", qos)
            publisher.publish(path_message(load_xy_csv(directory / f"{name}_real.csv")))
            self._rddf_publishers.append(publisher)

    def _on_pose(
        self,
        message: Odometry,
        source: str = "pose",
        topic: str = "/pose",
    ) -> None:
        if message.header.frame_id != GLOBAL_FRAME:
            self._warn_once(
                (source, "global_frame", message.header.frame_id),
                f"ignoring {topic} in {message.header.frame_id!r}; "
                f"the project global frame is {GLOBAL_FRAME!r}",
            )
            return
        if message.child_frame_id != LOCAL_FRAME:
            self._warn_once(
                (source, "child_frame", message.child_frame_id),
                f"ignoring {topic} for {message.child_frame_id!r}; "
                f"the project local frame is {LOCAL_FRAME!r}",
            )
            return
        if stamp_is_zero(message.header.stamp):
            self._warn_once(
                (source, "zero_stamp"),
                f"ignoring {topic} with a zero timestamp",
            )
            return
        try:
            pose = pose_from_odometry(message)
        except ValueError as error:
            self._warn_once(
                (source, "invalid_pose"), f"ignoring invalid {topic}: {error}"
            )
            return
        self._publish_marker(source, pose, message.header.stamp)

    def _on_calibration_pose(self, message: Odometry) -> None:
        self._on_pose(message, "calibration", "/pose/calibration")

    def _poll_sim_state(self) -> None:
        if time.monotonic() < self._next_sim_attempt:
            return
        try:
            with urlopen(self._sim_state_url, timeout=self._sim_timeout_sec) as response:
                pose, stamp = sim_pose_from_state(json.load(response))
            if stamp_is_zero(stamp):
                raise ValueError("SIM state has a zero timestamp")
        except (OSError, ValueError, KeyError, TypeError) as error:
            self._next_sim_attempt = time.monotonic() + self._sim_retry_sec
            if self._sim_online is not False:
                self.get_logger().warning(
                    f"SIM GT unavailable ({error}); /pose visualization remains active"
                )
            self._sim_online = False
            return

        if self._sim_online is not True:
            self.get_logger().info(f"SIM GT connected: {self._sim_state_url}")
        self._sim_online = True
        self._next_sim_attempt = 0.0
        self._publish_marker("sim", pose, stamp)

    def _publish_marker(self, source: str, pose: Pose, stamp: Time) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = GLOBAL_FRAME
        marker.ns = "ego"
        marker.id = ("sim", "pose", "calibration").index(source)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(pose)
        marker.scale.x = 0.28
        marker.scale.y = 0.20
        marker.scale.z = 0.08
        red, green, blue = MARKER_COLORS[source]
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.85
        marker.lifetime.nanosec = 500_000_000
        self._marker_publishers[source].publish(marker)

    def _relay_search_tree(self, message: SearchTree) -> None:
        try:
            marker = search_tree_marker(message)
        except ValueError as error:
            self._warn_once(
                ("invalid_search_tree", str(error)),
                f"ignoring invalid SearchTree: {error}",
            )
            return
        self._search_tree_publisher.publish(marker)

    def _relay_objects(self, message: Objects) -> None:
        try:
            marker = object_marker(message, self._obstacle_inflation_radius_m)
        except ValueError as error:
            self._warn_once(
                ("invalid_objects", str(error)),
                f"ignoring invalid /object_info: {error}",
            )
            return
        self._object_marker_publisher.publish(marker)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Visualizer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
