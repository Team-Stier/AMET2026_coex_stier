from __future__ import annotations

import json
import math
from pathlib import Path as FilePath
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from calibration.pose_geometry import (
    map_from_odom_pose,
    normalize_angle,
    transform_pose_2d,
)


class SimRvizBridge(Node):
    """Publish the calibrated color map and pose comparisons for RViz."""

    def __init__(self):
        super().__init__("calibration_sim_rviz_bridge")
        self.declare_parameter("map_file", "docs/sim_lane_map_world_color.png")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_resolution_m", 0.01)
        self.declare_parameter("map_origin_x_m", 0.0)
        self.declare_parameter("map_origin_y_m", 0.0)
        self.declare_parameter("map_marker_resolution_m", 0.02)
        self.declare_parameter("map_lane_hsv_lower", [14, 135, 145])
        self.declare_parameter("map_lane_hsv_upper", [32, 255, 255])
        self.declare_parameter("sim_state_url", "http://localhost/sim/api/state")
        self.declare_parameter("truth_poll_hz", 5.0)
        self.declare_parameter("truth_topic", "")
        self.declare_parameter("maximum_path_points", 1500)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.sim_state_url = str(self.get_parameter("sim_state_url").value)
        self.maximum_path_points = int(self.get_parameter("maximum_path_points").value)
        self.map_from_odom = None
        self.latest_truth = None
        self.latest_raw = None
        self.latest_corrected = None
        self.raw_path = Path()
        self.corrected_path = Path()
        self.truth_path = Path()

        # Keep both the coarse background and full-resolution lane marker for
        # RViz instances that subscribe after this node has already started.
        latched_qos = QoSProfile(depth=2)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        self.map_pub = self.create_publisher(
            Marker, "/calibration/rviz/map", latched_qos
        )
        self.pose_pub = self.create_publisher(
            MarkerArray, "/calibration/rviz/poses", 10
        )
        self.observed_pub = self.create_publisher(
            Marker, "/calibration/rviz/observed_lane", 10
        )
        self.raw_path_pub = self.create_publisher(
            Path, "/calibration/rviz/raw_path", 10
        )
        self.corrected_path_pub = self.create_publisher(
            Path, "/calibration/rviz/corrected_path", 10
        )
        self.truth_path_pub = self.create_publisher(
            Path, "/calibration/rviz/truth_path", 10
        )
        self.static_tf = StaticTransformBroadcaster(self)

        self.create_subscription(Odometry, "/odom", self.on_raw_odom, 10)
        self.create_subscription(
            Odometry, "/odom/calibride", self.on_corrected_odom, 10
        )
        self.create_subscription(
            Path,
            "/calibration/detected_centerline",
            self.on_observed_lane,
            10,
        )
        truth_topic = str(self.get_parameter("truth_topic").value)
        if truth_topic:
            self.create_subscription(TFMessage, truth_topic, self.on_truth_tf, 50)
        else:
            poll_hz = max(0.5, float(self.get_parameter("truth_poll_hz").value))
            self.create_timer(1.0 / poll_hz, self.poll_truth)
        self.create_timer(1.0, self.publish_map)
        self.map_marker, self.map_lane_marker = self._create_map_markers()
        self.publish_map()

    @staticmethod
    def _resolve_path(value: str) -> FilePath:
        path = FilePath(value).expanduser()
        if path.is_absolute():
            return path
        source_root = FilePath(__file__).resolve().parents[1]
        candidates = [FilePath.cwd() / path, source_root / path]
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(FilePath(get_package_share_directory("calibration")) / path)
        except (ImportError, LookupError):
            pass
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        return (source_root / path).resolve()

    @staticmethod
    def _append_image_point(
        marker: Marker,
        image,
        row: int,
        column: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> None:
        blue, green, red = image[row, column]
        marker.points.append(
            Point(
                x=origin_x + (column + 0.5) * resolution,
                y=origin_y + (height - row - 0.5) * resolution,
                z=0.0,
            )
        )
        marker.colors.append(
            ColorRGBA(
                r=float(red) / 255.0,
                g=float(green) / 255.0,
                b=float(blue) / 255.0,
                a=1.0,
            )
        )

    def _create_map_markers(self) -> tuple[Marker, Marker]:
        path = self._resolve_path(str(self.get_parameter("map_file").value))
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise OSError(f"failed to load RViz map image: {path}")
        source_resolution = float(self.get_parameter("map_resolution_m").value)
        marker_resolution = float(
            self.get_parameter("map_marker_resolution_m").value
        )
        stride = max(1, int(round(marker_resolution / source_resolution)))
        origin_x = float(self.get_parameter("map_origin_x_m").value)
        origin_y = float(self.get_parameter("map_origin_y_m").value)

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.ns = "calibrated_color_map"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = stride * source_resolution * 1.04
        marker.scale.y = stride * source_resolution * 1.04
        marker.lifetime = Duration(sec=0, nanosec=0)
        height, width = image.shape[:2]
        for row in range(stride // 2, height, stride):
            for column in range(stride // 2, width, stride):
                self._append_image_point(
                    marker,
                    image,
                    row,
                    column,
                    height,
                    source_resolution,
                    origin_x,
                    origin_y,
                )

        lane_marker = Marker()
        lane_marker.header.frame_id = self.map_frame
        lane_marker.ns = "calibrated_color_map_lane_full_resolution"
        lane_marker.id = 1
        lane_marker.type = Marker.POINTS
        lane_marker.action = Marker.ADD
        lane_marker.pose.orientation.w = 1.0
        lane_marker.scale.x = source_resolution * 1.15
        lane_marker.scale.y = source_resolution * 1.15
        lane_marker.lifetime = Duration(sec=0, nanosec=0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = tuple(int(value) for value in self.get_parameter("map_lane_hsv_lower").value)
        upper = tuple(int(value) for value in self.get_parameter("map_lane_hsv_upper").value)
        lane_mask = cv2.inRange(hsv, lower, upper)
        lane_rows, lane_columns = lane_mask.nonzero()
        for row, column in zip(lane_rows, lane_columns):
            self._append_image_point(
                lane_marker,
                image,
                int(row),
                int(column),
                height,
                source_resolution,
                origin_x,
                origin_y,
            )
        self.get_logger().info(
            "prepared RViz color map with "
            f"{len(marker.points)} background points and "
            f"{len(lane_marker.points)} full-resolution lane points from {path}"
        )
        return marker, lane_marker

    def publish_map(self) -> None:
        if not hasattr(self, "map_marker"):
            return
        stamp = self.get_clock().now().to_msg()
        self.map_marker.header.stamp = stamp
        self.map_lane_marker.header.stamp = stamp
        self.map_pub.publish(self.map_marker)
        self.map_pub.publish(self.map_lane_marker)

    def poll_truth(self) -> None:
        try:
            with urlopen(self.sim_state_url, timeout=0.15) as response:
                state = json.load(response)
            vehicle = state["vehicle"]
            self.latest_truth = (
                float(vehicle["x"]),
                float(vehicle["y"]),
                float(vehicle["yaw"]),
            )
        except (URLError, TimeoutError, KeyError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f"sim ground-truth request failed: {error}",
                throttle_duration_sec=2.0,
            )
            return
        self._try_bootstrap_tf()
        self._append_path(self.truth_path, self.latest_truth)
        self.truth_path_pub.publish(self.truth_path)
        self._publish_pose_markers()

    def on_truth_tf(self, message: TFMessage) -> None:
        """Consume simulator-world truth recorded in a calibration rosbag."""
        if not message.transforms:
            return
        transform = message.transforms[0].transform
        yaw = math.atan2(
            2.0 * (transform.rotation.w * transform.rotation.z
                   + transform.rotation.x * transform.rotation.y),
            1.0 - 2.0 * (transform.rotation.y * transform.rotation.y
                         + transform.rotation.z * transform.rotation.z),
        )
        self.latest_truth = (
            float(transform.translation.x),
            float(transform.translation.y),
            yaw,
        )
        self._try_bootstrap_tf()
        self._append_path(self.truth_path, self.latest_truth)
        self.truth_path_pub.publish(self.truth_path)
        self._publish_pose_markers()

    def on_raw_odom(self, message: Odometry) -> None:
        self.latest_raw = self._pose_from_odometry(message)
        self._try_bootstrap_tf()
        if self.map_from_odom is None:
            return
        pose_map = transform_pose_2d(self.latest_raw, self.map_from_odom)
        self._append_path(self.raw_path, pose_map)
        self.raw_path_pub.publish(self.raw_path)
        self._publish_pose_markers()

    def on_corrected_odom(self, message: Odometry) -> None:
        self.latest_corrected = self._pose_from_odometry(message)
        if self.map_from_odom is None:
            return
        pose_map = transform_pose_2d(self.latest_corrected, self.map_from_odom)
        self._append_path(self.corrected_path, pose_map)
        self.corrected_path_pub.publish(self.corrected_path)
        self._publish_pose_markers()

    def _try_bootstrap_tf(self) -> None:
        if (
            self.map_from_odom is not None
            or self.latest_truth is None
            or self.latest_raw is None
        ):
            return
        self.map_from_odom = map_from_odom_pose(self.latest_truth, self.latest_raw)
        x, y, yaw = self.map_from_odom
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        self.static_tf.sendTransform(transform)
        self.get_logger().info(
            "bootstrapped fixed map->odom from one simultaneous simulator pose: "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad"
        )

    def on_observed_lane(self, message: Path) -> None:
        if self.map_from_odom is None or self.latest_corrected is None:
            return
        base_map = transform_pose_2d(self.latest_corrected, self.map_from_odom)
        cosine = math.cos(base_map[2])
        sine = math.sin(base_map[2])
        marker = Marker()
        marker.header.stamp = message.header.stamp
        marker.header.frame_id = self.map_frame
        marker.ns = "observed_lane"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)
        marker.lifetime = Duration(sec=0, nanosec=400_000_000)
        for pose in message.poses:
            local = pose.pose.position
            marker.points.append(
                Point(
                    x=base_map[0] + cosine * local.x - sine * local.y,
                    y=base_map[1] + sine * local.x + cosine * local.y,
                    z=0.12,
                )
            )
        self.observed_pub.publish(marker)

    def _publish_pose_markers(self) -> None:
        if self.map_from_odom is None:
            return
        poses = []
        if self.latest_raw is not None:
            poses.append(
                ("RAW", transform_pose_2d(self.latest_raw, self.map_from_odom), (1.0, 0.1, 0.1))
            )
        if self.latest_corrected is not None:
            poses.append(
                (
                    "CORRECTED",
                    transform_pose_2d(self.latest_corrected, self.map_from_odom),
                    (0.0, 0.9, 1.0),
                )
            )
        if self.latest_truth is not None:
            poses.append(("TRUTH", self.latest_truth, (0.1, 1.0, 0.1)))

        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, (name, pose, color) in enumerate(poses):
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = stamp
            arrow.ns = "vehicle_pose"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = pose[0]
            arrow.pose.position.y = pose[1]
            arrow.pose.position.z = 0.16 + index * 0.04
            arrow.pose.orientation.z = math.sin(pose[2] * 0.5)
            arrow.pose.orientation.w = math.cos(pose[2] * 0.5)
            arrow.scale.x = 0.42
            arrow.scale.y = 0.13
            arrow.scale.z = 0.10
            arrow.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            array.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = "vehicle_label"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pose[0]
            label.pose.position.y = pose[1]
            label.pose.position.z = 0.45 + index * 0.06
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color = arrow.color
            label.text = name
            if name != "TRUTH" and self.latest_truth is not None:
                position_error = math.hypot(
                    pose[0] - self.latest_truth[0],
                    pose[1] - self.latest_truth[1],
                )
                yaw_error_deg = math.degrees(
                    abs(normalize_angle(pose[2] - self.latest_truth[2]))
                )
                label.text = (
                    f"{name}  {position_error:.2f} m / {yaw_error_deg:.1f} deg"
                )
            array.markers.append(label)
        self.pose_pub.publish(array)

    def _append_path(self, path: Path, pose: tuple[float, float, float]) -> None:
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        stamped = PoseStamped()
        stamped.header = path.header
        stamped.pose.position.x = pose[0]
        stamped.pose.position.y = pose[1]
        stamped.pose.position.z = 0.08
        stamped.pose.orientation.z = math.sin(pose[2] * 0.5)
        stamped.pose.orientation.w = math.cos(pose[2] * 0.5)
        path.poses.append(stamped)
        if len(path.poses) > self.maximum_path_points:
            del path.poses[: len(path.poses) - self.maximum_path_points]

    @staticmethod
    def _pose_from_odometry(message: Odometry) -> tuple[float, float, float]:
        pose = message.pose.pose
        quaternion = pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
        )
        return float(pose.position.x), float(pose.position.y), yaw


def main(args=None):
    rclpy.init(args=args)
    node = SimRvizBridge()
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
