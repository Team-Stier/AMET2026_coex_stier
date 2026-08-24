from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from calibration.pose_geometry import (
    map_from_odom_pose,
    normalize_angle,
    transform_pose_2d,
)


class FenceRvizBridge(Node):
    """Keep complete truth, odometry, LiDAR-odometry, and fence-corrected paths."""

    COLORS = {
        "ODOM": (1.0, 0.12, 0.12),
        "ODOM_LASER": (1.0, 0.62, 0.0),
        "FENCE_CORRECTED": (0.0, 0.90, 1.0),
        "TRUTH": (0.12, 1.0, 0.12),
    }

    def __init__(self) -> None:
        super().__init__("calibration_fence_rviz_bridge")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("truth_topic", "/sim/ground_truth/tf")
        self.declare_parameter("maximum_path_points", 20000)
        self.declare_parameter("fixed_start_map_x_m", 1.4)
        self.declare_parameter("fixed_start_map_y_m", 3.4)
        self.declare_parameter("fixed_start_map_yaw_rad", -1.5707963267948966)
        self.declare_parameter("fence_minimum_x_m", 0.0)
        self.declare_parameter("fence_maximum_x_m", 12.0)
        self.declare_parameter("fence_minimum_y_m", 0.0)
        self.declare_parameter("fence_maximum_y_m", 7.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.maximum_path_points = int(
            self.get_parameter("maximum_path_points").value
        )
        self.fixed_start_pose = (
            float(self.get_parameter("fixed_start_map_x_m").value),
            float(self.get_parameter("fixed_start_map_y_m").value),
            float(self.get_parameter("fixed_start_map_yaw_rad").value),
        )
        self.map_from_odom = None
        self.map_from_laser_odom = None
        self.latest_raw = None
        self.latest_laser = None
        self.latest_corrected = None
        self.latest_truth = None
        self.last_raw_stamp_sec = None

        self.raw_path = Path()
        self.laser_path = Path()
        self.corrected_path = Path()
        self.truth_path = Path()

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.fence_pub = self.create_publisher(
            Marker, "/calibration/rviz/fence", latched
        )
        self.pose_pub = self.create_publisher(
            MarkerArray, "/calibration/rviz/poses", 10
        )
        self.raw_path_pub = self.create_publisher(
            Path, "/calibration/rviz/raw_path", 10
        )
        self.laser_path_pub = self.create_publisher(
            Path, "/calibration/rviz/laser_path", 10
        )
        self.corrected_path_pub = self.create_publisher(
            Path, "/calibration/rviz/corrected_path", 10
        )
        self.truth_path_pub = self.create_publisher(
            Path, "/calibration/rviz/truth_path", 10
        )

        self.create_subscription(Odometry, "/odom", self.on_raw_odom, 20)
        self.create_subscription(
            Odometry, "/odom/laser", self.on_laser_odometry, 20
        )
        self.create_subscription(
            Odometry, "/odom/calibride", self.on_corrected_odom, 20
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("truth_topic").value),
            self.on_truth_tf,
            50,
        )
        self.fence_marker = self._create_fence_marker()
        self.create_timer(1.0, self.publish_fence)
        self.create_timer(1.0, self.publish_retained_paths)
        self.publish_fence()

    def _create_fence_marker(self) -> Marker:
        minimum_x = float(self.get_parameter("fence_minimum_x_m").value)
        maximum_x = float(self.get_parameter("fence_maximum_x_m").value)
        minimum_y = float(self.get_parameter("fence_minimum_y_m").value)
        maximum_y = float(self.get_parameter("fence_maximum_y_m").value)
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.ns = "lidar_fence_reference"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color = ColorRGBA(r=0.72, g=0.72, b=0.72, a=1.0)
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker.points = [
            Point(x=minimum_x, y=minimum_y, z=0.0),
            Point(x=maximum_x, y=minimum_y, z=0.0),
            Point(x=maximum_x, y=maximum_y, z=0.0),
            Point(x=minimum_x, y=maximum_y, z=0.0),
            Point(x=minimum_x, y=minimum_y, z=0.0),
        ]
        return marker

    def publish_fence(self) -> None:
        self.fence_marker.header.stamp = self.get_clock().now().to_msg()
        self.fence_pub.publish(self.fence_marker)

    def publish_retained_paths(self) -> None:
        """Keep the completed run visible after bag playback has stopped."""
        if self.raw_path.poses:
            self.raw_path_pub.publish(self.raw_path)
        if self.laser_path.poses:
            self.laser_path_pub.publish(self.laser_path)
        if self.corrected_path.poses:
            self.corrected_path_pub.publish(self.corrected_path)
        if self.truth_path.poses:
            self.truth_path_pub.publish(self.truth_path)
        self._publish_pose_markers()

    def on_raw_odom(self, message: Odometry) -> None:
        stamp_sec = self._stamp_seconds(message.header.stamp)
        if self.last_raw_stamp_sec is not None and stamp_sec < self.last_raw_stamp_sec:
            self._reset_paths()
        self.last_raw_stamp_sec = stamp_sec
        self.latest_raw = self._pose_from_odometry(message)
        if self.map_from_odom is None:
            self.map_from_odom = map_from_odom_pose(
                self.fixed_start_pose, self.latest_raw
            )
            self.get_logger().info(
                "RViz paths aligned from fixed start pose; map->odom="
                f"({self.map_from_odom[0]:.3f}, {self.map_from_odom[1]:.3f}, "
                f"{self.map_from_odom[2]:.3f})"
            )
        pose_map = transform_pose_2d(self.latest_raw, self.map_from_odom)
        self._append_path(self.raw_path, pose_map, message.header.stamp)
        self.raw_path_pub.publish(self.raw_path)
        self._publish_pose_markers()

    def on_laser_odometry(self, message: Odometry) -> None:
        self.latest_laser = self._pose_from_odometry(message)
        if self.map_from_laser_odom is None:
            self.map_from_laser_odom = map_from_odom_pose(
                self.fixed_start_pose, self.latest_laser
            )
        pose_map = transform_pose_2d(
            self.latest_laser, self.map_from_laser_odom
        )
        self._append_path(self.laser_path, pose_map, message.header.stamp)
        self.laser_path_pub.publish(self.laser_path)
        self._publish_pose_markers()

    def on_corrected_odom(self, message: Odometry) -> None:
        self.latest_corrected = self._pose_from_odometry(message)
        if self.map_from_odom is None:
            return
        pose_map = transform_pose_2d(self.latest_corrected, self.map_from_odom)
        self._append_path(self.corrected_path, pose_map, message.header.stamp)
        self.corrected_path_pub.publish(self.corrected_path)
        self._publish_pose_markers()

    def on_truth_tf(self, message: TFMessage) -> None:
        if not message.transforms:
            return
        transform = message.transforms[0]
        rotation = transform.transform.rotation
        self.latest_truth = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            ),
        )
        self._append_path(
            self.truth_path, self.latest_truth, transform.header.stamp
        )
        self.truth_path_pub.publish(self.truth_path)
        self._publish_pose_markers()

    def _publish_pose_markers(self) -> None:
        poses = []
        if self.latest_raw is not None and self.map_from_odom is not None:
            poses.append(
                ("ODOM", transform_pose_2d(self.latest_raw, self.map_from_odom))
            )
        if self.latest_laser is not None and self.map_from_laser_odom is not None:
            poses.append(
                (
                    "ODOM_LASER",
                    transform_pose_2d(
                        self.latest_laser, self.map_from_laser_odom
                    ),
                )
            )
        if self.latest_corrected is not None and self.map_from_odom is not None:
            poses.append(
                (
                    "FENCE_CORRECTED",
                    transform_pose_2d(
                        self.latest_corrected, self.map_from_odom
                    ),
                )
            )
        if self.latest_truth is not None:
            poses.append(("TRUTH", self.latest_truth))

        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, (name, pose) in enumerate(poses):
            color = self.COLORS[name]
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = stamp
            arrow.ns = "vehicle_pose"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = pose[0]
            arrow.pose.position.y = pose[1]
            arrow.pose.position.z = 0.12 + index * 0.05
            arrow.pose.orientation.z = math.sin(pose[2] * 0.5)
            arrow.pose.orientation.w = math.cos(pose[2] * 0.5)
            arrow.scale.x = 0.42
            arrow.scale.y = 0.12
            arrow.scale.z = 0.08
            arrow.color = ColorRGBA(
                r=color[0], g=color[1], b=color[2], a=1.0
            )
            array.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = "vehicle_label"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pose[0]
            label.pose.position.y = pose[1]
            label.pose.position.z = 0.42 + index * 0.07
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color = arrow.color
            label.text = name
            if name != "TRUTH" and self.latest_truth is not None:
                position_error = math.hypot(
                    pose[0] - self.latest_truth[0],
                    pose[1] - self.latest_truth[1],
                )
                yaw_error = math.degrees(
                    abs(normalize_angle(pose[2] - self.latest_truth[2]))
                )
                label.text += f"  {position_error:.3f} m / {yaw_error:.2f} deg"
            array.markers.append(label)
        self.pose_pub.publish(array)

    def _append_path(self, path: Path, pose, stamp) -> None:
        path.header.frame_id = self.map_frame
        path.header.stamp = stamp
        stamped = PoseStamped()
        stamped.header = path.header
        stamped.pose.position.x = pose[0]
        stamped.pose.position.y = pose[1]
        stamped.pose.orientation.z = math.sin(pose[2] * 0.5)
        stamped.pose.orientation.w = math.cos(pose[2] * 0.5)
        path.poses.append(stamped)
        if len(path.poses) > self.maximum_path_points:
            del path.poses[: len(path.poses) - self.maximum_path_points]

    def _reset_paths(self) -> None:
        self.raw_path = Path()
        self.laser_path = Path()
        self.corrected_path = Path()
        self.truth_path = Path()
        self.map_from_odom = None
        self.map_from_laser_odom = None

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _pose_from_odometry(message: Odometry):
        pose = message.pose.pose
        rotation = pose.orientation
        return (
            float(pose.position.x),
            float(pose.position.y),
            math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            ),
        )


def main(args=None):
    rclpy.init(args=args)
    node = FenceRvizBridge()
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
