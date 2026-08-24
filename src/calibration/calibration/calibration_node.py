from __future__ import annotations

import copy
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from calibration.bev import (
    BevGeometry,
    BevProjector,
    CameraModel,
    transform_to_matrix,
)
from calibration.lane_detector import LaneDetection, YellowLaneDetector
from calibration.odom_corrector import (
    LaneOdomCorrector,
    load_centerline_csv,
    transform_points,
)


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")
        self._declare_parameters()

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.frame_step = int(self.get_parameter("frame_step").value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_sec").value)
        )
        self.minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        self.publish_debug_images = bool(
            self.get_parameter("publish_debug_images").value
        )
        self.reference_frame = self.get_parameter("reference_frame").value
        self.maximum_detection_age_sec = float(
            self.get_parameter("maximum_detection_age_sec").value
        )

        camera = CameraModel(
            width=int(self.get_parameter("camera_width").value),
            height=int(self.get_parameter("camera_height").value),
            horizontal_fov_rad=float(
                self.get_parameter("horizontal_fov_rad").value
            ),
            distortion=tuple(float(value) for value in self.get_parameter("distortion").value),
        )
        geometry = BevGeometry(
            x_min_m=float(self.get_parameter("bev_x_min_m").value),
            x_max_m=float(self.get_parameter("bev_x_max_m").value),
            y_left_m=float(self.get_parameter("bev_y_left_m").value),
            y_right_m=float(self.get_parameter("bev_y_right_m").value),
            resolution_m=float(self.get_parameter("bev_resolution_m").value),
            ground_z_m=float(self.get_parameter("ground_z_m").value),
        )
        self.projector = BevProjector(camera, geometry)
        self.detector = YellowLaneDetector(
            geometry=geometry,
            hsv_lower=tuple(
                int(value) for value in self.get_parameter("yellow_hsv_lower").value
            ),
            hsv_upper=tuple(
                int(value) for value in self.get_parameter("yellow_hsv_upper").value
            ),
            minimum_points=int(self.get_parameter("minimum_lane_points").value),
            minimum_span_m=float(self.get_parameter("minimum_lane_span_m").value),
            maximum_residual_m=float(
                self.get_parameter("maximum_lane_residual_m").value
            ),
        )
        self.corrector = LaneOdomCorrector(
            maximum_match_distance_m=float(
                self.get_parameter("maximum_match_distance_m").value
            ),
            minimum_matches=int(self.get_parameter("minimum_correction_matches").value),
            maximum_lateral_correction_m=float(
                self.get_parameter("maximum_lateral_correction_m").value
            ),
            maximum_yaw_correction_rad=float(
                self.get_parameter("maximum_yaw_correction_rad").value
            ),
            smoothing_alpha=float(self.get_parameter("correction_smoothing_alpha").value),
        )
        self.reference_centerline = self._load_reference_centerline()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frame_count = 0
        self.latest_odometry = None
        self.latest_camera_pan = None
        self.latest_lane_points = None
        self.latest_lane_stamp = None

        self.calibrated_odom_pub = self.create_publisher(
            Odometry, "/odom/calibride", 10
        )
        self.centerline_pub = self.create_publisher(
            Path, "/calibration/detected_centerline", 10
        )
        self.bev_pub = self.create_publisher(
            CompressedImage, "/calibration/debug/bev/compressed", 1
        )
        self.mask_pub = self.create_publisher(
            CompressedImage, "/calibration/debug/lane_mask/compressed", 1
        )
        self.overlay_pub = self.create_publisher(
            CompressedImage, "/calibration/debug/lane_overlay/compressed", 1
        )

        image_topic = self.get_parameter("image_topic").value
        self.image_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odometry, 10
        )
        self.camera_pan_sub = self.create_subscription(
            Float64, "/camera/pan", self.on_camera_pan, 10
        )
        correction_state = "enabled" if self.reference_centerline is not None else "fallback"
        self.get_logger().info(f"BEV lane detection ready; odom correction mode={correction_state}")

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("camera_width", 480)
        self.declare_parameter("camera_height", 360)
        self.declare_parameter("horizontal_fov_rad", 1.7453)
        self.declare_parameter(
            "distortion", [-0.045, -0.0001, -0.0003, -0.0001, 0.001]
        )
        self.declare_parameter("frame_step", 5)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("bev_x_min_m", 0.15)
        self.declare_parameter("bev_x_max_m", 3.0)
        self.declare_parameter("bev_y_left_m", 1.2)
        self.declare_parameter("bev_y_right_m", 1.2)
        self.declare_parameter("bev_resolution_m", 0.02)
        self.declare_parameter("ground_z_m", 0.0)
        self.declare_parameter("yellow_hsv_lower", [8, 80, 80])
        self.declare_parameter("yellow_hsv_upper", [40, 255, 255])
        self.declare_parameter("minimum_lane_points", 30)
        self.declare_parameter("minimum_lane_span_m", 0.4)
        self.declare_parameter("maximum_lane_residual_m", 0.08)
        self.declare_parameter("minimum_confidence", 0.35)
        self.declare_parameter("publish_debug_images", True)
        self.declare_parameter("reference_centerline_file", "")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("maximum_detection_age_sec", 0.25)
        self.declare_parameter("maximum_match_distance_m", 0.35)
        self.declare_parameter("minimum_correction_matches", 8)
        self.declare_parameter("maximum_lateral_correction_m", 0.20)
        self.declare_parameter("maximum_yaw_correction_rad", 0.12)
        self.declare_parameter("correction_smoothing_alpha", 0.25)

    def _load_reference_centerline(self):
        path = str(self.get_parameter("reference_centerline_file").value)
        if not path:
            self.get_logger().warning(
                "reference_centerline_file is empty; publishing unmodified /odom"
            )
            return None
        try:
            points = load_centerline_csv(path)
        except (OSError, ValueError) as error:
            self.get_logger().error(
                f"failed to load centerline '{path}': {error}; correction disabled"
            )
            return None
        self.get_logger().info(f"loaded {len(points)} centerline points from {path}")
        return points

    def on_image(self, image_message: CompressedImage) -> None:
        self.frame_count += 1
        if self.frame_step > 1 and (self.frame_count - 1) % self.frame_step != 0:
            return

        encoded = np.frombuffer(image_message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("failed to decode compressed camera image")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame,
                Time.from_msg(image_message.header.stamp),
                timeout=self.tf_timeout,
            )
            bev = self.projector.project(
                image, transform_to_matrix(transform.transform)
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(
                f"skipping camera frame because BEV transform failed: {error}",
                throttle_duration_sec=2.0,
            )
            return

        detection = self.detector.detect(bev)
        if detection is not None and detection.confidence >= self.minimum_confidence:
            x_values = np.linspace(detection.x_min_m, detection.x_max_m, 40)
            self.latest_lane_points = np.column_stack(
                (x_values, detection.evaluate(x_values))
            )
            self.latest_lane_stamp = image_message.header.stamp
            self.centerline_pub.publish(
                self._create_centerline_path(image_message, detection)
            )

        if self.publish_debug_images:
            mask = detection.mask if detection is not None else self.detector.create_mask(bev)
            overlay = self.detector.draw_overlay(bev, detection)
            self._publish_compressed(self.bev_pub, image_message, bev)
            self._publish_compressed(self.mask_pub, image_message, mask)
            self._publish_compressed(self.overlay_pub, image_message, overlay)

    def _create_centerline_path(
        self, image_message: CompressedImage, detection: LaneDetection
    ) -> Path:
        path = Path()
        path.header.stamp = image_message.header.stamp
        path.header.frame_id = self.base_frame
        x_values = np.linspace(detection.x_min_m, detection.x_max_m, 40)
        y_values = detection.evaluate(x_values)
        for x_m, y_m in zip(x_values, y_values):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x_m)
            pose.pose.position.y = float(y_m)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    @staticmethod
    def _publish_compressed(
        publisher, source_message: CompressedImage, image: np.ndarray
    ) -> None:
        success, encoded = cv2.imencode(".jpg", image)
        if not success:
            return
        message = CompressedImage()
        message.header = source_message.header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        publisher.publish(message)

    def on_odometry(self, odometry: Odometry) -> None:
        self.latest_odometry = odometry
        output = copy.deepcopy(odometry)
        correction = self._estimate_odometry_correction(odometry)
        if correction is not None:
            pose = output.pose.pose
            yaw = self._yaw_from_quaternion(pose.orientation)
            pose.position.x -= math.sin(yaw) * correction.lateral_m
            pose.position.y += math.cos(yaw) * correction.lateral_m
            corrected_yaw = yaw + correction.yaw_rad
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(corrected_yaw * 0.5)
            pose.orientation.w = math.cos(corrected_yaw * 0.5)
        self.calibrated_odom_pub.publish(output)

    def _estimate_odometry_correction(self, odometry: Odometry):
        if self.reference_centerline is None or self.latest_lane_points is None:
            return None
        age = self._stamp_seconds(odometry.header.stamp) - self._stamp_seconds(
            self.latest_lane_stamp
        )
        if age < 0.0 or age > self.maximum_detection_age_sec:
            return None

        odom_frame = odometry.header.frame_id or "odom"
        if self.reference_frame == odom_frame:
            reference_odom = self.reference_centerline
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    odom_frame,
                    self.reference_frame,
                    Time.from_msg(odometry.header.stamp),
                    timeout=self.tf_timeout,
                )
                reference_odom = transform_points(
                    self.reference_centerline,
                    transform_to_matrix(transform.transform),
                )
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f"centerline transform failed; publishing raw odom: {error}",
                    throttle_duration_sec=2.0,
                )
                return None

        pose = odometry.pose.pose
        return self.corrector.estimate(
            self.latest_lane_points,
            reference_odom,
            pose.position.x,
            pose.position.y,
            self._yaw_from_quaternion(pose.orientation),
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_camera_pan(self, camera_pan: Float64) -> None:
        # Geometry uses timestamped TF. This command is retained only for diagnostics.
        self.latest_camera_pan = camera_pan.data


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
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
