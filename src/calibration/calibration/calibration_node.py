from __future__ import annotations

import copy
import math
from pathlib import Path as FilePath

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from calibration.bev import (
    BevGeometry,
    BevProjector,
    CameraModel,
    transform_to_matrix,
)
from calibration.lane_detector import LaneDetection, YellowLaneDetector
from calibration.correction_ekf import PoseCorrectionEkf
from calibration.fence_corrector import (
    FenceCorrectionResult,
    FenceOdomCorrector,
    FenceReference,
)
from calibration.lane_map import (
    load_lane_reference_image,
    polyline_reference,
    transform_reference,
)
from calibration.odom_corrector import (
    CorrectionQualityGate,
    LaneOdomCorrector,
    is_meaningful_correction,
    load_centerline_csv,
)
from calibration.pose_geometry import map_from_odom_pose


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
        self.maximum_camera_tf_age_sec = float(
            self.get_parameter("maximum_camera_tf_age_sec").value
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
        self.correction_source = str(
            self.get_parameter("correction_source").value
        ).strip().lower()
        if self.correction_source not in {"lane", "fence"}:
            raise ValueError("correction_source must be either 'lane' or 'fence'")

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
            minimum_component_area_px=int(
                self.get_parameter("minimum_lane_component_area_px").value
            ),
            point_spacing_m=float(self.get_parameter("observed_point_spacing_m").value),
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
            local_fit_radius_m=float(self.get_parameter("local_fit_radius_m").value),
            minimum_local_fit_points=int(
                self.get_parameter("minimum_local_fit_points").value
            ),
            maximum_tangent_angle_difference_rad=float(
                self.get_parameter("maximum_tangent_angle_difference_rad").value
            ),
        )
        self.correction_ekf = PoseCorrectionEkf(
            process_position_variance_per_sec=float(
                self.get_parameter("correction_ekf_process_position_variance_per_sec").value
            ),
            process_yaw_variance_per_sec=float(
                self.get_parameter("correction_ekf_process_yaw_variance_per_sec").value
            ),
            minimum_measurement_position_variance=float(
                self.get_parameter("correction_ekf_minimum_position_variance").value
            ),
            measurement_yaw_variance=float(
                self.get_parameter("correction_ekf_measurement_yaw_variance").value
            ),
            maximum_output_position_rate_m_s=float(
                self.get_parameter("maximum_correction_position_rate_m_s").value
            ),
            maximum_output_yaw_rate_rad_s=float(
                self.get_parameter("maximum_correction_yaw_rate_rad_s").value
            ),
        )
        self.use_lane_yaw_correction = bool(
            self.get_parameter("use_lane_yaw_correction").value
        )
        self.minimum_lateral_correction_update_m = float(
            self.get_parameter("minimum_lateral_correction_update_m").value
        )
        self.correction_quality_gate = CorrectionQualityGate(
            minimum_speed_m_s=float(
                self.get_parameter("minimum_correction_speed_m_s").value
            ),
            maximum_rms_error_m=float(
                self.get_parameter("maximum_correction_rms_error_m").value
            ),
            maximum_abs_lateral_m=float(
                self.get_parameter("maximum_accepted_lateral_correction_m").value
            ),
            maximum_abs_yaw_rad=float(
                self.get_parameter("maximum_accepted_yaw_correction_rad").value
            ),
            maximum_lateral_jump_m=float(
                self.get_parameter("maximum_correction_lateral_jump_m").value
            ),
            maximum_yaw_jump_rad=float(
                self.get_parameter("maximum_correction_yaw_jump_rad").value
            ),
            required_consistent_measurements=int(
                self.get_parameter("required_consistent_corrections").value
            ),
        )
        self.reference_lane = (
            self._load_reference_lane() if self.correction_source == "lane" else None
        )
        self.reference_fence = FenceReference.rectangle(
            float(self.get_parameter("fence_minimum_x_m").value),
            float(self.get_parameter("fence_maximum_x_m").value),
            float(self.get_parameter("fence_minimum_y_m").value),
            float(self.get_parameter("fence_maximum_y_m").value),
        )
        self.fence_corrector = FenceOdomCorrector(
            maximum_match_distance_m=float(
                self.get_parameter("fence_maximum_match_distance_m").value
            ),
            segment_endpoint_margin_m=float(
                self.get_parameter("fence_segment_endpoint_margin_m").value
            ),
            minimum_matches=int(
                self.get_parameter("fence_minimum_matches").value
            ),
            minimum_segments=int(
                self.get_parameter("fence_minimum_segments").value
            ),
            minimum_matches_per_segment=int(
                self.get_parameter("fence_minimum_matches_per_segment").value
            ),
            maximum_position_correction_m=float(
                self.get_parameter("fence_maximum_position_correction_m").value
            ),
            maximum_yaw_correction_rad=float(
                self.get_parameter("fence_maximum_yaw_correction_rad").value
            ),
            huber_delta_m=float(self.get_parameter("fence_huber_delta_m").value),
        )
        self.maximum_scan_age_sec = float(
            self.get_parameter("maximum_scan_age_sec").value
        )
        self.scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self.fence_maximum_rms_error_m = float(
            self.get_parameter("fence_maximum_rms_error_m").value
        )
        self.use_fixed_start_pose = bool(
            self.get_parameter("use_fixed_start_pose").value
        )
        self.fixed_start_pose = (
            float(self.get_parameter("fixed_start_map_x_m").value),
            float(self.get_parameter("fixed_start_map_y_m").value),
            float(self.get_parameter("fixed_start_map_yaw_rad").value),
        )
        self.fixed_map_from_odom = None
        self.fixed_odom_from_map_matrix = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_odom_broadcaster = StaticTransformBroadcaster(self)
        self.frame_count = 0
        self.latest_odometry = None
        self.latest_camera_pan = None
        self.latest_lane_points = None
        self.latest_lane_stamp = None
        self.latest_detection = None
        self.latest_bev = None
        self.latest_source_image = None
        self.latest_correction = None
        self.latest_measurement_pose = None
        self.latest_correction_stamp = None
        self.last_processed_lane_stamp = None
        self.latest_scan_points = None
        self.latest_scan_stamp = None
        self.last_processed_scan_stamp = None
        self.latest_fence_correction: FenceCorrectionResult | None = None
        self.last_odom_time_sec = None

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

        self.image_sub = None
        self.camera_pan_sub = None
        if self.correction_source == "lane":
            image_topic = self.get_parameter("image_topic").value
            self.image_sub = self.create_subscription(
                CompressedImage,
                image_topic,
                self.on_image,
                qos_profile_sensor_data,
            )
            self.camera_pan_sub = self.create_subscription(
                Float64, "/camera/pan", self.on_camera_pan, 10
            )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odometry, 10
        )
        self.scan_sub = None
        if self.correction_source == "fence":
            self.scan_sub = self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self.on_scan,
                qos_profile_sensor_data,
            )
        correction_state = (
            "enabled"
            if self.correction_source == "fence" or self.reference_lane is not None
            else "fallback"
        )
        self.get_logger().info(
            f"calibration ready; source={self.correction_source}, mode={correction_state}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("correction_source", "fence")
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("camera_width", 480)
        self.declare_parameter("camera_height", 360)
        self.declare_parameter("horizontal_fov_rad", 1.7453)
        self.declare_parameter(
            "distortion", [-0.045, -0.0001, -0.0003, -0.0001, 0.001]
        )
        self.declare_parameter("frame_step", 3)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("maximum_camera_tf_age_sec", 0.05)
        self.declare_parameter("bev_x_min_m", 0.15)
        self.declare_parameter("bev_x_max_m", 1.5)
        self.declare_parameter("bev_y_left_m", 1.2)
        self.declare_parameter("bev_y_right_m", 1.2)
        self.declare_parameter("bev_resolution_m", 0.02)
        self.declare_parameter("ground_z_m", 0.0)
        self.declare_parameter("yellow_hsv_lower", [15, 30, 30])
        self.declare_parameter("yellow_hsv_upper", [31, 220, 220])
        self.declare_parameter("minimum_lane_points", 12)
        self.declare_parameter("minimum_lane_span_m", 0.3)
        self.declare_parameter("minimum_lane_component_area_px", 4)
        self.declare_parameter("observed_point_spacing_m", 0.04)
        self.declare_parameter("minimum_confidence", 0.20)
        self.declare_parameter("publish_debug_images", True)
        self.declare_parameter("reference_lane_map_file", "")
        self.declare_parameter("reference_lane_map_resolution_m", 0.01)
        self.declare_parameter("reference_lane_map_origin_x_m", 0.0)
        self.declare_parameter("reference_lane_map_origin_y_m", 0.0)
        self.declare_parameter("reference_lane_map_hsv_lower", [14, 135, 145])
        self.declare_parameter("reference_lane_map_hsv_upper", [32, 255, 255])
        self.declare_parameter("reference_lane_point_spacing_m", 0.02)
        self.declare_parameter("reference_lane_tangent_radius_m", 0.10)
        self.declare_parameter("reference_centerline_file", "")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("maximum_detection_age_sec", 0.30)
        self.declare_parameter("maximum_match_distance_m", 0.35)
        self.declare_parameter("minimum_correction_matches", 8)
        self.declare_parameter("maximum_lateral_correction_m", 0.20)
        self.declare_parameter("maximum_yaw_correction_rad", 0.12)
        self.declare_parameter("minimum_correction_speed_m_s", 0.10)
        self.declare_parameter("maximum_correction_rms_error_m", 0.25)
        self.declare_parameter("maximum_accepted_lateral_correction_m", 0.20)
        self.declare_parameter("maximum_accepted_yaw_correction_rad", 0.12)
        self.declare_parameter("maximum_correction_lateral_jump_m", 0.25)
        self.declare_parameter("maximum_correction_yaw_jump_rad", 0.20)
        self.declare_parameter("required_consistent_corrections", 1)
        self.declare_parameter("use_lane_yaw_correction", False)
        self.declare_parameter("minimum_lateral_correction_update_m", 0.01)
        self.declare_parameter("correction_smoothing_alpha", 0.25)
        self.declare_parameter(
            "correction_ekf_process_position_variance_per_sec", 0.0025
        )
        self.declare_parameter(
            "correction_ekf_process_yaw_variance_per_sec", 0.0016
        )
        self.declare_parameter("correction_ekf_minimum_position_variance", 0.0025)
        self.declare_parameter("correction_ekf_measurement_yaw_variance", 0.0076)
        self.declare_parameter("maximum_correction_position_rate_m_s", 0.08)
        self.declare_parameter("maximum_correction_yaw_rate_rad_s", 0.08)
        self.declare_parameter("local_fit_radius_m", 0.20)
        self.declare_parameter("minimum_local_fit_points", 3)
        self.declare_parameter("maximum_tangent_angle_difference_rad", 0.44)
        self.declare_parameter("maximum_scan_age_sec", 0.15)
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("fence_minimum_x_m", 0.0)
        self.declare_parameter("fence_maximum_x_m", 12.0)
        self.declare_parameter("fence_minimum_y_m", 0.0)
        self.declare_parameter("fence_maximum_y_m", 7.0)
        self.declare_parameter("fence_maximum_match_distance_m", 0.25)
        self.declare_parameter("fence_segment_endpoint_margin_m", 0.10)
        self.declare_parameter("fence_minimum_matches", 80)
        self.declare_parameter("fence_minimum_segments", 3)
        self.declare_parameter("fence_minimum_matches_per_segment", 10)
        self.declare_parameter("fence_maximum_position_correction_m", 0.35)
        self.declare_parameter("fence_maximum_yaw_correction_rad", 0.15)
        self.declare_parameter("fence_huber_delta_m", 0.03)
        self.declare_parameter("fence_maximum_rms_error_m", 0.08)
        self.declare_parameter("use_fixed_start_pose", True)
        self.declare_parameter("fixed_start_map_x_m", 1.4)
        self.declare_parameter("fixed_start_map_y_m", 3.4)
        self.declare_parameter("fixed_start_map_yaw_rad", -1.5707963267948966)

    @staticmethod
    def _resolve_reference_path(value: str) -> FilePath:
        path = FilePath(value).expanduser()
        if path.is_absolute():
            return path
        source_package_root = FilePath(__file__).resolve().parents[1]
        candidates = [FilePath.cwd() / path, source_package_root / path]
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(FilePath(get_package_share_directory("calibration")) / path)
        except (ImportError, LookupError):
            pass
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        return (source_package_root / path).resolve()

    def _load_reference_lane(self):
        map_value = str(self.get_parameter("reference_lane_map_file").value)
        if map_value:
            path = self._resolve_reference_path(map_value)
            try:
                reference = load_lane_reference_image(
                    str(path),
                    resolution_m=float(
                        self.get_parameter("reference_lane_map_resolution_m").value
                    ),
                    origin_x_m=float(
                        self.get_parameter("reference_lane_map_origin_x_m").value
                    ),
                    origin_y_m=float(
                        self.get_parameter("reference_lane_map_origin_y_m").value
                    ),
                    hsv_lower=tuple(
                        int(value)
                        for value in self.get_parameter(
                            "reference_lane_map_hsv_lower"
                        ).value
                    ),
                    hsv_upper=tuple(
                        int(value)
                        for value in self.get_parameter(
                            "reference_lane_map_hsv_upper"
                        ).value
                    ),
                    point_spacing_m=float(
                        self.get_parameter("reference_lane_point_spacing_m").value
                    ),
                    tangent_radius_m=float(
                        self.get_parameter("reference_lane_tangent_radius_m").value
                    ),
                )
            except (OSError, ValueError) as error:
                self.get_logger().error(
                    f"failed to load lane map '{path}': {error}; correction disabled"
                )
                return None
            self.get_logger().info(
                f"loaded {len(reference.points)} geometric lane-map points from {path}"
            )
            return reference

        csv_value = str(self.get_parameter("reference_centerline_file").value)
        if not csv_value:
            self.get_logger().warning(
                "reference_lane_map_file is empty; publishing unmodified /odom"
            )
            return None
        path = self._resolve_reference_path(csv_value)
        try:
            reference = polyline_reference(load_centerline_csv(str(path)))
        except (OSError, ValueError) as error:
            self.get_logger().error(
                f"failed to load centerline '{path}': {error}; correction disabled"
            )
            return None
        self.get_logger().info(
            f"loaded {len(reference.points)} legacy centerline points from {path}"
        )
        return reference

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
            transform = self._lookup_camera_transform(image_message.header.stamp)
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
        self.latest_source_image = image
        self.latest_bev = bev
        self.latest_detection = detection
        if detection is not None and detection.confidence >= self.minimum_confidence:
            self.latest_lane_points = detection.points_m
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

    def _lookup_camera_transform(self, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame,
                Time.from_msg(stamp),
                timeout=self.tf_timeout,
            )
        except TransformException as exact_error:
            latest = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.base_frame,
                Time(),
                timeout=self.tf_timeout,
            )
            age = self._stamp_seconds(stamp) - self._stamp_seconds(latest.header.stamp)
            if (
                self.maximum_camera_tf_age_sec >= 0.0
                and abs(age) > self.maximum_camera_tf_age_sec
            ):
                raise exact_error
            return latest

    def _create_centerline_path(
        self, image_message: CompressedImage, detection: LaneDetection
    ) -> Path:
        path = Path()
        path.header.stamp = image_message.header.stamp
        path.header.frame_id = self.base_frame
        points = detection.points_m[np.argsort(detection.points_m[:, 0])]
        if len(points) > 80:
            indices = np.linspace(0, len(points) - 1, 80).round().astype(int)
            points = points[indices]
        for x_m, y_m in points:
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
        # Lossless debug output keeps HSV values identical to the image used by
        # the detector; JPEG artifacts otherwise make threshold tuning misleading.
        success, encoded = cv2.imencode(".png", image)
        if not success:
            return
        message = CompressedImage()
        message.header = source_message.header
        message.format = "png"
        message.data = encoded.tobytes()
        publisher.publish(message)

    def on_scan(self, scan: LaserScan) -> None:
        if not scan.header.frame_id or not math.isfinite(scan.angle_increment):
            self.get_logger().warning(
                "skipping LaserScan with an invalid frame or angle increment",
                throttle_duration_sec=2.0,
            )
            return
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        indices = np.arange(0, len(ranges), self.scan_stride)
        selected_ranges = ranges[indices]
        valid = (
            np.isfinite(selected_ranges)
            & (selected_ranges >= float(scan.range_min))
            & (selected_ranges < float(scan.range_max))
        )
        if int(np.count_nonzero(valid)) < self.fence_corrector.minimum_matches:
            return
        angles = float(scan.angle_min) + indices[valid] * float(scan.angle_increment)
        distances = selected_ranges[valid]
        scan_points = np.column_stack(
            (distances * np.cos(angles), distances * np.sin(angles))
        )
        if scan.header.frame_id != self.base_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan.header.frame_id,
                    Time.from_msg(scan.header.stamp),
                    timeout=self.tf_timeout,
                )
                matrix = transform_to_matrix(transform.transform)
                homogeneous = np.column_stack(
                    (
                        scan_points,
                        np.zeros(len(scan_points)),
                        np.ones(len(scan_points)),
                    )
                )
                scan_points = (matrix @ homogeneous.T).T[:, :2]
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f"scan transform failed; ignoring scan: {error}",
                    throttle_duration_sec=2.0,
                )
                return
        self.latest_scan_points = scan_points
        self.latest_scan_stamp = scan.header.stamp

    def on_odometry(self, odometry: Odometry) -> None:
        self.latest_odometry = odometry
        output = copy.deepcopy(odometry)
        odom_time_sec = self._stamp_seconds(odometry.header.stamp)
        previous_odom_time_sec = self.last_odom_time_sec
        if (
            previous_odom_time_sec is not None
            and odom_time_sec + 1.0e-6 < previous_odom_time_sec
        ):
            self.last_processed_lane_stamp = None
            self.last_processed_scan_stamp = None
            self.corrector.reset()
            self.correction_ekf.reset()
            self.correction_quality_gate.reset()
            self.get_logger().warning(
                "odometry time moved backwards; cleared persistent correction state"
            )
            previous_odom_time_sec = None
        self.last_odom_time_sec = odom_time_sec

        raw_pose = self._pose_2d(odometry.pose.pose)
        self._initialize_fixed_map_from_odom(
            raw_pose,
            odometry.header.frame_id or "odom",
            odometry.header.stamp,
        )
        dt_sec = 0.0
        if previous_odom_time_sec is not None:
            dt_sec = max(0.0, odom_time_sec - previous_odom_time_sec)
        self.correction_ekf.predict(
            raw_pose,
            dt_sec,
            pose_covariance=odometry.pose.covariance,
            twist_covariance=odometry.twist.covariance,
        )

        if self.correction_source == "fence":
            scan_stamp = self._stamp_key(self.latest_scan_stamp)
            fence_correction = None
            if scan_stamp is not None and scan_stamp != self.last_processed_scan_stamp:
                fence_correction = self._estimate_fence_correction(odometry)
                scan_age = odom_time_sec - self._stamp_seconds(self.latest_scan_stamp)
                if scan_age >= 0.0:
                    self.last_processed_scan_stamp = scan_stamp
            if (
                fence_correction is not None
                and fence_correction.rms_error_m <= self.fence_maximum_rms_error_m
            ):
                self.correction_ekf.correct(
                    fence_correction.measured_pose,
                    rms_error_m=fence_correction.rms_error_m,
                    match_count=fence_correction.match_count,
                )
                self.latest_fence_correction = fence_correction
                self.latest_measurement_pose = fence_correction.measured_pose
                self.latest_correction_stamp = odometry.header.stamp
        else:
            correction = None
            lane_stamp = self._stamp_key(self.latest_lane_stamp)
            if lane_stamp is not None and lane_stamp != self.last_processed_lane_stamp:
                correction = self._estimate_odometry_correction(odometry)
                self.last_processed_lane_stamp = lane_stamp
            if correction is not None:
                speed_m_s = math.hypot(
                    odometry.twist.twist.linear.x,
                    odometry.twist.twist.linear.y,
                )
                if not self.correction_quality_gate.accept(correction, speed_m_s):
                    correction = None
            if correction is not None and not is_meaningful_correction(
                correction,
                minimum_lateral_m=self.minimum_lateral_correction_update_m,
                use_yaw=self.use_lane_yaw_correction,
            ):
                # A near-zero residual confirms the held correction.  Rebuilding
                # the measurement from raw odometry here would erase it.
                correction = None
            if correction is not None:
                measurement_yaw = (
                    correction.yaw_rad if self.use_lane_yaw_correction else 0.0
                )
                measured_pose = self.correction_ekf.correct_local_residual(
                    correction.lateral_m,
                    measurement_yaw,
                    rms_error_m=correction.rms_error_m,
                    match_count=correction.match_count,
                )
                self.latest_correction = correction
                self.latest_measurement_pose = measured_pose
                self.latest_correction_stamp = odometry.header.stamp
        self.correction_ekf.advance_output(dt_sec)
        corrected_pose = self.correction_ekf.output_pose or raw_pose
        pose = output.pose.pose
        pose.position.x = corrected_pose[0]
        pose.position.y = corrected_pose[1]
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(corrected_pose[2] * 0.5)
        pose.orientation.w = math.cos(corrected_pose[2] * 0.5)
        posterior_covariance = self.correction_ekf.output_covariance()
        planar_indices = (0, 1, 5)
        covariance = list(output.pose.covariance)
        for index in planar_indices:
            for column in range(6):
                covariance[index * 6 + column] = 0.0
                covariance[column * 6 + index] = 0.0
        for row_index, row in enumerate(planar_indices):
            for column_index, column in enumerate(planar_indices):
                covariance[row * 6 + column] = float(
                    posterior_covariance[row_index, column_index]
                )
        output.pose.covariance = covariance
        self.calibrated_odom_pub.publish(output)

    def _estimate_odometry_correction(self, odometry: Odometry):
        if self.reference_lane is None or self.latest_lane_points is None:
            return None
        age = self._stamp_seconds(odometry.header.stamp) - self._stamp_seconds(
            self.latest_lane_stamp
        )
        if age < 0.0 or age > self.maximum_detection_age_sec:
            return None

        odom_frame = odometry.header.frame_id or "odom"
        if self.reference_frame == odom_frame:
            reference_odom = self.reference_lane
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    odom_frame,
                    self.reference_frame,
                    Time.from_msg(odometry.header.stamp),
                    timeout=self.tf_timeout,
                )
                reference_odom = transform_reference(
                    self.reference_lane,
                    transform_to_matrix(transform.transform),
                )
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f"centerline transform failed; publishing raw odom: {error}",
                    throttle_duration_sec=2.0,
                )
                return None

        corrected_pose = self.correction_ekf.state
        if corrected_pose is None:
            return None
        return self.corrector.estimate(
            self.latest_lane_points,
            reference_odom,
            float(corrected_pose[0]),
            float(corrected_pose[1]),
            float(corrected_pose[2]),
        )

    def _estimate_fence_correction(self, odometry: Odometry):
        if self.latest_scan_points is None or self.latest_scan_stamp is None:
            return None
        age = self._stamp_seconds(odometry.header.stamp) - self._stamp_seconds(
            self.latest_scan_stamp
        )
        if age < 0.0 or age > self.maximum_scan_age_sec:
            return None

        odom_frame = odometry.header.frame_id or "odom"
        if self.reference_frame == odom_frame:
            reference_odom = self.reference_fence
        elif self.fixed_odom_from_map_matrix is not None:
            reference_odom = self.reference_fence.transformed(
                self.fixed_odom_from_map_matrix
            )
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    odom_frame,
                    self.reference_frame,
                    Time.from_msg(odometry.header.stamp),
                    timeout=self.tf_timeout,
                )
                reference_odom = self.reference_fence.transformed(
                    transform_to_matrix(transform.transform)
                )
            except (TransformException, ValueError) as error:
                self.get_logger().warning(
                    f"fence-map transform failed; publishing raw odom: {error}",
                    throttle_duration_sec=2.0,
                )
                return None

        corrected_pose = self.correction_ekf.state
        if corrected_pose is None:
            return None
        return self.fence_corrector.estimate(
            self.latest_scan_points,
            reference_odom,
            (
                float(corrected_pose[0]),
                float(corrected_pose[1]),
                float(corrected_pose[2]),
            ),
        )

    def _initialize_fixed_map_from_odom(
        self,
        raw_pose: tuple[float, float, float],
        odom_frame: str,
        stamp,
    ) -> None:
        if (
            not self.use_fixed_start_pose
            or self.reference_frame == odom_frame
            or self.fixed_map_from_odom is not None
        ):
            return
        self.fixed_map_from_odom = map_from_odom_pose(
            self.fixed_start_pose, raw_pose
        )
        x_m, y_m, yaw_rad = self.fixed_map_from_odom
        cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
        map_from_odom_matrix = np.asarray(
            [
                [cosine, -sine, 0.0, x_m],
                [sine, cosine, 0.0, y_m],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.fixed_odom_from_map_matrix = np.linalg.inv(map_from_odom_matrix)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.reference_frame
        transform.child_frame_id = odom_frame
        transform.transform.translation.x = x_m
        transform.transform.translation.y = y_m
        transform.transform.rotation.z = math.sin(yaw_rad * 0.5)
        transform.transform.rotation.w = math.cos(yaw_rad * 0.5)
        self.map_odom_broadcaster.sendTransform(transform)
        self.get_logger().info(
            "initialized fixed map->odom from configured start pose: "
            f"start=({self.fixed_start_pose[0]:.3f}, "
            f"{self.fixed_start_pose[1]:.3f}, {self.fixed_start_pose[2]:.3f}), "
            f"transform=({x_m:.3f}, {y_m:.3f}, {yaw_rad:.3f})"
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    @classmethod
    def _pose_2d(cls, pose) -> tuple[float, float, float]:
        return (
            float(pose.position.x),
            float(pose.position.y),
            cls._yaw_from_quaternion(pose.orientation),
        )

    @staticmethod
    def _stamp_key(stamp):
        if stamp is None:
            return None
        return int(stamp.sec), int(stamp.nanosec)

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
