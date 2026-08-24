from __future__ import annotations

import copy
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from calibration.correction_ekf import PoseCorrectionEkf
from calibration.fence_corrector import (
    FenceCorrectionResult,
    FenceOdomCorrector,
    FenceReference,
)
from calibration.pose_geometry import map_from_odom_pose


class CalibrationNode(Node):
    """Correct planar odometry by matching 2D LiDAR points to four map fences."""

    def __init__(self) -> None:
        super().__init__("calibration_node")
        self._declare_parameters()

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.reference_frame = str(self.get_parameter("reference_frame").value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter("tf_timeout_sec").value)
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
            minimum_matches=int(self.get_parameter("fence_minimum_matches").value),
            minimum_segments=int(self.get_parameter("fence_minimum_segments").value),
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
        self.correction_ekf = PoseCorrectionEkf(
            process_position_variance_per_sec=float(
                self.get_parameter(
                    "correction_ekf_process_position_variance_per_sec"
                ).value
            ),
            process_yaw_variance_per_sec=float(
                self.get_parameter("correction_ekf_process_yaw_variance_per_sec").value
            ),
            minimum_measurement_position_variance=float(
                self.get_parameter(
                    "correction_ekf_minimum_position_variance"
                ).value
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

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_odom_broadcaster = StaticTransformBroadcaster(self)

        self.fixed_map_from_odom = None
        self.fixed_odom_from_map_matrix = None
        self.latest_scan_points = None
        self.latest_scan_stamp = None
        self.last_processed_scan_stamp = None
        self.latest_fence_correction: FenceCorrectionResult | None = None
        self.last_odom_time_sec = None

        self.calibrated_odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.on_odometry,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "fence calibration ready; inputs=/odom + /scan_filtered, "
            "output=/odom/calibride"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("output_topic", "/odom/calibride")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.05)
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
                matrix = self._transform_to_matrix(transform.transform)
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
        output = copy.deepcopy(odometry)
        odom_time_sec = self._stamp_seconds(odometry.header.stamp)
        previous_odom_time_sec = self.last_odom_time_sec
        if (
            previous_odom_time_sec is not None
            and odom_time_sec + 1.0e-6 < previous_odom_time_sec
        ):
            self._reset_runtime_state()
            previous_odom_time_sec = None
            self.get_logger().warning(
                "odometry time moved backwards; cleared fence correction state"
            )
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

        scan_stamp = self._stamp_key(self.latest_scan_stamp)
        if scan_stamp is not None and scan_stamp != self.last_processed_scan_stamp:
            correction = self._estimate_fence_correction(odometry)
            scan_age = odom_time_sec - self._stamp_seconds(self.latest_scan_stamp)
            if scan_age >= 0.0:
                self.last_processed_scan_stamp = scan_stamp
            if (
                correction is not None
                and correction.rms_error_m <= self.fence_maximum_rms_error_m
            ):
                self.correction_ekf.correct(
                    correction.measured_pose,
                    rms_error_m=correction.rms_error_m,
                    match_count=correction.match_count,
                )
                self.latest_fence_correction = correction

        self.correction_ekf.advance_output(dt_sec)
        self._write_corrected_pose(output, raw_pose)
        self.calibrated_odom_pub.publish(output)

    def _estimate_fence_correction(
        self, odometry: Odometry
    ) -> FenceCorrectionResult | None:
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
                    self._transform_to_matrix(transform.transform)
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
            tuple(float(value) for value in corrected_pose),
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

    def _write_corrected_pose(
        self,
        output: Odometry,
        raw_pose: tuple[float, float, float],
    ) -> None:
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

    def _reset_runtime_state(self) -> None:
        self.last_processed_scan_stamp = None
        self.latest_fence_correction = None
        self.fixed_map_from_odom = None
        self.fixed_odom_from_map_matrix = None
        self.correction_ekf.reset()

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        rotation = transform.rotation
        quaternion = np.asarray(
            [rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float64
        )
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm <= 1.0e-12:
            raise ValueError("transform quaternion is invalid")
        x, y, z, w = quaternion / norm
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = np.asarray(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )
        matrix[0, 3] = float(transform.translation.x)
        matrix[1, 3] = float(transform.translation.y)
        matrix[2, 3] = float(transform.translation.z)
        return matrix

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
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


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
