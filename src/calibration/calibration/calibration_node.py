from __future__ import annotations

import copy
import math

from geometry_msgs.msg import TwistWithCovariance
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from calibration.wall_fitter import (
    FitResult,
    RectangleWallFitter,
    ego_pose_from_lidar,
    lidar_pose_from_ego,
)


def propagate_pose_with_relative_motion(
    calibrated_anchor: tuple[float, float, float],
    prior_anchor: tuple[float, float, float],
    current_prior: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Apply only a prior's relative SE(2) motion to a calibrated map pose."""
    anchor_x, anchor_y, anchor_yaw = prior_anchor
    current_x, current_y, current_yaw = current_prior
    calibrated_x, calibrated_y, calibrated_yaw = calibrated_anchor

    delta_x = current_x - anchor_x
    delta_y = current_y - anchor_y
    local_x = math.cos(anchor_yaw) * delta_x + math.sin(anchor_yaw) * delta_y
    local_y = -math.sin(anchor_yaw) * delta_x + math.cos(anchor_yaw) * delta_y
    predicted_x = (
        calibrated_x
        + math.cos(calibrated_yaw) * local_x
        - math.sin(calibrated_yaw) * local_y
    )
    predicted_y = (
        calibrated_y
        + math.sin(calibrated_yaw) * local_x
        + math.cos(calibrated_yaw) * local_y
    )
    predicted_yaw = math.atan2(
        math.sin(calibrated_yaw + current_yaw - anchor_yaw),
        math.cos(calibrated_yaw + current_yaw - anchor_yaw),
    )
    return predicted_x, predicted_y, predicted_yaw


class CalibrationNode(Node):
    """Publish a map pose by fitting /scan returns to the rectangular walls."""

    def __init__(self) -> None:
        super().__init__("calibration_node")
        self._declare_parameters()

        self._lidar_frame = str(self.get_parameter("lidar_frame").value)
        self._lidar_offset_x_m = float(
            self.get_parameter("lidar_offset_x_m").value
        )
        self._scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self._maximum_rms_error_m = float(
            self.get_parameter("maximum_rms_error_m").value
        )
        self._fallback_position_variance_m2 = float(
            self.get_parameter("fallback_position_variance_m2").value
        )
        self._fallback_yaw_variance_rad2 = float(
            self.get_parameter("fallback_yaw_variance_rad2").value
        )
        maximum_prior_age_sec = float(
            self.get_parameter("maximum_prior_age_sec").value
        )
        initial_ego_pose = (
            float(self.get_parameter("initial_pose_x_m").value),
            float(self.get_parameter("initial_pose_y_m").value),
            float(self.get_parameter("initial_pose_yaw_rad").value),
        )
        if (
            not math.isfinite(self._lidar_offset_x_m)
            or not math.isfinite(self._maximum_rms_error_m)
            or self._maximum_rms_error_m <= 0.0
            or not math.isfinite(maximum_prior_age_sec)
            or maximum_prior_age_sec <= 0.0
            or not math.isfinite(self._fallback_position_variance_m2)
            or self._fallback_position_variance_m2 <= 0.0
            or not math.isfinite(self._fallback_yaw_variance_rad2)
            or self._fallback_yaw_variance_rad2 <= 0.0
            or not all(math.isfinite(value) for value in initial_ego_pose)
        ):
            raise ValueError("pose parameters must be finite and thresholds positive")
        self._maximum_prior_age_ns = int(maximum_prior_age_sec * 1.0e9)
        self._initial_pose = lidar_pose_from_ego(
            initial_ego_pose, self._lidar_offset_x_m
        )
        self._pose = self._initial_pose
        self._tracking_initialized = False
        self._last_stamp_ns: int | None = None
        self._latest_prior_pose: tuple[float, float, float] | None = None
        self._latest_prior_stamp_ns: int | None = None
        self._latest_prior_twist: TwistWithCovariance | None = None
        self._latest_prior_twist_stamp_ns: int | None = None
        self._tracking_pose_anchor: tuple[float, float, float] | None = None
        self._tracking_prior_anchor: tuple[float, float, float] | None = None

        wall_bounds = (
            float(self.get_parameter("wall_minimum_x_m").value),
            float(self.get_parameter("wall_maximum_x_m").value),
            float(self.get_parameter("wall_minimum_y_m").value),
            float(self.get_parameter("wall_maximum_y_m").value),
        )
        minimum_walls = int(self.get_parameter("minimum_walls").value)
        self._two_wall_maximum_rms_error_m = float(
            self.get_parameter("two_wall_maximum_rms_error_m").value
        )
        if minimum_walls < 2 or minimum_walls > 4:
            raise ValueError("minimum_walls must be between 2 and 4")
        if (
            not math.isfinite(self._two_wall_maximum_rms_error_m)
            or self._two_wall_maximum_rms_error_m <= 0.0
        ):
            raise ValueError("two-wall RMS threshold must be finite and positive")
        self._fitter = RectangleWallFitter(
            wall_bounds,
            maximum_match_distance_m=float(
                self.get_parameter("maximum_match_distance_m").value
            ),
            minimum_matches=int(self.get_parameter("minimum_matches").value),
            minimum_walls=max(3, minimum_walls),
            minimum_matches_per_wall=int(
                self.get_parameter("minimum_matches_per_wall").value
            ),
            maximum_position_step_m=float(
                self.get_parameter("maximum_position_step_m").value
            ),
            maximum_yaw_step_rad=float(
                self.get_parameter("maximum_yaw_step_rad").value
            ),
        )
        self._two_wall_fitter: RectangleWallFitter | None = None
        if minimum_walls == 2:
            self._two_wall_fitter = RectangleWallFitter(
                wall_bounds,
                maximum_match_distance_m=float(
                    self.get_parameter("maximum_match_distance_m").value
                ),
                minimum_matches=int(self.get_parameter("minimum_matches").value),
                minimum_walls=2,
                minimum_matches_per_wall=int(
                    self.get_parameter("two_wall_minimum_matches_per_wall").value
                ),
                maximum_position_step_m=float(
                    self.get_parameter("two_wall_maximum_position_step_m").value
                ),
                maximum_yaw_step_rad=float(
                    self.get_parameter("two_wall_maximum_yaw_step_rad").value
                ),
            )

        self._publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._prior_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("prior_topic").value),
            self._on_prior,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "rectangle-wall calibration ready: /scan + /pose prior -> "
            "/pose/calibration; "
            f"initial ego pose={initial_ego_pose}, minimum walls={minimum_walls}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("prior_topic", "/pose")
        self.declare_parameter("output_topic", "/pose/calibration")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("lidar_offset_x_m", -0.027)
        self.declare_parameter("wall_minimum_x_m", 0.0)
        self.declare_parameter("wall_maximum_x_m", 12.0)
        self.declare_parameter("wall_minimum_y_m", 0.0)
        self.declare_parameter("wall_maximum_y_m", 7.0)
        # A rectangle has symmetric absolute poses. The first accepted wall fit
        # always starts from this seed; later fits keep that solution continuous.
        # First center point in rddf/rddf_real.csv, facing map -Y.
        self.declare_parameter("initial_pose_x_m", 1.400001)
        self.declare_parameter("initial_pose_y_m", 3.394607)
        self.declare_parameter("initial_pose_yaw_rad", -math.pi / 2.0)
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("maximum_match_distance_m", 0.30)
        self.declare_parameter("minimum_matches", 50)
        self.declare_parameter("minimum_walls", 2)
        self.declare_parameter("minimum_matches_per_wall", 8)
        self.declare_parameter("maximum_position_step_m", 0.45)
        self.declare_parameter("maximum_yaw_step_rad", 0.35)
        self.declare_parameter("maximum_rms_error_m", 0.10)
        # Fallback is deliberately less certain than a wall fit. These values
        # are exposed even though the current planner does not consume covariance.
        self.declare_parameter("fallback_position_variance_m2", 0.04)
        self.declare_parameter("fallback_yaw_variance_rad2", 0.030461741978670857)
        self.declare_parameter("two_wall_minimum_matches_per_wall", 12)
        self.declare_parameter("two_wall_maximum_position_step_m", 0.25)
        self.declare_parameter("two_wall_maximum_yaw_step_rad", 0.15)
        self.declare_parameter("two_wall_maximum_rms_error_m", 0.08)
        self.declare_parameter("maximum_prior_age_sec", 0.25)

    def _on_prior(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        quaternion = np.asarray(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quaternion))
        if (
            message.header.frame_id != "map"
            or not math.isfinite(position.x)
            or not math.isfinite(position.y)
            or not math.isfinite(norm)
            or norm < 1.0e-9
        ):
            return
        qx, qy, qz, qw = quaternion / norm
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self._latest_prior_pose = lidar_pose_from_ego(
            (float(position.x), float(position.y), yaw), self._lidar_offset_x_m
        )
        prior_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        self._latest_prior_stamp_ns = prior_stamp_ns

        twist = message.twist.twist
        twist_values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if not all(math.isfinite(value) for value in twist_values):
            self._latest_prior_twist = None
            self._latest_prior_twist_stamp_ns = None
            return

        forwarded_twist = copy.deepcopy(message.twist)
        if not all(math.isfinite(value) for value in forwarded_twist.covariance):
            forwarded_twist.covariance = [0.0] * 36
            for index in (0, 7, 14, 21, 28, 35):
                forwarded_twist.covariance[index] = 1.0e6
        self._latest_prior_twist = forwarded_twist
        self._latest_prior_twist_stamp_ns = prior_stamp_ns

    def _scan_points(self, scan: LaserScan) -> np.ndarray | None:
        if (
            scan.header.frame_id != self._lidar_frame
            or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or scan.range_max <= scan.range_min
        ):
            return None

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        indices = np.arange(0, len(ranges), self._scan_stride)
        selected = ranges[indices]
        valid = (
            np.isfinite(selected)
            & (selected >= float(scan.range_min))
            & (selected <= float(scan.range_max))
        )
        if int(np.count_nonzero(valid)) < self._fitter.minimum_matches:
            return None
        angles = float(scan.angle_min) + indices[valid] * float(scan.angle_increment)
        distances = selected[valid]
        return np.column_stack(
            (distances * np.cos(angles), distances * np.sin(angles))
        )

    def _reset_tracking(self) -> None:
        self._pose = self._initial_pose
        self._tracking_initialized = False
        self._latest_prior_pose = None
        self._latest_prior_stamp_ns = None
        self._latest_prior_twist = None
        self._latest_prior_twist_stamp_ns = None
        self._tracking_pose_anchor = None
        self._tracking_prior_anchor = None

    def _fresh_prior(self, stamp_ns: int) -> bool:
        return (
            self._latest_prior_pose is not None
            and self._latest_prior_stamp_ns is not None
            and abs(stamp_ns - self._latest_prior_stamp_ns)
            <= self._maximum_prior_age_ns
        )

    def _fresh_twist(self, stamp_ns: int) -> bool:
        return (
            self._latest_prior_twist is not None
            and self._latest_prior_twist_stamp_ns is not None
            and abs(stamp_ns - self._latest_prior_twist_stamp_ns)
            <= self._maximum_prior_age_ns
        )

    def _fit_initial_poses(
        self, stamp_ns: int
    ) -> tuple[tuple[float, float, float], ...]:
        if not self._tracking_initialized:
            return (self._initial_pose,)

        predicted = self._fallback_pose(stamp_ns)
        if predicted is not None and predicted != self._pose:
            return (predicted, self._pose)
        return (self._pose,)

    def _fallback_pose(
        self, stamp_ns: int
    ) -> tuple[float, float, float] | None:
        if (
            not self._tracking_initialized
            or not self._fresh_prior(stamp_ns)
            or self._tracking_pose_anchor is None
            or self._tracking_prior_anchor is None
            or self._latest_prior_pose is None
        ):
            return None
        return propagate_pose_with_relative_motion(
            self._tracking_pose_anchor,
            self._tracking_prior_anchor,
            self._latest_prior_pose,
        )

    def _accept_pose(
        self, pose: tuple[float, float, float], stamp_ns: int
    ) -> None:
        self._pose = pose
        self._tracking_initialized = True
        if self._fresh_prior(stamp_ns):
            self._tracking_pose_anchor = pose
            self._tracking_prior_anchor = self._latest_prior_pose
        else:
            # Never reuse an anchor from an older fit after recovery.
            self._tracking_pose_anchor = None
            self._tracking_prior_anchor = None

    def _fit(self, points: np.ndarray, stamp_ns: int) -> FitResult | None:
        initial_poses = self._fit_initial_poses(stamp_ns)
        result = self._fitter.fit_first(
            points, initial_poses, self._maximum_rms_error_m
        )
        if result is not None or self._two_wall_fitter is None:
            return result
        return self._two_wall_fitter.fit_first(
            points, initial_poses, self._two_wall_maximum_rms_error_m
        )

    def _on_scan(self, scan: LaserScan) -> None:
        stamp_ns = int(scan.header.stamp.sec) * 1_000_000_000 + int(
            scan.header.stamp.nanosec
        )
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            self._reset_tracking()
            self.get_logger().warning("scan time moved backwards; reset initial pose")
        self._last_stamp_ns = stamp_ns

        points = self._scan_points(scan)
        if points is None:
            fallback_pose = self._fallback_pose(stamp_ns)
            if fallback_pose is None:
                self.get_logger().warning(
                    "invalid /scan and odometry fallback unavailable",
                    throttle_duration_sec=2.0,
                )
                return
            self._pose = fallback_pose
            self._publisher.publish(self._odometry(scan, None))
            self.get_logger().warning(
                "invalid /scan; publishing odometry fallback",
                throttle_duration_sec=2.0,
            )
            return

        result = self._fit(points, stamp_ns)
        if result is None:
            fallback_pose = self._fallback_pose(stamp_ns)
            if fallback_pose is not None:
                self._pose = fallback_pose
                self._publisher.publish(self._odometry(scan, None))
                self.get_logger().warning(
                    "rectangle wall fit rejected; publishing odometry fallback",
                    throttle_duration_sec=2.0,
                )
                return
            self.get_logger().warning(
                "rectangle wall fit rejected and odometry fallback unavailable",
                throttle_duration_sec=2.0,
            )
            return

        self._accept_pose(result.pose, stamp_ns)
        self._publisher.publish(self._odometry(scan, result.rms_error_m))

    def _odometry(
        self, scan: LaserScan, rms_error_m: float | None
    ) -> Odometry:
        x, y, yaw = ego_pose_from_lidar(
            self._pose, self._lidar_offset_x_m
        )
        message = Odometry()
        message.header.stamp = scan.header.stamp
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.child_frame_id = self._lidar_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if rms_error_m is None:
            position_variance = self._fallback_position_variance_m2
            yaw_variance = self._fallback_yaw_variance_rad2
        else:
            position_variance = max(rms_error_m * rms_error_m, 1.0e-4)
            yaw_variance = position_variance
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[14] = 1.0e6
        message.pose.covariance[21] = 1.0e6
        message.pose.covariance[28] = 1.0e6
        message.pose.covariance[35] = yaw_variance
        scan_stamp_ns = int(scan.header.stamp.sec) * 1_000_000_000 + int(
            scan.header.stamp.nanosec
        )
        if self._fresh_twist(scan_stamp_ns):
            message.twist = copy.deepcopy(self._latest_prior_twist)
        else:
            for index in (0, 7, 14, 21, 28, 35):
                message.twist.covariance[index] = 1.0e6
        return message


def main(args=None) -> None:
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
