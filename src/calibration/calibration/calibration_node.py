from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from calibration.wall_fitter import (
    RectangleWallFitter,
    ego_pose_from_lidar,
    lidar_pose_from_ego,
)


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
            or not all(math.isfinite(value) for value in initial_ego_pose)
        ):
            raise ValueError("pose parameters must be finite and thresholds positive")
        self._maximum_prior_age_ns = int(maximum_prior_age_sec * 1.0e9)
        self._initial_pose = lidar_pose_from_ego(
            initial_ego_pose, self._lidar_offset_x_m
        )
        self._pose = self._initial_pose
        self._last_stamp_ns: int | None = None
        self._latest_prior_pose: tuple[float, float, float] | None = None
        self._latest_prior_stamp_ns: int | None = None

        self._fitter = RectangleWallFitter(
            (
                float(self.get_parameter("wall_minimum_x_m").value),
                float(self.get_parameter("wall_maximum_x_m").value),
                float(self.get_parameter("wall_minimum_y_m").value),
                float(self.get_parameter("wall_maximum_y_m").value),
            ),
            maximum_match_distance_m=float(
                self.get_parameter("maximum_match_distance_m").value
            ),
            minimum_matches=int(self.get_parameter("minimum_matches").value),
            minimum_walls=int(self.get_parameter("minimum_walls").value),
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
            f"initial ego pose={initial_ego_pose}"
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
        # A rectangle has symmetric absolute poses. This seed (or a fresh /pose
        # retry prior) chooses one; the previous wall fit keeps it continuous.
        # First center point in rddf/rddf_real.csv, facing map -Y.
        self.declare_parameter("initial_pose_x_m", 1.400001)
        self.declare_parameter("initial_pose_y_m", 3.394607)
        self.declare_parameter("initial_pose_yaw_rad", -math.pi / 2.0)
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("maximum_match_distance_m", 0.30)
        self.declare_parameter("minimum_matches", 50)
        self.declare_parameter("minimum_walls", 3)
        self.declare_parameter("minimum_matches_per_wall", 8)
        self.declare_parameter("maximum_position_step_m", 0.45)
        self.declare_parameter("maximum_yaw_step_rad", 0.35)
        self.declare_parameter("maximum_rms_error_m", 0.10)
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
        self._latest_prior_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

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
        self._latest_prior_pose = None
        self._latest_prior_stamp_ns = None

    def _on_scan(self, scan: LaserScan) -> None:
        points = self._scan_points(scan)
        if points is None:
            self.get_logger().warning(
                "ignoring invalid or undersampled /scan in an unexpected frame",
                throttle_duration_sec=2.0,
            )
            return

        stamp_ns = int(scan.header.stamp.sec) * 1_000_000_000 + int(
            scan.header.stamp.nanosec
        )
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            self._reset_tracking()
            self.get_logger().warning("scan time moved backwards; reset initial pose")
        self._last_stamp_ns = stamp_ns

        initial_poses = (self._pose,)
        if (
            self._latest_prior_pose is not None
            and self._latest_prior_stamp_ns is not None
            and abs(stamp_ns - self._latest_prior_stamp_ns)
            <= self._maximum_prior_age_ns
        ):
            initial_poses = (self._latest_prior_pose, self._pose)
        result = self._fitter.fit_first(
            points, initial_poses, self._maximum_rms_error_m
        )
        if result is None:
            self.get_logger().warning(
                "rectangle wall fit rejected",
                throttle_duration_sec=2.0,
            )
            return

        self._pose = result.pose
        self._publisher.publish(self._odometry(scan, result.rms_error_m))

    def _odometry(self, scan: LaserScan, rms_error_m: float) -> Odometry:
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

        variance = max(rms_error_m * rms_error_m, 1.0e-4)
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[14] = 1.0e6
        message.pose.covariance[21] = 1.0e6
        message.pose.covariance[28] = 1.0e6
        message.pose.covariance[35] = variance
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
