import math
import time

from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64

from .controller_core import ControllerCore
from .models import (
    AdaptiveControlConfig,
    ControllerConfig,
    PIDConfig,
    PurePursuitConfig,
    SpeedLookaheadConfig,
    VehicleState,
)
from .pid import PIDController
from .pure_pursuit import PurePursuit


# /pose/calibration currently publishes the ego/planner point while its TF
# applies the -0.027 m base-to-LiDAR offset. This compatibility default is
# separately configurable because it is not the user-facing tracking point.
DEFAULT_CALIBRATED_POSE_ORIGIN_X_FROM_LIDAR_M = 0.027


def reference_offset_from_calibrated_pose(
    reference_point_x_from_lidar_m: float,
    calibrated_pose_origin_x_from_lidar_m: float = (
        DEFAULT_CALIBRATED_POSE_ORIGIN_X_FROM_LIDAR_M
    ),
) -> float:
    """Convert a LiDAR-relative reference x to the calibrated-pose origin."""

    if not all(
        math.isfinite(value)
        for value in (
            reference_point_x_from_lidar_m,
            calibrated_pose_origin_x_from_lidar_m,
        )
    ):
        raise ValueError("LiDAR-relative reference offsets must be finite")
    offset = (
        reference_point_x_from_lidar_m
        - calibrated_pose_origin_x_from_lidar_m
    )
    if not math.isfinite(offset):
        raise ValueError("calibrated-pose reference offset must be finite")
    return offset


def lack_speed_limit_m_s(
    lack_m: float,
    min_speed_m_s: float,
    max_speed_m_s: float,
    tolerance_m: float,
    lack_at_min_speed_m: float,
) -> float:
    """Map local-path shortage to a bounded speed limit."""

    values = (
        lack_m,
        min_speed_m_s,
        max_speed_m_s,
        tolerance_m,
        lack_at_min_speed_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("lack speed control values must be finite")
    if lack_m < 0.0 or min_speed_m_s < 0.0 or tolerance_m < 0.0:
        raise ValueError("lack and minimum values must not be negative")
    if min_speed_m_s > max_speed_m_s:
        raise ValueError("min_speed_m_s must not exceed max_speed_m_s")
    if lack_at_min_speed_m <= tolerance_m:
        raise ValueError("lack_at_min_speed_m must exceed tolerance_m")

    ratio = max(0.0, min(1.0, (lack_m - tolerance_m) / (
        lack_at_min_speed_m - tolerance_m
    )))
    return max_speed_m_s - ratio * (max_speed_m_s - min_speed_m_s)


class ControlNode(Node):
    def __init__(self, **kwargs):
        super().__init__("control_node", **kwargs)

        self.target_speed = self._float_parameter("target_speed_m_s", 0.55)
        if self.target_speed < 0.0:
            raise ValueError("target_speed_m_s must not be negative")
        self.lack_min_speed_m_s = self._float_parameter(
            "lack_speed_control.min_speed_m_s", 1.0
        )
        self.lack_max_speed_m_s = self._float_parameter(
            "lack_speed_control.max_speed_m_s", 2.4
        )
        self.lack_tolerance_m = self._float_parameter(
            "lack_speed_control.tolerance_m", 0.1
        )
        self.lack_at_min_speed_m = self._float_parameter(
            "lack_speed_control.lack_at_min_speed_m", 1.0
        )
        lack_speed_limit_m_s(
            0.0,
            self.lack_min_speed_m_s,
            self.lack_max_speed_m_s,
            self.lack_tolerance_m,
            self.lack_at_min_speed_m,
        )
        self.lack_m = self.lack_at_min_speed_m
        self.camera_pan_command = self._float_parameter(
            "camera_pan_command_rad", 0.0
        )
        self.pose_timeout_sec = self._float_parameter(
            "pose_timeout_sec", 0.5
        )
        self.path_timeout_sec = self._float_parameter(
            "path_timeout_sec", 0.5
        )
        self.watchdog_period_sec = self._float_parameter(
            "watchdog_period_sec", 0.1
        )
        self.maximum_speed_variance_m2_s2 = self._float_parameter(
            "maximum_speed_variance_m2_s2", 1.0
        )
        if (
            self.pose_timeout_sec <= 0.0
            or self.path_timeout_sec <= 0.0
            or self.watchdog_period_sec <= 0.0
            or self.maximum_speed_variance_m2_s2 <= 0.0
        ):
            raise ValueError("control timeouts and speed variance must be positive")
        self.calibrated_pose_origin_x_from_lidar_m = self._float_parameter(
            "calibrated_pose.origin_x_from_lidar_m",
            DEFAULT_CALIBRATED_POSE_ORIGIN_X_FROM_LIDAR_M,
        )
        self.reference_point_x_from_lidar_m = self._float_parameter(
            "pure_pursuit.reference_point_x_from_lidar_m",
            DEFAULT_CALIBRATED_POSE_ORIGIN_X_FROM_LIDAR_M,
        )
        pure_pursuit_config = PurePursuitConfig(
            wheelbase_m=self._float_parameter(
                "pure_pursuit.wheelbase_m", 0.18
            ),
            lookahead_distance_m=self._float_parameter(
                "pure_pursuit.lookahead_distance_m", 0.45
            ),
            max_steering_rad=self._float_parameter(
                "pure_pursuit.max_steering_rad", 0.3491
            ),
            closed_loop=self._bool_parameter(
                "pure_pursuit.closed_loop", False
            ),
            reference_point_offset_m=reference_offset_from_calibrated_pose(
                self.reference_point_x_from_lidar_m,
                self.calibrated_pose_origin_x_from_lidar_m,
            ),
        )
        pid_config = PIDConfig(
            kp=self._float_parameter("longitudinal_pid.kp", 0.0),
            ki=self._float_parameter("longitudinal_pid.ki", 0.0),
            kd=self._float_parameter("longitudinal_pid.kd", 0.0),
            output_min=self._float_parameter(
                "longitudinal_pid.output_min", 0.0
            ),
            output_max=self._float_parameter(
                "longitudinal_pid.output_max", 0.0
            ),
            integral_min=self._float_parameter(
                "longitudinal_pid.integral_min", 0.0
            ),
            integral_max=self._float_parameter(
                "longitudinal_pid.integral_max", 0.0
            ),
        )
        adaptive_config = AdaptiveControlConfig(
            enabled=self._bool_parameter("adaptive_control.enabled", False),
            preview_distance_m=self._float_parameter(
                "adaptive_control.preview_distance_m", 1.0
            ),
            min_lookahead_m=self._float_parameter(
                "adaptive_control.min_lookahead_m", 0.25
            ),
            max_lookahead_m=self._float_parameter(
                "adaptive_control.max_lookahead_m", 1.50
            ),
            curvature_reference_inv_m=self._float_parameter(
                "adaptive_control.curvature_reference_inv_m", 2.0
            ),
            max_lateral_acceleration_m_s2=self._float_parameter(
                "adaptive_control.max_lateral_acceleration_m_s2", 0.8
            ),
            min_speed_limit_m_s=self._float_parameter(
                "adaptive_control.min_speed_limit_m_s", 0.30
            ),
            max_speed_limit_m_s=self._float_parameter(
                "adaptive_control.max_speed_limit_m_s", 0.80
            ),
        )
        speed_lookahead_config = SpeedLookaheadConfig(
            enabled=self._bool_parameter("speed_lookahead.enabled", True),
            lookahead_time_sec=self._float_parameter(
                "speed_lookahead.lookahead_time_sec", 0.55
            ),
            min_lookahead_m=self._float_parameter(
                "speed_lookahead.min_lookahead_m", 0.45
            ),
            max_lookahead_m=self._float_parameter(
                "speed_lookahead.max_lookahead_m", 1.50
            ),
        )
        controller_config = ControllerConfig(
            longitudinal_pid_enabled=self._bool_parameter(
                "longitudinal_pid.enabled", False
            ),
            max_speed_m_s=self._float_parameter("max_speed_m_s", 3.0),
            stop_speed_threshold_m_s=self._float_parameter(
                "stop_speed_threshold_m_s", 1.0e-6
            ),
            speed_lookahead=speed_lookahead_config,
            adaptive_control=adaptive_config,
        )
        self.controller = ControllerCore(
            PurePursuit(pure_pursuit_config),
            PIDController(pid_config),
            controller_config,
        )
        self.vehicle_state = None
        self.gosign = False
        self.last_update = time.monotonic()
        self.last_pose_time = None
        self.last_path_time = None

        self.speed_pub = self.create_publisher(Float64, "/speed", 10)
        self.steering_pub = self.create_publisher(Float64, "/steering", 10)
        self.camera_pan_pub = self.create_publisher(Float64, "/camera/pan", 10)
        self.path_sub = self.create_subscription(Path, "/path", self.on_path, 10)
        self.lack_sub = self.create_subscription(
            Float64, "/lack", self.on_lack, 10
        )
        self.gosign_sub = self.create_subscription(
            Bool, "/gosign", self.on_gosign, 10
        )
        self.pose_sub = self.create_subscription(
            Odometry, "/pose/calibration", self.on_pose, qos_profile_sensor_data
        )
        self.watchdog_timer = self.create_timer(
            self.watchdog_period_sec, self.on_watchdog
        )

    def _float_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _bool_parameter(self, name: str, default: bool) -> bool:
        return bool(self.declare_parameter(name, default).value)

    def on_pose(self, pose: Odometry) -> None:
        position = pose.pose.pose.position
        orientation = pose.pose.pose.orientation
        speed = pose.twist.twist.linear.x
        speed_variance = pose.twist.covariance[0]
        quaternion_norm = math.hypot(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if (
            pose.header.frame_id != "map"
            or not math.isfinite(position.x)
            or not math.isfinite(position.y)
            or not math.isfinite(speed)
            or not math.isfinite(speed_variance)
            or speed_variance < 0.0
            or speed_variance > self.maximum_speed_variance_m2_s2
            or not math.isfinite(quaternion_norm)
            or quaternion_norm < 1.0e-9
        ):
            self._stop_control(clear_vehicle_state=True)
            return

        qx = orientation.x / quaternion_norm
        qy = orientation.y / quaternion_norm
        qz = orientation.z / quaternion_norm
        qw = orientation.w / quaternion_norm
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy ** 2 + qz ** 2),
        )
        self.vehicle_state = VehicleState(
            position.x, position.y, yaw, speed
        )
        self.last_pose_time = time.monotonic()

    def on_lack(self, lack: Float64) -> None:
        if math.isfinite(lack.data) and lack.data >= 0.0:
            self.lack_m = lack.data
        else:
            self.lack_m = self.lack_at_min_speed_m

    def on_path(self, path: Path) -> None:
        now = time.monotonic()
        path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
        ]
        valid_path = (
            path.header.frame_id == "map"
            and len(path_points) >= 2
            and all(
                math.isfinite(x) and math.isfinite(y)
                for x, y in path_points
            )
        )
        if valid_path:
            self.last_path_time = now
        else:
            self.last_path_time = None

        pose_stale = (
            self.last_pose_time is None
            or now - self.last_pose_time > self.pose_timeout_sec
        )
        if (
            not self.gosign
            or self.vehicle_state is None
            or pose_stale
            or not valid_path
        ):
            self._stop_control(clear_vehicle_state=pose_stale)
            return

        result = self.controller.update(
            self.vehicle_state,
            path_points,
            self.target_speed,
            max(now - self.last_update, 1.0e-4),
        )
        self.last_update = now
        if not math.isfinite(result.speed_command_m_s + result.steering_rad):
            self._stop_control()
            return
        lack_speed_limit = lack_speed_limit_m_s(
            self.lack_m,
            self.lack_min_speed_m_s,
            self.lack_max_speed_m_s,
            self.lack_tolerance_m,
            self.lack_at_min_speed_m,
        )
        self.publish_commands(
            min(result.speed_command_m_s, lack_speed_limit),
            result.steering_rad,
        )

    def on_watchdog(self) -> None:
        now = time.monotonic()
        pose_stale = (
            self.last_pose_time is None
            or now - self.last_pose_time > self.pose_timeout_sec
        )
        path_stale = (
            self.last_path_time is None
            or now - self.last_path_time > self.path_timeout_sec
        )
        if not self.gosign or pose_stale or path_stale:
            self._stop_control(clear_vehicle_state=pose_stale)

    def on_gosign(self, gosign: Bool) -> None:
        if not self.gosign and gosign.data:
            self.gosign = True

    def publish_commands(self, speed: float, steering: float) -> None:
        self.speed_pub.publish(Float64(data=float(speed)))
        self.steering_pub.publish(Float64(data=float(steering)))
        self.camera_pan_pub.publish(
            Float64(data=float(self.camera_pan_command))
        )

    def _stop_control(self, clear_vehicle_state: bool = False) -> None:
        if clear_vehicle_state:
            self.vehicle_state = None
            self.last_pose_time = None
        self.controller.pid.reset()
        self.last_update = time.monotonic()
        self.publish_stop()

    def publish_stop(self) -> None:
        self.publish_commands(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
