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
    VehicleState,
)
from .pid import PIDController
from .pure_pursuit import PurePursuit


class ControlNode(Node):
    def __init__(self, **kwargs):
        super().__init__("control_node", **kwargs)

        self.target_speed = self._float_parameter("target_speed_m_s", 0.55)
        if self.target_speed < 0.0:
            raise ValueError("target_speed_m_s must not be negative")
        self.camera_pan_command = self._float_parameter(
            "camera_pan_command_rad", 0.0
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
                "adaptive_control.max_lookahead_m", 0.40
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
        controller_config = ControllerConfig(
            longitudinal_pid_enabled=self._bool_parameter(
                "longitudinal_pid.enabled", False
            ),
            max_speed_m_s=self._float_parameter("max_speed_m_s", 3.0),
            stop_speed_threshold_m_s=self._float_parameter(
                "stop_speed_threshold_m_s", 1.0e-6
            ),
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

        self.speed_pub = self.create_publisher(Float64, "/speed", 10)
        self.steering_pub = self.create_publisher(Float64, "/steering", 10)
        self.camera_pan_pub = self.create_publisher(Float64, "/camera/pan", 10)
        self.path_sub = self.create_subscription(Path, "/path", self.on_path, 10)
        self.gosign_sub = self.create_subscription(
            Bool, "/gosign", self.on_gosign, 10
        )
        self.pose_sub = self.create_subscription(
            Odometry, "/pose", self.on_pose, qos_profile_sensor_data
        )

    def _float_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _bool_parameter(self, name: str, default: bool) -> bool:
        return bool(self.declare_parameter(name, default).value)

    def on_pose(self, pose: Odometry) -> None:
        if pose.header.frame_id != "map":
            self.vehicle_state = None
            self.publish_stop()
            return

        orientation = pose.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        position = pose.pose.pose.position
        self.vehicle_state = VehicleState(
            position.x, position.y, yaw, pose.twist.twist.linear.x
        )

    def on_path(self, path: Path) -> None:
        if (
            not self.gosign
            or self.vehicle_state is None
            or path.header.frame_id != "map"
            or len(path.poses) < 2
        ):
            self.publish_stop()
            return

        now = time.monotonic()
        result = self.controller.update(
            self.vehicle_state,
            [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses],
            self.target_speed,
            max(now - self.last_update, 1.0e-4),
        )
        self.last_update = now
        if not math.isfinite(result.speed_command_m_s + result.steering_rad):
            self.publish_stop()
            return
        self.publish_commands(result.speed_command_m_s, result.steering_rad)

    def on_gosign(self, gosign: Bool) -> None:
        if not self.gosign and gosign.data:
            self.gosign = True

    def publish_commands(self, speed: float, steering: float) -> None:
        self.speed_pub.publish(Float64(data=float(speed)))
        self.steering_pub.publish(Float64(data=float(steering)))
        self.camera_pan_pub.publish(
            Float64(data=float(self.camera_pan_command))
        )

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
