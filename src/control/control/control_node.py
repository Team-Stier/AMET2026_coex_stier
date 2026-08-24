import math
import time

from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64

from .controller_core import ControllerCore
from .models import ControllerConfig, PIDConfig, PurePursuitConfig, VehicleState
from .pid import PIDController
from .pure_pursuit import PurePursuit


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.target_speed = float(
            self.declare_parameter("target_speed_m_s", 0.55).value
        )
        self.controller = ControllerCore(
            PurePursuit(PurePursuitConfig(0.18, 0.45, 0.3491)),
            PIDController(PIDConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            ControllerConfig(False, self.target_speed),
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
        self.camera_pan_pub.publish(Float64(data=0.0))

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
