import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.speed_pub = self.create_publisher(Float64, "/speed", 10)
        self.steering_pub = self.create_publisher(Float64, "/steering", 10)
        self.camera_pan_pub = self.create_publisher(Float64, "/camera/pan", 10)
        self.path_sub = self.create_subscription(Path, "/path", self.on_path, 10)
        self.gosign_sub = self.create_subscription(
            Bool, "/gosign", self.on_gosign, 10
        )
        self.gosign = False

    def on_path(self, path: Path) -> None:
        self.get_logger().debug(f"received path with {len(path.poses)} poses")

    def on_gosign(self, gosign: Bool) -> None:
        self.gosign = gosign.data


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
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
