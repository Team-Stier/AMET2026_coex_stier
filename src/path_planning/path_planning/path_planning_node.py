import rclpy
from interfaces.msg import Objects
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        self.declare_parameter("rddf_file", "")
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odometry, 10
        )
        self.calibrated_odom_sub = self.create_subscription(
            Odometry, "/odom/calibride", self.on_calibrated_odometry, 10
        )
        self.object_info_sub = self.create_subscription(
            Objects, "/object_info", self.on_object_info, 10
        )

    def on_odometry(self, odometry: Odometry) -> None:
        self.get_logger().debug(f"received odometry stamp {odometry.header.stamp}")

    def on_calibrated_odometry(self, odometry: Odometry) -> None:
        self.get_logger().debug(
            f"received calibrated odometry stamp {odometry.header.stamp}"
        )

    def on_object_info(self, objects: Objects) -> None:
        self.get_logger().debug(f"received {objects.length} fitted objects")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
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
