import rclpy
from interfaces.msg import Objects
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")
        self.object_info_pub = self.create_publisher(Objects, "/object_info", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

    def on_scan(self, scan: LaserScan) -> None:
        self.get_logger().debug(
            f"received LaserScan with {len(scan.ranges)} ranges"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
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
