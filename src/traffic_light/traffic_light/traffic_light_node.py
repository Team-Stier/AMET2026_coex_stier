import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__("traffic_light_node")
        self.gosign_pub = self.create_publisher(Bool, "/gosign", 10)
        self.image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.on_image,
            qos_profile_sensor_data,
        )

    def on_image(self, image: CompressedImage) -> None:
        self.get_logger().debug(
            f"received compressed image with {len(image.data)} bytes"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
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
