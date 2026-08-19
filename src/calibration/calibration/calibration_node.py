import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")
        self.calibrated_odom_pub = self.create_publisher(
            Odometry, "/odom/calibride", 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.on_image,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odometry, 10
        )
        self.camera_pan_sub = self.create_subscription(
            Float64, "/camera/pan", self.on_camera_pan, 10
        )
        self.latest_camera_pan = None

    def on_image(self, image: CompressedImage) -> None:
        self.get_logger().debug(f"received image stamp {image.header.stamp}")

    def on_odometry(self, odometry: Odometry) -> None:
        self.get_logger().debug(f"received odometry stamp {odometry.header.stamp}")

    def on_camera_pan(self, camera_pan: Float64) -> None:
        self.latest_camera_pan = camera_pan.data


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
