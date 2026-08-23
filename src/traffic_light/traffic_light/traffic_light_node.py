from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from ultralytics import YOLO


STOP_CLASS_IDS = {0, 1}
GREEN_CLASS_ID = 2
EXPECTED_NAMES = {0: "red", 1: "yellow", 2: "green"}


class GoSignDecision:
    def __init__(self, required_green_frames: int):
        if required_green_frames < 1:
            raise ValueError("required_green_frames must be at least 1")
        self.required_green_frames = required_green_frames
        self.green_streak = 0

    def reset(self) -> None:
        self.green_streak = 0

    def update(self, detected_class_ids) -> bool:
        class_ids = set(detected_class_ids)
        if class_ids & STOP_CLASS_IDS or GREEN_CLASS_ID not in class_ids:
            self.reset()
            return False

        self.green_streak += 1
        return self.green_streak >= self.required_green_frames


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__("traffic_light_node")

        default_model_path = (
            Path(get_package_share_directory("traffic_light"))
            / "models"
            / "traffic_light_yolo26n-3"
            / "deploy"
            / "best_ncnn_model"
        )
        model_path = Path(
            self.declare_parameter("model_path", str(default_model_path)).value
        )
        self.confidence = float(
            self.declare_parameter("confidence", 0.5).value
        )
        self.image_size = int(self.declare_parameter("image_size", 640).value)
        required_green_frames = int(
            self.declare_parameter("green_confirm_frames", 3).value
        )
        self.image_timeout_seconds = float(
            self.declare_parameter("image_timeout_seconds", 1.0).value
        )

        if not model_path.is_dir():
            raise FileNotFoundError(f"NCNN model directory not found: {model_path}")
        if not 0.0 < self.confidence <= 1.0:
            raise ValueError("confidence must be in the range (0, 1]")
        if self.image_size < 1:
            raise ValueError("image_size must be positive")
        if self.image_timeout_seconds <= 0.0:
            raise ValueError("image_timeout_seconds must be positive")

        self.model = YOLO(model_path, task="detect")
        if self.model.names != EXPECTED_NAMES:
            raise ValueError(
                f"unexpected model classes: {self.model.names}; "
                f"expected {EXPECTED_NAMES}"
            )

        self.decision = GoSignDecision(required_green_frames)
        self.last_image_time = self.get_clock().now()
        self.last_gosign = False
        self.gosign_pub = self.create_publisher(Bool, "/gosign", 10)
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.on_image,
            image_qos,
        )
        self.watchdog = self.create_timer(0.1, self.on_watchdog)
        self.get_logger().info(f"loaded traffic-light model: {model_path}")

    def publish_gosign(self, allowed: bool) -> None:
        message = Bool()
        message.data = allowed
        self.gosign_pub.publish(message)
        self.last_gosign = allowed

    def publish_stop(self) -> None:
        self.decision.reset()
        self.publish_gosign(False)

    def on_image(self, image: CompressedImage) -> None:
        self.last_image_time = self.get_clock().now()
        frame = cv2.imdecode(
            np.frombuffer(image.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if frame is None:
            self.get_logger().error("failed to decode compressed camera image")
            self.publish_stop()
            return

        try:
            result = self.model.predict(
                frame,
                imgsz=self.image_size,
                conf=self.confidence,
                verbose=False,
            )[0]
        except Exception as error:
            self.get_logger().error(f"traffic-light inference failed: {error}")
            self.publish_stop()
            return

        class_ids = (
            [int(class_id) for class_id in result.boxes.cls.tolist()]
            if result.boxes is not None
            else []
        )
        self.publish_gosign(self.decision.update(class_ids))

    def on_watchdog(self) -> None:
        age_seconds = (
            self.get_clock().now() - self.last_image_time
        ).nanoseconds / 1_000_000_000
        if age_seconds > self.image_timeout_seconds and self.last_gosign:
            self.get_logger().warning("camera image timeout; publishing stop")
            self.publish_stop()


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
