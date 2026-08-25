from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from ultralytics import YOLO
from ultralytics.utils import YAML


STOP_CLASS_IDS = {0, 1}
GREEN_CLASS_ID = 2
EXPECTED_NAMES = {0: "red", 1: "yellow", 2: "green"}


def read_model_image_size(model_path: Path) -> int:
    metadata_path = model_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"NCNN metadata not found: {metadata_path}")

    image_size = YAML.load(metadata_path).get("imgsz")
    if (
        not isinstance(image_size, (list, tuple))
        or len(image_size) != 2
        or image_size[0] != image_size[1]
    ):
        raise ValueError(
            f"expected a square NCNN image size in {metadata_path}; "
            f"got {image_size}"
        )
    return int(image_size[0])


def resolve_model_image_size(
    model_path: Path, requested_image_size: int
) -> int:
    model_image_size = read_model_image_size(model_path)
    if requested_image_size not in (0, model_image_size):
        raise ValueError(
            f"image_size={requested_image_size} does not match the fixed "
            f"NCNN model size {model_image_size}: {model_path}"
        )
    return model_image_size


def make_debug_image(frame, source_image: CompressedImage):
    encoded_ok, encoded_frame = cv2.imencode(".jpg", frame)
    if not encoded_ok:
        return None

    debug_image = CompressedImage()
    debug_image.header = source_image.header
    debug_image.format = "jpeg"
    debug_image.data = encoded_frame.tobytes()
    return debug_image


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
            / "traffic_light_320_ncnn_model"
        )
        model_path = Path(
            self.declare_parameter("model_path", str(default_model_path)).value
        )
        self.confidence = float(
            self.declare_parameter("confidence", 0.75).value
        )
        requested_image_size = int(
            self.declare_parameter("image_size", 0).value
        )
        required_green_frames = int(
            self.declare_parameter("green_confirm_frames", 3).value
        )
        self.image_timeout_seconds = float(
            self.declare_parameter("image_timeout_seconds", 1.0).value
        )
        self.visualizer_enabled = bool(
            self.declare_parameter("visualizer_enabled", False).value
        )

        if not model_path.is_dir():
            raise FileNotFoundError(f"NCNN model directory not found: {model_path}")
        if not 0.0 < self.confidence <= 1.0:
            raise ValueError("confidence must be in the range (0, 1]")
        if self.image_timeout_seconds <= 0.0:
            raise ValueError("image_timeout_seconds must be positive")

        self.image_size = resolve_model_image_size(
            model_path, requested_image_size
        )

        self.model = YOLO(model_path, task="detect")
        if self.model.names != EXPECTED_NAMES:
            raise ValueError(
                f"unexpected model classes: {self.model.names}; "
                f"expected {EXPECTED_NAMES}"
            )

        self.decision = GoSignDecision(required_green_frames)
        self.last_image_time = self.get_clock().now()
        self.last_gosign = False
        self.completed = False
        self.gosign_pub = self.create_publisher(Bool, "/gosign", 10)
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.visualizer_pub = (
            self.create_publisher(
                CompressedImage,
                "/traffic_light/debug/compressed",
                image_qos,
            )
            if self.visualizer_enabled
            else None
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.on_image,
            image_qos,
        )
        self.watchdog = self.create_timer(0.1, self.on_watchdog)
        self.get_logger().info(
            f"loaded traffic-light model: {model_path} "
            f"({self.image_size}x{self.image_size})"
        )
        if self.visualizer_enabled:
            self.get_logger().info(
                "publishing visualizations on /traffic_light/debug/compressed"
            )

    def publish_gosign(self, allowed: bool) -> None:
        if self.completed or (
            allowed and self.gosign_pub.get_subscription_count() == 0
        ):
            return

        if allowed:
            self.completed = True
        message = Bool()
        message.data = allowed
        self.gosign_pub.publish(message)
        self.last_gosign = allowed
        if allowed:
            if not self.gosign_pub.wait_for_all_acked(Duration(seconds=1.0)):
                self.get_logger().warning("/gosign acknowledgment timed out")
            self.context.try_shutdown()

    def publish_stop(self) -> None:
        self.decision.reset()
        self.publish_gosign(False)

    def on_image(self, image: CompressedImage) -> None:
        if self.completed:
            return

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

        if self.visualizer_pub is not None:
            try:
                debug_image = make_debug_image(result.plot(), image)
                if debug_image is None:
                    self.get_logger().error("failed to encode visualization")
                else:
                    self.visualizer_pub.publish(debug_image)
            except Exception as error:
                self.get_logger().error(
                    f"traffic-light visualization failed: {error}"
                )

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
