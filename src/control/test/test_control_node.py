from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import pytest
import rclpy
from std_msgs.msg import Bool

from control.control_node import ControlNode


class Recorder:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


def test_control_node_latches_first_true_gosign():
    rclpy.init()
    node = ControlNode()
    node.speed_pub = Recorder()
    node.steering_pub = Recorder()
    node.camera_pan_pub = Recorder()
    try:
        pose = Odometry()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        node.on_pose(pose)

        path = Path()
        path.header.frame_id = "map"
        path.poses = [PoseStamped(), PoseStamped()]
        path.poses[1].pose.position.x = 1.0

        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.0)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)

        node.on_gosign(Bool(data=True))
        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.55)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
        assert node.camera_pan_pub.values[-1] == pytest.approx(0.0)

        published_speed_count = len(node.speed_pub.values)
        node.on_gosign(Bool(data=False))
        assert len(node.speed_pub.values) == published_speed_count

        node.on_path(path)
        assert node.speed_pub.values[-1] == pytest.approx(0.55)
        assert node.steering_pub.values[-1] == pytest.approx(0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
