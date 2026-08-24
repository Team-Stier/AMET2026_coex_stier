from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("calibration"))
    return LaunchDescription(
        [
            Node(
                package="calibration",
                executable="calibration_node",
                name="calibration_node",
                parameters=[str(share / "config" / "fence_localization.yaml")],
                output="screen",
            ),
            Node(
                package="calibration",
                executable="fence_rviz_bridge",
                name="calibration_fence_rviz_bridge",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="calibration_rviz",
                arguments=["-d", str(share / "rviz" / "sim_localization.rviz")],
                output="screen",
            ),
        ]
    )
