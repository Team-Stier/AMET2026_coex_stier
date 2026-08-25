from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("pose_tf")
    return LaunchDescription(
        [
            Node(
                package="pose_tf",
                executable="pose_tf_node",
                name="pose_tf_node",
                parameters=[
                    PathJoinSubstitution(
                        [package_share, "config", "pose_tf.yaml"]
                    )
                ],
                output="screen",
            )
        ]
    )
