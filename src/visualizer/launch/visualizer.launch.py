from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("visualizer")
    visualizer = Node(
        package="visualizer",
        executable="visualizer",
        name="visualizer",
        parameters=[PathJoinSubstitution([package_share, "config", "visualizer.yaml"])],
        output="screen",
    )
    return LaunchDescription(
        [
            visualizer,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=visualizer,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="visualizer node stopped"))
                    ],
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [package_share, "rviz", "visualizer.rviz"]
                    ),
                ],
                output="screen",
            ),
        ]
    )
