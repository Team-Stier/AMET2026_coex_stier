from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rddf_visualizer"), "rviz", "rddf_visualizer.rviz"]
    )
    return LaunchDescription(
        [
            Node(
                package="rddf_visualizer",
                executable="rddf_visualizer_node",
                parameters=[{"enable_sim_gt": True}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )

