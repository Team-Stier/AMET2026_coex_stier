from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rddf_visualizer"), "rviz", "rddf_visualizer.rviz"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_yaw_offset_deg", default_value="-90.0"),
            DeclareLaunchArgument("laser_odom_yaw_offset_deg", default_value="-90.0"),
            Node(
                package="rddf_visualizer",
                executable="rddf_visualizer_node",
                parameters=[
                    {
                        "enable_sim_gt": True,
                        "odom_yaw_offset_deg": LaunchConfiguration(
                            "odom_yaw_offset_deg"
                        ),
                        "laser_odom_yaw_offset_deg": LaunchConfiguration(
                            "laser_odom_yaw_offset_deg"
                        ),
                    }
                ],
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

