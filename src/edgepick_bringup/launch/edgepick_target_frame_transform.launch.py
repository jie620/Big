"""Transform perception target points into a robot planning frame before real hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic", default_value="/edgepick/perception/target_point"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/edgepick/perception/target_point_base"
            ),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("transform_timeout_ms", default_value="100"),
            Node(
                package="edgepick_perception",
                executable="target_frame_transform_node",
                name="edgepick_target_frame_transform",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "target_frame": LaunchConfiguration("target_frame"),
                        "transform_timeout_ms": LaunchConfiguration("transform_timeout_ms"),
                    }
                ],
            ),
        ]
    )
