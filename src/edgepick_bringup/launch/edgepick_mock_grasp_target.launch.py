"""Build mock-safe grasp and pregrasp targets from a base-frame perception point."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    grasp_target_builder = Node(
        package="edgepick_task",
        executable="grasp_target_builder_node",
        name="edgepick_grasp_target_builder",
        output="screen",
        parameters=[
            {
                "target_point_topic": "/edgepick/perception/target_point_base",
                "pregrasp_pose_topic": "/edgepick/task/pregrasp_pose",
                "grasp_pose_topic": "/edgepick/task/grasp_pose",
                "expected_frame": LaunchConfiguration("target_frame"),
                "pregrasp_offset_m": LaunchConfiguration("pregrasp_offset_m"),
                "grasp_z_offset_m": LaunchConfiguration("grasp_z_offset_m"),
                "end_effector_orientation_xyzw": [0.0, 1.0, 0.0, 0.0],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("pregrasp_offset_m", default_value="0.08"),
            DeclareLaunchArgument("grasp_z_offset_m", default_value="0.02"),
            grasp_target_builder,
        ]
    )
