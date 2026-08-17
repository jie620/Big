"""Start the EdgePick RGB-D target candidate foundation node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgbd_target_candidate = Node(
        package="edgepick_perception",
        executable="rgbd_target_candidate_node",
        name="edgepick_rgbd_target_candidate",
        output="screen",
        parameters=[
            {
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "target_topic": "/edgepick/perception/target_point",
                "event_topic": "/edgepick/task/event",
                "target_pixel_u": LaunchConfiguration("target_pixel_u"),
                "target_pixel_v": LaunchConfiguration("target_pixel_v"),
                "min_depth_m": LaunchConfiguration("min_depth_m"),
                "max_depth_m": LaunchConfiguration("max_depth_m"),
                "publish_task_events": LaunchConfiguration("publish_task_events"),
                "publish_target_lost": LaunchConfiguration("publish_target_lost"),
                "publish_event_once": LaunchConfiguration("publish_event_once"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("target_pixel_u", default_value="-1"),
            DeclareLaunchArgument("target_pixel_v", default_value="-1"),
            DeclareLaunchArgument("min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("max_depth_m", default_value="1.20"),
            DeclareLaunchArgument("publish_task_events", default_value="true"),
            DeclareLaunchArgument("publish_target_lost", default_value="true"),
            DeclareLaunchArgument("publish_event_once", default_value="true"),
            rgbd_target_candidate,
        ]
    )
