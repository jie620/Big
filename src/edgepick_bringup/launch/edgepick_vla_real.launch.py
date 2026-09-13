"""Run the local SmolVLA policy against real RGB-D/ros2_control inputs."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory("edgepick_bringup"))
    camera_share = Path(get_package_share_directory("orbbec_camera"))
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(camera_share / "launch" / "dabai_dcw2.launch.py"))
    )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "edgepick_real_control.launch.py")),
        launch_arguments={"use_real_i2c": "true"}.items(),
    )
    vla = Node(
        package="edgepick_vla", executable="smolvla_node", output="screen",
        # SmolVLA dependencies (draccus/LeRobot) live in the Jetson Python
        # 3.10 environment; the system ROS Python does not contain them.
        prefix="/home/jetson/Codex_Projects/Edge_AI/.venv-smolvla-jetson/bin/python",
        parameters=[{
            "task": LaunchConfiguration("task"),
            "checkpoint": LaunchConfiguration("checkpoint"),
            "image_topic": LaunchConfiguration("image_topic"),
            "joints_topic": LaunchConfiguration("joints_topic"),
            "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "execute_trajectory": ParameterValue(LaunchConfiguration("execute_trajectory"), value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("task", default_value="抓取橘子"),
        DeclareLaunchArgument("checkpoint", default_value="/home/jetson/Codex_Projects/Edge_AI/smolvla_v3_360000/best_pretrained"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("joints_topic", default_value="/joint_states"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/camera/depth/points"),
        DeclareLaunchArgument("rate_hz", default_value="5.0"),
        DeclareLaunchArgument("execute_trajectory", default_value="true"),
        camera, control, vla,
    ])
