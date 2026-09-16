"""One safe route for mock, MuJoCo, and explicitly enabled real hardware."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    mode=value("mode")
    if mode not in ("mock","mujoco","real"): raise ValueError("mode must be mock, mujoco or real")
    real=mode=="real"
    truth=lambda key:value(key).lower()=="true"
    if real and not (truth("calibration_confirmed") and truth("registered_depth_confirmed")):
        raise ValueError("Real launch requires calibrated camera extrinsics and verified RGB-D registration")
    if real and not value("checkpoint"): raise ValueError("checkpoint is required for real mode")
    if real and not value("yolo_model"): raise ValueError("yolo_model is required for real mode")
    share=Path(get_package_share_directory("edgepick_safe"))
    bringup=Path(get_package_share_directory("edgepick_bringup"))
    config=(MoveItConfigsBuilder("DOFBOT_Pro-V24",package_name="dofbot_pro_moveit")
            .robot_description(file_path=str(bringup/"urdf/edgepick_dofbot.urdf.xacro"),
                mappings={"initial_positions_file":str(bringup/"config/initial_positions.yaml"),
                          "use_real_i2c":str(real).lower(),"motion_time_ms":"30"})
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"],default_planning_pipeline="ompl").to_moveit_configs())
    # Preserve vendor files; distrust collision pairs disabled only by sampling.
    srdf=ET.fromstring(config.robot_description_semantic["robot_description_semantic"])
    for pair in list(srdf.findall("disable_collisions")):
        if pair.get("reason")=="Never": srdf.remove(pair)
    config.robot_description_semantic["robot_description_semantic"]=ET.tostring(srdf,encoding="unicode")
    params=str(share/"config/safe.yaml")
    actions=[]
    critical=[]
    rsp=Node(package="robot_state_publisher",executable="robot_state_publisher",parameters=[config.robot_description],output="screen")
    actions.append(rsp)
    if mode!="mujoco":
        control=Node(package="controller_manager",executable="ros2_control_node",output="screen",parameters=[config.robot_description,str(bringup/"config/edgepick_ros2_controllers.yaml")])
        actions.append(control);critical.append(control)
        for name in ("joint_state_broadcaster","arm_group_controller","grip_group_controller"):
            actions.append(Node(package="controller_manager",executable="spawner",arguments=[name,"--controller-manager","/controller_manager"],output="screen"))
    else:
        sim=Node(package="edgepick_safe",executable="mujoco_backend",prefix=value("python"),output="screen",parameters=[{"model_xml":value("model_xml"),"render":truth("render")}])
        actions.append(sim);critical.append(sim)
    move=Node(package="moveit_ros_move_group",executable="move_group",parameters=[config.to_dict(),{"publish_robot_description_semantic":True,"publish_planning_scene":True,"publish_geometry_updates":True,"publish_state_updates":True}],output="screen")
    gate=Node(package="edgepick_safe",executable="safety_executor",parameters=[config.to_dict(),params,{"real_hardware":real,"calibration_confirmed":truth("calibration_confirmed")}],output="screen")
    actions.extend([move,gate]);critical.extend([move,gate])
    if mode=="mock" and truth("mock_inputs"):
        actions.append(Node(package="edgepick_safe",executable="mock_inputs",parameters=[{"auto_arm":truth("auto_arm"),"scenario":value("scenario")}],output="screen"))
    if value("checkpoint"):
        policy=Node(package="edgepick_safe",executable="policy_node",prefix=value("python"),parameters=[params,{"checkpoint":value("checkpoint"),"lerobot_source":value("lerobot_source"),"task":value("task"),"image_topic":value("image_topic"),"device":value("device")}],output="screen")
        actions.append(policy);critical.append(policy)
    if real or (mode=="mujoco" and value("yolo_model")):
        perception=Node(package="edgepick_safe",executable="perception_node",prefix=value("python"),parameters=[params,{"yolo_model":value("yolo_model"),"registered_depth_confirmed":truth("registered_depth_confirmed"),"image_topic":value("image_topic"),"depth_topic":value("depth_topic"),"camera_info_topic":value("camera_info_topic")}],output="screen")
        actions.append(perception);critical.append(perception)
    if real:
        # Camera driver is a separate explicit launch: names and registration vary by firmware.
        camera_tf=value("camera_transform").split(",")
        if len(camera_tf)!=6: raise ValueError("camera_transform must be measured x,y,z,roll,pitch,yaw")
        numbers=[float(x) for x in camera_tf]
        import math
        if not all(math.isfinite(x) for x in numbers): raise ValueError("Nonfinite camera transform")
        actions.append(Node(package="tf2_ros",executable="static_transform_publisher",arguments=["--x",camera_tf[0],"--y",camera_tf[1],"--z",camera_tf[2],"--roll",camera_tf[3],"--pitch",camera_tf[4],"--yaw",camera_tf[5],"--frame-id","DaBai_DCW2_Link","--child-frame-id",value("camera_frame")]))
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason="critical safe-chain process exited"))])))
    return actions


def generate_launch_description():
    root=Path(get_package_share_directory("edgepick_safe")).parents[3]
    defaults={"mode":"mock","checkpoint":"","task":"把橘子放到右边","yolo_model":str(root/"models/yolo/orange.pt"), "python":str(root/".venv-vla/bin/python"),
              "lerobot_source":str(root/"vendor/lerobot-smolvla-jetson/src"),"model_xml":str(root/"simulation/dofbot_pro/dofbot_pro.xml"),"render":"false","device":"cuda","auto_arm":"false","mock_inputs":"true","scenario":"normal",
              "calibration_confirmed":"false","registered_depth_confirmed":"false","camera_transform":"",
              "camera_frame":"camera_link","image_topic":"/camera/color/image_raw",
              "depth_topic":"/camera/aligned_depth_to_color/image_raw","camera_info_topic":"/camera/color/camera_info"}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=build)])
