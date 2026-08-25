# Orange Perception Validation

## 目标

阶段 20 验证真实相机下的橘子 perception 链路。重点是检测、目标点投影、目标点基座变换和抓取位姿构造，不包含任务闭环和 MoveIt 执行。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_perception_validation.launch.py
```

如果需要看画面和检测框：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_perception_validation.launch.py show_viewer:=true
```

如果现场没有已有 TF，可以临时打开静态 camera transform：

```text
publish_camera_static_tf:=true
camera_tf_x:=0.0
camera_tf_y:=0.0
camera_tf_z:=0.0
camera_tf_roll:=0.0
camera_tf_pitch:=0.0
camera_tf_yaw:=0.0
```

## 预期链路

```text
edgepick_coco_detector_node.py
  -> /edgepick/perception/detections
detected_target_candidate_node
  -> /edgepick/perception/target_point
target_frame_transform_node
  -> /edgepick/perception/target_point_base
grasp_target_builder_node
  -> /edgepick/task/pregrasp_pose
  -> /edgepick/task/grasp_pose
perception_metrics_node
  -> /edgepick/perception/metrics
```

## 验证

```bash
ros2 topic echo /edgepick/perception/detections --once
ros2 topic echo /edgepick/perception/target_point --once
ros2 topic echo /edgepick/perception/target_point_base --once
ros2 topic echo /edgepick/task/pregrasp_pose --once
ros2 topic echo /edgepick/task/grasp_pose --once
ros2 topic echo /edgepick/perception/metrics --once
```

## 边界

- 不启动 task node。
- 不启动 mock driver 或 MoveIt adapter。
- 不发送真实抓取 goal。
- 只验证 perception 输出是否稳定进入 base frame 和抓取位姿边界。

## 备注

如果 `target_point_base` 没有输出，先确认：

1. 相机 `color/depth` topic 是否正常。
2. `base_link <- camera_color_optical_frame` 的 TF 是否存在。
3. 橘子 detector 是否真的发布了 `orange`。
