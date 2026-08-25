# Orange Task Rehearsal

## 目标

阶段 19 把 COCO 橘子检测接入任务/抓取联调。目标不是新增检测模型，而是验证橘子检测结果能稳定推进到任务状态机、目标点转换、抓取目标构造和 MoveIt 结果适配。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_task_rehearsal.launch.py
```

如果需要看相机画面和检测框：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_task_rehearsal.launch.py show_viewer:=true
```

如果现场没有现成 TF，可临时启用静态相机位姿：

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
  -> /edgepick/task/event == target_acquired
target_frame_transform_node
  -> /edgepick/perception/target_point_base
grasp_target_builder_node
  -> /edgepick/task/pregrasp_pose
  -> /edgepick/task/grasp_pose
task_node + mock_task_driver_node + moveit_action_adapter_node
  -> /edgepick/task/state == succeeded
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
ros2 topic echo /edgepick/task/state --once
ros2 topic echo /edgepick/perception/metrics --once
```

## 安全边界

- 该入口仍然只做联调，不直接发真实抓取 goal。
- 默认仍使用 mock-safe 的 MoveIt 结果适配，不依赖真实 action server。
- 真机 TF 若已经由系统或相机驱动发布，就不要同时打开 `publish_camera_static_tf`。

## 备注

阶段 19 继续沿用阶段 9 的检测契约和阶段 12 的抓取目标构造，只把橘子检测结果接入更上层的任务链路。
