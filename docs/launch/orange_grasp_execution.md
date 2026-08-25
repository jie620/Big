# Orange Grasp Execution

## 目标

阶段 21 把真实橘子 detection、TF、抓取位姿构造、任务状态机、真实 MoveIt 和真实夹爪控制接起来，执行一次完整的抓取动作。

启动时姿态序列现在固定为：

1. 先直接发送 Yahboom 归零姿态 `[90, 90, 90, 90, 90, 30]`。
2. 再直接发送抓取启动姿态 `[90, 165, 18, 0, 90, 30]`。
3. 只有两步都成功并发布 `/edgepick/startup_pose/ready` 后，才启动橘子检测和后续抓取链路。

默认恢复姿态使用舵机角度参数：

```text
restore_servo_angle1=90
restore_servo_angle2=165
restore_servo_angle3=18
restore_servo_angle4=0
restore_servo_angle5=90
restore_servo_angle6=30
```

橘子抓取执行器会等待 `/edgepick/startup_pose/ready`，只有上述序列完成后才会进入真实抓取。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_grasp_execution.launch.py
```

带真实 I2C 参数时建议直接使用单行命令，避免 shell 将参数拆成独立命令：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_grasp_execution.launch.py use_real_i2c:=true i2c_device:=/dev/i2c-7 i2c_address:=0x15 motion_time_ms:=30 show_viewer:=false
```

完整真机验证，包含相机启动：

```bash
cd /home/jetson/Codex_Projects/Big
bash scripts/run_orange_grasp_validation.sh
```

如果需要看相机画面和检测框：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_grasp_execution.launch.py show_viewer:=true
```

夹爪保守值建议：

```bash
gripper_close_position:=-0.9
```

当前代码默认的闭合值是 `-1.4939`，更接近“关到底”；做橘子验证时先用更保守的闭合位置，再按实际抓持效果微调。

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
  -> /edgepick/task/event == target_acquired
target_frame_transform_node
  -> /edgepick/perception/target_point_base
grasp_target_builder_node
  -> /edgepick/task/pregrasp_pose
  -> /edgepick/task/grasp_pose
task_node + mock_task_driver_node(start_only)
  -> /edgepick/task/state == planning/executing/verifying/succeeded
orange_grasp_executor_node
  -> 真实 MoveIt 运动 + 夹爪开合
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
ros2 topic echo /edgepick/task/failure --once
ros2 topic echo /edgepick/perception/metrics --once
```

## 边界

- 这里已经不是 rehearsal。
- 默认会启动真实 MoveIt 和真实夹爪 action。
- `start_only` 任务驱动只负责起步，不会伪造规划、执行或验证结果。
- 启动姿态序列失败时不会发布 ready 信号，抓取执行器会保持等待，不会直接进入抓取动作。
- 如果 `target_point_base` 没有输出，先确认相机 TF 和橘子 detector。

## 备注

抓取成功后，任务状态机应进入 `succeeded`。如果抓取过程中任一步失败，任务状态机应进入 `recovering`，并由失败事件反映原因。
