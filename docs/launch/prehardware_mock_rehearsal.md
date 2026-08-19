# Pre-Hardware Mock Rehearsal

## 目标

阶段 13 提供真实硬件接入前的系统级 mock rehearsal。它用于确认 topic、TF、目标构造、metrics 和任务事件流可以在同一进程组内跑通。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_prehardware_mock_rehearsal.launch.py
```

## 预期链路

```text
mock_rgbd_source_node
  -> /camera/depth/image_raw
  -> /camera/depth/camera_info
mock_detector_node
  -> /edgepick/perception/detections
detected_target_candidate_node
  -> /edgepick/perception/target_point
target_frame_transform_node
  -> /edgepick/perception/target_point_base
grasp_target_builder_node
  -> /edgepick/task/pregrasp_pose
  -> /edgepick/task/grasp_pose
task_node + moveit_action_adapter_node + mock_task_driver_node
  -> /edgepick/task/state == succeeded
perception_metrics_node
  -> /edgepick/perception/metrics
```

## 验证命令

```bash
ros2 topic echo /edgepick/perception/target_point --once
ros2 topic echo /edgepick/perception/target_point_base --once
ros2 topic echo /edgepick/task/pregrasp_pose --once
ros2 topic echo /edgepick/task/grasp_pose --once
ros2 topic echo /edgepick/task/state --once
ros2 topic echo /edgepick/perception/metrics --once
```

## Readiness Checklist

- `colcon build` 五包通过。
- `colcon test` 五包通过。
- `colcon test-result --test-result-base build --all --verbose` 无 errors/failures/skips。
- `edgepick_prehardware_mock_rehearsal.launch.py --show-args` 通过。
- 短时启动能看到 `task_node` 进入 `succeeded`。
- `/edgepick/perception/metrics` 显示 detection、target point 和 task events 均有计数。
- `/edgepick/task/pregrasp_pose` 与 `/edgepick/task/grasp_pose` frame 为 `base_link` 或配置的目标 frame。
- 默认路径不访问 `/dev/i2c-7`，不加载真实硬件后端，不发送真实 MoveIt goal。

## 边界

该 rehearsal 是真实硬件前最后的 mock 系统检查，不等同于真机执行。进入阶段 14 时，真实 I2C 后端必须通过独立参数或独立 launch 显式启用，并继续保留当前 mock rehearsal 作为回归测试入口。
