# edgepick_task

该包保存可单测的任务状态机和 mock rehearsal 适配器。它不是 VLA 的真机执行器，也不直接访问 I2C；真机动作只能从 `edgepick_bringup/edgepick_system.launch.py mode:=real` 进入 `edgepick_safe/safety_executor`。

## 组成

- `grasp_state_machine.*`：感知、规划、执行、复核和恢复状态转移。
- `task_node.cpp`：把状态机暴露为 ROS 2 状态、失败和 diagnostics topic。
- `mock_task_driver_node.cpp`：按状态驱动 mock 场景。
- `moveit_action_adapter_node.cpp`：仅用于 mock action 结果适配，不构造真机目标。
- `grasp_target_builder.*`：把 mock/诊断目标点转换成预抓取和抓取 pose。

## Mock 任务链路

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/local_setup.bash
source install/local_setup.bash
ros2 launch edgepick_bringup edgepick_task_closed_loop.launch.py scenario:=success
```

这些 launch 只用于状态机和 topic 契约回归。完整的系统 mock 使用：

```bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 \
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=mock auto_arm:=true
```

状态机 topic 为 `/edgepick/task/event`、`/edgepick/task/state`、`/edgepick/task/failure` 和 `/diagnostics`。VLA/Safety Gate 主路线使用 `/edgepick/vla/proposal`、`/edgepick/safe/*`，两者的边界保持独立。
