# Task Node Topic Contract

## 当前阶段

阶段 5：`edgepick_task/task_node` 提供任务状态机的 ROS 2 topic 边界。

## Topic

| 方向 | Topic | 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/edgepick/task/event` | `std_msgs/msg/String` | 外部模块发布任务事件 |
| 输出 | `/edgepick/task/state` | `std_msgs/msg/String` | 当前任务状态 |
| 输出 | `/edgepick/task/failure` | `std_msgs/msg/String` | 最近一次失败原因 |
| 输出 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 状态机诊断快照 |

## Event Vocabulary

```text
start_requested
target_acquired
target_lost
plan_succeeded
plan_failed
execution_succeeded
execution_failed
verification_succeeded
verification_failed
recovery_succeeded
recovery_failed
timeout
cancel_requested
reset
```

输入事件会先去除首尾空白并转成小写；未知事件会被拒绝，节点发布 WARN diagnostics，但不会改变状态机状态。

## Diagnostics 字段

`/diagnostics` 中 `edgepick_task/state_machine` 状态项包含：

- `reason`：触发本次发布的事件或原因。
- `state`：状态机当前状态。
- `failure`：最近一次失败原因。
- `recovery_attempts`：当前恢复尝试次数。
- `terminal`：是否已经进入 `succeeded`、`failed` 或 `canceled`。

## 手动验证

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_task_mock.launch.py
```

另开终端发布事件：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source /home/jetson/Codex_Projects/Big/install/setup.bash
ros2 topic pub --once /edgepick/task/event std_msgs/msg/String "{data: start_requested}"
ros2 topic echo /edgepick/task/state --once
ros2 topic echo /edgepick/task/failure --once
ros2 topic echo /diagnostics --once
```

## 结构反思

task node 现在只接受事件、输出状态和诊断；它不直接执行感知、规划或硬件控制。这个边界让阶段 6 可以安全地添加 mock 任务闭环，而不会把 ROS action、相机回调和硬件写入混到同一个节点中。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
