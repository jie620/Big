# Mock Task Closed Loop

## 当前阶段

阶段 6：EdgePick 已具备自动 mock 任务闭环。

`mock_task_driver_node` 会根据 `/edgepick/task/state` 发布下一步 `/edgepick/task/event`。它不是固定睡眠后乱发事件，而是等待状态机进入脚本期望状态后再推进。

## 场景

| 场景 | 作用 |
| --- | --- |
| `success` | 从 `idle` 自动推进到 `succeeded` |
| `perception_recovery` | 首次感知丢失，恢复后重新感知并成功 |
| `planning_recovery` | 首次规划失败，恢复后重新感知/规划并成功 |
| `execution_recovery` | 首次执行失败，恢复后重新感知/规划/执行并成功 |
| `verification_recovery` | 首次验证失败，恢复后重新感知/规划/执行/验证并成功 |

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_task_closed_loop.launch.py scenario:=success
```

切换恢复场景：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_task_closed_loop.launch.py scenario:=planning_recovery
```

## 观察

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source /home/jetson/Codex_Projects/Big/install/setup.bash
ros2 topic echo /edgepick/task/state
ros2 topic echo /edgepick/task/failure
ros2 topic echo /diagnostics
```

期望结果：

- `success` 最终进入 `succeeded`，失败原因为 `none`。
- 恢复场景会短暂进入 `recovering`，然后重新进入 `perceiving` 并最终 `succeeded`。
- `/diagnostics` 会记录 `reason`、`state`、`failure`、`recovery_attempts` 和 `terminal`。

当前验证记录：2026-08-16 短时启动 `scenario:=success` 时，mock 驱动已自动发布完整成功路径并让 task node 进入 `succeeded`。受限沙箱会出现 DDS UDP socket 权限警告，真实桌面终端仍需补充 topic echo 或 rosbag 证据。

## 结构反思

mock 闭环只是自动化任务流，不是真实感知、真实 MoveIt action 或真实硬件控制。它的价值是先把状态、失败、恢复和 diagnostics 的工程骨架跑通。后续 Stage 7 可以把规划/执行步骤替换成 MoveIt action 适配器，Stage 8/9 再把感知步骤替换成 RGB-D 和 TensorRT 结果。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
