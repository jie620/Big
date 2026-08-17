# MoveIt Action Adapter

## 当前阶段

阶段 7：EdgePick 已具备 mock-safe 的 MoveIt action 适配层。

`moveit_action_adapter_node` 的职责是把规划/执行 action result 转换为任务事件：

| 状态机状态 | 适配阶段 | 成功事件 | 失败事件 |
| --- | --- | --- | --- |
| `planning` | MoveGroup | `plan_succeeded` | `plan_failed` |
| `executing` | ExecuteTrajectory | `execution_succeeded` | `execution_failed` |

`timeout` outcome 会统一转换为 `timeout` 事件，由状态机按当前状态处理恢复。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_moveit_action_mock.launch.py
```

默认参数：

```text
use_mock_action_results:=true
planning_outcome:=success
execution_outcome:=success
```

失败路径示例：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_moveit_action_mock.launch.py planning_outcome:=failure
```

## 边界

- 已完成：typed action client、server 可用性检查、mock action outcome、任务事件映射、launch 和单元测试。
- 未完成：真实 MoveIt goal 构造、目标位姿输入、轨迹执行策略、碰撞目标更新。
- 安全边界：默认不发送真实 MoveIt goal，不启动真实硬件，不访问 `/dev/i2c-7`。

## 验证记录

2026-08-16 短时启动 `edgepick_moveit_action_mock.launch.py` 时，事件流如下：

```text
mock driver: start_requested
mock driver: target_acquired
moveit adapter: plan_succeeded
moveit adapter: execution_succeeded
mock driver: verification_succeeded
task node: succeeded
```

沙箱仍有 DDS UDP socket 权限警告，真实桌面终端需要补充 topic echo 或 rosbag 证据。

## 结构反思

这一步不是“已经能让 MoveIt 控制机械臂抓取”，而是先建立任务层和 MoveIt action 层之间的转换接口。先把接口做薄、做可测，后面接真实目标位姿和轨迹执行时更容易定位问题。

## 下一步

阶段 9：新增目标检测/TensorRT 推理层，为后续真实 MoveIt goal 提供更可靠的目标像素和候选点。
