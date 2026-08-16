# edgepick_task

EdgePick 的任务逻辑包。阶段 4 先实现纯 C++ 抓取状态机；阶段 5 增加 ROS 2 task node，用 topic 事件驱动状态机并发布 diagnostics。当前仍不调用 MoveIt、不接触真实机械臂。

## 结构

- `include/edgepick_task/grasp_state_machine.hpp`：状态、事件、失败码、配置和状态机接口。
- `include/edgepick_task/task_event_io.hpp`：事件字符串解析、状态快照和 diagnostics 文本格式化。
- `src/grasp_state_machine.cpp`：状态转移、恢复次数、取消、重置和字符串化实现。
- `src/task_event_io.cpp`：统一维护 ROS topic、CLI 示例和测试共用的事件词表。
- `src/task_node.cpp`：ROS 2 节点，订阅任务事件并发布状态、失败原因和 diagnostics。
- `test/grasp_state_machine_test.cpp`：成功路径、失败恢复、恢复预算耗尽、超时、取消和非法转移测试。
- `test/task_event_io_test.cpp`：事件词表、字符串解析和状态快照格式测试。

## 状态路径

```text
idle
  -> perceiving
  -> planning
  -> executing
  -> verifying
  -> succeeded
```

失败或超时路径：

```text
perceiving/planning/executing/verifying
  -> recovering
  -> perceiving
```

恢复次数耗尽、恢复失败或恢复超时后进入 `failed`；主动取消进入 `canceled`。

## ROS 2 topic 接口

- 输入事件：`/edgepick/task/event`，类型 `std_msgs/msg/String`。
- 当前状态：`/edgepick/task/state`，类型 `std_msgs/msg/String`。
- 最后失败原因：`/edgepick/task/failure`，类型 `std_msgs/msg/String`。
- 诊断输出：`/diagnostics`，类型 `diagnostic_msgs/msg/DiagnosticArray`。

事件字符串：

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

最小手动验证：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ros2 launch edgepick_bringup edgepick_task_mock.launch.py
ros2 topic pub --once /edgepick/task/event std_msgs/msg/String "{data: start_requested}"
ros2 topic echo /edgepick/task/state --once
ros2 topic echo /edgepick/task/failure --once
ros2 topic echo /diagnostics --once
```

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 4：抓取任务状态机核心

当前阶段：新增 `edgepick_task`，先用无 ROS 依赖的 C++ 状态机定义抓取任务生命周期。

完成内容：实现状态转移、失败码、恢复预算、超时处理、取消、重置和日志稳定字符串。

结构反思：任务状态机先做成纯逻辑库是正确的；这样可以在不启动 MoveIt、相机或真实机械臂的情况下测试恢复策略，后续 ROS 节点只负责把外部 action/topic 结果翻译成状态机事件。

### 阶段 5：ROS 2 task node 与 diagnostics

当前阶段：新增 `task_node`，把纯状态机包装成 ROS 2 可观测节点。

完成内容：节点订阅 `/edgepick/task/event`，接受稳定事件字符串，发布 `/edgepick/task/state`、`/edgepick/task/failure` 和 `/diagnostics`。未知事件只产生 WARN diagnostics，不进入状态机。

结构反思：topic 事件层是正确的过渡层。它让后续 mock 感知、MoveIt action 适配器、验证器都能用同一套状态词表接入，不需要互相知道内部实现。

验证记录：`grasp_state_machine_test` 和 `task_event_io_test` 通过，`ros2 pkg executables edgepick_task` 可列出 `edgepick_task task_node`。

## 下一步目标

阶段 6：新增 mock 任务闭环适配层，让 mock 感知、规划、执行和验证节点自动发布事件，形成完整可回放的抓取任务流程。
