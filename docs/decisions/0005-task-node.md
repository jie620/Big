# ADR 0005: 用 ROS 2 task node 暴露状态机事件边界

## 决策

阶段 5 将 `GraspStateMachine` 包装为 `edgepick_task/task_node`。节点订阅 `/edgepick/task/event`，发布 `/edgepick/task/state`、`/edgepick/task/failure` 和 `/diagnostics`。

该节点当前只负责事件适配、状态发布和诊断发布，不直接调用 MoveIt action、相机节点或真实 I2C。外部模块需要把感知、规划、执行、验证和恢复结果翻译成稳定的 `TaskEvent` 字符串。

## 背景

EdgePick 后续会同时接入 RGB-D 感知、MoveIt 规划、轨迹执行、抓取验证和故障恢复。如果 task node 现在就直接依赖这些具体模块，状态逻辑会很快变成难以测试的“大节点”。

因此阶段 5 先固定 ROS 边界：上游只发布事件，下游只观察状态、失败原因和 diagnostics。这样可以先用命令行和 mock 节点验证任务流，再逐步替换成真实感知和规划结果。

## 后果

- 可以用 `ros2 topic pub` 手动驱动状态机，不需要相机、RViz 或真实机械臂。
- diagnostics 中包含状态、失败原因、恢复次数、终止状态和触发原因，便于 rosbag 回放和故障复盘。
- 未知事件会被拒绝，并只发布 WARN diagnostics；不会以猜测方式进入状态机。
- 下一阶段需要新增 mock 任务闭环适配层，把自动感知/规划/执行/验证结果转换成同一组事件。

## 结构反思

阶段 5 的结构是正确的：`edgepick_task` 保留任务决策，`edgepick_bringup` 提供启动入口，`edgepick_hardware` 继续守住硬件边界。ROS topic 是状态机与外部模块之间的契约，不是绕过分层的快捷通道。

## 当前阶段

阶段 5：ROS 2 task node 与 diagnostics 已实现。

## 下一步

阶段 6：新增 mock 任务闭环适配层，让 mock 感知、规划、执行和验证节点自动发布任务事件，并记录 rosbag/diagnostics 验证证据。
