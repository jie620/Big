# ADR 0006: 用状态门控 mock 驱动形成任务闭环

## 决策

阶段 6 新增 `MockTaskScript` 和 `mock_task_driver_node`。mock 驱动节点订阅 `/edgepick/task/state`，只有当状态机进入脚本期望状态时，才向 `/edgepick/task/event` 发布下一步事件。

支持的场景包括：

- `success`
- `perception_recovery`
- `planning_recovery`
- `execution_recovery`
- `verification_recovery`

这些场景模拟 mock 感知、mock 规划、mock 执行、mock 验证和 mock 恢复模块的输出。

## 背景

阶段 5 已经能用手动 `ros2 topic pub` 驱动状态机，但手动发事件不能证明系统具备自动任务流，也不能稳定复现失败恢复路径。

阶段 6 的目标是形成第一条可自动跑完的 mock 抓取闭环，同时继续避免真实硬件风险：不访问 `/dev/i2c-7`，不启动真实机械臂控制，不把 MoveIt action 和视觉节点强塞进状态机。

## 后果

- 成功路径和一次恢复路径可以通过单元测试验证。
- ROS launch 可以一键启动 task node 和 mock 驱动节点。
- 后续接 MoveIt action 时，只需要替换规划/执行事件来源，不需要改变 `task_node` 的状态接口。
- 当前 mock 驱动仍是脚本化场景，不代表真实感知或真实规划已接入。

## 结构反思

阶段 6 的关键是“mock 外部世界”和“任务核心”分开：`MockTaskScript` 是可单测的场景脚本，`mock_task_driver_node` 是 ROS topic 适配器，`task_node` 仍只管理状态机和 diagnostics。这个分层让项目开始具备端到端流程感，但没有提前背上真实硬件和视觉调试复杂度。

## 当前阶段

阶段 6：mock 任务闭环适配层已实现。

## 下一步

阶段 7：新增 MoveIt action 接入层，把 mock 规划/执行事件替换为可测试的 MoveIt action 结果适配。
