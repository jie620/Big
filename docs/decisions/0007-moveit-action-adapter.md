# ADR 0007: 拆出 MoveIt action 结果适配层

## 决策

阶段 7 新增 `moveit_action_adapter_node`。该节点订阅 `/edgepick/task/state`，当状态进入 `planning` 时产生规划结果事件，当状态进入 `executing` 时产生执行结果事件。

默认模式为 `use_mock_action_results:=true`，只模拟 MoveGroup 和 ExecuteTrajectory 的结果，不构造真实 MoveIt goal，不发送真实轨迹，也不访问 `/dev/i2c-7`。

## 背景

阶段 6 的 mock 闭环已经可以自动发出 `plan_succeeded` 和 `execution_succeeded`，但这些事件仍来自同一个 mock 脚本。为了后续接真实 MoveIt，需要先把“任务状态机”和“规划/执行 action 结果来源”拆开。

## 后果

- `task_node` 不需要知道 MoveIt action 类型，只继续接收稳定 `TaskEvent`。
- `moveit_action_adapter_node` 已创建 `moveit_msgs/action/MoveGroup` 和 `moveit_msgs/action/ExecuteTrajectory` 的 typed action client。
- 当前真实 action goal 构造仍未实现；若关闭 mock outcome，只做 action server 可用性检查，并把不可用映射为失败事件。
- `mock_task_driver_node` 可以使用 `moveit_success` 场景，只模拟操作者、感知和验证，把规划/执行留给 action adapter。

## 结构反思

阶段 7 的边界是正确的：MoveIt action 适配器是“结果翻译层”，不是感知节点、不是状态机、也不是硬件控制节点。这样后续新增目标位姿、规划请求和真实执行时，可以局部修改 action adapter，不污染任务状态机。

## 当前阶段

阶段 7：MoveIt action 适配层已实现。

## 下一步

阶段 8：新增 RGB-D 感知基础层，把目标候选点从相机/深度数据中提取出来，并与任务事件边界对接。
