# 工程文档

本目录后续记录架构决策、坐标系/手眼标定、接口契约、性能基线和故障注入实验。实验结论应包含环境、命令、原始数据位置与可复现步骤。

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 1：硬件控制边界

当前阶段：新增 ADR 0001，记录 `edgepick_hardware` 先使用 mock 传输、不连接真实 I2C 的决策。

完成内容：说明 `CommandGateway` 的职责、厂商 `Arm_Lib` 的边界，以及真实 I2C 适配器为什么必须延后。

结构反思：阶段 1 文档先把安全边界写清楚是必要的，否则后续每一层都可能绕过硬件安全门。

当时下一步：记录 mock `ros2_control` 系统接口的职责和边界。

### 阶段 2：mock ros2_control 系统接口

当前阶段：新增 ADR 0002，记录 EdgePick mock `ros2_control` 系统接口的职责、边界和后续风险。

完成内容：把“硬件安全边界”和“mock ros2_control 接口”拆成两个 ADR，便于以后回溯为什么先做 mock、为什么不直接接真实 I2C。

结构反思：文档结构现在能按阶段追踪设计选择，阶段 2 没有覆盖阶段 1，而是在 ADR 0001 之后追加 ADR 0002。

### 阶段 3：EdgePick mock bringup

当前阶段：新增 ADR 0003 和 `docs/launch/edgepick_bringup_mock_chain.md`，记录启动覆盖层和 mock 控制链路。

完成内容：说明为什么不能直接 include vendor demo launch，以及 EdgePick xacro、controller manager、MoveIt、RViz 与 MockSystemInterface 的连接关系。

结构反思：文档现在按“硬件边界 -> mock 系统接口 -> bringup 覆盖层”递进，能回溯每一层为什么存在。

验证记录：已记录 mock control 与 MoveIt mock 的短时启动结果，沙箱限制下未打开 RViz GUI，真实桌面终端仍需补充 RViz 执行截图/日志。

### 阶段 4：抓取任务状态机核心

当前阶段：新增 ADR 0004、`docs/task/grasp_state_machine.md` 和 `docs/launch/rviz_mock_verification.md`。

完成内容：记录状态机职责、恢复策略、RViz mock 真实终端验证步骤和阶段 5 的 ROS 节点接入方向。

结构反思：文档现在从硬件边界、启动覆盖层推进到任务决策层，能说明 EdgePick 为什么不是一个简单固定动作 demo。

### 阶段 5：ROS 2 task node 与 diagnostics

当前阶段：新增 ADR 0005 和 `docs/task/task_node_topics.md`，记录 task node 的 ROS topic、事件词表、失败原因和 diagnostics 输出。

完成内容：把“外部模块如何驱动状态机”和“用户如何用命令行验证状态变化”写成可复现接口文档。

结构反思：阶段 5 文档把 task node 定义为事件适配边界，而不是让它直接承担感知、规划、执行或硬件控制职责，方便后续逐层替换 mock 组件。

验证记录：新增文档与实现一起通过 36 个自动化测试；沙箱中 launch 可显示参数并启动进程，但 DDS 网络 socket 受限，真实终端仍需补充 topic pub/echo 截图或日志。

## 下一步目标

阶段 6：记录 mock 任务闭环适配层设计，并开始沉淀 rosbag/diagnostics 的验证证据。
