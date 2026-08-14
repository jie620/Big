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

## 下一步目标

阶段 3：在 `docs/` 中补充 bringup 链路说明，记录 xacro、controller manager、joint trajectory controller 和 MoveIt 执行路径的对应关系。
