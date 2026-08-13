# 自研 ROS 2 包

后续按职责创建独立包，第一批预计包括：

- `edgepick_hardware`：C++ I2C 传输抽象、mock 后端和 ros2_control `SystemInterface`。
- `edgepick_bringup`：生命周期管理、参数和启动编排。
- `edgepick_task`：抓取任务状态机、超时和恢复策略。

任何真实 I2C 写入必须经 `edgepick_hardware`，不得在感知或任务节点中直接调用 `Arm_Lib`。
