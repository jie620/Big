# ADR 0002: 第二阶段先实现 EdgePick mock ros2_control 系统接口

## 决策

`edgepick_hardware` 新增 `edgepick_hardware/MockSystemInterface`，作为 `hardware_interface::SystemInterface` 插件导出。该插件只做 mock 控制链验证，不打开 I2C、不调用 `Arm_Lib`、不移动真实机械臂。

接口行为：

- 对外导出 6 个 position command interface；
- 按 ros2_control 配置导出 position 和 velocity state interface；
- 将 MoveIt/ros2_control 的弧度关节命令线性转换为 DOFBOT 舵机角度；
- 继续复用 `CommandGateway` 做角度范围、非有限值、限频、重复抑制和传输失败统计；
- `read()` 为 mock 状态读取，`write()` 在命令被接受后让状态跟随命令。

## 背景

厂商 MoveIt 配置当前使用 `mock_components/GenericSystem`，能移动 RViz 模型，但没有经过 EdgePick 的命令安全门。第二阶段的目标不是控制真机，而是把 MoveIt/控制器链路先接到我们自己的硬件边界上。

## 结构反思

这个阶段的正确边界是：`SystemInterface` 只适配 ros2_control 生命周期和接口格式，安全策略仍在 `CommandGateway`，实际写入仍在 `CommandTransport`。因此 mock、未来真实 I2C、故障注入和统计可以沿同一条路径增长。

风险是 DOFBOT 的真实舵机角度映射可能还需要实机标定。当前采用 vendor URDF 的关节范围做线性映射，足够支撑 mock 控制链和单元测试；真实 I2C 阶段必须重新用低速动作验证每个关节的方向、零点和限位。

## 当前阶段

阶段 2：自研 mock `ros2_control` 系统接口已进入 `edgepick_hardware`，用于后续 MoveIt/RViz mock 控制链。

## 下一步

阶段 3：创建 `edgepick_bringup`，提供 xacro、controllers yaml 和 launch，把 vendor MoveIt 配置中的 `mock_components/GenericSystem` 替换为 `edgepick_hardware/MockSystemInterface` 进行 RViz 链路验证。
