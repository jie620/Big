# edgepick_hardware

EdgePick 的硬件边界包。它目前只提供可测试的 mock 链路，不连接真实 `/dev/i2c-7`，也不会移动机械臂。

## 结构

- `include/edgepick_hardware/command.hpp`：六关节命令与舵机角度限制。
- `include/edgepick_hardware/command_gateway.hpp`：命令校验、限频、重复抑制和统计。
- `include/edgepick_hardware/transport.hpp`：底层写入抽象。
- `include/edgepick_hardware/mock_transport.hpp`：内存传输和失败注入。
- `include/edgepick_hardware/mock_system_interface.hpp`：`ros2_control` mock `SystemInterface`。
- `src/`：上述接口实现。
- `test/`：不依赖相机、MoveIt 或真实机械臂的单元测试。

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 1：命令网关与 mock 传输

当前阶段：完成 `CommandGateway`、`CommandTransport` 和 `MockTransport`。

完成内容：支持六关节命令模型、关节角度校验、运动时间校验、非有限浮点拒绝、重复命令抑制、20 ms 限频、传输失败注入和统计。

结构反思：阶段 1 把命令安全策略和底层传输拆开，便于后续用同一个网关服务 mock、真实 I2C 和故障注入测试。

当时下一步：把命令网关封装成 mock `ros2_control` 系统接口。

### 阶段 2：mock ros2_control 系统接口

当前阶段：新增 `edgepick_hardware/MockSystemInterface` 插件。它导出 6 个 position command interface，并按 URDF 中的 position/velocity state interface 暴露状态；写入时把 MoveIt/ros2_control 使用的弧度位置转换为 DOFBOT 舵机角度，再交给 `CommandGateway` 校验和 mock 传输记录。

完成内容：接入 `hardware_interface`、`pluginlib` 和 `rclcpp`，新增插件描述和 mock 系统接口测试，确认该包可以作为 controller manager 的硬件插件入口。

结构反思：当前结构保持了三层边界：控制器只看 `hardware_interface::SystemInterface`，系统接口负责 ROS 弧度与舵机角度转换，`CommandGateway` 继续作为唯一命令安全门。这样后续真实 I2C 适配器可以替换 `CommandTransport`，不需要让 MoveIt、任务状态机或视觉节点直接知道 `Arm_Lib`。

补充记录：已为 `command_gateway` 和 `mock_system_interface` 增加模块级注释，说明命令校验顺序、限频/重复抑制意图、mock `read/write` 行为、ros2_control handle 存储生命周期和弧度到舵机角度映射风险。

阶段 3 修正：bringup 冒烟测试发现 pluginlib 需要加载动态库，`edgepick_hardware` 已改为 `SHARED` 库，并新增 `plugin_loading_test.cpp`，确保 `edgepick_hardware/MockSystemInterface` 能通过 pluginlib 实例化。

## 下一步目标

阶段 3：新增 `edgepick_bringup`，生成使用 `edgepick_hardware/MockSystemInterface` 的 xacro/launch/config，让 MoveIt 和 controller manager 在 RViz 中通过 EdgePick 自己的 mock 硬件接口跑通。
