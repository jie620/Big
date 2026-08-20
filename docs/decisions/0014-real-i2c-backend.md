# ADR 0014: Explicit Real I2C Backend

## 决策

阶段 14 在 `edgepick_hardware` 内新增 `DofbotI2cTransport`，并在 `edgepick_bringup` 内新增 `edgepick_real_control.launch.py` 作为真实 I2C 的显式入口。

默认 mock 控制链不变。只有 ros2_control 硬件参数 `use_real_i2c` 被设置为 `true` 时，`MockSystemInterface` 才会创建真实 I2C 传输适配器并尝试打开配置的 I2C 设备。

## 背景

前 13 个阶段已经完成真实硬件前 mock rehearsal。下一步需要让 controller-manager 的命令可以到达 DOFBOT 主控板，但不能让普通构建、测试、mock launch 或 RViz 验证意外触碰 `/dev/i2c-7`。

厂商 `Arm_Lib` 的 DOFBOT 控制路径通过 I2C 写入运动时间和六舵机位置帧。阶段 14 只复刻这条低层传输契约，不改变任务状态机、感知链路或 MoveIt mock 链路。

## 影响

- `DofbotI2cTransport` 将命令编码为两次 I2C block write：先写运动时间到 `0x1e`，再写六舵机帧到 `0x1d`。
- 默认 I2C address 为 `0x15`，默认设备为 `/dev/i2c-7`，二者都通过 launch/xacro 参数显式传入。
- `MockSystemInterface` 继续保留 mock 传输为默认路径，并让 real/mock 两条路径共用 `CommandGateway` 的校验、限频和重复命令抑制。
- `edgepick_real_control.launch.py` 是真机控制入口；`edgepick_mock_control.launch.py` 和 prehardware rehearsal 仍保持 mock-safe。
- 如果 I2C 设备无法打开、地址解析失败或真实 bus 构造失败，硬件组件初始化返回错误，不静默降级为 mock。

## 状态

阶段 14：显式 real I2C 后端已实现。

验证记录：2026-08-20 `edgepick_hardware` 与 `edgepick_bringup` 构建通过；新增 `dofbot_i2c_transport_test` 覆盖厂商帧编码、显式启用、禁用拒绝和写失败路径；`mock_system_interface_test` 覆盖 real I2C 显式失败路径；`bringup_config_test` 覆盖 `edgepick_real_control.launch.py` 参数透传；`edgepick_real_control.launch.py --show-args` 通过。

## 下一步

阶段 15：在真实 DOFBOT 上执行低速单关节验证，记录供电、权限、I2C 探测、控制器状态和回滚命令。
