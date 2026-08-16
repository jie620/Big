# ADR 0003: 新增 edgepick_bringup 作为启动覆盖层

## 决策

阶段 3 新增 `edgepick_bringup` 包，集中维护 EdgePick 的 xacro、controller yaml 和 launch 文件。该包复用厂商 DOFBOT URDF 与 MoveIt 配置，但用 `edgepick_hardware/MockSystemInterface` 替换 vendor `mock_components/GenericSystem`。

## 背景

阶段 2 已经实现 EdgePick 自己的 mock `ros2_control` 系统接口。如果继续直接启动厂商 `dofbot_pro_moveit/demo.launch.py`，它会重新加载厂商 xacro，硬件插件仍然是 `mock_components/GenericSystem`，不会经过 EdgePick 的 `CommandGateway`。

## 后果

`edgepick_bringup` 成为启动层边界：

- `edgepick_hardware` 继续只负责命令安全门和硬件接口；
- `edgepick_bringup` 负责将 robot description、controller manager、MoveIt 和 RViz 连接起来；
- vendor `dofbot_pro_description` 和 `dofbot_pro_moveit` 保持只读运行依赖；
- 真实 I2C 仍不启用，阶段 3 只验证 mock 链路。

## 结构反思

这个拆分避免把 launch、MoveIt 参数和底层硬件安全策略塞进同一个包。后续真实 I2C 接入时，应继续沿用同一 bringup 结构，只新增显式启用的 real hardware 配置，而不是修改 vendor 文件。

## 当前阶段

阶段 3：`edgepick_bringup` 已提供 mock control launch、MoveIt mock launch、EdgePick xacro 覆盖层和 controller 配置测试。

## 下一步

阶段 4：运行 mock launch 并记录 controller manager、MoveIt action、`/joint_states` 与 RViz 的链路状态；随后开始 `edgepick_task` 状态机设计。
