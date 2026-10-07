# 自研 ROS 2 包

```text
edgepick_interfaces  消息类型和动作契约
edgepick_hardware    Mock ros2_control 与真实 I2C 边界
edgepick_perception  RGB-D、检测框、投影和感知指标
edgepick_task        状态机与 mock rehearsal 适配器
edgepick_safe        输入契约、静态场景、安全仲裁和策略节点
edgepick_bringup     唯一系统入口、URDF 和 controller 配置
```

职责边界：

- 只有 `edgepick_hardware` 可以接触 ros2_control 硬件插件和 I2C。
- 只有 `edgepick_safe/safety_executor` 可以接受 VLA 动作建议并调用 MoveIt。
- `edgepick_task` 不再安装直接 I2C、姿态恢复或旧的真机验证 executable。
- `edgepick_bringup/launch/edgepick_system.launch.py` 是唯一总编排入口；其他 launch 仅用于 mock/诊断。

从仓库根目录按 [README](../README.md) 构建和运行。
