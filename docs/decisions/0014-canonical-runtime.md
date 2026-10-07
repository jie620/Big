# ADR 0014：统一运行入口

旧的 real control、MoveIt 验证和直接舵机命令路径已经合并为 `edgepick_bringup/launch/edgepick_system.launch.py`。入口通过 `mode` 选择 Mock、MuJoCo 或真实 I2C 后端，共享 MoveIt、Safety Gate 和 VLA 动作契约。

真机模式必须显式提供 checkpoint、YOLO 模型、RGB-D 注册确认、相机外参和 I2C 参数；默认不会 arm。低层硬件仍由 `edgepick_hardware` 管理，任务包不再安装直接电机控制 executable。
