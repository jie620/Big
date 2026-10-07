# edgepick_bringup

该包负责 URDF/xacro、controller 配置和系统编排。真实运行只使用一个入口：

```bash
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=mock
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=mujoco checkpoint:=... yolo_model:=... registered_depth_confirmed:=true
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=real checkpoint:=... yolo_model:=... calibration_confirmed:=true registered_depth_confirmed:=true camera_transform:=...
```

`edgepick_system.launch.py` 共享 robot description、MoveIt、Safety Gate 和动作契约，只替换底层控制边界：

- `mock`：`edgepick_hardware/MockSystemInterface`，可选 `edgepick_safe/mock_inputs`。
- `mujoco`：`edgepick_safe/mujoco_backend`，同时启动真实 SmolVLA 和 YOLO/RGB-D 路径。
- `real`：显式启用 I2C，要求 checkpoint、YOLO 模型、实测相机外参和 RGB-D 注册确认。

MuJoCo/real 模式会在 launch 构建阶段检查模型文件、注册深度确认和真机标定确认。真机模式还会透传 `/dev/i2c-7`、`0x15` 和运动时间参数。总入口不会自动 arm。

目录中的其它 launch 文件只服务于 mock 单包测试、感知诊断或 RViz 调试，不能作为第二条真机运行路径。修改启动参数后先运行：

```bash
python3 -m py_compile launch/edgepick_system.launch.py
```

再从仓库根目录构建并运行 `scripts/run_mock_checks.sh`。
