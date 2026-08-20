# Real Control Launch

## 目标

阶段 14 提供真实 DOFBOT I2C 控制入口。该入口只负责启动 robot_state_publisher、controller-manager 和控制器 spawner，并把 I2C 参数显式写入 ros2_control 的硬件描述。

## 启动前检查

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
```

真机验证前确认：

- 机械臂供电稳定，工作空间内没有遮挡。
- 当前用户可访问 `/dev/i2c-7`。
- 系统中没有同时运行厂商 Arm_Lib 脚本或其他 I2C 写入进程。
- 已先通过 `edgepick_prehardware_mock_rehearsal.launch.py` 做 mock 回归。

## 参数解析验证

该命令只解析 launch 参数，不启动 controller-manager，不打开 `/dev/i2c-7`：

```bash
ros2 launch edgepick_bringup edgepick_real_control.launch.py --show-args
```

预期参数：

```text
publish_frequency
use_real_i2c
i2c_device
i2c_address
```

## 启动真实控制链

默认 real control launch 已将 `use_real_i2c` 设为 `true`：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_real_control.launch.py
```

如需覆盖 I2C 设备或地址：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_real_control.launch.py i2c_device:=/dev/i2c-7 i2c_address:=0x15
```

## 安全边界

- 默认 mock launch 不访问真实 I2C。
- real control launch 是唯一真机控制入口。
- `use_real_i2c:=true` 才会构造 `DofbotI2cTransport` 并打开 I2C 设备。
- real/mock 两条路径都经过同一个 `CommandGateway`，继续执行关节角度、运动时间、限频和重复命令校验。
- I2C 初始化失败时 controller-manager 应失败退出，不能自动退回 mock。

## 阶段 15 低速验证建议

阶段 15 才执行真实运动。建议先只验证一个关节、小角度、低速，并同时观察：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

验证完成后停止 launch，确认没有其他进程继续写 I2C。
