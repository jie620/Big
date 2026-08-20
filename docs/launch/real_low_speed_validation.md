# Real Low-Speed Validation

## 目标

阶段 15 只验证真实 DOFBOT 的最小硬件闭环：I2C、controller-manager、控制器状态和单关节低速小角度动作。

本流程不启动感知、不启动 task node、不执行抓取任务。

## 1. 构建和 mock 回归

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash

colcon build --base-paths src --packages-select edgepick_hardware edgepick_bringup
colcon test --base-paths src --packages-select edgepick_hardware edgepick_bringup
colcon test-result --test-result-base build --all --verbose
```

## 2. I2C 预检

先做不扫描的打开/选址检查：

```bash
python3 scripts/check_dofbot_i2c.py --device /dev/i2c-7 --address 0x15
```

如需确认地址是否可见，再显式扫描总线：

```bash
python3 scripts/check_dofbot_i2c.py --device /dev/i2c-7 --address 0x15 --scan
```

脚本会优先执行厂商 `Arm_Lib` 读探针：

- `Arm_get_hardversion()` 应返回类似 `0.20`
- `Arm_ping_servo(1)` 应返回 `218`，也就是 `0xda`

预期结果：

- `OK opened /dev/i2c-7 and selected address 0x15`
- `OK Arm_Lib bus=7 version=0.20 ping1=218`
- `i2cdetect` 如果没看到 `0x15` 也不作为失败依据

如果失败，先处理 `/dev/i2c-7` 是否存在、当前用户权限、机械臂供电、SDA/SCL/GND 接线和是否有其他进程占用 I2C。`/sys/class/i2c-adapter/i2c-7/bus_clk_rate` 在某些系统上可能不存在，不作为失败依据。

## 3. launch 参数验证

该命令只解析参数，不启动真实控制链：

```bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_real_control.launch.py --show-args
```

确认默认值：

```text
use_real_i2c: true
i2c_device: /dev/i2c-7
i2c_address: 0x15
```

## 4. 启动真实控制链

确认机械臂供电稳定、工作空间清空、手边可以立刻停止进程后再执行：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_real_control.launch.py
```

注意：该 launch 会在控制器激活后立即写入目标姿态，机械臂可能立刻抬起，不是静默预检。

另开一个终端观察：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source /home/jetson/Codex_Projects/Big/install/setup.bash

ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

预期：

- `joint_state_broadcaster` 为 `active`
- `arm_group_controller` 为 `active`
- `grip_group_controller` 为 `active`
- `/joint_states` 包含 `Arm1_Joint` 到 `grip_joint`

## 5. 单关节低速小角度动作

只验证 `Arm1_Joint`，其他关节保持 0 rad。先发送很小目标：

```bash
ros2 action send_goal /arm_group_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Arm1_Joint, Arm2_Joint, Arm3_Joint, Arm4_Joint, Arm5_Joint],
    points: [
      {positions: [0.05, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}
    ]
  }
}"
```

观察方向、速度、噪声和供电。成功后回到 0 rad：

```bash
ros2 action send_goal /arm_group_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Arm1_Joint, Arm2_Joint, Arm3_Joint, Arm4_Joint, Arm5_Joint],
    points: [
      {positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}
    ]
  }
}"
```

## 6. 回滚和停止

停止 launch：

```bash
pkill -f edgepick_real_control.launch.py
pkill -f ros2_control_node
```

如果动作方向错误、抖动明显、供电异常或控制器报错，停止后记录日志，不继续验证其他关节。

## 已完成记录

用户真实终端已完成以下动作：

- `Arm_Lib` 读探针确认 `version=0.20`、`ping1=218`
- `edgepick_real_control.launch.py` 启动后机械臂进入控制器激活姿态
- `Arm1_Joint` 以 `0.05 rad`、`3 s` 小步前进
- `Arm1_Joint` 回到 `0 rad`
- 两次 `follow_joint_trajectory` 目标均返回 `SUCCEEDED`

## 记录模板

```text
date:
operator:
i2c_device:
i2c_address:
i2c_precheck:
controller_state:
joint_tested:
target_delta_rad:
duration_sec:
observed_direction:
returned_to_zero:
issues:
logs:
```
