# Real MoveIt Validation

## 目标

阶段 17 在阶段 16 的 real control + MoveGroup 基础上，再加一个最小关节验证节点。

验证流程：

1. 启动 real control 和 MoveGroup。
2. 等待 MoveIt state monitor 可用。
3. 捕获当前关节值，作为 home。
4. 对单个关节做小幅度前进。
5. 回到捕获的 home。
6. 比较回零误差。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_moveit_real_validation.launch.py
```

默认参数：

```text
use_real_i2c: true
i2c_device: /dev/i2c-7
i2c_address: 0x15
use_rviz: false
validation_start_delay_sec: 8.0
move_group_name: arm_group
test_joint_index: 0
test_joint_delta_rad: 0.05
planning_time_sec: 5.0
planning_attempts: 10
validation_attempts: 3
state_monitor_wait_sec: 2.0
settle_time_ms: 500
home_tolerance_rad: 0.03
velocity_scaling_factor: 0.1
acceleration_scaling_factor: 0.1
```

## 预期

- 节点先输出捕获到的 home 关节值。
- 节点规划并执行一次小角度前进。
- 节点规划并执行回 home。
- 回零误差应小于 `home_tolerance_rad`。
- 运行结束后 launch 自动退出。

## 安全边界

- 该入口仍不启动 task node、perception、mock 驱动或抓取目标构造。
- 仍只建议在机械臂周围清空后执行。
- 若动作方向不符预期，先停止，不要继续提高幅度。

## 验证记录模板

```text
date:
operator:
target_joint:
delta_rad:
planning_attempts:
execution_attempts:
home_error_rad:
result:
logs:
```
