# Real MoveIt Control

## 目标

阶段 16 把 MoveIt 直接接到真实 controller-manager 上。这个入口只负责 real control + MoveGroup，不启动 task node、perception 或 mock 驱动。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_moveit_real.launch.py
```

默认参数：

```text
use_real_i2c: true
i2c_device: /dev/i2c-7
i2c_address: 0x15
use_rviz: false
```

## 预期

- real control 启动后机械臂会进入控制器激活姿态。
- `move_group` 连接同名 `arm_group_controller` 和 `grip_group_controller`.
- 默认不打开 RViz，适合先做 headless 验证。

## 安全边界

- 该 launch 不包含 task node、mock detector、metrics 或 prehardware rehearsal。
- 先确认机械臂周围没有人和障碍，再启动。
- 若要看图形界面，可显式追加 `use_rviz:=true`。

## 下一步

阶段 17 见 `docs/launch/real_moveit_validation.md`，它会在 real control + MoveGroup 之上做最小关节目标执行和回零确认。
