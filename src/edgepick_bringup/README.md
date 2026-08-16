# edgepick_bringup

EdgePick 的启动编排包。阶段 3 只启动 mock 控制链，不访问真实 `/dev/i2c-7`，也不会移动机械臂。

## 结构

- `urdf/edgepick_dofbot.urdf.xacro`：复用厂商 DOFBOT URDF 几何，并接入 EdgePick ros2_control 配置。
- `urdf/edgepick_dofbot.ros2_control.xacro`：把硬件插件替换为 `edgepick_hardware/MockSystemInterface`。
- `config/edgepick_ros2_controllers.yaml`：controller manager、轨迹控制器、夹爪控制器和 joint state broadcaster 配置。
- `config/initial_positions.yaml`：mock 控制链的初始关节位置。
- `launch/edgepick_mock_control.launch.py`：只启动 robot_state_publisher、controller manager 和控制器 spawner。
- `launch/edgepick_moveit_mock.launch.py`：启动 MoveIt、RViz 和 EdgePick mock ros2_control 链路。
- `launch/edgepick_task_mock.launch.py`：启动 `edgepick_task/task_node`，用于手动或 mock 节点发布任务事件。
- `test/test_bringup_config.py`：验证 xacro、controller yaml 和 MoveIt launch 关键连接点。

## 使用

先 source ROS 2、厂商 workspace 和本仓库 install：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
```

只验证 controller manager 到 EdgePick mock 硬件接口：

```bash
ros2 launch edgepick_bringup edgepick_mock_control.launch.py
```

验证 MoveIt/RViz 到 EdgePick mock 硬件接口：

```bash
ros2 launch edgepick_bringup edgepick_moveit_mock.launch.py
```

只验证 task node 的事件入口和 diagnostics 输出：

```bash
ros2 launch edgepick_bringup edgepick_task_mock.launch.py
```

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 3：EdgePick mock bringup

当前阶段：新增 `edgepick_bringup`，把 xacro、controller yaml 和 launch 编排集中起来，使 MoveIt/RViz 可以通过 `edgepick_hardware/MockSystemInterface` 运行。

完成内容：提供 mock control launch、MoveIt mock launch、EdgePick xacro 覆盖层、controller 配置和配置测试。

结构反思：bringup 独立成包是正确的；`edgepick_hardware` 保持硬件边界，`edgepick_bringup` 只负责启动编排，vendor MoveIt/URDF 仍作为外部运行资源复用。

验证记录：mock control launch 已确认 `EdgePickMockSystem` 和三个控制器能加载、配置并激活；MoveIt mock launch 在 `use_rviz:=false` 下确认 `move_group` 能加载 robot model 并监听 `joint_states`。沙箱中的 DDS UDP socket 权限警告不代表真实终端失败。

### 阶段 5：task node mock launch

当前阶段：`edgepick_bringup` 新增 `edgepick_task_mock.launch.py`，作为任务状态机 ROS 节点的最小启动入口。

完成内容：launch 传入恢复次数参数和 topic 名称，并由配置测试确认 `edgepick_task/task_node` 与 `/edgepick/task/*`、`/diagnostics` 的接口契约。

结构反思：bringup 继续只做启动编排，不承载状态机逻辑，也不接触真实硬件；这能保持“逻辑包”和“启动包”的职责分离。

验证记录：`bringup_config_test` 当前覆盖 4 项配置检查，其中包含 `edgepick_task_mock.launch.py` 的节点和 topic 契约。

## 下一步目标

阶段 6：为 mock 任务闭环新增组合 launch，让 task node、mock 事件适配器和 mock MoveIt 链路可以一键启动。
