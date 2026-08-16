# EdgePick mock bringup chain

阶段 3 的启动链路不控制真实机械臂。它只验证 MoveIt、controller manager 和 `edgepick_hardware/MockSystemInterface` 能在同一条 mock 路径上工作。

## 启动层

`edgepick_mock_control.launch.py`：

- 展开 `edgepick_dofbot.urdf.xacro`；
- 启动 `robot_state_publisher`；
- 启动 `controller_manager/ros2_control_node`；
- 加载 `joint_state_broadcaster`、`arm_group_controller` 和 `grip_group_controller`。

`edgepick_moveit_mock.launch.py`：

- 复用 vendor `dofbot_pro_moveit` 的 SRDF、kinematics、joint limits、MoveIt controller 配置和 RViz 配置；
- 将 `robot_description` 替换为 EdgePick xacro；
- 启动 `move_group`、RViz 和 EdgePick mock controller manager。

## 控制路径

```text
RViz Plan/Execute
  -> move_group
  -> arm_group_controller/follow_joint_trajectory
  -> controller_manager
  -> edgepick_hardware/MockSystemInterface
  -> CommandGateway
  -> MockTransport
  -> joint state update
  -> joint_state_broadcaster
  -> /joint_states
  -> robot_state_publisher / RViz
```

## 当前阶段

阶段 3：启动覆盖层已经进入仓库，并通过配置测试验证 xacro 与 controller yaml 的关键连接点。

## 验证记录

2026-08-16，在受限沙箱中完成两次短时启动验证：

- `edgepick_mock_control.launch.py`：`EdgePickMockSystem` 完成 initialize/configure/activate，`joint_state_broadcaster`、`arm_group_controller` 和 `grip_group_controller` 均 loaded/configured/activated。
- `edgepick_moveit_mock.launch.py use_rviz:=false`：`move_group` 加载 `DOFBOT_Pro-V24` robot model，启动 planning scene monitor，并监听 `joint_states`。

当前沙箱会出现 DDS UDP socket 权限警告，这是环境网络权限限制；真实桌面终端中仍需补充 RViz 执行路径验证。

## 下一步

阶段 4：在真实桌面终端启动 RViz，记录 controller 列表、action 名称、`/joint_states` 发布情况和 RViz 执行结果。
