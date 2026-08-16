# RViz mock verification runbook

这份手册用于在真实桌面终端补齐阶段 4 的 GUI 验证。它仍然只运行 mock 硬件，不会控制真实机械臂。

## 启动

Terminal 1：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ros2 launch edgepick_bringup edgepick_moveit_mock.launch.py
```

Terminal 2：

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source /home/jetson/Codex_Projects/Big/install/setup.bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}"
ros2 action list | grep -E "follow_joint_trajectory|gripper_cmd|move_action"
ros2 topic echo /joint_states --once
```

## RViz 操作

1. 在 RViz MotionPlanning 面板选择 `arm_group`。
2. 选择一个命名姿态或轻微拖动 interactive marker。
3. 点击 `Plan`。
4. 确认规划成功后点击 `Execute`。
5. 观察 RViz 模型运动，确认真实机械臂没有动作。

## 记录模板

```text
日期：
启动命令：
controller 状态：
action 列表：
/joint_states 是否发布：
RViz Plan 结果：
RViz Execute 结果：
真实机械臂是否保持不动：
异常日志：
结论：
```

## 当前阶段

阶段 4：运行手册已进入仓库，等待真实桌面终端补充执行记录。

## 下一步

阶段 5：把 `edgepick_task` 状态机接到 ROS 2 节点和 mock MoveIt 执行结果上。
