# Mock rehearsal

系统级 mock 不接相机、I2C 或真实 MoveIt 控制器，用于验证消息、场景更新、Safety Gate 和故障注入。

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/local_setup.bash
source install/local_setup.bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 \
ros2 launch edgepick_bringup edgepick_system.launch.py \
  mode:=mock auto_arm:=true scenario:=normal
```

可用场景：`normal`、`estop`、`target_loss`、`depth_timeout`、`policy_timeout`、`nan`、`jump`、`replay`、`collision` 和 `destination_blocked`。场景由 `edgepick_safe/mock_inputs` 注入，所有动作仍经过 `/edgepick/safe/arm` 和 Safety Gate。

运行结束后检查：

```bash
ros2 topic echo /edgepick/safe/state --once
ros2 topic echo /edgepick/safe/event
```

另有 `edgepick_task_closed_loop.launch.py` 等单包 mock launch，用于状态机单测和 topic 契约回归；它们不启动真实硬件，也不是新的真机入口。
