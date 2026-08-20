# Grasp Target Builder

## 当前阶段

阶段 12：EdgePick 已具备 mock-safe 的抓取/预抓取目标构造边界。

`grasp_target_builder_node` 订阅 `/edgepick/perception/target_point_base`，把基座坐标系目标点转换成两个规划目标：

| 方向 | Topic | 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/edgepick/perception/target_point_base` | `geometry_msgs/msg/PointStamped` | `base_link` 或配置目标 frame 下的目标候选点 |
| 输出 | `/edgepick/task/pregrasp_pose` | `geometry_msgs/msg/PoseStamped` | 抓取前的上方接近位姿 |
| 输出 | `/edgepick/task/grasp_pose` | `geometry_msgs/msg/PoseStamped` | 抓取目标位姿 |

## 默认策略

默认参数：

```text
target_frame:=base_link
pregrasp_offset_m:=0.08
grasp_z_offset_m:=0.02
end_effector_orientation_xyzw:=[0.0, 1.0, 0.0, 0.0]
```

计算规则：

```text
grasp_pose.position = target_point + [0, 0, grasp_z_offset_m]
pregrasp_pose.position = grasp_pose.position + [0, 0, pregrasp_offset_m]
```

节点会拒绝空 frame、与 `target_frame` 不匹配的输入、非有限坐标、负 offset 和零长度四元数。

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_mock_grasp_target.launch.py
```

验证 topic：

```bash
ros2 topic echo /edgepick/task/pregrasp_pose --once
ros2 topic echo /edgepick/task/grasp_pose --once
```

## 边界

- 已完成：目标点到抓取/预抓取 pose 的纯函数、ROS 节点、launch 和单元测试。
- 未完成：MoveIt `MotionPlanRequest` 或 `MoveGroup` goal 组装、碰撞体更新、轨迹执行策略。
- 安全边界：默认只发布 `PoseStamped`，不发送轨迹，不启动真实硬件，不访问 `/dev/i2c-7`。

## 验证记录

2026-08-19 代码侧验证：`edgepick_task` 与 `edgepick_bringup` 构建通过；`grasp_target_builder_test` 覆盖 offset、orientation 归一化和无效输入；`bringup_config_test` 覆盖 `edgepick_mock_grasp_target.launch.py` 的 topic、默认 frame 和 offset 参数；`ros2 pkg executables edgepick_task` 可列出 `grasp_target_builder_node`。

## 结构反思

阶段 12 没有把抓取 pose 计算塞进 MoveIt action adapter。这个分层让“目标位姿是否合理”和“MoveIt 是否规划成功”可以分开验证，也让真实硬件接入前仍保持所有输出可 echo、可录包、可单测。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
