# Target Frame Transform

## 当前阶段

阶段 11：EdgePick 已具备把相机坐标系目标点转换到机器人规划坐标系的边界。

`target_frame_transform_node` 订阅 `/edgepick/perception/target_point`，从 TF 树查找源 frame 到目标 frame 的变换，并发布 `/edgepick/perception/target_point_base`。默认目标 frame 是 `base_link`。

## Topic

| 方向 | Topic | 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/edgepick/perception/target_point` | `geometry_msgs/msg/PointStamped` | 相机坐标系下的目标候选点 |
| 输出 | `/edgepick/perception/target_point_base` | `geometry_msgs/msg/PointStamped` | 机器人规划坐标系下的目标候选点 |
| 依赖 | `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 相机 frame 到目标 frame 的 TF |

## 启动

先确保 TF 树中存在 `base_link <- camera_color_optical_frame` 或等价链路，然后运行：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_target_frame_transform.launch.py
```

常用参数：

```text
input_topic:=/edgepick/perception/target_point
output_topic:=/edgepick/perception/target_point_base
target_frame:=base_link
transform_timeout_ms:=100
```

如果只有临时标定值，可以在另一个终端发布 static transform，再启动本节点：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_color_optical_frame
```

该 static transform 只是链路验证占位；正式抓取前必须替换为实测手眼标定结果。

## 验证

```bash
ros2 topic echo /edgepick/perception/target_point --once
ros2 topic echo /edgepick/perception/target_point_base --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

2026-08-19 代码侧验证：五包构建通过；`target_frame_transform_test` 覆盖平移、Z 轴旋转和零四元数回退；`bringup_config_test` 覆盖 `edgepick_target_frame_transform.launch.py` 的 topic、默认目标 frame 和超时参数。

## 结构反思

阶段 11 只做坐标转换，不做抓取姿态、不做规划、不碰真实硬件。这个分层能把“目标点是否在正确机器人坐标系”单独验证清楚，避免后续把 TF、MoveIt goal 和真实执行失败混在一起。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
