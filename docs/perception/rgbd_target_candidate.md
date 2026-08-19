# RGB-D Target Candidate

## 当前阶段

阶段 8：EdgePick 已具备 RGB-D 目标候选点基础链路。

`rgbd_target_candidate_node` 订阅相机内参和深度图，选择一个目标像素，读取该像素深度，再用 pinhole 模型投影到相机坐标系。

## Topic

| 方向 | Topic | 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图，支持 `16UC1` 和 `32FC1` |
| 输入 | `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| 输出 | `/edgepick/perception/target_point` | `geometry_msgs/msg/PointStamped` | 相机坐标系下的候选点 |
| 输出 | `/edgepick/task/event` | `std_msgs/msg/String` | 可选发布 `target_acquired` 或 `target_lost` |

## 启动

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_rgbd_perception.launch.py
```

常用参数：

```text
depth_topic:=/camera/depth/image_raw
camera_info_topic:=/camera/depth/camera_info
target_pixel_u:=-1
target_pixel_v:=-1
min_depth_m:=0.05
max_depth_m:=1.20
publish_task_events:=true
```

`target_pixel_u/v=-1` 表示使用深度图中心点。Stage 9 会把这里替换为检测结果提供的目标像素。

## 数学

```text
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = depth_m
```

其中 `fx/fy/cx/cy` 来自 `CameraInfo.k`，输出点仍在相机光学坐标系中。

## 验证记录

2026-08-16：

- `rgbd_projection_test` 覆盖内参解析、`16UC1` 深度、`32FC1` 深度、投影和异常输入。
- `edgepick_rgbd_perception.launch.py --show-args` 通过。
- 短时启动可创建 `rgbd_target_candidate_node` 并等待相机 topic。
- 沙箱仍有 DDS UDP socket 权限警告，真实桌面终端需使用 Orbbec topic 复验。

2026-08-17：

- 真实终端确认 DaBai DCW2 发布 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/depth/camera_info`、`/camera/depth/points`、`/camera/depth_registered/points` 和 `/camera/ir/image_raw` 等 topic。
- `/camera/depth/image_raw` 约 10 Hz；`/camera/depth/camera_info` 返回 640x480 内参，`fx≈478.65`、`fy≈478.39`、`cx≈319.88`、`cy≈236.72`。
- `edgepick_rgbd_perception.launch.py` 已在本仓库 install 环境中启动 `rgbd_target_candidate_node`，等待的默认 topic 与相机实际 topic 一致。
- 仍需补最后一条输出验证：`ros2 topic echo /edgepick/perception/target_point`；如果没有输出，先检查 `/camera/depth/image_raw` 的 `encoding` 是否为当前支持的 `16UC1` 或 `32FC1`。

## 结构反思

这一步不是目标检测，也不是手眼标定。它只把 RGB-D 数据变成相机坐标系下的候选点，故意保持薄而可测。阶段 10 已增加旁路 metrics 节点；如果后续真实模型或 rosbag 的候选点异常，可以先判断问题来自检测像素、深度投影还是目标点稳定性。

## 下一步

阶段 11：用真实模型或 rosbag 的 metrics 输出调整检测阈值、深度范围和目标点稳定性策略。
