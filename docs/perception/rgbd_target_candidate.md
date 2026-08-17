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

## 结构反思

这一步不是目标检测，也不是手眼标定。它只把 RGB-D 数据变成相机坐标系下的候选点，故意保持薄而可测。这样下一步加 YOLO/TensorRT 时，如果候选点异常，可以先判断问题来自检测像素还是深度投影。

## 下一步

阶段 9：新增目标检测/TensorRT 推理层，让目标像素来自检测框中心、分割质心或其他检测结果。
