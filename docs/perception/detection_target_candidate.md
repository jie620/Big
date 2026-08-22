# Detection Target Candidate

## 当前阶段

阶段 9：EdgePick 已具备检测框驱动的 RGB-D 目标候选点链路。

`mock_detector_node` 先发布模拟检测结果，`detected_target_candidate_node` 根据类别、标签和置信度选择目标检测框，再把检测框中心像素交给阶段 8 的深度投影逻辑。

## Topic

| 方向 | Topic | 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/edgepick/perception/detections` | `edgepick_interfaces/msg/TargetDetectionArray` | 检测器输出的一帧目标框 |
| 输入 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图，支持 `16UC1` 和 `32FC1` |
| 输入 | `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| 输出 | `/edgepick/perception/target_point` | `geometry_msgs/msg/PointStamped` | 相机坐标系下的候选点 |
| 输出 | `/edgepick/task/event` | `std_msgs/msg/String` | 可选发布 `target_acquired` 或 `target_lost` |

## 启动

先确保 Orbbec 相机已经发布 `/camera/depth/image_raw` 和 `/camera/depth/camera_info`，然后运行：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_detection_perception_mock.launch.py
```

常用参数：

```text
target_class_id:=1
target_label:=target
min_detection_score:=0.50
mock_center_u:=320.0
mock_center_v:=240.0
max_detection_age_ms:=500
```

## 验证

```bash
ros2 topic echo /edgepick/perception/detections --once
ros2 topic echo /edgepick/perception/target_point
ros2 topic echo /edgepick/task/event
```

2026-08-17 代码侧验证：五包构建通过，自动化测试为 68 tests、0 errors、0 failures、0 skipped；`edgepick_detection_perception_mock.launch.py --show-args` 通过，短时启动可创建 mock detector 与 detected target candidate 节点。真实相机终端还需补 `/edgepick/perception/target_point` 的持续输出证据。

2026-08-19 阶段 10 补充：新增 `perception_metrics_node` 后，可通过 `/edgepick/perception/metrics` 同时观察 detection、target point 和 task event 的量测结果。

如果 `/edgepick/perception/detections` 有输出但 `/edgepick/perception/target_point` 没有输出，优先检查：

- depth topic 是否仍在发布：`ros2 topic hz /camera/depth/image_raw`
- depth 编码是否为当前支持类型：`ros2 topic echo /camera/depth/image_raw --once | grep encoding`
- mock 检测框中心是否落在图像范围内。
- `target_label`、`target_class_id` 和 `min_detection_score` 是否把 mock 结果过滤掉。

## 结构反思

阶段 9 仍不是最终 TensorRT 部署层，而是先把检测结果消息、目标选择策略和 RGB-D 投影边界固定。这个拆分能让后续真实模型出现误检、漏检或延迟抖动时，先判断问题在检测器还是在深度投影链路。

## 下一步

阶段 18：橘子目标检测桥接与真机感知链路验证。
