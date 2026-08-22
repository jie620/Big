# edgepick_perception

EdgePick 的 RGB-D 感知基础包。阶段 8 先实现相机内参、深度采样和像素到三维点的转换，不接 YOLO/TensorRT，也不控制机械臂。

## 结构

- `include/edgepick_perception/rgbd_projection.hpp`：相机内参、像素、深度范围和三维点投影接口。
- `include/edgepick_perception/mock_rgbd_source.hpp`：mock RGB-D source 的 depth/camera_info 构造接口。
- `include/edgepick_perception/target_detection_selection.hpp`：检测框过滤、排序和中心像素选择接口。
- `include/edgepick_perception/target_frame_transform.hpp`：目标点 TF 转换接口。
- `src/rgbd_projection.cpp`：`16UC1` 毫米深度、`32FC1` 米深度解析和 pinhole 投影实现。
- `src/rgbd_target_candidate_node.cpp`：ROS 2 节点，订阅 depth image 与 camera info，发布目标候选点和可选任务事件。
- `src/detected_target_candidate_node.cpp`：订阅检测结果、depth image 和 camera info，发布检测框驱动的三维候选点。
- `src/mock_detector_node.cpp`：发布可配置 mock 检测框，用于无模型验证阶段 9 链路。
- `scripts/edgepick_yolo_detector_node.py`：读取真实相机图像并发布厂商 YOLO 检测结果，保留给垃圾分类模型。
- `scripts/edgepick_coco_detector_node.py`：读取真实相机图像并发布 COCO 检测结果，当前用于橘子目标。
- `src/mock_rgbd_source_node.cpp`：发布固定 depth image 和 camera info，用于阶段 13 rehearsal。
- `src/perception_metrics_node.cpp`：旁路发布感知链路延迟、稳定性和事件计数。
- `src/target_frame_transform_node.cpp`：把相机坐标目标点转换到机器人规划 frame。
- `test/rgbd_projection_test.cpp`：内参解析、深度读取、三维投影和异常输入测试。
- `test/target_detection_selection_test.cpp`：检测框过滤、排序和像素选择测试。
- `test/perception_metrics_test.cpp`：感知指标统计测试。
- `test/mock_rgbd_source_test.cpp`：mock depth/camera_info 消息构造测试。
- `test/target_frame_transform_test.cpp`：目标点 TF 转换测试。

## Topic

- 输入：`/camera/depth/image_raw`，类型 `sensor_msgs/msg/Image`。
- 输入：`/camera/depth/camera_info`，类型 `sensor_msgs/msg/CameraInfo`。
- 输入：`/edgepick/perception/detections`，类型 `edgepick_interfaces/msg/TargetDetectionArray`。
- 输出：`/edgepick/perception/target_point`，类型 `geometry_msgs/msg/PointStamped`。
- 输出：`/edgepick/perception/target_point_base`，类型 `geometry_msgs/msg/PointStamped`。
- 输出：`/edgepick/perception/metrics`，类型 `std_msgs/msg/String`。
- 可选输出：`/edgepick/task/event`，类型 `std_msgs/msg/String`，发布 `target_acquired` 或 `target_lost`。

## 使用

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ros2 run edgepick_perception rgbd_target_candidate_node
```

```bash
ros2 run edgepick_perception edgepick_yolo_detector_node.py
```

```bash
ros2 run edgepick_perception edgepick_detection_viewer_node.py
```

默认使用 depth 图像中心点作为临时目标像素。后续 Stage 9 会由检测/分割结果提供目标像素。

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 8：RGB-D 感知基础层

当前阶段：新增 `edgepick_perception` 包，建立 RGB-D 目标候选点的最小可测链路。

完成内容：实现 `CameraInfo` 内参解析、`16UC1`/`32FC1` 深度读取、pinhole 三维投影、目标候选点发布和 `target_acquired`/`target_lost` 事件发布。

结构反思：感知基础层独立成包是正确的；任务状态机不需要知道图像格式，MoveIt adapter 不需要知道深度采样，硬件层仍完全不参与感知。

验证记录：`rgbd_projection_test` 覆盖内参、深度和投影边界；2026-08-16 四包测试结果为 61 tests、0 errors、0 failures、0 skipped。本包可作为 Stage 9 检测/TensorRT 的输入基础。

补充验证记录：2026-08-17 根据真实终端回传，DaBai DCW2 的 `/camera/depth/image_raw` 约 10 Hz，`/camera/depth/camera_info` 返回 640x480 内参，`fx≈478.65`、`fy≈478.39`、`cx≈319.88`、`cy≈236.72`。`rgbd_target_candidate_node` 已通过 bringup launch 启动并等待默认 depth/camera_info topic。下一步仍需 echo `/edgepick/perception/target_point`，确认候选点实际持续发布。

### 阶段 9：检测框驱动的目标候选点

当前阶段：新增检测结果选择库、mock detector 和检测框驱动的目标候选点节点。

完成内容：支持按 `target_class_id`、`target_label` 和 `min_detection_score` 筛选检测结果；在候选分数相同的情况下用检测框面积做确定性排序；将选中检测框中心像素交给阶段 8 深度投影。

结构反思：真实 TensorRT 推理还没有进入本包核心逻辑；当前先把检测输出到三维目标点的接口固定住，避免后续模型、深度、任务事件耦合在一起。

验证记录：`target_detection_selection_test` 覆盖 5 个检测选择用例；2026-08-17 五包测试结果为 68 tests、0 errors、0 failures、0 skipped。`ros2 pkg executables edgepick_perception` 可识别检测候选点节点和 mock detector。

### 阶段 10：感知量测入口

当前阶段：新增 `perception_metrics` 指标库和 `perception_metrics_node`，用于真实模型或 rosbag 回放时观察检测帧、目标点和任务事件。

完成内容：指标库统计检测消息数量、候选框数量、空检测帧、检测延迟、目标点延迟、目标点步长、Z 轴稳定性和任务事件计数；节点把摘要发布到 `/edgepick/perception/metrics`。

结构反思：metrics 节点是旁路观察者，不参与检测选择和深度投影。这样真实 YOLO/TensorRT 节点只需要按阶段 9 契约发布检测结果，指标层就能独立评估链路质量。

验证记录：`perception_metrics_test` 覆盖 4 个指标统计用例；2026-08-19 五包测试结果为 74 tests、0 errors、0 failures、0 skipped。`ros2 pkg executables edgepick_perception` 可识别 `perception_metrics_node`。

### 阶段 11：目标点基座坐标转换

当前阶段：新增 `target_frame_transform` helper 和 `target_frame_transform_node`，把 `/edgepick/perception/target_point` 转换为 `/edgepick/perception/target_point_base`。

完成内容：转换 helper 支持平移、旋转和零四元数回退；ROS 节点通过 TF 查询源 frame 到 `base_link` 的转换，并发布保留采样时间戳的目标点。

结构反思：目标点转换只处理 frame，不处理抓取姿态、规划目标或硬件控制。这样手眼标定和 TF 问题可以在 MoveIt 和真实机械臂之前单独验证。

验证记录：`target_frame_transform_test` 覆盖 3 个坐标转换用例；2026-08-19 五包测试通过。`ros2 pkg executables edgepick_perception` 可识别 `target_frame_transform_node`。

### 阶段 13：mock RGB-D source 与感知事件门控

当前阶段：新增 `mock_rgbd_source` helper 和 `mock_rgbd_source_node`，并为 `detected_target_candidate_node` 增加可选 task-state event gate。

完成内容：mock RGB-D source 发布固定 `16UC1` depth image 和 `CameraInfo`；rehearsal 中检测候选节点只在 `/edgepick/task/state == perceiving` 时发布一次 `target_acquired`。

结构反思：这让系统 rehearsal 不依赖真相机，也不会在任务进入 planning/executing 后继续刷感知事件。默认独立感知 launch 仍保持原行为。

验证记录：`mock_rgbd_source_test` 覆盖内参和深度图构造；2026-08-19 五包测试汇总为 90 tests、0 errors、0 failures、0 skipped；短时启动 rehearsal launch 后 metrics 显示 `target_points=50`、`target_acquired_events=1`。

### 阶段 18：真实目标检测桥接

当前阶段：新增 `scripts/edgepick_yolo_detector_node.py` 和 `scripts/edgepick_coco_detector_node.py`，分别承接厂商垃圾分类 YOLO 和橘子目标 COCO 检测，再由 `edgepick_detected_target_candidate_node` 继续做筛选和 RGB-D 投影。

完成内容：YOLO 分支继续读取厂商 `best.engine`；COCO 分支读取本机 `frozen_inference_graph.pb` 和 `object_detection_coco.txt`，默认筛选 `orange`。两条分支都按框发布 `class_id`、`label`、`score`、中心像素和框尺寸；下游仍保持阶段 9 的检测选择契约不变。

结构反思：目标检测必须跟任务对象对齐。垃圾分类模型可以保留作参考，但橘子抓取链路应走 COCO 橘子入口，避免把任务语义和模型语义混在一起。

验证记录：2026-08-22 该包构建通过；`ros2 pkg executables edgepick_perception` 识别到 `edgepick_yolo_detector_node.py` 和 `edgepick_coco_detector_node.py`；`edgepick_bringup/edgepick_orange_detection.launch.py --show-args` 成功展开橘子检测参数。待在真实 Jetson 上接相机后补充 `/edgepick/perception/detections` 实测。

## 下一步目标

阶段 19：把橘子检测结果稳定接到任务/抓取联调，并继续做真机链路观测。
