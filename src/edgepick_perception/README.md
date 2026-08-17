# edgepick_perception

EdgePick 的 RGB-D 感知基础包。阶段 8 先实现相机内参、深度采样和像素到三维点的转换，不接 YOLO/TensorRT，也不控制机械臂。

## 结构

- `include/edgepick_perception/rgbd_projection.hpp`：相机内参、像素、深度范围和三维点投影接口。
- `include/edgepick_perception/target_detection_selection.hpp`：检测框过滤、排序和中心像素选择接口。
- `src/rgbd_projection.cpp`：`16UC1` 毫米深度、`32FC1` 米深度解析和 pinhole 投影实现。
- `src/rgbd_target_candidate_node.cpp`：ROS 2 节点，订阅 depth image 与 camera info，发布目标候选点和可选任务事件。
- `src/detected_target_candidate_node.cpp`：订阅检测结果、depth image 和 camera info，发布检测框驱动的三维候选点。
- `src/mock_detector_node.cpp`：发布可配置 mock 检测框，用于无模型验证阶段 9 链路。
- `test/rgbd_projection_test.cpp`：内参解析、深度读取、三维投影和异常输入测试。
- `test/target_detection_selection_test.cpp`：检测框过滤、排序和像素选择测试。

## Topic

- 输入：`/camera/depth/image_raw`，类型 `sensor_msgs/msg/Image`。
- 输入：`/camera/depth/camera_info`，类型 `sensor_msgs/msg/CameraInfo`。
- 输入：`/edgepick/perception/detections`，类型 `edgepick_interfaces/msg/TargetDetectionArray`。
- 输出：`/edgepick/perception/target_point`，类型 `geometry_msgs/msg/PointStamped`。
- 可选输出：`/edgepick/task/event`，类型 `std_msgs/msg/String`，发布 `target_acquired` 或 `target_lost`。

## 使用

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ros2 run edgepick_perception rgbd_target_candidate_node
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

## 下一步目标

阶段 10：接入真实模型推理或 rosbag 回放，记录检测延迟、目标点稳定性和任务事件触发情况。
