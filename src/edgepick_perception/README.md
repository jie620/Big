# edgepick_perception

EdgePick 的 RGB-D 感知基础包。阶段 8 先实现相机内参、深度采样和像素到三维点的转换，不接 YOLO/TensorRT，也不控制机械臂。

## 结构

- `include/edgepick_perception/rgbd_projection.hpp`：相机内参、像素、深度范围和三维点投影接口。
- `src/rgbd_projection.cpp`：`16UC1` 毫米深度、`32FC1` 米深度解析和 pinhole 投影实现。
- `src/rgbd_target_candidate_node.cpp`：ROS 2 节点，订阅 depth image 与 camera info，发布目标候选点和可选任务事件。
- `test/rgbd_projection_test.cpp`：内参解析、深度读取、三维投影和异常输入测试。

## Topic

- 输入：`/camera/depth/image_raw`，类型 `sensor_msgs/msg/Image`。
- 输入：`/camera/depth/camera_info`，类型 `sensor_msgs/msg/CameraInfo`。
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

## 下一步目标

阶段 9：新增目标检测/TensorRT 推理层，让目标像素来自检测结果，而不是固定中心点。
