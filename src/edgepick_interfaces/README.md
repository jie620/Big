# edgepick_interfaces

EdgePick 的自定义 ROS 2 接口包。阶段 9 先加入目标检测结果消息，让 YOLO/TensorRT、mock detector 和 RGB-D 投影节点之间有稳定契约。

## 消息

- `TargetDetection`：单个二维检测框，包含类别、标签、置信度、中心像素和框尺寸。
- `TargetDetectionArray`：一帧图像对应的检测结果数组，带 `std_msgs/Header`。

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 9：目标检测接口

当前阶段：新增 `edgepick_interfaces`，为检测/TensorRT 层定义轻量消息契约。

完成内容：`TargetDetectionArray` 作为 `/edgepick/perception/detections` 的默认消息类型，后续真实 TensorRT 节点和当前 mock detector 都发布同一格式。

结构反思：把消息独立成接口包是正确的；检测节点、RGB-D 投影节点和任务节点不需要互相包含实现代码，只共享稳定消息定义。

验证记录：2026-08-17 `edgepick_interfaces` 随五包构建通过，`ros2 interface show edgepick_interfaces/msg/TargetDetectionArray` 能显示检测数组消息结构。

## 下一步目标

阶段 14：真实硬件接入暂不新增接口消息，继续保持检测接口稳定；若真机检查发现需要更丰富的抓取目标契约，再新增独立接口消息。
