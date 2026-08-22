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

### 阶段 18：接口继续稳定复用

当前阶段：橘子检测链路继续复用 `TargetDetection` 和 `TargetDetectionArray`，不新增接口消息。

完成内容：COCO 橘子 detector 直接发布现有检测消息，`detected_target_candidate_node` 继续吃同一条 `/edgepick/perception/detections` 约定。

结构反思：接口包的价值在于稳定。只要二维检测框语义没变，就不该为了换模型或换类别新增消息字段。

## 下一步目标

阶段 19：继续保持检测接口稳定；若以后确实需要更丰富的抓取目标契约，再新增独立接口消息。
