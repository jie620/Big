# ADR 0009: 先固定检测结果契约，再接真实 TensorRT

## 决策

阶段 9 新增 `edgepick_interfaces` 包，定义 `TargetDetection` 和 `TargetDetectionArray`，并在 `edgepick_perception` 中新增检测框选择逻辑、`mock_detector_node` 和 `detected_target_candidate_node`。

`detected_target_candidate_node` 订阅 `/edgepick/perception/detections`、`/camera/depth/image_raw` 和 `/camera/depth/camera_info`，先按类别、标签和置信度选出一个检测框，再取检测框中心像素复用阶段 8 的深度投影，发布 `/edgepick/perception/target_point` 和可选任务事件。

## 背景

真实 TensorRT 推理需要模型、输入预处理、CUDA/TensorRT runtime、类别表和性能统计。如果直接把这些和 RGB-D 投影写在同一个节点里，调试时会很难区分问题来自模型输出、检测框筛选、深度图编码、相机内参还是任务事件。

阶段 9 因此先固定检测消息和 mock detector：当前没有部署真实模型，但已经把“检测结果如何驱动三维候选点”这条接口打通。

## 后果

- 后续 YOLO/TensorRT 节点只要发布 `TargetDetectionArray`，就能替换 `mock_detector_node`。
- RGB-D 投影、检测框选择和任务状态事件仍然可以无模型测试。
- 当前目标像素来自检测框中心；分割质心、抓取点偏移和多目标策略留给后续阶段。
- 当前不访问真实 `/dev/i2c-7`，不会移动机械臂。

## 结构反思

阶段 9 的结构是正确的：接口包定义消息，perception 包处理检测选择和深度投影，bringup 包组合 mock detector 与候选点节点。这样模型推理可以独立迭代，不会污染任务状态机、MoveIt action 适配器或硬件安全边界。

## 当前阶段

阶段 9：检测框驱动的 RGB-D 目标候选点链路已实现。

验证记录：2026-08-17 五包构建和测试通过，自动化测试为 68 tests、0 errors、0 failures、0 skipped；阶段 9 launch 参数检查和短时节点启动通过。

阶段 10 补充：检测契约保持不变，新增 `perception_metrics_node` 作为旁路观察者记录检测延迟、目标点稳定性和任务事件计数。

## 下一步

阶段 18：橘子目标检测桥接与真机感知链路验证。
