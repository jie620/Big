# ADR 0008: 先建立 RGB-D 感知基础层

## 决策

阶段 8 新增 `edgepick_perception` 包，先实现 CameraInfo 内参解析、深度图采样和 pinhole 三维投影。ROS 节点 `rgbd_target_candidate_node` 订阅 depth image 与 camera info，发布 `/edgepick/perception/target_point`，并可选发布 `target_acquired` 或 `target_lost`。

当前默认目标像素为深度图中心点，或由参数 `target_pixel_u`、`target_pixel_v` 指定。目标检测、TensorRT 推理和手眼标定不在阶段 8 实施。

## 背景

如果一开始就把 YOLO/TensorRT、深度图、相机内参、坐标转换和任务事件写在同一个节点里，调试时很难判断错误来自模型、深度、内参还是状态机。阶段 8 先固定 RGB-D 基础数学和 topic 契约。

## 后果

- `edgepick_task` 继续只接收任务事件，不直接处理图像。
- `edgepick_perception` 可以在没有模型的情况下测试深度与投影。
- Stage 9 可以把目标像素来源从固定中心点替换为检测框中心或分割质心。
- 当前输出坐标仍在相机光学坐标系中；机器人基座坐标和手眼标定是后续阶段。

## 结构反思

阶段 8 的边界是正确的：感知基础层负责“从 RGB-D 数据得到相机坐标系下的候选点”，任务层负责状态，MoveIt 层负责规划/执行结果，硬件层负责命令安全。这个拆分能让后续模型推理、TF 和真机控制分别验证。

## 当前阶段

阶段 8：RGB-D 感知基础层已实现。

## 下一步

阶段 9：新增目标检测/TensorRT 推理层，让目标像素来自检测结果，并开始记录推理延迟。
