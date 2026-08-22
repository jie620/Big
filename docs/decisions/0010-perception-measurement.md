# ADR 0010: 第十阶段先建立感知量测入口

## 决策

阶段 10 新增 `perception_metrics_node` 和 `edgepick_perception_metrics.launch.py`，先把真实模型 publisher 或 rosbag 回放产生的检测链路变成可量测对象。

该阶段继续沿用阶段 9 的检测契约：真实 YOLO/TensorRT 节点或回放数据只需要发布 `/edgepick/perception/detections`，`detected_target_candidate_node` 仍负责把检测框中心与深度图投影成 `/edgepick/perception/target_point`。新增的 metrics 节点只订阅检测、目标点和任务事件，定期发布 `/edgepick/perception/metrics`，不修改感知输出，也不访问真实 `/dev/i2c-7`。

## 背景

阶段 9 已经固定 `TargetDetectionArray` 到 RGB-D 目标点的接口，但真实模型或 rosbag 接入后，最先需要回答的问题不是机械臂能不能抓，而是：

- 检测 topic 是否持续输出。
- 检测消息和目标点的时间戳年龄是否可接受。
- 目标点在相机坐标系下是否稳定。
- `target_acquired` 和 `target_lost` 是否符合预期触发。

如果这些证据缺失，后续直接进入 TF、MoveIt goal 或真实 I2C 会把模型延迟、深度噪声、目标选择和执行失败混在一起，难以定位。

## 后果

- `edgepick_perception` 增加纯 C++ 指标累积库，便于单元测试延迟、稳定性和事件计数。
- `perception_metrics_node` 发布一行稳定摘要，适合终端、rosbag 或日志采集。
- `edgepick_perception_metrics.launch.py` 可选择启动 `ros2 bag play --clock`，也可只观察外部真实 detector。
- 当前阶段仍不包含 TensorRT runtime、模型预处理或真实机械臂执行；这些继续作为可替换上游或后续阶段。

## 当前阶段

阶段 10：感知量测入口已实现。

验证记录：2026-08-19 五包构建通过；`colcon test-result --test-result-base build --all --verbose` 汇总为 74 tests、0 errors、0 failures、0 skipped。新增 `perception_metrics_test` 覆盖指标统计和摘要格式，`bringup_config_test` 覆盖阶段 10 launch 契约。

## 下一步

阶段 18：橘子目标检测桥接与真机感知链路验证。
