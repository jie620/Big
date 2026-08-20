# ADR 0012: Mock MoveIt Grasp Target Builder

## 决策

阶段 12 新增 `grasp_target_builder` 纯 C++ helper、`grasp_target_builder_node` 和 `edgepick_mock_grasp_target.launch.py`。该边界订阅 `/edgepick/perception/target_point_base`，生成 `/edgepick/task/pregrasp_pose` 和 `/edgepick/task/grasp_pose` 两个 `geometry_msgs/msg/PoseStamped` 目标。

该阶段仍然不发送 MoveIt goal、不执行轨迹、不访问真实 `/dev/i2c-7`。它只把已经进入 `base_link` 的目标点变成 mock MoveIt 链路可消费的规划目标输入。

## 背景

阶段 11 已经把相机坐标系目标点转换到机器人规划 frame。如果直接在 MoveIt action adapter 中同时做目标点订阅、抓取姿态构造、action goal 发送和结果映射，后续调试会把坐标、姿态、规划器和真实执行问题混在一起。

阶段 12 因此建立一个薄的目标构造层：目标点进来，抓取/预抓取 pose 出去。MoveIt action adapter 仍负责规划/执行结果到任务事件的转换，真实 goal 发送继续留到真实硬件前的最后系统演练之后。

## 影响

- `edgepick_task` 新增 `geometry_msgs` 依赖和 `grasp_target_builder_node` 可执行入口。
- 默认抓取 pose 使用目标点 `z + 0.02 m`，预抓取 pose 在抓取 pose 上方再加 `0.08 m`。
- 默认末端 orientation 为 `[x=0, y=1, z=0, w=0]`，表示 mock 阶段先采用向下抓取姿态。
- helper 会拒绝空 frame、非有限目标点、负 offset 和零长度四元数。
- 当前阶段不新增自定义接口消息，继续使用标准 `PointStamped` 和 `PoseStamped` 保持边界轻量。

## 状态

阶段 12：mock MoveIt 抓取目标构造边界已实现。

验证记录：2026-08-19 `edgepick_task` 与 `edgepick_bringup` 构建通过；新增 `grasp_target_builder_test` 覆盖抓取/预抓取 offset、orientation 归一化和无效输入拒绝；`bringup_config_test` 覆盖阶段 12 launch 契约；`ros2 pkg executables edgepick_task` 可识别 `grasp_target_builder_node`；`edgepick_mock_grasp_target.launch.py --show-args` 通过。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
