# ADR 0011: 第十一阶段建立目标点 TF 转换边界

## 决策

阶段 11 新增 `target_frame_transform_node` 和 `edgepick_target_frame_transform.launch.py`，把 `/edgepick/perception/target_point` 从相机光学坐标系转换到机器人规划坐标系，默认输出 `/edgepick/perception/target_point_base`，默认目标 frame 为 `base_link`。

该节点只依赖 TF 树和感知目标点，不构造抓取姿态，不发送 MoveIt goal，也不访问真实 `/dev/i2c-7`。它是接入真实硬件之前的坐标边界：先证明目标点能进入机器人基座坐标系，再进入规划和真机执行。

## 背景

阶段 8/9/10 的输出仍然是相机坐标系下的候选点。继续直接推进 MoveIt 或硬件会带来两个风险：

- 目标点 frame 与机器人规划 frame 不一致，错误会在规划或执行阶段才暴露。
- 手眼标定、TF 发布、深度投影和目标选择问题会混在一起，不容易定位。

阶段 11 因此先建立一个薄的 TF 转换层。真实标定结果可以由外部 static transform、robot_state_publisher 或后续标定节点提供；本阶段只固定 topic 和转换行为。

## 后果

- `edgepick_perception` 增加纯 C++ `transform_target_point` helper，可单测平移、旋转和异常四元数处理。
- `target_frame_transform_node` 订阅相机坐标目标点，查 TF 后发布规划坐标目标点。
- `edgepick_target_frame_transform.launch.py` 暴露输入 topic、输出 topic、目标 frame 和 TF 超时参数。
- 当前阶段仍不生成抓取姿态、不调用 MoveIt、不启用真实硬件。

## 当前阶段

阶段 11：目标点 TF 转换边界已实现。

验证记录：2026-08-19 五包构建通过；新增 `target_frame_transform_test` 覆盖平移、Z 轴旋转和零四元数回退；`bringup_config_test` 覆盖阶段 11 launch 契约。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
