# ADR 0013: Pre-Hardware Mock Rehearsal

## 决策

阶段 13 新增 `mock_rgbd_source_node`、`edgepick_prehardware_mock_rehearsal.launch.py` 和真实硬件前 readiness checklist。该 launch 在不接真实相机、不接真实 `/dev/i2c-7` 的情况下串起：

- mock RGB-D source
- mock detector
- detected target candidate
- static camera TF
- target frame transform
- grasp/pregrasp target builder
- perception metrics
- task node
- state-gated mock task driver
- MoveIt action mock adapter

## 背景

阶段 8 到阶段 12 已经分别建立了 RGB-D 投影、检测契约、metrics、TF 转换和抓取目标构造。但如果这些边界只能单独启动，真实硬件接入时仍会一次性暴露 topic、TF、事件时序和参数类型问题。

阶段 13 因此建立系统级 mock rehearsal：所有真实硬件前链路在同一个 launch 中启动，并通过 `target_acquired -> plan_succeeded -> execution_succeeded -> verification_succeeded` 跑到 `succeeded`。

## 影响

- `mock_rgbd_source_node` 发布固定 `16UC1` depth image 和 `CameraInfo`，用于无相机环境下复现感知链路。
- `detected_target_candidate_node` 新增可选 task-state gate，rehearsal 中只在 `/edgepick/task/state == perceiving` 时发布一次 `target_acquired`。
- `edgepick_prehardware_mock_rehearsal.launch.py` 明确使用 mock action results，不发送真实 MoveIt goal，不启动真实 ros2_control hardware，不访问真实 I2C。
- 短时启动已在沙箱中跑到 `succeeded`；日志仍出现 DDS UDP socket 权限警告，这是当前沙箱网络限制，需要在真实终端复验 topic echo。

## 状态

阶段 13：真实硬件前系统级 mock 演练已实现。

验证记录：2026-08-19 五包构建通过；五包测试汇总为 90 tests、0 errors、0 failures、0 skipped；新增 `mock_rgbd_source_test` 和 `system_rehearsal_success` 脚本测试；`bringup_config_test` 覆盖 rehearsal launch；`edgepick_prehardware_mock_rehearsal.launch.py --show-args` 通过；短时启动创建 10 个节点并完成 `idle -> perceiving -> planning -> executing -> verifying -> succeeded`。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认 launch 仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
