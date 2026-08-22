# ADR 0016: Real MoveIt on Real Control

## 决策

阶段 16 新增 `edgepick_moveit_real.launch.py`。该入口把 `edgepick_real_control.launch.py` 和 MoveGroup 组合在一起，默认不开 RViz，并继续沿用 vendor 的 MoveIt controller 映射。

## 背景

阶段 15 已经证明真实 I2C 后端和单关节低速动作可用。下一步不再是验证底层写帧，而是把 MoveIt 规划链路接到真实 controller-manager 上，先确认规划到执行的闭环能在低速条件下工作。

## 影响

- 真机 MoveIt 联动入口和底层 real control 入口分离，但共享同一套 ros2_control 控制器名。
- 默认 `use_rviz:=false`，避免把阶段 16 做成 GUI 依赖入口。
- real control 启动时会继续出现控制器激活姿态，这属于预期行为，不再视为异常。
- stage 16 不引入 task node 或 perception node，仍只覆盖 MoveIt -> controller-manager -> hardware 的最小链路。

## 状态

阶段 16：real MoveIt + real control 联动入口已新增，等待在真实 DOFBOT 上执行最小规划目标。

验证记录：2026-08-21 `edgepick_bringup` 构建通过；`bringup_config_test` 覆盖到 14 项检查；`edgepick_moveit_real.launch.py --show-args` 通过，参数包含 `publish_frequency`、`use_real_i2c`、`i2c_device`、`i2c_address` 和 `use_rviz`。

## 下一步

阶段 18：橘子目标检测桥接与真机感知链路验证。
