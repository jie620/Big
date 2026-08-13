# EdgePick

面向 Jetson Orin NX 与 Yahboom DOFBOT Pro 的资源自适应、可恢复 RGB-D 机器人抓取系统。

本仓库是 EdgePick 的唯一开发位置。厂商工程仅以只读参考快照保留在 `vendor/yahboom/`；所有新增或改造代码必须放在本仓库的 `src/`、`docs/`、`scripts/` 或 `test/` 中。

## 目标架构

```text
Orbbec RGB-D -> 三维目标位姿 -> 任务状态机 -> MoveIt 2
                                           -> FollowJointTrajectory
                                           -> C++ ros2_control 硬件接口
                                           -> I2C / DOFBOT 舵机

旁路：diagnostics、rosbag2、端到端延迟、温度/功耗、I2C 错误统计
```

系统定位为软实时边缘机器人系统，而非硬实时控制器。真实机械臂阶段必须由唯一的硬件接口进程持有 `/dev/i2c-7`。

## 目录

```text
src/                 自研 ROS 2 包，后续开发主目录
docs/                架构、接口、实验与标定记录
scripts/             可重复执行的开发和测量脚本
test/                独立于真机的测试资产
vendor/yahboom/      从本机厂商工作区提取的只读参考快照
```

## 开发边界

- 先实现 mock 硬件和自动化测试，再低速接入真实机械臂。
- 不修改 `vendor/` 中的文件；需要行为时在 `src/` 中重新实现并通过测试验证。
- 不将模型、TensorRT 引擎、ROS 构建产物、rosbag 或机械臂网格纳入 Git。
- 真实设备、相机驱动和厂商 ROS 接口仍由本机既有安装提供，具体来源见 `vendor/EXTERNAL_DEPENDENCIES.md`。

## 当前阶段

阶段 1：已完成 `edgepick_hardware` 的 C++ 命令网关和内存 mock 传输。它覆盖关节/时间校验、重复命令抑制、发送限频和传输失败注入，不连接真实机械臂。

在 ROS 2 Humble 终端中可复现构建和测试：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select edgepick_hardware
colcon test --base-paths src --packages-select edgepick_hardware
colcon test-result --all --verbose
```

下一步：将该库封装为 mock `ros2_control` 硬件接口，并让现有 MoveIt 配置先在 RViz 中通过该接口运行。真实 I2C 适配器不在当前阶段实施。
