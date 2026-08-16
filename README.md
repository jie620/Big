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

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 1：命令网关与 mock 传输

当前阶段：已完成 `edgepick_hardware` 的 C++ 命令网关和内存 mock 传输。它覆盖关节/时间校验、重复命令抑制、发送限频和传输失败注入，不连接真实机械臂。

完成内容：建立 `CommandGateway`、`CommandTransport` 和 `MockTransport`，让真实 I2C 接入前先有可测试的命令安全边界。

结构反思：阶段 1 的结构是正确的，因为它先把“能不能发命令、什么时候拒绝命令、失败如何统计”从 ROS 控制器和真实硬件中拆出来，避免一开始就把安全策略写死到 I2C 代码里。

当时下一步：将该库封装为 mock `ros2_control` 硬件接口，并让现有 MoveIt 配置先在 RViz 中通过该接口运行。真实 I2C 适配器不在阶段 1 实施。

### 阶段 2：mock ros2_control 系统接口

当前阶段：`edgepick_hardware` 已从命令网关推进到 mock `ros2_control` 系统接口。新增 `edgepick_hardware/MockSystemInterface` 插件后，控制器可以通过标准 position command interface 写入命令；插件内部把 MoveIt 的弧度关节位置转换为 DOFBOT 舵机角度，再进入 `CommandGateway` 和内存 mock 传输。该阶段仍不连接真实机械臂。

完成内容：新增 `hardware_interface::SystemInterface` 插件、pluginlib 描述、构建依赖和单元测试，确认控制器命令能进入 EdgePick 自己的硬件边界。

结构反思：阶段 2 的分层保持为 `ros2_control SystemInterface -> CommandGateway -> CommandTransport`。这能让 MoveIt/RViz 链路、mock 测试和未来真实 I2C 适配器共用同一条安全门，而不是让视觉或任务节点直接写硬件。

补充记录：为 `CommandGateway` 和 `MockSystemInterface` 增加模块级注释，明确安全门顺序、mock 状态更新、弧度到舵机角度转换和不触碰真实 I2C 的边界。后续代码也按“解释模块意图和关键边界，不逐行复述代码”的原则添加注释。

### 阶段 3：EdgePick mock bringup

当前阶段：新增 `edgepick_bringup`，提供 EdgePick 自己的 xacro、controller yaml 和 launch，让 controller manager、MoveIt 与 RViz 使用 `edgepick_hardware/MockSystemInterface`。该阶段仍不连接真实机械臂。

完成内容：新增 mock control launch、MoveIt mock launch、EdgePick xacro 覆盖层、controller 配置、配置测试、ADR 0003 和 mock 链路说明。

结构反思：阶段 3 把启动编排从 `edgepick_hardware` 中拆出来是正确的；硬件包只负责安全边界，bringup 包只负责把 vendor 运行资源和 EdgePick 自研接口连接起来。

验证记录：`edgepick_mock_control.launch.py` 短时启动成功，`EdgePickMockSystem` 完成 initialize/configure/activate，`joint_state_broadcaster`、`arm_group_controller`、`grip_group_controller` 均 loaded/configured/activated。`edgepick_moveit_mock.launch.py use_rviz:=false` 短时启动成功，`move_group` 加载 `DOFBOT_Pro-V24` robot model 并监听 `joint_states`。当前沙箱会报 DDS UDP socket 权限警告，属于受限环境限制；真实桌面终端应继续验证 RViz 执行路径。

### 阶段 4：抓取任务状态机核心

当前阶段：新增 `edgepick_task`，先实现无 ROS 依赖的 C++ 抓取任务状态机，并补充 RViz mock 验证手册。

完成内容：实现 `GraspStateMachine` 的状态、事件、失败码、恢复预算、超时、取消和重置；新增状态机测试、ADR 0004、状态机文档和 RViz mock 验证 runbook。

结构反思：阶段 4 继续保持分层：`edgepick_task` 只管任务决策，不直接调用 MoveIt 或硬件；后续 ROS 节点负责把感知、MoveIt 和验证结果翻译成状态机事件。

### 阶段 5：ROS 2 task node 与 diagnostics

当前阶段：`edgepick_task` 已从纯 C++ 状态机推进到 ROS 2 节点。新增 `task_node` 订阅 `/edgepick/task/event`，发布 `/edgepick/task/state`、`/edgepick/task/failure` 和 `/diagnostics`，让 mock 感知、规划、执行和验证组件可以先用稳定字符串事件驱动任务流程。

完成内容：新增事件字符串解析与状态快照工具、ROS 2 task node、task launch、事件 IO 测试、bringup launch 测试、ADR 0005 和 task node topic 文档。该阶段仍不调用 MoveIt action，也不连接真实 I2C。

结构反思：阶段 5 没有让状态机直接依赖相机、MoveIt 或硬件，而是先固定 ROS 边界和 diagnostics 面。这样后续接入真实规划/执行结果时，只需要把外部结果翻译成同一组 `TaskEvent`。

验证记录：2026-08-16 构建 `edgepick_hardware`、`edgepick_bringup`、`edgepick_task` 通过；测试结果为 36 tests、0 errors、0 failures、0 skipped。`ros2 pkg executables edgepick_task` 可识别 `edgepick_task task_node`。`edgepick_task_mock.launch.py --show-args` 在 `ROS_LOG_DIR=/tmp/edgepick_ros_logs` 下通过；短时启动能创建 `task_node` 进程，但当前沙箱仍会因 DDS UDP socket 权限限制报错，真实桌面终端需复验 topic 通信。

## 复现命令

在 ROS 2 Humble 终端中可复现构建和测试：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
colcon build --base-paths src --packages-select edgepick_hardware edgepick_bringup edgepick_task
colcon test --base-paths src --packages-select edgepick_hardware edgepick_bringup edgepick_task
colcon test-result --test-result-base build --all --verbose
```

## 下一步目标

阶段 6：新增 mock 任务闭环适配层，把 mock 感知、规划、执行和验证结果自动转换为 `TaskEvent`，形成可 rosbag 记录的完整抓取流程；仍不接真实 I2C。
