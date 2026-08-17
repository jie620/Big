# 自研 ROS 2 包

后续按职责创建独立包，第一批预计包括：

- `edgepick_hardware`：C++ I2C 传输抽象、mock 后端和 ros2_control `SystemInterface`。当前已完成命令网关与 mock 系统接口。
- `edgepick_bringup`：生命周期管理、参数和启动编排。当前已完成 mock control 与 MoveIt mock launch。
- `edgepick_interfaces`：EdgePick 自定义 ROS 2 消息。当前已完成目标检测结果消息。
- `edgepick_perception`：RGB-D 相机内参、深度采样、目标检测选择和目标候选点。当前已完成检测框中心到三维点的基础链路。
- `edgepick_task`：抓取任务状态机、超时、恢复策略、ROS 2 task node、mock 闭环驱动和 MoveIt action 适配层。当前已完成规划/执行 action 结果映射。

任何真实 I2C 写入必须经 `edgepick_hardware`，不得在感知或任务节点中直接调用 `Arm_Lib`。

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 1：命令网关与 mock 传输

当前阶段：`src/edgepick_hardware` 建立了命令网关、传输抽象和内存 mock 后端。

完成内容：先把命令校验、重复抑制、发送限频和传输失败注入放进自研包，不让感知、任务或启动层直接接触真实 I2C。

结构反思：`src/` 在阶段 1 只放一个自研硬件边界包是合理的，先把最低层契约固定住，后续包可以按职责追加。

当时下一步：在 `edgepick_hardware` 中继续封装 mock `ros2_control` `SystemInterface`。

### 阶段 2：mock ros2_control 系统接口

当前阶段：`edgepick_hardware` 已具备 mock `ros2_control` `SystemInterface` 插件，能让控制器以标准 position interface 写入六关节命令，并在包内完成弧度到舵机角度的转换与校验。

完成内容：新增系统接口、插件描述、构建依赖和测试，但没有把启动文件、MoveIt 配置和硬件接口混在一个包里。

结构反思：`src/` 目前仍只放自研包，vendor 代码保持在 `vendor/` 作为只读参考。这个结构是正确的，后续 `edgepick_bringup` 可以单独承担启动编排。

补充记录：阶段 2 代码已补充适量模块注释，重点说明接口职责、单位转换、安全拒绝路径和 mock 状态策略。以后新增包也遵循同样的注释粒度。

### 阶段 3：EdgePick mock bringup

当前阶段：新增 `src/edgepick_bringup`，把 xacro、controller 配置和 launch 编排集中起来。

完成内容：提供 `edgepick_mock_control.launch.py` 和 `edgepick_moveit_mock.launch.py`，并用测试确认 xacro 插件、controller 名称和 MoveIt launch 关键连接点。

结构反思：`src/` 现在开始形成清晰分层：`edgepick_hardware` 管硬件边界，`edgepick_bringup` 管启动编排，后续 `edgepick_task` 再管抓取任务逻辑。

### 阶段 4：抓取任务状态机核心

当前阶段：新增 `src/edgepick_task`，用纯 C++ 实现抓取任务状态机。

完成内容：覆盖成功路径、失败恢复、恢复预算耗尽、超时、取消、重置和非法转移测试。

结构反思：`edgepick_task` 没有直接依赖 ROS action 或硬件接口是正确的，状态逻辑先独立可测，后续节点只做事件适配。

### 阶段 5：ROS 2 task node 与 diagnostics

当前阶段：`src/edgepick_task` 新增 ROS 2 `task_node`，`src/edgepick_bringup` 新增 `edgepick_task_mock.launch.py`。

完成内容：`edgepick_task` 现在可以通过 `/edgepick/task/event` 接收任务事件，并把当前状态、最后失败原因和诊断快照发布到 ROS topic。

结构反思：`src/` 的职责边界仍然清楚：状态机逻辑留在 `edgepick_task`，启动入口留在 `edgepick_bringup`，硬件安全门仍由 `edgepick_hardware` 独占。

验证记录：三包构建和测试通过，`edgepick_task` 暴露 `task_node` 可执行入口，`edgepick_bringup` 配置测试已覆盖 `edgepick_task_mock.launch.py` 的 topic 契约。

### 阶段 6：mock 任务闭环适配层

当前阶段：`src/edgepick_task` 新增 mock 场景脚本和 `mock_task_driver_node`，`src/edgepick_bringup` 新增闭环启动入口。

完成内容：mock 驱动根据状态机当前状态自动发布下一步事件，支持成功路径和感知/规划/执行/验证的一次失败恢复路径。

结构反思：闭环驱动作为独立节点存在是正确的；它模拟外部模块，不污染状态机本体，也不触碰 `edgepick_hardware` 的硬件安全边界。

验证记录：`mock_task_script_test` 覆盖 5 个场景，三包测试总数更新为 45 个且全部通过。

### 阶段 7：MoveIt action 适配层

当前阶段：`src/edgepick_task` 新增 MoveIt action outcome 映射和 `moveit_action_adapter_node`，`src/edgepick_bringup` 新增 `edgepick_moveit_action_mock.launch.py`。

完成内容：规划/执行阶段不再必须由 `mock_task_driver_node` 直接发布结果；适配节点会根据 `planning`/`executing` 状态发布对应 action 结果事件。

结构反思：`src/edgepick_task` 继续只做任务和 action 结果适配，不构造真实抓取目标，也不接触硬件；真实运动仍被 `edgepick_hardware` 和 mock ros2_control 边界隔离。

验证记录：`moveit_action_event_mapper_test` 覆盖 action outcome 到任务事件的映射，三包测试总数更新为 52 个且全部通过。

### 阶段 8：RGB-D 感知基础层

当前阶段：新增 `src/edgepick_perception`，提供 RGB-D 目标候选点基础链路。

完成内容：实现 CameraInfo 内参解析、深度图像采样、三维点投影、目标点 topic 发布和感知事件发布。

结构反思：新增独立 perception 包是正确的；图像/深度格式和投影数学不进入 `edgepick_task`，也不污染 MoveIt action adapter 或硬件接口。

验证记录：`rgbd_projection_test` 覆盖 7 个感知基础用例，四包测试总数更新为 61 个且全部通过。

补充验证记录：2026-08-17 根据真实终端回传，DaBai DCW2 已发布 color/depth/IR 图像、depth camera_info 和点云 topic；`/camera/depth/image_raw` 约 10 Hz，`/camera/depth/camera_info` 返回有效 640x480 内参。`edgepick_rgbd_perception.launch.py` 已能在本仓库 install 环境中启动候选点节点并等待 `/camera/depth/image_raw` 与 `/camera/depth/camera_info`。

### 阶段 9：检测框驱动的目标候选点

当前阶段：新增 `src/edgepick_interfaces`，并扩展 `src/edgepick_perception` 与 `src/edgepick_bringup`，让 mock 检测结果可以驱动 RGB-D 目标点发布。

完成内容：`TargetDetectionArray` 定义检测结果；`target_detection_selection` 负责选择目标框；`detected_target_candidate_node` 负责把检测框中心投影成三维候选点；`mock_detector_node` 负责无模型验证。

结构反思：`src/` 的分层继续保持清楚：接口包只放消息，perception 包放感知逻辑，bringup 包放启动组合，硬件包仍独占真实控制边界。

验证记录：五包构建通过，测试总数更新为 68 个且全部通过。`edgepick_perception` 现在暴露 `detected_target_candidate_node`、`mock_detector_node` 和 `rgbd_target_candidate_node` 三个可执行入口。

## 下一步目标

阶段 10：接入真实模型推理或 rosbag 回放，记录检测延迟、目标点稳定性和任务事件触发情况。
