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

### 阶段 6：mock 任务闭环适配层

当前阶段：`edgepick_task` 新增 `MockTaskScript` 和 `mock_task_driver_node`，`edgepick_bringup` 新增 `edgepick_task_closed_loop.launch.py`。现在可以由 mock 驱动节点根据 `/edgepick/task/state` 自动发布下一步 `/edgepick/task/event`，不再只依赖手动 topic pub。

完成内容：新增成功场景和四类一次恢复场景：`success`、`perception_recovery`、`planning_recovery`、`execution_recovery`、`verification_recovery`。mock 驱动按状态门控发布事件，覆盖 mock 感知、规划、执行、验证和恢复适配器的最小闭环。

结构反思：阶段 6 没有把 mock 逻辑写进 `task_node`，而是拆成独立驱动节点和纯 C++ 脚本测试。这样 task node 继续只负责状态机边界，后续接真实 MoveIt action 或感知节点时可以替换 mock 驱动，不需要重写任务核心。

验证记录：2026-08-16 构建三包通过；自动化测试为 45 tests、0 errors、0 failures、0 skipped。新增 `mock_task_script_test` 验证所有 mock 场景能把状态机推进到 `succeeded`。`edgepick_task_closed_loop.launch.py scenario:=success` 短时启动成功创建 `task_node` 和 `mock_task_driver_node`，事件自动推进到 `succeeded`；沙箱仍有 DDS UDP socket 权限警告，真实终端需继续补 rosbag/topic 证据。

### 阶段 7：MoveIt action 适配层

当前阶段：`edgepick_task` 新增 `moveit_action_adapter_node` 和 `MoveItActionEventMapper`，`edgepick_bringup` 新增 `edgepick_moveit_action_mock.launch.py`。规划和执行阶段现在由独立 action 适配节点把 MoveIt-style 结果转换为 `plan_succeeded`、`plan_failed`、`execution_succeeded`、`execution_failed` 或 `timeout`。

完成内容：新增 `moveit_msgs`/`rclcpp_action` 编译依赖、MoveGroup 与 ExecuteTrajectory action client、action outcome 到 `TaskEvent` 的纯逻辑映射、`moveit_success` 场景、mapper 单元测试和 bringup launch 契约测试。默认 `use_mock_action_results:=true`，不会构造真实 MoveIt goal，也不会触碰真实 I2C。

结构反思：阶段 7 把规划/执行结果来源从 mock 脚本中拆出来，形成 `task_node <- event topic <- MoveIt action adapter` 的边界。这样后续真正构造 MoveIt 目标时，只替换 action client 的 goal/result 处理，不需要改状态机或感知/验证驱动。

验证记录：2026-08-16 构建三包通过；自动化测试为 52 tests、0 errors、0 failures、0 skipped。`edgepick_moveit_action_mock.launch.py` 短时启动创建 `task_node`、`mock_task_driver_node` 和 `moveit_action_adapter_node`，其中 `plan_succeeded` 与 `execution_succeeded` 由 action 适配节点发布，最终进入 `succeeded`。沙箱仍有 DDS UDP socket 权限警告，真实终端需继续补 topic/rosbag 证据。

### 阶段 8：RGB-D 感知基础层

当前阶段：新增 `edgepick_perception` 包和 `edgepick_rgbd_perception.launch.py`，先把 Orbbec depth image、camera info 和目标候选点 topic 接成最小可测链路。

完成内容：实现 CameraInfo 内参解析、`16UC1`/`32FC1` 深度读取、pinhole 三维投影、`/edgepick/perception/target_point` 发布，以及可选 `/edgepick/task/event` 的 `target_acquired`/`target_lost` 事件发布。

结构反思：阶段 8 继续保持“感知基础数学”和“目标检测模型”分离。当前节点只负责从 RGB-D 数据得到相机坐标系下的候选点，不构造 MoveIt 目标、不做手眼标定、不访问真实 `/dev/i2c-7`。

验证记录：2026-08-16 四包构建和测试通过；自动化测试为 61 tests、0 errors、0 failures、0 skipped。`edgepick_rgbd_perception.launch.py --show-args` 通过，短时启动可创建 RGB-D 候选点节点并等待 `/camera/depth/image_raw` 与 `/camera/depth/camera_info`。

补充验证记录：2026-08-17 根据真实终端回传，Orbbec DaBai DCW2 已发布 `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/depth/camera_info`、`/camera/depth/points`、`/camera/depth_registered/points` 和 `/camera/ir/image_raw` 等 topic；`/camera/depth/image_raw` 约 10 Hz，`/camera/depth/camera_info` 返回 640x480 内参，`fx≈478.65`、`fy≈478.39`、`cx≈319.88`、`cy≈236.72`。阶段 8 感知 launch 已在真机终端启动，并等待同一组 depth/camera_info topic。

### 阶段 9：检测框驱动的目标候选点

当前阶段：新增 `edgepick_interfaces`、检测框选择逻辑、`mock_detector_node`、`detected_target_candidate_node` 和 `edgepick_detection_perception_mock.launch.py`。目标像素现在可以来自检测框中心，而不再只能使用固定中心点。

完成内容：定义 `TargetDetectionArray` 消息，按类别、标签和置信度选择目标检测框，复用阶段 8 的深度采样和 pinhole 投影发布 `/edgepick/perception/target_point`，并保留 `/edgepick/task/event` 的 `target_acquired`/`target_lost` 边界。

结构反思：阶段 9 没有直接把 TensorRT runtime、模型预处理和深度投影写成一个大节点，而是先固定检测结果契约和 mock detector。后续真实 YOLO/TensorRT 节点只需替换 detection publisher，不需要改任务状态机、MoveIt action 适配器或硬件安全边界。

验证记录：2026-08-17 五包构建通过；自动化测试更新为 68 tests、0 errors、0 failures、0 skipped。`ros2 pkg executables edgepick_perception` 可识别 `detected_target_candidate_node`、`mock_detector_node` 和 `rgbd_target_candidate_node`；`edgepick_detection_perception_mock.launch.py --show-args` 通过。短时启动可创建 mock detector 与 detected target candidate 两个节点；当前沙箱仍有 DDS UDP socket 权限警告，真实终端需继续补 `/edgepick/perception/detections` 和 `/edgepick/perception/target_point` echo 证据。

### 阶段 10：真实模型或 rosbag 感知量测入口

当前阶段：`edgepick_perception` 新增 `perception_metrics_node` 和纯 C++ 指标累积库，`edgepick_bringup` 新增 `edgepick_perception_metrics.launch.py`。现在可以观察真实 detector 或 rosbag 回放产生的 `/edgepick/perception/detections`、`/edgepick/perception/target_point` 和 `/edgepick/task/event`，并发布 `/edgepick/perception/metrics`。

完成内容：指标覆盖检测帧数、候选框数量、空检测帧、检测消息年龄、目标点消息年龄、目标点步长、Z 轴稳定性，以及 `target_acquired`/`target_lost` 事件计数。launch 支持外部真实模型 publisher，也支持可选 `ros2 bag play --clock` 回放入口。

结构反思：阶段 10 没有把 TensorRT runtime、rosbag 播放和深度投影塞进一个大节点，而是新增旁路 metrics 节点。这样真实模型、回放数据和后续 TF/MoveIt 目标构造都能独立替换，同时保留统一的感知质量证据。

验证记录：2026-08-19 五包构建通过；`colcon test-result --test-result-base build --all --verbose` 汇总为 74 tests、0 errors、0 failures、0 skipped。新增 `perception_metrics_test` 覆盖指标统计和摘要格式，`bringup_config_test` 覆盖阶段 10 launch 契约。

### 阶段 11：目标点基座坐标转换

当前阶段：`edgepick_perception` 新增 `target_frame_transform_node` 和纯 C++ `transform_target_point` helper，`edgepick_bringup` 新增 `edgepick_target_frame_transform.launch.py`。现在可以把 `/edgepick/perception/target_point` 从相机 frame 转换到机器人规划 frame，并发布 `/edgepick/perception/target_point_base`。

完成内容：节点通过 TF 查找源 frame 到 `base_link` 的转换，保留目标点采样时间戳，只改变坐标 frame 和坐标值。launch 暴露输入 topic、输出 topic、目标 frame 和 TF 超时参数。

结构反思：阶段 11 仍然不构造抓取姿态、不调用 MoveIt、不访问真实 `/dev/i2c-7`。它先把“相机目标点能否进入机器人基座坐标系”做成可测边界，为后续 mock MoveIt 目标构造和手眼标定验证打基础。

验证记录：2026-08-19 五包构建通过；新增 `target_frame_transform_test` 覆盖平移、Z 轴旋转和零四元数回退，`bringup_config_test` 覆盖阶段 11 launch 契约。

### 阶段 12：mock MoveIt 抓取目标构造

当前阶段：`edgepick_task` 新增 `grasp_target_builder` helper 和 `grasp_target_builder_node`，`edgepick_bringup` 新增 `edgepick_mock_grasp_target.launch.py`。现在可以把 `/edgepick/perception/target_point_base` 转换为 `/edgepick/task/pregrasp_pose` 和 `/edgepick/task/grasp_pose`。

完成内容：默认规划 frame 为 `base_link`，抓取点在目标点上方 `0.02 m`，预抓取点再上方 `0.08 m`，末端 orientation 使用 mock 阶段固定的向下姿态。helper 会拒绝空 frame、非有限坐标、负 offset 和零长度四元数。

结构反思：阶段 12 仍然不发送 MoveIt goal、不执行轨迹、不访问真实 `/dev/i2c-7`。它先把“基座坐标目标点是否能变成可规划位姿”做成可测边界，避免把目标构造、规划器和真实执行混在一起。

验证记录：2026-08-19 `edgepick_task` 与 `edgepick_bringup` 构建通过；新增 `grasp_target_builder_test` 覆盖 offset、orientation 和无效输入，`bringup_config_test` 覆盖阶段 12 launch 契约；`ros2 pkg executables edgepick_task` 可识别 `grasp_target_builder_node`，`edgepick_mock_grasp_target.launch.py --show-args` 通过。

### 阶段 13：真实硬件前系统级 mock 演练

当前阶段：`edgepick_perception` 新增 `mock_rgbd_source_node`，`edgepick_bringup` 新增 `edgepick_prehardware_mock_rehearsal.launch.py`，`edgepick_task` 新增 `system_rehearsal_success` 脚本。现在可以不接真相机、不接真实 I2C，把 mock RGB-D、检测、TF、抓取目标、metrics、任务状态机和 MoveIt action mock 串成一条 rehearsal 链。

完成内容：rehearsal launch 启动 10 个节点，目标点从 mock depth/camera_info 和 mock detection 产生，经 TF 转成 `/edgepick/perception/target_point_base`，再生成 `/edgepick/task/pregrasp_pose` 与 `/edgepick/task/grasp_pose`。任务事件流由感知节点发布一次 `target_acquired`，MoveIt action mock 发布规划/执行结果，mock verifier 发布验证成功。

结构反思：阶段 13 仍然不发送真实 MoveIt goal、不启动真实硬件、不访问 `/dev/i2c-7`。它把真实硬件前最容易混在一起的 topic、TF、事件时序和参数类型问题提前暴露在 mock 环境中。

验证记录：2026-08-19 五包构建通过；五包测试汇总为 90 tests、0 errors、0 failures、0 skipped；`edgepick_prehardware_mock_rehearsal.launch.py --show-args` 通过。短时启动创建 10 个节点并完成 `idle -> perceiving -> planning -> executing -> verifying -> succeeded`，metrics 显示 detection、target point 和 task event 计数正常。沙箱仍有 DDS UDP socket 权限警告，真实桌面终端需要按 checklist 复验 topic echo。

### 阶段 14：显式 real I2C 后端

当前阶段：`edgepick_hardware` 新增 `DofbotI2cTransport`，`edgepick_bringup` 新增 `edgepick_real_control.launch.py`。默认仍是 mock；只有显式 `use_real_i2c:=true` 时才会打开真实 I2C。

完成内容：real I2C 路径对齐厂商 `Arm_Lib` 的六舵机写帧，`MockSystemInterface` 保持同一命令网关和相同安全门。

结构反思：阶段 14 把真机启用和 mock 控制链分离成两个 launch。这样默认回归不会碰硬件，而真机验证时又不需要改代码分支。

验证记录：2026-08-20 `edgepick_hardware` 与 `edgepick_bringup` 构建通过；新增 real I2C transport、显式失败路径测试和 bringup 参数测试。

### 阶段 15：真实 DOFBOT 低速单关节验证

当前阶段：在真实 DOFBOT 上做最小硬件闭环，只验证 I2C、controller-manager、控制器状态和单关节低速小角度动作。

完成内容：`check_dofbot_i2c.py` 先做 I2C 预检，再由真实终端确认 `/dev/i2c-7`、`Arm_get_hardversion()`、`Arm_ping_servo()` 和低速关节动作。

结构反思：阶段 15 把“能否安全移动一个关节”和“完整任务链能否抓取”拆开，避免第一次真机动作时同时调试感知、规划和硬件。

验证记录：2026-08-20 新增 I2C 预检脚本和低速验证手册；脚本通过语法检查。用户真实终端已确认 `/dev/i2c-7` 可打开、`Arm_get_hardversion()` 返回 `0.20`、`Arm_ping_servo(1)` 返回 `218`；`Arm1_Joint` 低速小角度前进和回零均返回 `SUCCEEDED`。

### 阶段 16：real MoveIt on real control

当前阶段：`edgepick_bringup` 新增 `edgepick_moveit_real.launch.py`，把 `edgepick_real_control.launch.py` 与 `move_group` 组合起来，默认不启动 RViz。

完成内容：launch 继续沿用 vendor MoveIt 控制器映射，并保留 `use_real_i2c`、`i2c_device`、`i2c_address` 参数透传。

结构反思：阶段 16 让 MoveIt 直接接到真实 controller-manager，但仍不把 task/perception 混进来。这样真实规划和真实执行的问题能单独被看见。

验证记录：2026-08-21 `edgepick_bringup` 构建通过；`bringup_config_test` 现在覆盖 14 项检查；`edgepick_moveit_real.launch.py --show-args` 通过，参数包含 `publish_frequency`、`use_real_i2c`、`i2c_device`、`i2c_address` 和 `use_rviz`。

### 阶段 17：real MoveIt 最小关节验证

当前阶段：`edgepick_bringup` 新增 `edgepick_moveit_real_validation.launch.py`，在阶段 16 的 real control + MoveGroup 基础上追加一个最小关节验证节点。

完成内容：launch 暴露最小目标关节索引、角度增量、规划时间、重试次数、稳定等待和回零容差参数；节点先抓取当前关节值，再对单个关节做小幅度前进，最后回到捕获的 home 状态。

结构反思：阶段 17 不再增加新的通用启动层，而是把“MoveIt 规划是否能稳定执行”和“回零是否仍然可控”合成一个最小验证动作。

验证记录：待在真实 DOFBOT 上执行阶段 17 launch，并记录规划成功、执行成功和回零误差。

### 阶段 18：真实目标检测桥接

当前阶段：`edgepick_bringup` 新增 `edgepick_orange_detection.launch.py`，把本机 COCO 橘子 detector 和阶段 9 的 `detected_target_candidate_node` 接到同一条检测契约，同时保留厂商 YOLO 垃圾分类入口作为参考。

完成内容：橘子 launch 暴露 `image_topic`、`model_path`、`config_path`、`label_path`、`target_label`、`conf_threshold`、`max_detections`，以及后续深度投影所需的 `depth_topic` 和 `camera_info_topic`。

结构反思：bringup 这里仍只做编排。真实视觉模型和 RGB-D 投影之间通过 `/edgepick/perception/detections` 交接，目标对象必须和任务语义一致。

验证记录：2026-08-22 `edgepick_bringup` 构建和静态测试通过；`ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_orange_detection.launch.py --show-args` 成功展开新参数；`ros2 pkg executables edgepick_perception` 可见 `edgepick_coco_detector_node.py`。

## 复现命令

在 ROS 2 Humble 终端中可复现构建和测试：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
colcon build --base-paths src --packages-select edgepick_interfaces edgepick_hardware edgepick_bringup edgepick_task edgepick_perception
colcon test --base-paths src --packages-select edgepick_interfaces edgepick_hardware edgepick_bringup edgepick_task edgepick_perception
colcon test-result --test-result-base build --all --verbose
```

## 下一步目标

阶段 19：把橘子检测结果接入任务/抓取联调，并继续做真机链路观测。
