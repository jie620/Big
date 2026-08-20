# 工程文档

本目录后续记录架构决策、坐标系/手眼标定、接口契约、性能基线和故障注入实验。实验结论应包含环境、命令、原始数据位置与可复现步骤。

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 1：硬件控制边界

当前阶段：新增 ADR 0001，记录 `edgepick_hardware` 先使用 mock 传输、不连接真实 I2C 的决策。

完成内容：说明 `CommandGateway` 的职责、厂商 `Arm_Lib` 的边界，以及真实 I2C 适配器为什么必须延后。

结构反思：阶段 1 文档先把安全边界写清楚是必要的，否则后续每一层都可能绕过硬件安全门。

当时下一步：记录 mock `ros2_control` 系统接口的职责和边界。

### 阶段 2：mock ros2_control 系统接口

当前阶段：新增 ADR 0002，记录 EdgePick mock `ros2_control` 系统接口的职责、边界和后续风险。

完成内容：把“硬件安全边界”和“mock ros2_control 接口”拆成两个 ADR，便于以后回溯为什么先做 mock、为什么不直接接真实 I2C。

结构反思：文档结构现在能按阶段追踪设计选择，阶段 2 没有覆盖阶段 1，而是在 ADR 0001 之后追加 ADR 0002。

### 阶段 3：EdgePick mock bringup

当前阶段：新增 ADR 0003 和 `docs/launch/edgepick_bringup_mock_chain.md`，记录启动覆盖层和 mock 控制链路。

完成内容：说明为什么不能直接 include vendor demo launch，以及 EdgePick xacro、controller manager、MoveIt、RViz 与 MockSystemInterface 的连接关系。

结构反思：文档现在按“硬件边界 -> mock 系统接口 -> bringup 覆盖层”递进，能回溯每一层为什么存在。

验证记录：已记录 mock control 与 MoveIt mock 的短时启动结果，沙箱限制下未打开 RViz GUI，真实桌面终端仍需补充 RViz 执行截图/日志。

### 阶段 4：抓取任务状态机核心

当前阶段：新增 ADR 0004、`docs/task/grasp_state_machine.md` 和 `docs/launch/rviz_mock_verification.md`。

完成内容：记录状态机职责、恢复策略、RViz mock 真实终端验证步骤和阶段 5 的 ROS 节点接入方向。

结构反思：文档现在从硬件边界、启动覆盖层推进到任务决策层，能说明 EdgePick 为什么不是一个简单固定动作 demo。

### 阶段 5：ROS 2 task node 与 diagnostics

当前阶段：新增 ADR 0005 和 `docs/task/task_node_topics.md`，记录 task node 的 ROS topic、事件词表、失败原因和 diagnostics 输出。

完成内容：把“外部模块如何驱动状态机”和“用户如何用命令行验证状态变化”写成可复现接口文档。

结构反思：阶段 5 文档把 task node 定义为事件适配边界，而不是让它直接承担感知、规划、执行或硬件控制职责，方便后续逐层替换 mock 组件。

验证记录：新增文档与实现一起通过 36 个自动化测试；沙箱中 launch 可显示参数并启动进程，但 DDS 网络 socket 受限，真实终端仍需补充 topic pub/echo 截图或日志。

### 阶段 6：mock 任务闭环适配层

当前阶段：新增 ADR 0006 和 `docs/task/mock_task_closed_loop.md`，记录 mock 任务闭环驱动的场景、topic、启动命令和验证方式。

完成内容：把“手动发布事件”升级为“状态门控自动发布事件”，并定义成功路径、感知恢复、规划恢复、执行恢复和验证恢复五个可测试场景。

结构反思：文档现在能说明 EdgePick 的任务层如何从纯逻辑、ROS topic 边界，推进到可自动回放的 mock 闭环，为后续 MoveIt action 接入留出清楚边界。

验证记录：阶段 6 实现后自动化测试更新为 45 tests、0 errors、0 failures、0 skipped。closed-loop launch 的 `success` 场景已在短时启动中自动推进到 `succeeded`；沙箱 DDS UDP 权限警告不作为真实桌面终端失败结论。

### 阶段 7：MoveIt action 适配层

当前阶段：新增 ADR 0007 和 `docs/task/moveit_action_adapter.md`，记录 MoveGroup/ExecuteTrajectory action 结果如何转换为任务事件。

完成内容：文档明确了 Stage 7 的边界：已经具备 action client、action server 可用性检查和 mock action result，但尚未构造真实 MoveIt goal 或发送真实轨迹。

结构反思：文档现在能区分三类东西：任务状态机、mock 感知/验证驱动、MoveIt action 结果适配器。这个边界能防止后续把感知、规划和硬件控制塞进一个大节点。

验证记录：阶段 7 实现后自动化测试更新为 52 tests、0 errors、0 failures、0 skipped。`edgepick_moveit_action_mock.launch.py` 的成功路径已在短时启动中自动推进到 `succeeded`。

### 阶段 8：RGB-D 感知基础层

当前阶段：新增 ADR 0008 和 `docs/perception/rgbd_target_candidate.md`，记录 depth image、camera info、目标候选点和任务事件之间的接口。

完成内容：文档明确了当前只做中心像素/参数像素的深度采样和三维投影，尚未接入 YOLO/TensorRT 检测或手眼标定。

结构反思：文档现在把“感知基础数学”和“目标检测模型”拆开，便于后续先验证深度/内参/坐标，再叠加模型推理。

验证记录：阶段 8 实现后自动化测试更新为 61 tests、0 errors、0 failures、0 skipped。`edgepick_rgbd_perception.launch.py` 可启动 RGB-D 候选点节点并等待相机 topic。

补充验证记录：2026-08-17 根据真实终端回传，DaBai DCW2 相机 topic 已确认存在：`/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/depth/camera_info`、`/camera/depth/points`、`/camera/depth_registered/points` 和 `/camera/ir/image_raw`。`/camera/depth/image_raw` 发布频率约 10 Hz，`/camera/depth/camera_info` 返回 640x480 内参；阶段 8 launch 已能启动 `rgbd_target_candidate_node` 并等待这些 topic。

### 阶段 9：检测框驱动的目标候选点

当前阶段：新增 ADR 0009 和 `docs/perception/detection_target_candidate.md`，记录检测消息、mock detector、检测框中心像素和 RGB-D 投影之间的接口。

完成内容：文档明确当前阶段先固定 `TargetDetectionArray` 和 mock 验证链路，真实 YOLO/TensorRT 模型推理延后到阶段 10。

结构反思：文档现在能区分“检测结果契约”和“真实模型部署”。这能避免后续把 TensorRT、深度图、任务事件和硬件动作混成不可测大节点。

验证记录：阶段 9 实现后自动化测试更新为 68 tests、0 errors、0 failures、0 skipped。`edgepick_detection_perception_mock.launch.py --show-args` 通过，短时启动可创建 mock detector 与 detected target candidate 节点。

### 阶段 10：感知量测入口

当前阶段：新增 ADR 0010 和 `docs/perception/perception_metrics.md`，记录真实模型或 rosbag 回放接入后的延迟、稳定性和任务事件量测方式。

完成内容：文档明确 `perception_metrics_node` 是旁路观察者，只订阅 `/edgepick/perception/detections`、`/edgepick/perception/target_point` 和 `/edgepick/task/event`，并发布 `/edgepick/perception/metrics`。

结构反思：阶段 10 文档把“真实 detector 输入”和“量测证据”拆开，避免后续把模型部署、深度投影、TF 和机械臂执行调试混成一个不可定位的问题。

验证记录：阶段 10 实现后自动化测试更新为 74 tests、0 errors、0 failures、0 skipped。`edgepick_perception_metrics.launch.py` 已进入 bringup 配置测试。

### 阶段 11：目标点 TF 转换边界

当前阶段：新增 ADR 0011 和 `docs/perception/target_frame_transform.md`，记录相机坐标目标点如何转换到机器人规划坐标系。

完成内容：文档明确 `target_frame_transform_node` 只订阅 `/edgepick/perception/target_point`、读取 TF，并发布 `/edgepick/perception/target_point_base`；它不构造抓取姿态、不调用 MoveIt、不访问真实硬件。

结构反思：阶段 11 把 TF/手眼标定边界提前单独验证，避免后续把坐标错误、规划失败和真实执行风险混在一起。

验证记录：阶段 11 实现后新增目标点转换单元测试和 bringup launch 契约测试。

### 阶段 12：mock MoveIt 目标构造边界

当前阶段：新增 ADR 0012 和 `docs/task/grasp_target_builder.md`，记录基座坐标目标点如何转换为抓取/预抓取规划目标。

完成内容：文档明确 `grasp_target_builder_node` 只订阅 `/edgepick/perception/target_point_base`，发布 `/edgepick/task/pregrasp_pose` 和 `/edgepick/task/grasp_pose`；它不发送 MoveIt goal、不执行轨迹、不访问真实硬件。

结构反思：阶段 12 把目标位姿构造独立出来，后续如果 MoveIt 规划失败，可以先判断问题来自目标点、抓取 offset/orientation 还是规划器本身。

验证记录：阶段 12 实现后新增抓取目标构造单元测试和 bringup launch 契约测试。

### 阶段 13：真实硬件前系统级 mock 演练

当前阶段：新增 ADR 0013 和 `docs/launch/prehardware_mock_rehearsal.md`，记录真实硬件前系统级 mock rehearsal 和 readiness checklist。

完成内容：文档明确 rehearsal launch 串起 mock RGB-D、mock detector、TF、目标构造、metrics、task node、mock driver 和 MoveIt action mock；该链路不启动真实硬件。

结构反思：阶段 13 把接入真实硬件前的最后一次系统检查写成可复现命令，而不是靠临时手动 topic pub。

验证记录：2026-08-19 五包构建通过；五包测试汇总为 90 tests、0 errors、0 failures、0 skipped；短时启动已跑到 `succeeded`，并记录需要在真实终端复验的 topic echo checklist。

### 阶段 14：显式 real I2C 后端

当前阶段：新增 ADR 0014 和 `docs/launch/real_control.md`，记录真实 I2C 后端如何在 `edgepick_hardware` 和 `edgepick_bringup` 中显式启用。

完成内容：文档明确 real I2C 入口不改变默认 mock 行为；`use_real_i2c:=true` 才会触碰 `/dev/i2c-7`，默认路径仍保持 mock-safe。

结构反思：阶段 14 继续沿用“默认安全、显式切换”的原则，避免把真机接入写成隐式副作用。

验证记录：2026-08-20 阶段 14 代码与配置测试通过，`edgepick_real_control.launch.py --show-args` 可解析参数。

### 阶段 15：真实 DOFBOT 低速单关节验证

当前阶段：新增 ADR 0015、`docs/launch/real_low_speed_validation.md` 和 `scripts/check_dofbot_i2c.py`，记录真机低速验证流程和 I2C 预检方式。

完成内容：文档明确阶段 15 只做最小硬件闭环，不启动感知、任务状态机或完整抓取流程；真实运动必须由操作者确认安全后手动执行。

结构反思：阶段 15 把“能否安全移动一个关节”和“完整任务链能否抓取”拆开，避免第一次真机动作时同时调试感知、规划和硬件。

验证记录：2026-08-20 新增 I2C 预检脚本和低速验证手册；脚本通过语法检查。用户真实终端已确认 `/dev/i2c-7` 可打开、`Arm_get_hardversion()` 返回 `0.20`、`Arm_ping_servo(1)` 返回 `218`。`i2cdetect` 未显示 `0x15`，因此阶段 15 以厂商 `Arm_Lib` 读探针作为 I2C 接入判据。随后用户真实终端完成 real control 启动、`Arm1_Joint` 低速小角度前进和回零，两个 action 目标均返回 `SUCCEEDED`。第一次启动时机械臂会立即进入控制器激活后的姿态，阶段 15 记录为已完成。

## 下一步目标

阶段 16：根据阶段 15 的真机结果修正关节方向、零点、限位和低速控制参数。
