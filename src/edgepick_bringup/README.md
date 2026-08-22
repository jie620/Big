# edgepick_bringup

EdgePick 的启动编排包。阶段 3 只启动 mock 控制链，不访问真实 `/dev/i2c-7`，也不会移动机械臂。

## 结构

- `urdf/edgepick_dofbot.urdf.xacro`：复用厂商 DOFBOT URDF 几何，并接入 EdgePick ros2_control 配置。
- `urdf/edgepick_dofbot.ros2_control.xacro`：把硬件插件替换为 `edgepick_hardware/MockSystemInterface`。
- `config/edgepick_ros2_controllers.yaml`：controller manager、轨迹控制器、夹爪控制器和 joint state broadcaster 配置。
- `config/initial_positions.yaml`：mock 控制链的初始关节位置。
- `launch/edgepick_mock_control.launch.py`：只启动 robot_state_publisher、controller manager 和控制器 spawner。
- `launch/edgepick_real_control.launch.py`：显式启用真实 I2C 的 controller-manager 启动入口。
- `launch/edgepick_moveit_mock.launch.py`：启动 MoveIt、RViz 和 EdgePick mock ros2_control 链路。
- `launch/edgepick_moveit_real.launch.py`：启动 real control + MoveGroup 的阶段 16 联动入口。
- `launch/edgepick_moveit_real_validation.launch.py`：启动 real MoveIt 并执行阶段 17 的最小关节验证。
- `launch/edgepick_moveit_action_mock.launch.py`：启动 task node、mock 感知/验证驱动和 MoveIt action 适配器。
- `launch/edgepick_detection_perception_mock.launch.py`：启动 mock detector 和检测框驱动的 RGB-D 候选点节点。
- `launch/edgepick_perception_metrics.launch.py`：启动检测链路量测入口，可选 rosbag 回放。
- `launch/edgepick_mock_grasp_target.launch.py`：启动 mock-safe 抓取/预抓取目标构造节点。
- `launch/edgepick_prehardware_mock_rehearsal.launch.py`：启动真实硬件前系统级 mock rehearsal。
- `launch/edgepick_rgbd_perception.launch.py`：启动 RGB-D 目标候选点基础节点。
- `launch/edgepick_target_frame_transform.launch.py`：启动目标点 TF 转换节点。
- `launch/edgepick_task_mock.launch.py`：启动 `edgepick_task/task_node`，用于手动或 mock 节点发布任务事件。
- `launch/edgepick_task_closed_loop.launch.py`：同时启动 task node 和 mock 任务驱动节点，自动跑任务事件闭环。
- `test/test_bringup_config.py`：验证 xacro、controller yaml 和 MoveIt launch 关键连接点。

## 使用

先 source ROS 2、厂商 workspace 和本仓库 install：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
```

只验证 controller manager 到 EdgePick mock 硬件接口：

```bash
ros2 launch edgepick_bringup edgepick_mock_control.launch.py
```

验证 MoveIt/RViz 到 EdgePick mock 硬件接口：

```bash
ros2 launch edgepick_bringup edgepick_moveit_mock.launch.py
```

验证 real MoveIt 最小关节动作和回零：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_moveit_real_validation.launch.py
```

只验证 task node 的事件入口和 diagnostics 输出：

```bash
ros2 launch edgepick_bringup edgepick_task_mock.launch.py
```

验证自动 mock 任务闭环：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_task_closed_loop.launch.py scenario:=success
```

验证 MoveIt action mock 适配链：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_moveit_action_mock.launch.py
```

验证 RGB-D 感知基础节点：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_rgbd_perception.launch.py
```

验证检测框驱动的 RGB-D 候选点链路：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_detection_perception_mock.launch.py
```

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 3：EdgePick mock bringup

当前阶段：新增 `edgepick_bringup`，把 xacro、controller yaml 和 launch 编排集中起来，使 MoveIt/RViz 可以通过 `edgepick_hardware/MockSystemInterface` 运行。

完成内容：提供 mock control launch、MoveIt mock launch、EdgePick xacro 覆盖层、controller 配置和配置测试。

结构反思：bringup 独立成包是正确的；`edgepick_hardware` 保持硬件边界，`edgepick_bringup` 只负责启动编排，vendor MoveIt/URDF 仍作为外部运行资源复用。

验证记录：mock control launch 已确认 `EdgePickMockSystem` 和三个控制器能加载、配置并激活；MoveIt mock launch 在 `use_rviz:=false` 下确认 `move_group` 能加载 robot model 并监听 `joint_states`。沙箱中的 DDS UDP socket 权限警告不代表真实终端失败。

### 阶段 5：task node mock launch

当前阶段：`edgepick_bringup` 新增 `edgepick_task_mock.launch.py`，作为任务状态机 ROS 节点的最小启动入口。

完成内容：launch 传入恢复次数参数和 topic 名称，并由配置测试确认 `edgepick_task/task_node` 与 `/edgepick/task/*`、`/diagnostics` 的接口契约。

结构反思：bringup 继续只做启动编排，不承载状态机逻辑，也不接触真实硬件；这能保持“逻辑包”和“启动包”的职责分离。

验证记录：`bringup_config_test` 当前覆盖 4 项配置检查，其中包含 `edgepick_task_mock.launch.py` 的节点和 topic 契约。

### 阶段 6：mock 任务闭环 launch

当前阶段：`edgepick_bringup` 新增 `edgepick_task_closed_loop.launch.py`，一键启动 `task_node` 和 `mock_task_driver_node`。

完成内容：launch 暴露 `scenario`、`event_period_ms`、`initial_delay_ms` 和 `max_recovery_attempts` 参数，默认跑 `success` 场景。

结构反思：组合 launch 属于启动编排层，仍不把 mock 场景逻辑写进 bringup；bringup 只负责把节点和 topic 契约接起来。

验证记录：`bringup_config_test` 当前覆盖 5 项配置检查，新增闭环 launch 的节点和 topic 契约检查。`edgepick_task_closed_loop.launch.py scenario:=success` 短时启动可创建两个节点并完成 `succeeded` 路径；沙箱 DDS UDP 权限警告需在真实终端复验。

### 阶段 7：MoveIt action mock launch

当前阶段：`edgepick_bringup` 新增 `edgepick_moveit_action_mock.launch.py`，一键启动 task node、mock 感知/验证驱动和 MoveIt action 适配器。

完成内容：launch 使用 `moveit_success` 场景，让 mock 驱动只发布 `start_requested`、`target_acquired` 和 `verification_succeeded`；规划/执行结果由 `moveit_action_adapter_node` 发布。

结构反思：bringup 仍只负责组合节点和参数，不承载 action 映射逻辑。MoveIt action adapter 位于 `edgepick_task`，便于后续替换成真实 MoveIt goal/result 处理。

验证记录：`bringup_config_test` 当前覆盖 6 项配置检查，新增 MoveIt action mock launch 检查；短时启动可创建三个节点并完成 `succeeded` 路径。

### 阶段 8：RGB-D 感知基础 launch

当前阶段：`edgepick_bringup` 新增 `edgepick_rgbd_perception.launch.py`，启动 `edgepick_perception/rgbd_target_candidate_node`。

完成内容：launch 暴露 depth topic、camera info topic、目标像素、深度范围和事件发布开关，默认等待 `/camera/depth/image_raw` 与 `/camera/depth/camera_info`。

结构反思：bringup 继续只做启动编排，感知算法留在 `edgepick_perception`，任务状态仍由 `edgepick_task` 管理。

验证记录：`bringup_config_test` 当前覆盖 7 项配置检查；短时启动可创建 RGB-D 候选点节点并等待相机 topic。

补充验证记录：2026-08-17 根据真实终端回传，DaBai DCW2 相机驱动已发布 `/camera/depth/image_raw` 与 `/camera/depth/camera_info`，depth image 约 10 Hz。使用 `ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_rgbd_perception.launch.py` 可启动 `rgbd_target_candidate_node`，节点等待的默认 topic 与相机实际 topic 一致。

### 阶段 9：检测框驱动的感知 launch

当前阶段：`edgepick_bringup` 新增 `edgepick_detection_perception_mock.launch.py`，把 `mock_detector_node` 与 `detected_target_candidate_node` 接到同一条检测 topic。

完成内容：launch 暴露目标类别、标签、置信度阈值、mock 检测框中心、depth topic 和 camera info topic。默认仍只读相机数据，不访问真实 `/dev/i2c-7`，不会移动机械臂。

结构反思：bringup 只负责节点组合，不把检测选择逻辑写进 launch；后续真实 TensorRT detector 可以替换 `mock_detector_node`，保留 `/edgepick/perception/detections` 和目标点输出契约。

验证记录：`bringup_config_test` 当前覆盖 8 项配置检查，新增检测感知 mock launch 检查。`edgepick_detection_perception_mock.launch.py --show-args` 通过；短时启动可创建 `mock_detector_node` 和 `detected_target_candidate_node`。沙箱 DDS UDP socket 权限警告不作为真实桌面终端失败结论。

### 阶段 10：感知量测 launch

当前阶段：`edgepick_bringup` 新增 `edgepick_perception_metrics.launch.py`，把检测框驱动目标点节点和指标节点组合起来，并提供可选 rosbag 回放入口。

完成内容：launch 暴露 `play_bag`、`bag_path`、`use_sim_time`、`run_candidate_node`、目标筛选参数、depth/camera_info topic 和 `metrics_period_ms`。默认只观察感知链路，不启动 mock detector，不访问真实 `/dev/i2c-7`，不会移动机械臂。

结构反思：bringup 只负责组合真实 detector 或 rosbag 与 metrics 观察者，不把模型推理或指标计算写进 launch。后续真实 TensorRT 节点仍可以独立替换 detection publisher。

验证记录：`bringup_config_test` 当前覆盖 9 项配置检查，新增阶段 10 metrics launch 检查。五包测试结果为 74 tests、0 errors、0 failures、0 skipped。

### 阶段 11：目标点 TF 转换 launch

当前阶段：`edgepick_bringup` 新增 `edgepick_target_frame_transform.launch.py`，启动 `target_frame_transform_node`。

完成内容：launch 暴露 `input_topic`、`output_topic`、`target_frame` 和 `transform_timeout_ms`。默认把 `/edgepick/perception/target_point` 转换为 `/edgepick/perception/target_point_base`，目标 frame 为 `base_link`。

结构反思：bringup 继续只组合节点和参数，不发布临时标定值，也不启动真实硬件。TF 来源必须由 robot_state_publisher、static transform 或后续标定节点明确提供。

验证记录：`bringup_config_test` 当前覆盖 10 项配置检查，新增阶段 11 target frame transform launch 检查。

### 阶段 12：mock 抓取目标构造 launch

当前阶段：`edgepick_bringup` 新增 `edgepick_mock_grasp_target.launch.py`，启动 `grasp_target_builder_node`。

完成内容：launch 暴露 `target_frame`、`pregrasp_offset_m` 和 `grasp_z_offset_m`。默认订阅 `/edgepick/perception/target_point_base`，发布 `/edgepick/task/pregrasp_pose` 和 `/edgepick/task/grasp_pose`。

结构反思：bringup 继续只组合节点和参数，不生成 MoveIt goal，也不启动真实硬件。抓取 offset/orientation 先作为 mock-safe 参数固定，后续系统演练再决定是否进入 MoveIt goal 构造。

验证记录：`bringup_config_test` 当前覆盖 11 项配置检查，新增阶段 12 mock grasp target launch 检查。

### 阶段 13：pre-hardware mock rehearsal launch

当前阶段：`edgepick_bringup` 新增 `edgepick_prehardware_mock_rehearsal.launch.py`，一键启动真实硬件前系统级 mock chain。

完成内容：launch 组合 mock RGB-D、mock detector、检测候选点、static TF、目标点 TF 转换、抓取目标构造、metrics、task node、系统 rehearsal driver 和 MoveIt action mock adapter。

结构反思：bringup 仍只做启动编排。真实 I2C 后端没有被加入默认 launch；阶段 13 只证明 topic/TF/事件时序在 mock 环境中可以闭环。

验证记录：`bringup_config_test` 当前覆盖 12 项配置检查；2026-08-19 五包测试汇总为 90 tests、0 errors、0 failures、0 skipped；短时启动创建 10 个节点并跑到 `succeeded`。

### 阶段 14：显式 real control launch

当前阶段：`edgepick_bringup` 新增 `edgepick_real_control.launch.py`，把 `use_real_i2c`、`i2c_device` 和 `i2c_address` 参数显式透传到 ros2_control 的硬件描述。

完成内容：默认仍走 `MockSystemInterface`；只有在 `use_real_i2c:=true` 时，`edgepick_hardware` 才会创建真实 I2C 传输适配器并尝试打开 `/dev/i2c-7`。

结构反思：阶段 14 把真机启用和 mock 控制链分离成两个 launch。这样默认回归不会碰硬件，而真机验证时又不需要改代码分支。

验证记录：2026-08-20 `edgepick_bringup` 配置测试覆盖了 `edgepick_real_control.launch.py` 的参数注入；`edgepick_real_control.launch.py --show-args` 通过。

### 阶段 16：real MoveIt on real control

当前阶段：新增 `edgepick_moveit_real.launch.py`，把 `edgepick_real_control.launch.py` 与 `move_group` 组合起来，默认不启动 RViz。

完成内容：launch 继续沿用 vendor MoveIt 控制器映射，并保留 `use_real_i2c`、`i2c_device`、`i2c_address` 参数透传。

结构反思：阶段 16 让 MoveIt 直接接到真实 controller-manager，但仍不把 task/perception 混进来。这样真实规划和真实执行的问题能单独被看见。

验证记录：2026-08-21 `edgepick_bringup` 构建通过；`bringup_config_test` 现在覆盖 14 项检查；`edgepick_moveit_real.launch.py --show-args` 通过，参数包含 `publish_frequency`、`use_real_i2c`、`i2c_device`、`i2c_address` 和 `use_rviz`。真实 MoveIt 最小目标执行待进入阶段 17。

### 阶段 17：real MoveIt 最小关节验证

当前阶段：`edgepick_bringup` 新增 `edgepick_moveit_real_validation.launch.py`，在阶段 16 的 real control + MoveGroup 基础上追加一个最小关节验证节点。

完成内容：launch 暴露最小目标关节索引、角度增量、规划时间、重试次数、稳定等待和回零容差参数；节点先抓取当前关节值，再对单个关节做小幅度前进，最后回到捕获的 home 状态。

结构反思：阶段 17 不再增加新的通用启动层，而是把“MoveIt 规划是否能稳定执行”和“回零是否仍然可控”合成一个最小验证动作。这样若失败，能直接区分是规划、执行还是回零误差问题。

验证记录：待在真实 DOFBOT 上执行阶段 17 launch，并记录规划成功、执行成功和回零误差。

## 下一步目标

阶段 17：从 MoveIt 发送最小关节目标并确认规划、执行和回零稳定完成。
