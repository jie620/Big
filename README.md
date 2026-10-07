# EdgePick

EdgePick 是面向 Jetson Orin Nano 的六自由度机械臂方块抓取系统。当前软件路线固定为：

```text
RGB-D + JointState -> SmolVLA proposal -> Safety Gate -> MoveIt -> ros2_control/I2C
                                      \-> 静态障碍场景、目的地复核和执行取消
```

系统只保留一个总启动入口；`mode` 选择硬件边界：

```bash
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=mock
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=mujoco ...
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=real ...
```

`mock` 不接相机和机械臂，适合回归和故障注入；`mujoco` 启动 MuJoCo 关节、RGB-D、模型感知和同一个 Safety Gate；`real` 是唯一允许启用 I2C 的真机路线。其它 launch 文件只用于 mock 单元链路或感知诊断，不是新的真机入口。

## 目录

```text
src/edgepick_interfaces/   ROS 消息契约
src/edgepick_hardware/     ros2_control、MockSystemInterface 和 I2C 后端
src/edgepick_perception/   RGB-D、YOLO 检测和投影诊断组件
src/edgepick_task/         任务状态机和 mock rehearsal 适配器
src/edgepick_safe/         SmolVLA 输入、场景更新和 Safety Gate
src/edgepick_bringup/      唯一总入口、URDF 和 controller 配置
simulation/                MuJoCo 场景及离线检查
scripts/                   构建、训练、检查和记录脚本
docs/                      契约、验收边界和操作说明
vendor/                    固定版本的外部依赖，只作本地来源
```

## 构建

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/local_setup.bash
colcon build --base-paths src --symlink-install \
  --packages-select edgepick_interfaces edgepick_hardware edgepick_perception \
  edgepick_task edgepick_bringup edgepick_safe
source install/local_setup.bash
```

无硬件回归：

```bash
bash scripts/run_mock_checks.sh
```

## 三条运行路线

### Mock

```bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 \
ros2 launch edgepick_bringup edgepick_system.launch.py \
  mode:=mock auto_arm:=true scenario:=normal
```

`scenario` 可选 `estop`、`target_loss`、`depth_timeout`、`policy_timeout`、`nan`、`jump`、`replay`、`collision` 和 `destination_blocked`。Mock 只验证门控、规划调用、碰撞场景更新和故障闭锁，不代表真机动作。

### MuJoCo

MuJoCo 模式要求真实存在的 SmolVLA checkpoint、YOLO 方块模型和注册深度确认；否则总入口会在启动前拒绝。

默认场景是 3 cm 黄色方块、两个 3 x 3 x 6 cm 的绿色/蓝色静态障碍和 5 cm 红色放置区。释放后必须有新的目标观测，且方块 footprint 覆盖放置区的比例达到 50%（比例以方块 footprint 为分母）。

```bash
CHECKPOINT="$PWD/models/vla"
YOLO_MODEL="$PWD/models/yolo/cube.engine"
ros2 launch edgepick_bringup edgepick_system.launch.py \
  mode:=mujoco checkpoint:="$CHECKPOINT" yolo_model:="$YOLO_MODEL" \
  registered_depth_confirmed:=true render:=true
```

checkpoint 至少要有 `config.json` 和 `model.safetensors`，并且契约必须是 RGB、注册深度、六维 ROS 弧度绝对关节动作，`chunk_size=10`、`n_action_steps=1`。运行时每次只执行第一个安全动作，然后重新采集观测。

### 真机

先确认 Orbbec 发布的是注册到彩色图像的深度流，再运行唯一真机入口：

```bash
ros2 launch orbbec_camera dabai_dcw2.launch.py
ros2 launch edgepick_bringup edgepick_system.launch.py \
  mode:=real \
  checkpoint:="$PWD/models/vla" \
  yolo_model:="$PWD/models/yolo/cube.engine" \
  calibration_confirmed:=true registered_depth_confirmed:=true \
  camera_transform:='X,Y,Z,ROLL,PITCH,YAW' \
  camera_frame:=camera_color_optical_frame \
  i2c_device:=/dev/i2c-7 i2c_address:=0x15
```

`camera_transform` 必须是实测的 `DaBai_DCW2_Link -> camera_frame` 外参；topic、分辨率、frame 和时间同步必须按当前驱动实测值替换。总入口不会自动 arm，独立硬件急停仍然是必需的。软件取消不能撤回已经送达伺服板的命令。

策略和感知使用最新帧单槽队列，推理不会阻塞 ROS 图像回调，过期结果会在发布前丢弃。默认策略 FP32 保持 checkpoint 数值一致；Jetson 上可在确认动作和延迟后使用 `use_amp:=true`，检测器默认 `detector_device:=auto detector_half:=true`。先用 `python3 scripts/profile_ros_runtime.py --duration 60 --output artifacts/runtime_profile.json` 检查话题频率和时间戳新鲜度。

## 训练和模型

```bash
bash scripts/setup_vla_env.sh
bash scripts/train_vla.sh BASE_CHECKPOINT DATASET LOCAL_VLM_ASSETS
```

训练数据和转换脚本位于 `/home/jetson/Codex_Projects/Edge_AI/mujoco`，本仓库只保存运行契约、checkpoint 打包和部署代码。检测器的剪枝、蒸馏、ONNX/TensorRT 导出和基准测试使用 `scripts/compress_detector.py`、`scripts/export_detector.py` 和 `scripts/benchmark_detector.py`。

## 验收边界

自动化测试可以证明编译、消息契约、Safety Gate、场景更新、MuJoCo 数值步进和 Mock 故障路径。它不能证明最终模型的 rollout、相机外参和深度注册、Jetson 延迟、真实抓取或真实放置成功率；这些必须用带事件、视频、关节反馈和运行日志的独立实测记录确认。

详细契约见 [`docs/SAFE_VLA.md`](docs/SAFE_VLA.md)。
