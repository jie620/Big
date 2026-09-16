# 安全 VLA 系统

项目统一位于 `/home/jetson/Codex_Projects/Big`。DOFBOT 的六维动作是五个机械臂关节加一个夹爪关节，单位为 ROS 弧度。

## 执行链

```
RGB + JointState → SmolVLA → /edgepick/vla/proposal
                                   ↓
注册 RGB-D → 目标/障碍/目的地复核 → Safety Gate
                                   ↓
                  MoveIt 规划、轨迹碰撞检查、执行中取消
                                   ↓
           ros2_control → EdgePick CommandGateway → I2C
```

策略节点只能发布动作建议，没有控制器、I2C 或电机接口。旧的 `edgepick_vla/smolvla_node` 也转入该入口，旧 VLA launch 默认改为 Mock。

安全门默认关闭。急停和急停心跳、目标、关节、场景、推理心跳失效都会阻断执行；故障后必须显式重新 arm。模型动作校验包括关节顺序、有限数、上下限、最大单步跳变、时间戳、有效期和序列号。一次仅接收一个动作，执行后采集新观测，不复用旧 action chunk。

MoveIt 为每个机械臂/夹爪目标规划并检查插值路径。目标区域必须有新的深度覆盖且没有障碍，才允许释放；释放后使用新目标观测核实位置。规划失败最多重试两次，运动故障不自动重试。携带物按球形碰撞体保守建模，半径可标定。视觉接近检查不等同于力传感器的抓持证明。

## 构建和无模型验收

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/local_setup.bash
colcon build --packages-select edgepick_interfaces edgepick_hardware edgepick_perception edgepick_task edgepick_bringup edgepick_safe edgepick_vla
source install/local_setup.bash
colcon test --packages-select edgepick_safe
colcon test-result --verbose
bash scripts/run_mock_checks.sh
```

`run_mock_checks.sh` 固定使用隔离 ROS 域 186 和 Mock 硬件。运行记录默认保存在 `/tmp/edgepick_safe_acceptance`，可以通过 `EDGEPICK_TEST_OUTPUT` 改到项目的 `artifacts/`。

交互式 Mock：

```bash
ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 ros2 launch edgepick_safe safe_system.launch.py mode:=mock auto_arm:=true
```

可用 `scenario:=estop|target_loss|depth_timeout|policy_timeout|nan|jump|replay|collision|destination_blocked` 注入异常。Mock 注入器只适用于仿真，不应与真机控制器共用 ROS 域。

## 交付模型

需要完整的 SmolVLA 预训练目录（`config.json`、`model.safetensors`），以及训练时使用的本地 VLM/tokenizer 资源。不能仅凭裸权重猜测单位、关节顺序和图像视角。支持契约：单张 RGB `observation.image`、六维 `observation.state`、六维绝对关节角 `action`；位置增量、末端位姿或其他机器人模型会被拒绝。

```bash
cd /home/jetson/Codex_Projects/Big
# 新机器创建依赖环境；本机已准备的环境可直接使用。
bash scripts/setup_vla_env.sh
.venv-vla/bin/python scripts/prepare_checkpoint.py /path/to/pretrained_model \
  --absolute-ros-radians --vlm-assets /path/to/local_vlm_assets
```

打包脚本复制模型和 VLM 资源到 `models/vla/`，不覆盖已有模型；`--absolute-ros-radians` 是对训练数据语义的明确声明。运行时严格加载权重，离线模式下不会偷偷下载或降级为随机模型。训练图像视角、颜色、动作单位和关节零点须与部署一致。

## MuJoCo

```bash
source install/local_setup.bash
.venv-vla/bin/python simulation/check_scene.py
ros2 launch edgepick_safe safe_system.launch.py mode:=mujoco \
  checkpoint:="$PWD/models/vla" render:=true registered_depth_confirmed:=true \
  image_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

MuJoCo 后端实现与 MoveIt 配置相同的 FollowJointTrajectory/GripperCommand 接口。VLA 仍走同一个安全门。场景含机械臂、橘子和路线障碍；相机发布 RGB、米制深度、内参与 TF。底层物理模型使用原项目的近距抓持辅助，模拟 attachment/release 不是完整的接触抓取动力学验证。`simulation/check_scene.py` 仅验证关节/执行器映射、障碍和数值稳定性。

启动后先保证持续的急停输入和新鲜观测，再在另一终端 arm；真机应使用实际急停接口发布该状态。

```bash
# 仅仿真：模拟健康的急停输入，保持运行。
ros2 topic pub -r 10 /edgepick/safe/estop std_msgs/msg/Bool '{data: false}'
# 另一个终端：满足条件后才能解锁。
ros2 service call /edgepick/safe/arm std_srvs/srv/Trigger '{}'
```

模型 rollout 不会自动启动或声称成功；可用 `record_safe_run.py --scope mujoco --model models/vla --output artifacts/run_001.json` 保存终态、事件和超时，所有失败/超时都应计入成功率分母。

## 真机接入

先独立启动 Orbbec 驱动，并确认是注册到 RGB 的深度流。相同分辨率不能证明注册。驱动版本的 topic 名称不同，须按实际接口传参；节点核对 RGB/depth/CameraInfo 的帧、尺寸、时间差和内参。相机到 `DaBai_DCW2_Link` 的外参必须实测。

```bash
ros2 launch orbbec_camera dabai_dcw2.launch.py
# 以下外参、frame、topic 必须替换为实测值；未确认标定时启动会拒绝。
ros2 launch edgepick_safe safe_system.launch.py mode:=real \
  checkpoint:="$PWD/models/vla" \
  calibration_confirmed:=true registered_depth_confirmed:=true \
  camera_transform:='X,Y,Z,ROLL,PITCH,YAW' camera_frame:=camera_link \
  depth_topic:=/实际注册深度话题 camera_info_topic:=/实际彩色内参话题
```

`mode:=real` 才启用 I2C。启动不会自动 arm，也不会自动执行启动姿态。必须保证实测关节姿态与模型训练起始分布相容；超出 `max_joint_step` 的建议会被拒绝，不通过裁剪制造动作。目的地、物体半径、支撑面高度和时限在 `src/edgepick_safe/config/safe.yaml` 中配置。

深度障碍使用保守体素，机器人自身点依据碰撞几何过滤；自滤波有 15 mm 容差，体素默认 25 mm。遮挡区不等于自由空间：仅对当前观测范围内的障碍负责，真实部署须验证视场覆盖、制动距离和盲区。软件取消无法撤回已送达伺服板的指令，独立硬件急停仍是必需的物理保护。ROS 域是假定受信任的控制网络；这些 topic 不是对恶意 ROS 节点的权限隔离。

## 数据、训练与感知优化

- `simulation/collect_vla_dataset.py`：原有仿真数据采集入口；不访问 I2C。
- `simulation/convert_mujoco_to_lerobot.py`：RGB、文本、状态、动作转换。
- `scripts/train_vla.sh BASE_CHECKPOINT DATASET LOCAL_VLM_ASSETS`：项目内保存训练输出。
- `scripts/compress_detector.py TEACHER --data DATA.yaml --output artifacts/compression`：依赖图传播的结构化通道剪枝和检测头蒸馏。只剪枝独立卷积层，保留 split/attention 内部形状约束与检测头输出语义。
- `scripts/export_detector.py WEIGHTS --format onnx --precision fp32`；TensorRT 用 `--format engine --precision fp16`，INT8 必须提供代表性标定数据 `--data DATA.yaml`。
- `scripts/benchmark_detector.py MODEL --data DATA.yaml --image SAMPLE.jpg --output artifacts/detector.json`：mAP50/mAP50-95、文件体积、端到端延迟中位数/P95、PyTorch 显存峰值。TensorRT 总内存需另采 `tegrastats`。

默认 `models/yolo/orange.pt` 是原项目的橘子检测基线，尚未证明真机精度。厂商 `best.engine` 是垃圾分类模型，无橘子类别，未复用。`mAP 0.93→0.85`、`体积下降60%+`、`仿真90%+`、`真机60%+` 均不是本次代码交付的测量结果。剪枝 smoke check 的参数下降与最终文件体积、mAP 是不同指标。

## 验证边界

本次验证包括 C++ 编译、门控/模型契约/目的地深度检查、真实 ROS Mock 规划执行和故障注入、蒸馏梯度、剪枝后前向、MuJoCo 数值步进。最终模型尚未提供，完整 VLA rollout、真实抓取、外参/深度注册和性能指标均需后续实测。未经最终验收不能把本代码称为已完成真机部署。
