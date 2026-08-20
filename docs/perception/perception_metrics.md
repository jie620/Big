# Perception Metrics

## 当前阶段

阶段 10：EdgePick 已具备真实模型或 rosbag 回放的感知量测入口。

`perception_metrics_node` 订阅检测结果、三维目标点和任务事件，定期发布 `/edgepick/perception/metrics`。它只记录链路质量，不改变检测选择、不构造 MoveIt 目标，也不访问真实机械臂。

## Topic

| 方向 | Topic | 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/edgepick/perception/detections` | `edgepick_interfaces/msg/TargetDetectionArray` | 真实 detector、mock detector 或 rosbag 回放的检测帧 |
| 输入 | `/edgepick/perception/target_point` | `geometry_msgs/msg/PointStamped` | RGB-D 投影后的相机坐标系候选点 |
| 输入 | `/edgepick/task/event` | `std_msgs/msg/String` | `target_acquired`、`target_lost` 等任务事件 |
| 输出 | `/edgepick/perception/metrics` | `std_msgs/msg/String` | 延迟、目标点稳定性和事件计数摘要 |

## 启动

使用真实 detector 或外部 rosbag publisher 时：

```bash
cd /home/jetson/Codex_Projects/Big
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_perception_metrics.launch.py
```

使用 rosbag 回放时：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_perception_metrics.launch.py \
  play_bag:=true \
  bag_path:=/path/to/edgepick_perception_bag \
  use_sim_time:=true
```

如果外部链路已经发布 `/edgepick/perception/target_point`，可以只运行指标节点：

```bash
ROS_LOG_DIR=/tmp/edgepick_ros_logs ros2 launch edgepick_bringup edgepick_perception_metrics.launch.py \
  run_candidate_node:=false
```

常用参数：

```text
depth_topic:=/camera/depth/image_raw
camera_info_topic:=/camera/depth/camera_info
target_class_id:=1
target_label:=target
min_detection_score:=0.50
max_detection_age_ms:=500
metrics_period_ms:=5000
```

## 指标

`/edgepick/perception/metrics` 输出一行文本，字段稳定，适合直接写入日志：

```text
detections=120 candidates=120 empty_detections=0 detection_latency_ms_mean=12.3 detection_score_mean=0.872 target_points=118 target_latency_ms_mean=18.6 target_step_m_mean=0.0042 target_step_m_stddev=0.0011 target_z_m_mean=0.4310 target_z_m_stddev=0.0030 target_acquired_events=1 target_lost_events=0 other_task_events=0
```

重点观察：

- `detections` 和 `candidates` 是否持续增长。
- `empty_detections` 是否频繁出现。
- `detection_latency_ms_mean` 和 `target_latency_ms_mean` 是否随回放或模型负载变差。
- `target_step_m_mean` 与 `target_z_m_stddev` 是否显示目标点抖动过大。
- `target_acquired_events` 与 `target_lost_events` 是否符合预期。

## 验证

```bash
ros2 topic echo /edgepick/perception/metrics
ros2 topic echo /edgepick/perception/detections --once
ros2 topic echo /edgepick/perception/target_point --once
```

2026-08-19 代码侧验证：五包构建通过；`colcon test-result --test-result-base build --all --verbose` 汇总为 74 tests、0 errors、0 failures、0 skipped。`perception_metrics_test` 覆盖延迟、目标点步长、Z 轴稳定性、事件计数和摘要格式；`bringup_config_test` 覆盖阶段 10 launch 中的 rosbag 与 metrics 节点契约。

## 结构反思

阶段 10 没有把 TensorRT、rosbag 播放和指标统计塞进一个检测节点，而是让 metrics 成为旁路观察者。这样真实模型、回放数据和后续 TF/MoveIt 目标构造都可以独立替换，同时保留统一的感知质量证据。

## 下一步

阶段 14 已完成显式 real I2C 后端接入，默认仍保持 mock-safe。阶段 15 将在真实 DOFBOT 上做低速单关节验证。
