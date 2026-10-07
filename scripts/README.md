# scripts

脚本按用途分为四类：

- `run_mock_checks.sh`、`integration_safe_mock.py`、`verify_safe_runtime.py`：无硬件回归。
- `setup_vla_env.sh`、`prepare_checkpoint.py`、`train_vla.sh`：SmolVLA 环境、checkpoint 和训练。
- `prepare_pt_checkpoint.py`：把自定义训练产生的 `best.pt` 转成 Big 可加载的 `model.safetensors` checkpoint。它会保留真实输入模态；如果源模型只有 RGB，不会伪造深度策略输入，深度仍由 Safety Gate 感知链路强制使用。
- `compress_detector.py`、`export_detector.py`、`benchmark_detector.py`：YOLO 方块模型压缩和 TensorRT 测量。`benchmark_detector.py` 会在 CUDA 同步后报告端到端 P50/P95/P99，以及 Ultralytics 能提供的预处理、推理和后处理分项耗时。
- `profile_ros_runtime.py`：只读观察运行中的 RGB、深度、关节和 VLA proposal 话题，记录本地到达周期和消息头时间戳年龄，不发布任何动作。
- `check_dofbot_i2c.py`、`check_mujoco_backend.py`、`record_safe_run.py`：设备、仿真和运行证据检查。

脚本不提供第二条真机启动路线。真实动作必须通过：

```bash
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=real ...
```

任何物理动作前仍需独立硬件急停和实测 RGB-D/TF 验收。

运行时 profiling 示例：

```bash
python3 scripts/profile_ros_runtime.py --duration 60 \
  --output run_logs/runtime_profile.json
```

检测器基准会执行一次验证、若干预热推理，然后进行 CUDA 同步的计时循环：

```bash
python3 scripts/benchmark_detector.py models/yolo/cube.engine \
  --data models/yolo/data.yaml --image /path/to/rgb.png \
  --device 0 --warmup 20 --runs 100 \
  --output run_logs/yolo_benchmark.json
```

`latency_ms` 是完整 `predict` 调用的时间；`preprocess_ms`、`inference_ms` 和 `postprocess_ms` 只在当前 Ultralytics 后端提供分项数据时出现。Jetson 的 TensorRT 或统一内存总占用仍应配合 `tegrastats` 观察。

从当前方块训练快照转换示例：

```bash
python3 scripts/prepare_pt_checkpoint.py /home/jetson/Codex_Projects/Edge_AI/best.pt \
  --output models/vla_cube_best \
  --vlm-assets /home/jetson/.cache/huggingface/hub/models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct/snapshots/7b375e1b73b11138ff12fe22c8f2822d8fe03467
```
