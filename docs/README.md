# 文档索引

## 运行入口

所有系统运行都从同一个 launch 文件进入：

```bash
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=mock
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=mujoco checkpoint:=... yolo_model:=... registered_depth_confirmed:=true
ros2 launch edgepick_bringup edgepick_system.launch.py mode:=real checkpoint:=... yolo_model:=... calibration_confirmed:=true registered_depth_confirmed:=true camera_transform:=...
```

- [`SAFE_VLA.md`](SAFE_VLA.md)：输入契约、Safety Gate、静态障碍和真机前置条件。
- [`launch/real_system.md`](launch/real_system.md)：真机启动前检查和唯一 real 命令。
- [`launch/prehardware_mock_rehearsal.md`](launch/prehardware_mock_rehearsal.md)：不接硬件的系统级 rehearsal。
- [`task/`](task/)：任务状态机、目标构造和 mock action 适配器。
- [`perception/`](perception/)：RGB-D、检测框、TF 和指标诊断。
- [`decisions/`](decisions/)：已经被当前总入口替代的历史决策记录。

历史决策不再作为运行说明；操作时以根 README 和 `SAFE_VLA.md` 为准。
