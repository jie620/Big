# 真机系统入口

真机只允许使用 `edgepick_system.launch.py mode:=real`。它会启动同一套 robot description、MoveIt、Safety Gate、SmolVLA、YOLO/RGB-D 感知和 ros2_control，并显式打开 I2C。

## 启动前

确认以下条件后再启动：

1. `/dev/i2c-7` 可访问，地址和厂商读探针已确认。
2. 相机已经发布注册到彩色图像的深度、彩色 CameraInfo 和稳定 frame。
3. `camera_transform` 是 `DaBai_DCW2_Link -> camera_frame` 的实测外参。
4. checkpoint 含 `config.json`、`model.safetensors` 和本地 VLM/tokenizer 资源。
5. YOLO 模型是针对黄色方块训练或验证过的部署文件。
6. 独立急停处于可用状态，机械臂周围有足够的停止距离。

## 命令

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/local_setup.bash
source /home/jetson/Codex_Projects/Big/install/local_setup.bash
ros2 launch orbbec_camera dabai_dcw2.launch.py
ros2 launch edgepick_bringup edgepick_system.launch.py \
  mode:=real \
  checkpoint:=/home/jetson/Codex_Projects/Big/models/vla \
  yolo_model:=/home/jetson/Codex_Projects/Big/models/yolo/cube.engine \
  calibration_confirmed:=true registered_depth_confirmed:=true \
  camera_transform:='X,Y,Z,ROLL,PITCH,YAW' \
  camera_frame:=camera_color_optical_frame \
  i2c_device:=/dev/i2c-7 i2c_address:=0x15
```

入口不会自动 arm。先持续发布真实急停心跳，确认 `/joint_states`、目标、场景和策略心跳均新鲜，再调用 `/edgepick/safe/arm`。任何标定、注册深度、模型或设备检查失败都会在 launch 阶段拒绝启动。

这份文档只证明软件启动路径；它不证明真实抓取、放置、延迟或碰撞安全。真机验收必须同时保存 Safety Gate 事件、控制器反馈、关节轨迹、RGB-D 记录和独立急停测试结果。
