# 开发脚本

只放置可重复执行的构建、启动、测量和数据导出脚本。脚本默认必须使用 mock 后端；任何触发真实机械臂动作的脚本需要显式参数开关。

- `check_dofbot_i2c.py`：阶段 15 真机前 I2C 预检脚本。默认只打开设备并选择地址，然后通过厂商 `Arm_Lib` 做读探针；`--scan` 会额外调用 `i2cdetect`，但扫描结果只作辅助参考。
- `run_orange_grasp_validation.sh`：橘子抓取验证脚本。先启动 `orbbec_camera` 的 `dabai_dcw2.launch.py`，确认 `/camera/color/image_raw`、`/camera/depth/image_raw` 和 `/camera/depth/camera_info` 已发布，再启动抓取链路。默认 `USE_REAL_I2C=false` 做 dry-run（仍需相机）；真机必须显式设置 `USE_REAL_I2C=true`，其他 launch 参数可追加在脚本后。

夹爪使用度数参数，例如 `gripper_close_angle_deg:=120.0`，旧的 `gripper_close_position` 弧度参数已移除。
脚本的 topic 存在检查不证明 RGB-D 已注册；配套深度、内参与 frame 必须按[根 README](../README.md)核对。
无设备测试使用 `python3 src/edgepick_task/test/runtime_safety_check.py`（仓库根目录、先 source install）。
