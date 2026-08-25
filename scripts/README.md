# 开发脚本

只放置可重复执行的构建、启动、测量和数据导出脚本。脚本默认必须使用 mock 后端；任何触发真实机械臂动作的脚本需要显式参数开关。

- `check_dofbot_i2c.py`：阶段 15 真机前 I2C 预检脚本。默认只打开设备并选择地址，然后通过厂商 `Arm_Lib` 做读探针；`--scan` 会额外调用 `i2cdetect`，但扫描结果只作辅助参考。
- `run_orange_grasp_validation.sh`：真实橘子抓取验证脚本。先启动 `orbbec_camera` 的 `dabai_dcw2.launch.py`，确认 `/camera/color/image_raw`、`/camera/depth/image_raw` 和 `/camera/depth/camera_info` 已发布，再启动橘子抓取链路；默认把夹爪闭合值压到更保守的 `-0.9`，防止直接关到底。
