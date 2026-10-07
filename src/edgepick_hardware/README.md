# edgepick_hardware

该包是唯一的硬件边界，提供：

- `MockSystemInterface`：默认回归后端，不访问设备。
- `DofbotI2cTransport`：由总入口 `edgepick_system.launch.py mode:=real` 显式启用的 I2C 后端。
- 命令网关和舵机映射：统一关节顺序、ROS 弧度和厂商角度转换。

硬件包不提供独立真机 launch。I2C 设备、地址和运动时间从总入口透传；默认值为 `/dev/i2c-7`、`0x15` 和 `80 ms`，必须按实际设备检查。
