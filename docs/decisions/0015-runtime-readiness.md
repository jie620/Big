# ADR 0015：真机 readiness 边界

真机启动前的 I2C、RGB-D、CameraInfo、TF、checkpoint 和独立急停检查属于 readiness 条件，不再通过单独的运行入口绕过 Safety Gate。检查失败时总入口拒绝启动或保持未 arm。

真实执行的验收必须区分软件构建、控制器反馈、视觉注册和物理动作证据；任一项缺失都不能宣称抓取成功。
