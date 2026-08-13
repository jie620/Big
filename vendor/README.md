# 厂商参考快照

该目录保存 2026-07-27 从本机 Yahboom DOFBOT 环境选择性复制的源码和配置，用于理解既有协议、消息和 MoveIt 配置。它们不是 EdgePick 的可编辑实现，也不会作为本项目的构建输入。

复制时已排除 `.ipynb_checkpoints/`、`__pycache__/`、Python egg、TensorRT 引擎和其他生成文件。

| 快照 | 原始路径 | 用途 |
| --- | --- | --- |
| `dofbot_pro_moveit` | `/home/jetson/dofbot_pro_ws/src/dofbot_pro_moveit` | MoveIt 配置、启动方式和 C++ 示例 |
| `dofbot_pro_description` | `/home/jetson/dofbot_pro_ws/src/dofbot_pro_description` | URDF/关节描述参考；网格未复制 |
| `dofbot_pro_driver` | `/home/jetson/dofbot_pro_ws/src/dofbot_pro_driver` | 现有 Python 驱动行为基线 |
| `dofbot_pro_yolov11` | `/home/jetson/dofbot_pro_ws/src/dofbot_pro_yolov11` | YOLO 节点与抓取控制流程参考；引擎未复制 |
| `Arm_Lib` | `/home/jetson/colcon_ws/src/Arm_Lib` | I2C 协议行为参考 |
| `dofbot_pro_interface` | `/home/jetson/dofbot_pro_ws/src/dofbot_pro_interface` | ROS 2 消息与服务定义参考 |

新增依赖和二次分发前须确认其各自许可证；当前源目录未提供可识别的 LICENSE 文件。
