# 未复制的大型外部依赖

这些资源保留在机器现有位置，EdgePick 在需要时通过已安装 ROS 2 环境或明确的参数路径使用它们，不复制进本仓库。

| 资源 | 原始位置 | 未复制原因 |
| --- | --- | --- |
| DOFBOT 网格 | `/home/jetson/dofbot_pro_ws/src/dofbot_pro_description/meshes` | 约 51 MB；仅 RViz 可视化需要 |
| YOLO TensorRT 引擎 | `/home/jetson/dofbot_pro_ws/src/dofbot_pro_yolov11/dofbot_pro_yolov11/best.engine` | 约 12 MB；二进制模型应由项目脚本可重复生成或显式配置 |
| Orbbec ROS 2 驱动 | `/home/jetson/dofbot_pro_ws/src/OrbbecSDK_ROS2-main` | 大型第三方相机 SDK，暂时复用系统现有工作区 |
| ROS 2 / MoveIt 2 | `/opt/ros/humble` 与现有工作区 install 层 | 系统依赖，不应提交到应用仓库 |

后续若需要可复现实验，优先记录版本、校验值、下载或构建步骤，而不是提交二进制文件。
