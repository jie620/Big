# 自研 ROS 2 包

后续按职责创建独立包，第一批预计包括：

- `edgepick_hardware`：C++ I2C 传输抽象、mock 后端和 ros2_control `SystemInterface`。当前已完成命令网关与 mock 系统接口。
- `edgepick_bringup`：生命周期管理、参数和启动编排。
- `edgepick_task`：抓取任务状态机、超时和恢复策略。

任何真实 I2C 写入必须经 `edgepick_hardware`，不得在感知或任务节点中直接调用 `Arm_Lib`。

## 阶段记录

记录规则：阶段记录只追加，不覆盖旧阶段；每次任务完成后只更新“下一步目标”。

### 阶段 1：命令网关与 mock 传输

当前阶段：`src/edgepick_hardware` 建立了命令网关、传输抽象和内存 mock 后端。

完成内容：先把命令校验、重复抑制、发送限频和传输失败注入放进自研包，不让感知、任务或启动层直接接触真实 I2C。

结构反思：`src/` 在阶段 1 只放一个自研硬件边界包是合理的，先把最低层契约固定住，后续包可以按职责追加。

当时下一步：在 `edgepick_hardware` 中继续封装 mock `ros2_control` `SystemInterface`。

### 阶段 2：mock ros2_control 系统接口

当前阶段：`edgepick_hardware` 已具备 mock `ros2_control` `SystemInterface` 插件，能让控制器以标准 position interface 写入六关节命令，并在包内完成弧度到舵机角度的转换与校验。

完成内容：新增系统接口、插件描述、构建依赖和测试，但没有把启动文件、MoveIt 配置和硬件接口混在一个包里。

结构反思：`src/` 目前仍只放自研包，vendor 代码保持在 `vendor/` 作为只读参考。这个结构是正确的，后续 `edgepick_bringup` 可以单独承担启动编排。

补充记录：阶段 2 代码已补充适量模块注释，重点说明接口职责、单位转换、安全拒绝路径和 mock 状态策略。以后新增包也遵循同样的注释粒度。

## 下一步目标

阶段 3：新增 `edgepick_bringup` 包，把 xacro、controller 配置和 launch 编排集中起来，验证 RViz/MoveIt 到 EdgePick mock 硬件接口的完整链路。
