# ADR 0015: Real Low-Speed Single-Joint Validation

## 决策

阶段 15 将真实 DOFBOT 验证限制为低速、单关节、小角度动作。验证前必须先完成 I2C 预检、mock rehearsal 回归和 controller-manager 状态检查。

仓库提供 `scripts/check_dofbot_i2c.py` 作为 I2C 预检工具。该脚本默认只打开 I2C 设备并选择地址，不发送舵机命令帧；`--scan` 会调用 `i2cdetect` 探测总线地址。

## 背景

阶段 14 已经把真实 I2C 后端接入 `edgepick_hardware`，但真机第一次动作仍存在方向、零点、限位、供电和权限风险。直接运行完整任务链会把感知、规划、执行和硬件风险混在一起，出问题时难以定位。

阶段 15 因此只验证最小硬件闭环：I2C 可访问、controller-manager 可启动、控制器可激活、单个关节可低速移动并返回安全位置。

## 影响

- 真机验证不使用 task node、perception、MoveIt action mock 或完整抓取流程。
- 默认 mock launch 和 prehardware rehearsal 仍作为回归入口。
- 真实运动命令必须由操作者在确认机械臂空间安全后手动执行。
- 验证记录需要保存供电、I2C 地址、控制器状态、动作幅度、方向、零点、异常和回滚命令。

## 状态

阶段 15：低速真机验证已完成。

验证记录：2026-08-20 新增 I2C 预检脚本和 `docs/launch/real_low_speed_validation.md`，脚本通过本地语法检查。用户真实终端已确认 `/dev/i2c-7` 可打开、`Arm_get_hardversion()` 返回 `0.20`、`Arm_ping_servo(1)` 返回 `218`。`i2cdetect` 未显示 `0x15`，因此阶段 15 以厂商 `Arm_Lib` 读探针作为 I2C 接入判据。随后用户真实终端完成 real control 启动、`Arm1_Joint` 低速小角度前进和回零，两个 action 目标均返回 `SUCCEEDED`。

## 下一步

阶段 18：保持 real control 边界稳定，继续配合橘子感知联调。
