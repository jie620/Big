# ADR 0017: Real MoveIt Minimal Validation

## 决策

阶段 17 新增 `edgepick_moveit_real_validation.launch.py`。该入口复用阶段 16 的 real control + MoveGroup，再追加一个最小验证节点，执行单关节小幅度前进并回到捕获的 home 状态。

## 背景

阶段 16 已经证明 MoveIt 可以接到真实 controller-manager。下一步不该立刻把任务链、感知或抓取逻辑接进来，而是先验证最小的真实闭环：MoveIt 能否稳定规划、执行，并回到当前 home。

## 影响

- 验证动作保持单关节、小角度、低速。
- 验证前先捕获当前关节值，回零目标不依赖手写常量。
- launch 结束后自动关闭，方便直接把运行结果当作一次性验证记录。
- 该阶段仍不引入 task node、perception node 或抓取目标构造。

## 状态

阶段 17：最小 MoveIt 关节验证入口已新增，等待在真实 DOFBOT 上执行。

## 下一步

阶段 19：把橘子检测结果稳定接到任务/抓取联调，并继续做真机链路观测。
