# Phase 5：运动控制管理层

Phase 5 将已有的轨迹规划器、三轴伺服和动力学模型组织成可测试的 `MotionController`。
这是离线确定性软件仿真，不是实时控制器，也不替代真实驱动器的 STO、急停继电器或安全 PLC。

## 调度顺序

每个 1 ms `step(dt)` 按下面顺序执行：

1. 检查 `dt` 和 Stop/EmergencyStop 高优先级事件；
2. 如果处于 Homing，推进回零传感器、防抖和三轴动力学；
3. 检查状态、软限位、故障锁存和活动指令；
4. 由 Phase 3/4 插补器采样参考位置和速度；
5. 将同一时刻的三轴参考一次性送入 `MultiAxisServoSystem`；
6. 更新跟踪误差、限位裕量、故障和 CSV/事件日志。

回零按 `SearchFast → Backoff → SearchSlow → ApplyOffset → Homed` 执行。传感器触发只改变
回零状态和坐标偏移，实际轴位置始终来自动力学更新；最后还要等待真实速度下降，不能把速度或位置瞬间写成零。

## 状态机和命令队列

状态为 `Disabled`、`NotHomed`、`Homing`、`Ready`、`Running`、`ControlledStopping`、`Fault` 和
`EmergencyStopped`。普通 MoveLine/MoveArc/Dwell 使用 FIFO 队列，Stop 和 EmergencyStop 使用高优先级队列。
Fault、EmergencyStopped 都是锁存状态，必须显式复位，复位后回到 `NotHomed`。

每条命令保留 ID、几何参数、约束、排队/开始/结束时间和终态。运动开始前会再次进行有限线段或有限圆弧的软限位检查，
圆弧检查四个角度极值，不能只检查端点。

## 限位和预测制动

每轴保存位置范围、速度/加速度和可实现制动减速度。预测停止距离为：

```text
d_stop = |v| * update_delay + v² / (2 a_brake)
```

若沿当前方向的停止位置越过带安全余量的限位，命令被拒绝或进入故障策略。这个计算不会裁剪实际位置；EmergencyStop
只锁存软件状态，禁能后的轴仍按惯性和阻尼继续运动。

Phase 5.1 在每次伺服更新后再次检查实际位置。位置超出边界会锁存
`SoftLimitViolation`，位置仍保持动力学模型给出的真实值并写入事件日志；位置尚未越界但预测停止距离已经超过可用裕量时，
同样锁存该故障，避免把无法安全完成的轨迹报告为 `Ready`。接近边界的静止浮点抖动使用 `0.005 mm` 观察容差，
不会改变模型位置。故障复位必须先满足对应原因已消除，再回到 `NotHomed` 重新建立坐标。

Phase 4 导出的 Jerk 分量 `jx_ref_mm_s3`、`jy_ref_mm_s3`、`jz_ref_mm_s3` 及模长
`jerk_ref_magnitude_mm_s3` 均为 mm/s³；模长由 `sqrt(jx²+jy²+jz²)` 计算，CSV 测试逐行核验。

## 构建、场景和日志

```bash
cd /mnt/d/运动控制项目
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/motioncore_phase5 --scenario homing --output results/phase5
./build/motioncore_phase5 --scenario nominal --output results/phase5
./build/motioncore_phase5 --scenario queue --output results/phase5
./build/motioncore_phase5 --scenario soft-limit --output results/phase5
./build/motioncore_phase5 --scenario controlled-stop --output results/phase5
./build/motioncore_phase5 --scenario emergency-stop --output results/phase5
./build/motioncore_phase5 --scenario fault-recovery --output results/phase5
python3 scripts/plot_phase5.py results/phase5/nominal.csv --output-dir results/phase5
```

CSV 使用统一仿真时间，包含状态、当前命令、三轴参考/实际位置和速度、驱动力、回零标志、跟踪误差、故障码及三轴软限位裕量。
`events.csv` 记录状态转换、回零、入队、完成、停止、急停、故障和复位。数值使用 SI 计算，CSV 的位置和速度换算为 mm、mm/s。

## 已知限制

本阶段只支持零起止速度、离线固定步长、三轴直线/XY 圆弧和拐角停车；没有在线路径前瞻、非零速度连续拐角、真实传感器噪声模型、
EtherCAT、Windows 实时线程或硬件安全认证。回零传感器是确定性软件模型，急停场景只能用于验证软件状态和惯性滑行逻辑。
