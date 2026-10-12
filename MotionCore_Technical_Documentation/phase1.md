# MotionCore 第一阶段：单轴运动控制

## 阶段目标

在没有真实电机和驱动器的情况下，使用一个确定性的离散模型完成单轴位置闭环。每次运行都得到相同的结果，便于调试、测试和写实验报告。

单位约定：位置使用 mm，速度使用 mm/s，加速度使用 mm/s²，时间使用 s。

## 控制循环

每个采样周期按下面顺序执行：

```text
读取当前 AxisState
    ↓
根据目标位置计算期望速度
    ↓
根据期望速度计算加速度命令
    ↓
AxisSimulator 限制加速度和速度并积分
    ↓
写入 CSV 日志
```

当前基本控制器是位置 P 环和速度 P 环的串联形式：

```text
position_error = target_position - position
desired_velocity = clamp(Kp_position * position_error)
acceleration_command = clamp(Kp_velocity * (desired_velocity - velocity))
```

PID 不放在这一阶段的核心路径中，因为下一阶段会专门加入电机模型、积分抗饱和和位置环/速度环对比。

## 验收标准

- `cmake -S . -B build` 配置成功；
- `cmake --build build` 无编译警告；
- `ctest --test-dir build --output-on-failure` 全部通过；
- 加速度请求永远不会超过 `max_acceleration`；
- 速度永远不会超过 `max_velocity`；
- 100 mm 阶跃目标在 8 s 内的绝对位置误差小于 0.5 mm；
- 日志包含时间、目标、位置、速度、加速度、误差和命令。

在 VS Code 中可以直接按 `F5`，选择 `Debug MotionCore Phase 1`；首次运行会自动执行 CMake 构建并使用 Ubuntu 的 `/usr/bin/gdb`。

## 练习

1. 将 `apps/motioncore_phase1.cpp` 中的目标位置从 100 mm 改为 -50 mm，观察控制方向；
2. 把最大速度改小，比较到达时间；
3. 在 `AxisSimulator::step()` 中设置断点，用 GDB 观察一次离散积分；
4. 解释为什么位置更新使用旧速度和新速度的平均值；
5. 记录一次参数变化前后的最终误差和最大速度。
