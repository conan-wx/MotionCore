# Phase 2 技术说明：动力学与 PID 伺服

## 模型和单位

`DynamicAxisModel` 的状态完全使用 SI 单位。参数含义是：等效质量 `m`（kg）、黏性阻尼 `b`（N·s/m）、最大驱动力（N）、初始位置（m）、初始速度（m/s）和负载曲线。负载力在动力学方程中被减掉，所以正负号都可以表达运动方向上的阻力或助力。

```text
a[k] = (clamp(F_command) - b*v[k] - F_load[k]) / m
x[k+1] = x[k] + v[k]*dt + 0.5*a[k]*dt*dt
v[k+1] = v[k] + a[k]*dt
```

这是零阶保持驱动力下的常加速度显式积分。实验固定 `dt=1 ms`，模型接口仍检查 `dt` 必须为有限正数，便于单元测试非法输入。矩形负载脉冲使用 `[start, start+duration)` 半开区间，边界行为确定且可复现。

## 控制器

独立的 P、PI、PID 类都检查参数和输入是否有限。PI/PID 将积分候选值限制在积分范围内；当候选输出已经饱和且误差仍会把输出推向同一饱和方向时，冻结积分状态，这就是条件积分抗饱和。控制器仍把本拍限幅后的输出交给模型，`reset()` 会清空积分和历史状态。

PID 的微分项使用 `-d(measurement)/dt`，而不是 `d(setpoint-measurement)/dt`，因此目标阶跃不会产生 derivative kick；导数再经过一阶离散低通滤波，减小测量噪声放大。

串级控制器的外环计算 `v_target = clamp(Kp_position * position_error)`，随后将每拍变化限制在 `amax*dt`。速度环可以选择 P 或 PI。阻尼前馈打开时，额外加入 `b*v_target`，最后仍进行总驱动力限幅。该前馈只补偿粘性阻尼，不补偿恒定外部负载；实验 D 的结果应按这个定义解读。

## 数据和指标

仿真日志先保存内部 SI 值，CSV 输出层再转换成 mm、mm/s 和 mm/s²。位置误差定义为 `target_position - actual_position`。最大超调按目标阶跃方向计算，最终误差和速度取最后一个采样点，最大速度/加速度取整个日志中的绝对值最大值。

调节时间判定使用 ±0.2 mm 误差带和 0.2 s 保持时间：从前往后寻找第一个采样点 `i`，使 `i` 到 `i+ceil(0.2/dt)`（含两端）所有位置误差都在误差带内；该点时间就是调节时间。扰动恢复时间采用相同判定，但搜索从扰动结束时刻开始计时。

## 学习和调试入口

推荐先阅读 `dynamic_axis.hpp/.cpp`，再阅读 `p_controller.hpp`、`pi_controller.hpp`、`pid_controller.hpp` 和 `cascade_servo_controller.cpp`。开环模型可在测试中直接构造 `DynamicAxisModel` 并调用 `step(force_n, 0.001)`；闭环程序入口是 `apps/motioncore_phase2.cpp`。所有 Phase 1 类和测试仍在原位置，便于对比学习。
