# Phase 3 技术说明：工业运动轨迹规划

Phase 3 在 Phase 1 的运动学轴和 Phase 2 的动力学/串级伺服控制之上，增加可单独验证的单轴轨迹规划模块。规划器只负责生成参考轨迹，动力学模型仍然负责计算实际位置、速度和加速度；两者的数据不能混用。Phase 3 不引入三轴插补、Qt、ROS2 或实体硬件。

## 统一约定

规划器内部统一使用 SI 单位：位置 `m`、速度 `m/s`、加速度 `m/s²` 和 Jerk `m/s³`。命令行和 CSV 为便于阅读使用 `mm`、`mm/s`、`mm/s²` 和 `mm/s³`。仿真默认采样周期是 1 ms，但采样接口接受任意非负时间，因此总时间不是采样周期整数倍时，最后一次采样仍能精确取到终点。

轨迹状态至少包含：`t、x_ref(t)、v_ref(t)、a_ref(t)、j_ref(t)`。起点和终点位置必须有限，最大速度、最大加速度及最大 Jerk 必须为正且有限。本阶段实际规划只接受起止速度均为零；传入非零边界速度时接口应明确抛出参数错误，避免调用者误以为非零边界速度已被静默支持。

## 梯形/三角形速度规划

令运动距离 `D = |x1 - x0|`，运动方向 `s = sign(x1 - x0)`。先在正方向坐标 `q = s (x - x0)` 中规划，采样时再乘回 `s`。达到最大速度所需的加速时间和单侧位移为：

```text
ta = vmax / amax
da = 0.5 * amax * ta² = vmax² / (2 * amax)
```

当 `D >= 2*da` 时使用标准梯形：

```text
tc = (D - 2*da) / vmax
T  = 2*ta + tc
```

当 `D < 2*da` 时无法达到 `vmax`，自动退化为三角形：

```text
vpeak = sqrt(D * amax)
ta = vpeak / amax
tc = 0
T  = 2*ta
```

正方向的分段解析式如下，`td = t - ta - tc` 表示减速段局部时间：

```text
加速 0 <= t < ta:
    a = +amax
    v = amax*t
    q = 0.5*amax*t²

匀速 ta <= t < ta+tc:
    a = 0
    v = vpeak
    q = da + vpeak*(t-ta)

减速 ta+tc <= t <= T:
    a = -amax
    v = vpeak - amax*td
    q = da + vpeak*tc + vpeak*td - 0.5*amax*td²
```

在最终时刻直接返回 `x1` 和 `v=0`，避免双精度舍入使终点出现微小漂移。加速度在两个切换点发生跳变，因此梯形规划的理论 Jerk 是不连续冲激，不能宣称满足有限 Jerk 约束；需要受限 Jerk 时应选择 `SCurveProfile`。

## 七段限 Jerk S 曲线

完整的对称七段轨迹按下列顺序排列：

```text
1  +J：加速度从 0 增加到 +A
2   0：保持 +A
3  -J：加速度回到 0，速度达到 vmax
4   0：匀速
5  -J：加速度从 0 降到 -A
6   0：保持 -A
7  +J：加速度回到 0，速度降到 0
```

每一段都用常 Jerk 的解析积分计算。对称静止到静止运动，常见的阶段时间记为 `tj`（Jerk 段）、`ta`（恒加速度段）和 `tv`（匀速段）；具体实现根据距离和约束自动求解，不把超出的段时间简单裁剪为零，并始终保证 `tj <= amax / jmax`、`vpeak <= vmax`。

实现中的通用分段公式是：对一段长度为 `tau`、起始状态为 `(x0,v0,a0)` 且
Jerk 为常数 `j` 的区间，直接计算

```text
a(tau) = a0 + j*tau
v(tau) = v0 + a0*tau + 0.5*j*tau²
x(tau) = x0 + v0*tau + 0.5*a0*tau² + (j*tau³)/6
```

七段的 Jerk 序列为 `[+J, 0, -J, 0, -J, 0, +J]`；每段结束状态作为下一段的起始状态。因此连续性来自状态传递，而不是逐毫秒累加位置。

若距离不足以同时达到 `vmax` 和 `amax`，规划器依次退化为无匀速段、无恒加速度段的七段子集，极短距离使用四段 Jerk 轨迹（`+J, -J, -J, +J`）。纯 Jerk 受限四段的每段时间满足：

```text
D = 2*jmax*tj³
T = 4*tj
```

方向处理与梯形规划相同。每段边界保存上一段的 `x、v、a`，下一段从该状态继续积分，所以位置、速度和加速度连续；Jerk 只在边界按约束允许地切换。

标准验收参数 `D=100 mm, vmax=50 mm/s, amax=100 mm/s², jmax=500 mm/s³` 下有 `tj=0.2 s`、`ta=0.3 s`、半程加速时间 `0.7 s`、匀速时间 `1.3 s`，理论总时间 `2.7 s`。例如：

```text
t=0.2 s: x=0.6666667 mm, v=10 mm/s, a=100 mm/s²
t=0.7 s: x=17.5 mm,    v=50 mm/s
```

极短距离 `D=1 mm` 时使用四段 Jerk 子集，理论每段 `tj=0.1 s`，总时间为 `0.4 s`。这些数值由独立公式写入单元测试，不从规划结果反推测试期望。

## 采样和 Phase 2 闭环集成

`TrajectorySampler` 根据指定时间返回参考状态；时间小于零、超过总时间时分别返回起点和终点状态，终点强制为目标位置和零速度。规划器单独运行时只验证解析轨迹约束和边界精度。

闭环模式将参考位置和参考速度送入 Phase 2 串级伺服。位置环使用轨迹前馈加位置误差修正：

```text
v_command = v_ref + Kp_position * (x_ref - x_actual)
```

随后继续使用既有的目标速度限幅、速度 PI、驱动力限幅和动力学模型。`x_ref`不能直接写入实际轴状态；闭环日志同时保存参考位置、实际位置、跟踪误差及驱动力，便于分别评估规划器数值精度和虚拟电机跟踪能力。驱动力饱和或控制器动态不足时，闭环误差可以大于规划器的 `1e-6 mm` 数值指标，这属于两个独立验收项。

## 命令、数据和可视化

Phase 3 可执行程序名为 `motioncore_phase3`，主要参数如下：

```text
--profile trapezoid|scurve
--mode planner|closed-loop
--start-mm <number>
--target-mm <number>
--vmax-mm-s <number>
--amax-mm-s2 <number>
--jmax-mm-s3 <number>
--dt-ms <number>
--output <csv path>
```

规划器 CSV 至少包含：

```text
time_s,reference_position_mm,reference_velocity_mm_s,
reference_acceleration_mm_s2,reference_jerk_mm_s3
```

闭环 CSV 在此基础上增加 `actual_position_mm`、`actual_velocity_mm_s`、`actual_acceleration_mm_s2`、`position_error_mm` 和 `drive_force_N`。所有输出写入 `results/phase3/`，可用标准库脚本一次生成五张 SVG：

```bash
python3 scripts/plot_phase3.py \
  --trapezoid results/phase3/trapezoid.csv \
  --scurve results/phase3/scurve.csv \
  --closed-loop results/phase3/closed_loop.csv \
  --output-dir results/phase3
```

生成的图分别是 `velocity_comparison.svg`、`acceleration_comparison.svg`、`scurve_jerk.svg`、`closed_loop_position.svg` 和 `tracking_error.svg`。

## 验证方法和限制

CTest 同时运行 Phase 1、Phase 2 原有测试及命名清晰的 Phase 3 测试。Phase 3 测试覆盖理论总时间、边界状态、速度/加速度/Jerk 约束、分段连续性、梯形三角形分支、S 曲线退化分支、正反向运动、任意时间采样、零位移、非法参数和闭环跟踪。规划器默认目标为最终位置误差不超过 `1e-6 mm`、最终速度不超过 `1e-6 mm/s`、标准理论时间误差不超过 `1e-7 s`。

当前模型仍然是 Phase 2 的一维等效质量模型，未包含电机电气时间常数、摩擦死区、编码器量化、实时线程抖动或真实功放。非零边界速度、三轴协调插补和在线重规划也不属于本阶段；调用者应等待后续阶段明确支持后再使用。
