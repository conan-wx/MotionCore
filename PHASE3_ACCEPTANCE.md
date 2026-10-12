# MotionCore Phase 3 验收记录

记录日期：2026-10-10。命令均在 Windows 的 WSL2 Ubuntu 中、项目目录 `/mnt/d/运动控制项目` 执行。规划器内部使用 SI 单位，标准仿真采样周期为 1 ms。

## 1. Debug Clean Build 与回归测试

实际执行：

```bash
wsl -d Ubuntu -- bash -lc 'cd /mnt/d/运动控制项目 && cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build --clean-first -j2 && ctest --test-dir build --output-on-failure'
```

实际 CTest 输出：

```text
Test project /mnt/d/运动控制项目/build
    Start 1: pid_unit_test
1/4 Test #1: pid_unit_test ....................   Passed
    Start 2: motioncore_phase1_test
2/4 Test #2: motioncore_phase1_test ...........   Passed
    Start 3: motioncore_phase2_test
3/4 Test #3: motioncore_phase2_test ...........   Passed
    Start 4: motioncore_phase3_test
4/4 Test #4: motioncore_phase3_test ...........   Passed

100% tests passed, 0 tests failed out of 4
Total Test time (real) = 0.05 sec
```

Phase 3 测试程序直接运行输出：

```text
MotionCore Phase 3 tests passed
```

Phase 3 CTest 覆盖标准梯形、三角形、S 曲线七段和四段退化、正反向、零位移、临界距离、任意时刻采样、约束上限、分段连续性、非法参数以及 Phase 2 动力学闭环跟踪。Phase 1/2 的三个原有 CTest 没有修改测试要求，均继续通过。

## 2. 独立规划器验收阈值

| 参数 | 标准值 |
| --- | ---: |
| 起点 | 0 mm |
| 终点 | 100 mm |
| 起止速度 | 0 mm/s |
| 最大速度 | 50 mm/s |
| 最大加速度 | 100 mm/s² |
| 最大 Jerk | 500 mm/s³ |

规划器精度目标为最终位置误差不超过 `1e-6 mm`、最终速度绝对值不超过 `1e-6 mm/s`，标准理论时间误差不超过 `1e-7 s`。终点采样会强制返回请求目标和零速度，避免浮点累计误差。

## 3. 标准场景实际终端输出

实际执行的三个命令：

```bash
./build/motioncore_phase3 --profile trapezoid --mode planner \
  --start-mm 0 --target-mm 100 --vmax-mm-s 50 --amax-mm-s2 100 \
  --jmax-mm-s3 500 --dt-ms 1 --output results/phase3/trapezoid.csv

./build/motioncore_phase3 --profile scurve --mode planner \
  --start-mm 0 --target-mm 100 --vmax-mm-s 50 --amax-mm-s2 100 \
  --jmax-mm-s3 500 --dt-ms 1 --output results/phase3/scurve.csv

./build/motioncore_phase3 --profile scurve --mode closed-loop \
  --start-mm 0 --target-mm 100 --vmax-mm-s 50 --amax-mm-s2 100 \
  --jmax-mm-s3 500 --dt-ms 1 --output results/phase3/closed_loop.csv
```

实际输出：

```text
profile=trapezoid, mode=planner, duration_s=2.5, final_reference_mm=100, csv=results/phase3/trapezoid.csv
profile=scurve, mode=planner, duration_s=2.7, final_reference_mm=100, csv=results/phase3/scurve.csv
profile=scurve, mode=closed-loop, duration_s=2.7, final_reference_mm=100, final_actual_mm=103.526269715, final_actual_velocity_mm_s=-1.19774804185, peak_drive_force_N=0.239009681226, max_tracking_error_mm=3.53352679785, csv=results/phase3/closed_loop.csv
```

标准 S 曲线 CSV 的关键采样值：

```text
t=0.2 s: x=0.666666666667 mm, v=10 mm/s, a=100 mm/s²
t=0.7 s: x=17.5 mm,        v=50 mm/s, a=0 mm/s²
```

这与独立公式 `tj=amax/jmax=0.2 s`、`ta=vmax/amax-tj=0.3 s`、`tv=1.3 s`、`T=2.7 s` 一致。规划 CSV 的最终行是 100 mm、0 mm/s；闭环误差单独报告，未套用规划器的 `1e-6 mm` 精度阈值。

## 4. 分支场景实际结果

实际执行了 10 mm 三角形、1 mm 四段 Jerk、反向、零位移和临界距离场景：

```text
profile=trapezoid, mode=planner, duration_s=0.632455532034, final_reference_mm=10
profile=scurve, mode=planner, duration_s=0.4, final_reference_mm=1
profile=scurve, mode=planner, duration_s=2.7, final_reference_mm=-100
profile=scurve, mode=planner, duration_s=0, final_reference_mm=0
profile=trapezoid, mode=planner, duration_s=1, final_reference_mm=25
```

其中 10 mm 梯形规划满足 `T=2*sqrt(D/a)=0.632455532034 s`；1 mm S 曲线为四段 Jerk、总时间 0.4 s；反向轨迹终点为 -100 mm；零位移轨迹为零时长；25 mm 是达到最大速度的临界距离。

## 5. 可视化验收

实际执行：

```bash
python3 scripts/plot_phase3.py \
  --trapezoid results/phase3/trapezoid.csv \
  --scurve results/phase3/scurve.csv \
  --closed-loop results/phase3/closed_loop.csv \
  --output-dir results/phase3
```

实际生成：

```text
results/phase3/velocity_comparison.svg
results/phase3/acceleration_comparison.svg
results/phase3/scurve_jerk.svg
results/phase3/closed_loop_position.svg
results/phase3/tracking_error.svg
```

图中分别展示梯形/S 曲线速度、加速度、S 曲线 Jerk、规划位置与实际位置、闭环位置误差，单位与 CSV 列名一致。

## 6. 已知限制和风险

当前规划器只支持零起止速度；非零边界速度、三轴协调插补、在线重规划和 Phase 4 功能没有实现。梯形规划的加速度边界不连续，因此其 `reference_jerk_mm_s3` 使用 `NaN` 标记理论上的冲激，不伪装成有限 Jerk 轨迹。闭环最终位置为 103.526 mm、最大跟踪误差为 3.534 mm，说明当前 Phase 2 控制器可以完成轨迹跟踪但仍有明显动态误差；这属于控制器整定和动力学限制，不是规划器解析误差。当前模型仍不包含真实电机电气动态、摩擦死区、编码器量化、实时线程抖动或硬件功放。
