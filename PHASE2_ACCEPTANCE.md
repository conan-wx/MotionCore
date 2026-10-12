# MotionCore Phase 2 验收记录

记录日期：2026-10-10。所有命令在 Windows 的 WSL2 Ubuntu 中、项目目录 `/mnt/d/运动控制项目` 执行；仿真固定步长为 1 ms，时长 8 s。

## 构建与 CTest

实际执行的 Debug clean build 命令：

```bash
wsl -d Ubuntu -- bash -lc 'cd /mnt/d/运动控制项目 && cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build --clean-first -j2 && ctest --test-dir build --output-on-failure'
```

关键终端输出：

```text
[100%] Built target motioncore_phase2_tests
Test project /mnt/d/运动控制项目/build
    Start 1: pid_unit_test
1/3 Test #1: pid_unit_test ....................   Passed
    Start 2: motioncore_phase1_test
2/3 Test #2: motioncore_phase1_test ...........   Passed
    Start 3: motioncore_phase2_test
3/3 Test #3: motioncore_phase2_test ...........   Passed

100% tests passed, 0 tests failed out of 3
Total Test time (real) = 0.04 sec
```

Phase 2 测试程序的直接输出为 `MotionCore 第二阶段测试全部通过`。新增测试覆盖：动力学离散公式和驱动力饱和、正反向运动、P/PI/PID 输出限幅、积分抗饱和和 reset、PID 测量微分与设定值突变、目标速度斜率限制且不覆盖实际速度、串级闭环到位、0.5 s 扰动恢复、非法参数、CSV 表头/字段/行数/单位转换。

## 验收阈值

默认无扰动到位实验 A 的初始阈值为：最终位置误差 ≤ 0.2 mm，最终速度绝对值 ≤ 0.5 mm/s，最大超调量 ≤ 5 mm。调节时间使用 ±0.2 mm 误差带，并要求连续 0.2 s 内所有采样点保持在带内。实验 B 另外从负载结束时刻开始报告相同判定得到的恢复时间，并检查全程数值有限。

## 四组实验的实际输出

命令格式：

```bash
./build/motioncore_phase2 --scenario nominal
./build/motioncore_phase2 --scenario disturbance
./build/motioncore_phase2 --scenario comparison
./build/motioncore_phase2 --scenario feedforward
```

实际输出（科学计数法数值来自程序原样打印）：

```text
[nominal] max_overshoot_mm=0.0146303, final_position_error_mm=2.81719e-12, final_velocity_mm_s=1.15862e-11, settling_time_s=1.212, max_velocity_mm_s=101.972, max_acceleration_mm_s2=759.549, settled=true

[disturbance] max_overshoot_mm=6.15247, final_position_error_mm=1.04927e-08, final_velocity_mm_s=-4.03304e-08, settling_time_s=1.212, max_velocity_mm_s=101.972, max_acceleration_mm_s2=2000.47, settled=true
[disturbance] recovery_time_after_pulse_s=1.14 (error band +/-0.2 mm held for 0.2 s)

[comparison_p] max_overshoot_mm=0, final_position_error_mm=6.25, final_velocity_mm_s=2.45979e-12, settling_time_s=-1, max_velocity_mm_s=43.4783, max_acceleration_mm_s2=984, settled=false
[comparison_pi] max_overshoot_mm=0, final_position_error_mm=4.77396e-12, final_velocity_mm_s=1.78856e-11, settling_time_s=1.342, max_velocity_mm_s=99.4654, max_acceleration_mm_s2=983.936, settled=true
[comparison] final_speed_error_p_mm_s=50, final_speed_error_pi_mm_s=2.04171e-11, mean_last_0.5s_speed_error_p_mm_s=50, mean_last_0.5s_speed_error_pi_mm_s=6.01268e-11

[feedforward_off] max_overshoot_mm=0.0146303, final_position_error_mm=2.81719e-12, final_velocity_mm_s=1.15862e-11, settling_time_s=1.212, max_velocity_mm_s=101.972, max_acceleration_mm_s2=759.549, settled=true
[feedforward_on] max_overshoot_mm=0, final_position_error_mm=6.06598e-10, final_velocity_mm_s=1.93032e-09, settling_time_s=1.835, max_velocity_mm_s=109.088, max_acceleration_mm_s2=840.921, settled=true
[feedforward] peak_drive_force_off_N=0.954001, peak_drive_force_on_N=1.05955
```

实验 A 满足全部默认阈值。实验 B 的外部负载会造成约 6.15 mm 峰值偏差，但扰动结束后 1.14 s 恢复到 ±0.2 mm 并保持，最终状态误差和速度都回到接近零，未出现数值发散。实验 C 显示速度 P 在 1 N 恒载下留下 50 mm/s 的稳态速度误差，而 PI 将其降至约 `2e-11 mm/s`；对应位置 P 也留下 6.25 mm 的静差，PI 到位。实验 D 的前馈是阻尼前馈，不是恒载补偿，因此在当前参数下开启后峰值驱动力和调节时间分别为 1.05955 N、1.835 s，结果用于真实对比而非人为宣称一定改善。

## 数据和图形验收

每次运行生成独立的 CSV 和 `<scenario>_metrics.csv`，CSV 含 9 列要求字段，包含时间、目标/实际位置、目标/实际速度、实际加速度、驱动力、负载力和位置误差。绘图脚本实际执行：

```bash
python3 scripts/plot_phase2.py results/phase2/nominal.csv
python3 scripts/plot_phase2.py results/phase2/disturbance.csv
python3 scripts/plot_phase2.py results/phase2/comparison_p.csv
python3 scripts/plot_phase2.py results/phase2/comparison_pi.csv
python3 scripts/plot_phase2.py results/phase2/feedforward_off.csv
python3 scripts/plot_phase2.py results/phase2/feedforward_on.csv
```

六次命令均输出 `已生成: results/phase2/<name>.svg`。生成的 SVG 展示位置、速度、位置误差和驱动力随时间变化。

## 已知局限

Phase 2 使用等效质量、黏性阻尼和外部负载的单轴模型，没有电机电气时间常数、反电动势、摩擦死区、编码器量化、实时线程抖动或真实功放限制。阻尼前馈只补偿 `b*v_target`，不会自动抵消恒定外部负载。Phase 3 的三轴协调、S 曲线规划、Qt/ROS2/Gazebo 和硬件接口没有提前实现。
