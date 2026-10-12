# MotionCore Phase 4 验收记录

本记录对应最终提交的 Debug 构建和真实程序输出。所有命令均在 WSL Ubuntu 的
`/mnt/d/运动控制项目` 执行，规划器内部使用 SI 单位，CSV 和下文指标使用 mm 系列单位。

## 1. 验收阈值和判定

| 项目 | 阈值或独立理论值 |
| --- | --- |
| 几何位置、终点位置 | 绝对误差 ≤ `1e-6 mm` |
| 标准直线长度 | `sqrt(30²+40²+50²) = 70.7106781187 mm` |
| 标准直线梯形时间 | `1.9142135624 s` |
| 标准直线 S 曲线时间 | `2.1142135624 s` |
| 标准直线半程 | `(15,20,25) mm` |
| 标准圆弧长度 | `20*pi = 31.4159265359 mm` |
| 标准圆弧半程 | `(14.1421356237,14.1421356237,0) mm` |
| 闭环保持 | 轨迹结束后继续 `2 s`，最终误差和速度单独报告 |
| 圆弧约束 | 参考轨迹合成加速度、Jerk 不超过给定上限；超限构造直接拒绝 |

调节/恢复时间采用固定采样判定：进入 `±0.2 mm` 误差带后，后续采样点连续保持
`0.2 s` 才算稳定。Phase 4 的程序同时记录闭环瞬态最大误差、RMSE、路径偏差、同步误差、
实际速度/加速度和驱动力；规划器误差和虚拟电机跟踪误差不混合验收。

## 2. Debug Clean Build 与全部回归测试

实际执行：

```text
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --clean-first -j2
ctest --test-dir build --output-on-failure
```

构建目标包含 `motioncore_phase4` 和 `motioncore_phase4_tests`。CTest 实际输出：

```text
1/6 Test #1: pid_unit_test ....................   Passed    0.00 sec
2/6 Test #2: motioncore_phase1_test ...........   Passed    0.00 sec
3/6 Test #3: motioncore_phase2_test ...........   Passed    0.01 sec
4/6 Test #4: motioncore_phase3_test ...........   Passed    0.00 sec
5/6 Test #5: motioncore_phase4_test ...........   Passed    0.01 sec
6/6 Test #6: motioncore_phase4_csv_test .......   Passed    0.04 sec

100% tests passed, 0 tests failed out of 6
Total Test time (real) =   0.11 sec
```

Phase 4 测试包含 Vector3 运算、短距离三角形、直线正反向和单轴分支、标准梯形/S 曲线
时间、任意时刻采样、S 曲线边界连续性、圆弧 CW/CCW/跨 0°/180°/大于 180°/整圆、
合成约束和非法参数、多段拐角停速、不同轴动力学参数、三轴闭环保持、异常输入及
`motioncore_phase4_csv_test` 的 CSV 头字段和数据行完整性检查。

## 3. Phase 3 闭环问题复核

复核命令使用标准 0→100 mm S 曲线（总规划时间 2.7 s）：

```text
./build/motioncore_phase3 --profile scurve --mode closed-loop ... \
  --output results/phase3/phase4_review_transient.csv
profile=scurve, mode=closed-loop, duration_s=2.7, final_reference_mm=100,
final_actual_mm=103.526269715, final_actual_velocity_mm_s=-1.19774804185,
peak_drive_force_N=0.239009681226, max_tracking_error_mm=3.53352679785

./build/motioncore_phase3 --profile scurve --mode closed-loop ... --hold-s 2 \
  --output results/phase3/phase4_review_hold2s.csv
profile=scurve, mode=closed-loop, duration_s=2.7, final_reference_mm=100,
final_actual_mm=100.000021399, final_actual_velocity_mm_s=-8.1653336092e-05,
peak_drive_force_N=0.239009681226, max_tracking_error_mm=3.53352679785
```

`103.526 mm` 是轨迹结束采样点的瞬态超调/跟踪误差，不是最终稳定误差。保持阶段没有
覆盖实际状态，也没有删除误差数据；原有积分抗饱和控制器继续工作，2 s 后位置误差约
`0.000021399 mm`、速度绝对值约 `0.0000816533 mm/s`。因此 Phase 3 闭环问题定位为
结束时刻的瞬态，未发现数值发散或积分状态泄漏。

## 4. 独立规划器标准结果

### A：三轴直线（0,0,0）→（30,40,50）mm

独立公式给出 `L=sqrt(30²+40²+50²)=70.7106781187 mm`。实际程序输出：

```text
geometry=line, profile=trapezoid, mode=planner,
duration_s=1.91421356237, final_reference_xyz_mm=30,40,50,
final_reference_path_mm=70.7106781187

geometry=line, profile=scurve, mode=planner,
duration_s=2.11421356237, final_reference_xyz_mm=30,40,50,
final_reference_path_mm=70.7106781187
```

插补器单元测试用独立路径长度和相应梯形公式验算时间，并在路径进度 50% 验证参考点
`(15,20,25) mm`；终点位置误差小于 `1e-12 mm`，终点速度为零。

### B：XY 圆弧

圆心 `(0,0)`、起点 `(20,0)`、终点 `(0,20)` mm、CCW、`vmax=20`、`amax=40`、
`jmax=200`（mm 系列）以及合成限制 `100/500` 的实际输出：

```text
geometry=arc, profile=scurve, mode=planner,
duration_s=2.27079632679, final_reference_xyz_mm=0,20,0,
final_reference_path_mm=31.4159265359
```

独立几何公式给出 `L=R*pi/2=31.4159265359 mm`，弧长 50% 的参考点为
`(14.1421356237,14.1421356237,0) mm`。`ArcInterpolator::kinematics()` 分别计算切向、
法向和合成加速度/Jerk；标准场景满足给定合成限制。CW、180°、跨越 0°、大于 180°和显式
整圆均由 CTest 数值断言覆盖；CW 标准端点的实际规划时间为 `5.41238898038 s`（270°长弧）。

### C～F：反向、单轴、多段和非法参数

CTest 实际通过了反向直线、仅 X 轴、仅 XY 轴、零位移、短距离三角形、临界距离、1 mm
S 曲线、相邻多段几何连续和拐角速度归零，以及零半径、半径不一致、NaN/无穷约束、非法
方向和非有限采样时间。多段应用示例为直线—圆弧—直线队列，实际总规划时间
`3.19438534175 s`。

## 5. 三轴闭环与误差指标

所有闭环命令均采用 1 ms 步长并在轨迹结束后保持 2 s。以下是程序实际输出：

| 场景 | 规划时间(s) | 最终三维误差(mm) | 最终速度(mm/s) | 最大跟踪误差(mm) | RMSE(mm) | 最大路径偏差(mm) | 最大同步误差 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 直线 S 曲线 | 2.11421356237 | 0.0000548808 | 0.000210955 | 0.555626 | 0.231071 | 6.09e-14 | 1.89e-15 |
| XY 圆弧 S 曲线 | 2.27079632679 | 0.0000213802 | 0.000082183 | 0.228104 | 0.106270 | 0.112914（径向） | 0.00713788 |
| 多段直线—圆弧—直线 | 3.19438534175 | 0.0000319657 | 0.000122879 | 0.581743 | 0.308519 | 0.481710 | 0.0334301 |

直线场景峰值驱动力 `0.225803 N`，圆弧 `0.091616 N`，多段 `0.214310 N`，均未触发
20 N 驱动力饱和。闭环的实际峰值速度/加速度/Jerk 分别为直线
`52.5693 mm/s / 115.6683 mm/s² / 603.233 mm/s³`、圆弧
`21.0764 mm/s / 47.6872 mm/s² / 268.201 mm/s³`；这些是动力学跟踪瞬态，规划器参考
轨迹本身仍满足标量和圆弧合成约束，二者在报告中分开统计。

## 6. CSV 和图形

真实生成的 CSV 位于 `results/phase4/`：

```text
line_trapezoid_planner.csv
line_scurve_planner.csv
line_scurve_closed_loop.csv
arc_scurve_planner.csv
arc_scurve_closed_loop.csv
arc_cw_planner.csv
multi_scurve_closed_loop.csv
```

绘图命令：

```text
python3 scripts/plot_phase4.py --line results/phase4/line_scurve_closed_loop.csv \
  --arc results/phase4/arc_scurve_closed_loop.csv --output-dir results/phase4
```

实际生成 `xyz_position.svg`、`xyz_velocity.svg`、`trajectory_3d.svg`、`arc_xy.svg`、
`tracking_error.svg`、`synchronization_error.svg`。CSV 中规划参考与闭环实际状态使用独立列，
梯形规划的理论 Jerk 以 NaN 表示不适用的边界冲激，没有伪装成有限 Jerk。

## 7. 未实现和技术限制

当前只支持零起止速度和 XY 平面圆弧；不包含在线重规划、非零边界速度、路径前瞻、连续
拐角非零速度、五轴姿态、真实编码器噪声、电机电气模型、Qt、ROS2、Gazebo 或 EtherCAT。
闭环虚拟电机的速度/加速度瞬态可能高于参考约束，已作为独立性能指标记录；若实际设备需要
硬约束，应在控制器或执行器层增加限幅和前馈整定，而不能把实际状态直接改写成参考轨迹。
