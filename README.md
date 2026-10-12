# MotionCore：工业运动控制与数字孪生仿真平台

MotionCore 是一个面向机器人 C++ 软件、半导体设备软件和自动化软件岗位的可复现学习项目。项目在 Windows + WSL2 + Ubuntu + VS Code 中运行，不需要实体电机或控制器。Phase 1 的单轴运动学代码和 `cpp_learning` 学习代码保持不变；当前新增的是 Phase 2 电机动力学与伺服控制。

## Phase 2 做了什么

动力学模型在内部统一使用 SI 单位：位置 m、速度 m/s、加速度 m/s²、质量 kg、力 N。模型满足

```text
m * dv/dt = F_drive - b * v - F_load
dx/dt = v
```

每个 1 ms 周期先根据当前速度计算加速度，再用常加速度积分更新位置和速度：

```text
x[k+1] = x[k] + v[k] * dt + 0.5 * a[k] * dt²
v[k+1] = v[k] + a[k] * dt
```

驱动力在模型入口处限制到 `[-max_drive_force, +max_drive_force]`。目标速度、实际速度和实际加速度是三个独立量，控制器不会把目标速度直接写入模型状态。`DynamicAxisModel` 也可以直接用 `step(force, dt)` 做开环动力学测试。

串级控制器采用外位置 P 环和内速度 P/PI 环：

```text
位置误差 -> 位置 P -> 目标速度限幅 -> 目标速度加速度斜率限制
          -> 速度误差 -> 速度 P 或 PI -> 驱动力 -> 动力学模型
```

`PController`、`PIController` 和 `PIDController` 可独立复用。PI/PID 包含输出限幅、积分限幅、条件积分抗饱和和 `reset()`；PID 微分项对测量值求导并做一阶滤波，因此设定值突变不会产生微分冲击。串级控制器可选粘性阻尼前馈 `F_ff = b * v_target`。

## 固定参数和四组实验

默认参数为 `m=1 kg`、`b=3 N·s/m`、最大驱动力 `20 N`、步长 `0.001 s`、时长 `8 s`、目标位置 `100 mm`。位置验收指标使用 ±0.2 mm 误差带，并要求连续保持 0.2 s；调节时间定义为首次进入该误差带且之后 0.2 s 内每个采样点都保持在带内。指标脚本还输出最大超调、最终位置误差、最终速度、最大实际速度和最大实际加速度。

在项目根目录运行：

```bash
./build/motioncore_phase2 --scenario nominal      # A：无扰动到 100 mm
./build/motioncore_phase2 --scenario disturbance  # B：2 N 负载，2.0~2.5 s
./build/motioncore_phase2 --scenario comparison   # C：恒定负载下速度 P/PI 对比
./build/motioncore_phase2 --scenario feedforward  # D：阻尼前馈开关对比
```

每组实验会在 `results/phase2/` 写入 CSV 和对应的 `<scenario>_metrics.csv`。CSV 至少包含：

```text
time_s,target_position_mm,actual_position_mm,target_velocity_mm_s,
actual_velocity_mm_s,actual_acceleration_mm_s2,drive_force_N,
load_force_N,position_error_mm
```

可用标准库脚本绘制位置、速度、误差和驱动力四联 SVG 图：

```bash
python3 scripts/plot_phase2.py results/phase2/nominal.csv
```

## 构建和测试

在 VS Code 的 WSL 终端执行：

```bash
cd /mnt/d/运动控制项目
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

原有 `pid_unit_test`、`motioncore_phase1_test` 和 Phase 1 可执行程序都保留；新增 `motioncore_phase2_test` 覆盖动力学公式、正反向运动、控制器限幅与抗饱和、PID 测量微分、闭环到位、扰动恢复、非法参数和 CSV 完整性。完整验收记录见 [PHASE2_ACCEPTANCE.md](PHASE2_ACCEPTANCE.md)，技术说明见 [docs/phase2.md](docs/phase2.md)。

当前完整回归还包括 `motioncore_phase3_test`；运行 `ctest --test-dir build --output-on-failure` 会同时执行前三阶段测试。

## 当前阶段：Phase 3 轨迹规划

Phase 3 新增 `TrapezoidalProfile` 和限 Jerk 的 `SCurveProfile`。两者都通过 `TrajectoryProfile` 和 `TrajectorySampler` 提供任意时刻解析采样，输出时间、参考位置、参考速度、参考加速度和参考 Jerk。梯形规划自动在标准梯形和三角形之间切换；S 曲线根据距离自动退化为无匀速、无恒加速度或四段 Jerk 轨迹。当前统一接口明确只接受零起止速度，非零边界速度会抛出参数错误。

标准验收参数为起点 0 mm、终点 100 mm、最大速度 50 mm/s、最大加速度 100 mm/s²、最大 Jerk 500 mm/s³。梯形理论总时间为 2.5 s，七段 S 曲线理论总时间为 2.7 s。Phase 3 还提供把 `v_ref + Kp*(x_ref-x_actual)` 接入 Phase 2 速度 PI 和动力学模型的闭环模式。

```bash
./build/motioncore_phase3 --profile trapezoid --mode planner \
  --start-mm 0 --target-mm 100 --vmax-mm-s 50 \
  --amax-mm-s2 100 --jmax-mm-s3 500 --dt-ms 1 \
  --output results/phase3/trapezoid.csv

./build/motioncore_phase3 --profile scurve --mode planner \
  --start-mm 0 --target-mm 100 --vmax-mm-s 50 \
  --amax-mm-s2 100 --jmax-mm-s3 500 --dt-ms 1 \
  --output results/phase3/scurve.csv

./build/motioncore_phase3 --profile scurve --mode closed-loop \
  --start-mm 0 --target-mm 100 --vmax-mm-s 50 \
  --amax-mm-s2 100 --jmax-mm-s3 500 --dt-ms 1 \
  --output results/phase3/closed_loop.csv
```

轨迹 CSV 和图形写入 `results/phase3/`。运行 `scripts/plot_phase3.py` 可以生成速度对比、加速度对比、S 曲线 Jerk、规划/实际位置和跟踪误差五张 SVG 图。Phase 3 的完整验收记录见 [PHASE3_ACCEPTANCE.md](PHASE3_ACCEPTANCE.md)，技术说明见 [docs/phase3.md](docs/phase3.md)。

## 当前阶段：Phase 4 XYZ 三轴协调插补

Phase 4 在 Phase 3 标量轨迹之上增加了 `Vector3`、`LineInterpolator`、`ArcInterpolator`、
`CoordinatedTrajectoryPlanner` 和 `MultiAxisServoSystem`。直线和 XY 圆弧都只使用一个
标量路径进度，因此三轴共享统一的运动时间；圆弧额外计算向心加速度和曲率 Jerk，有限
合成约束下先保守降速，仍无法建立正约束时拒绝指令。多段路径采用拐角减速到零的保守衔接策略。内部单位为 m、m/s、
m/s²、m/s³，命令行和 CSV 使用 mm 系列单位。

```bash
./build/motioncore_phase4 --geometry line --profile scurve --mode planner \
  --start-mm 0,0,0 --target-mm 30,40,50 --vmax-mm-s 50 \
  --amax-mm-s2 100 --jmax-mm-s3 500 --dt-ms 1 \
  --output results/phase4/line_scurve_planner.csv

./build/motioncore_phase4 --geometry arc --profile scurve --mode closed-loop \
  --start-mm 20,0,0 --target-mm 0,20,0 --center-mm 0,0,0 \
  --direction ccw --vmax-mm-s 20 --amax-mm-s2 40 --jmax-mm-s3 200 \
  --dt-ms 1 --hold-s 2 --output results/phase4/arc_scurve_closed_loop.csv

python3 scripts/plot_phase4.py --line results/phase4/line_scurve_closed_loop.csv \
  --arc results/phase4/arc_scurve_closed_loop.csv --output-dir results/phase4
```

`--geometry multi-demo` 可运行一条直线—圆弧—直线的固定多段示例；该示例会显式传递
圆弧合成加速度/Jerk 限制并在曲率超限时保守降速。固定示例拒绝显式几何坐标参数，避免
悄悄忽略用户输入。`--mode closed-loop` 会用三个
独立动力学轴和串级伺服跟踪参考轨迹，并在轨迹结束后继续保持。完整公式、指标定义、
限制和真实验收输出见 [docs/phase4.md](docs/phase4.md)、[PHASE4_FIX_ACCEPTANCE.md](PHASE4_FIX_ACCEPTANCE.md)
与 [PHASE4_ACCEPTANCE.md](PHASE4_ACCEPTANCE.md)。

## 目录结构

```text
include/motioncore/model/       Phase 1/2 被控对象模型
include/motioncore/control/     P、PI、PID 和串级控制器
include/motioncore/planning/    梯形、三角形和限 Jerk S 曲线规划器
include/motioncore/multiaxis/   XYZ 向量、直线/圆弧插补、路径队列和三轴伺服
include/motioncore/metrics/     仿真指标计算
include/motioncore/io/          CSV 输出
src/motioncore/                 对应实现
apps/                           Phase 1/2/3/4/5 仿真程序
gui/                            Phase 6 Qt Widgets 上位机与仿真 worker
tests/                          CTest 单元测试
scripts/                        Phase 2/3/4/5 绘图脚本
docs/phase6.md                  Qt 线程、数据流和已知限制
results/phase2/                 Phase 2 CSV、指标和 SVG
results/phase3/                 Phase 3 轨迹 CSV 和 SVG
results/phase4/                 Phase 4 三轴 CSV 和 SVG
results/phase5/                 Phase 5 管理器 CSV、事件日志和 SVG
```

Phase 4 当前仍只承诺零起止速度、XY 平面圆弧和离线仿真，不包含在线重规划、非零边界速度、
五轴姿态、真实编码器噪声、电机电气模型、ROS2、Gazebo 或 EtherCAT。Phase 6 提供可选
Qt 上位机，但仍属于软件数字孪生，不连接真实硬件。

## 当前阶段：Phase 5 运动控制管理系统

Phase 5 新增 `MotionStateMachine`、`MotionCommandQueue`、`HomingController`、
`SoftLimitManager`、`FaultManager` 和统一 `MotionController`。管理层复用 Phase 3/4 的
插补器与 `MultiAxisServoSystem`，不直接覆盖实际位置。回零通过 SearchFast、Backoff、
SearchSlow、ApplyOffset 和真实动力学减速完成；Stop 是受控减速，EmergencyStop 锁存软件
状态并保留惯性滑行。所有轴在同一个 1 ms 调度周期内更新。

```bash
./build/motioncore_phase5 --scenario homing --output results/phase5
./build/motioncore_phase5 --scenario nominal --output results/phase5
./build/motioncore_phase5 --scenario queue --output results/phase5
./build/motioncore_phase5 --scenario soft-limit --output results/phase5
./build/motioncore_phase5 --scenario controlled-stop --output results/phase5
./build/motioncore_phase5 --scenario emergency-stop --output results/phase5
./build/motioncore_phase5 --scenario fault-recovery --output results/phase5
python3 scripts/plot_phase5.py results/phase5/nominal.csv --output-dir results/phase5
```

Phase 5 的状态、指令、回零、故障、限位裕量和驱动力写入 CSV，并将状态转换和命令生命周期写入事件日志。
完整调度顺序、停止距离公式和限制见 [docs/phase5.md](docs/phase5.md) 与 [PHASE5_ACCEPTANCE.md](PHASE5_ACCEPTANCE.md)。

### Phase 5.1 缺陷修复

Phase 5.1 对急停取消命令、运行中实际软限位和 Phase 4 Jerk CSV 单位进行了回归修复。
队列只有 `Queued` 命令能够进入 `Running`；实际轴每个 1 ms 步检查位置和
`v*dt+v²/(2*a_brake)` 预测制动距离，危险时锁存 `SoftLimitViolation`。Phase 4 的
`jx_ref_mm_s3`、`jy_ref_mm_s3`、`jz_ref_mm_s3` 和
`jerk_ref_magnitude_mm_s3` 全部以 mm/s³ 导出，模长由数值测试验证为三个分量的欧氏模。
具体修复和真实输出见 [PHASE5_FIX_ACCEPTANCE.md](PHASE5_FIX_ACCEPTANCE.md)。

Phase 5.1 回归命令：

```bash
./build_phase5/phase5_1_queue_regression_test
./build_phase5/phase5_1_dynamic_limit_test
python3 tests/check_phase4_csv.py results/phase4/multi_demo_phase5_1.csv
```

Qt 6 已在 WSL Ubuntu 中安装并实际检测为 6.4.2。GUI 通过 `MOTIONCORE_BUILD_GUI=ON`
单独构建，核心库、仿真程序和既有测试仍不依赖 Qt：

```bash
cmake -S . -B build_phase6 -DCMAKE_BUILD_TYPE=Debug -DMOTIONCORE_BUILD_GUI=ON
cmake --build build_phase6 -j2
QT_QPA_PLATFORM=offscreen ./build_phase6/motioncore_gui
ctest --test-dir build_phase6 --output-on-failure
```

GUI 架构、线程模型和已知限制见 [docs/phase6.md](docs/phase6.md)，实际验收见
[PHASE6_ACCEPTANCE.md](PHASE6_ACCEPTANCE.md)。无显示环境使用 offscreen 进行 Qt 对象、
信号槽、控制器数据和 CSV 导出测试；这不代表真实硬件或硬实时验收。

## English overview

MotionCore is a C++17/CMake digital-twin platform for XYZ industrial motion control. It
contains a mass-damping-load model, reusable P/PI/PID and cascaded servo control,
trapezoidal and jerk-limited S-curve profiles, coordinated line and XY arc interpolation,
homing, software limits, controlled stop, emergency-stop and fault simulation. An optional
Qt 6 Widgets application connects to the same `MotionController` through a worker thread.
CTest, CSV analysis scripts and sanitizer builds provide reproducible numerical checks.

The project is software simulation only: it does not connect to real drives, EtherCAT,
STO or safety PLCs, and a 1 ms numerical step is not a hard-real-time guarantee. The GUI
currently provides a clearly labelled XY trajectory projection rather than a full 3D renderer.
See the technical documents in `docs/`, especially [PROJECT_OVERVIEW](docs/PROJECT_OVERVIEW.md),
[PROJECT_ARCHITECTURE](docs/PROJECT_ARCHITECTURE.md), [BUILD_AND_RUN](docs/BUILD_AND_RUN.md),
and [FINAL_RELEASE_ACCEPTANCE](FINAL_RELEASE_ACCEPTANCE.md).


## 最终本地验收与交付

最终源码包为 `MotionCore_v1.0.0_Source.zip`，不包含构建目录、旧压缩包或二进制缓存。
本轮实际结果为：核心 Debug 19/19、核心 Release 19/19、Qt offscreen 22/22、ASan+UBSan 核心 19/19；完整终端摘要见 `MotionCore_Test_Results/`，技术文档汇总见 `MotionCore_Technical_Documentation/`。

Qt 界面支持 Load CSV Replay。回放会校验表头、有限数值和严格递增时间，只更新快照，不调用运动控制器。界面轨迹图是 XY 二维投影，人工图形操作仍需在有显示环境的机器上完成；本项目没有真实硬件或工业安全认证。

GitHub 远程、仓库可见性、许可证和 tag 尚未由仓库所有者确认，因此当前没有伪称已上传或已发布。
