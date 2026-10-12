# MotionCore Phase 4：XYZ 三轴协调运动与插补

Phase 4 在 Phase 1 的动力学模型、Phase 2 的串级伺服和 Phase 3 的解析轨迹规划器
之上增加三轴协调层。规划器负责“应该走哪一条几何路径”和“路径在什么时候走到哪
里”，轴模型仍然只根据驱动力计算实际运动。参考坐标绝不会写入实际轴状态，因此
规划误差和闭环跟踪误差可以独立验收。

## 单位和时钟

模块内部统一使用 SI 单位：位置为 m，速度为 m/s，加速度为 m/s²，Jerk 为 m/s³，
时间为 s。命令行参数和 CSV 为便于阅读使用 mm、mm/s、mm/s²、mm/s³。三个轴共享
同一个仿真时钟，默认步长为 1 ms；采样器接受任意非负时间，最后一个采样点可以
精确落在轨迹总时间上，不要求总时间是步长的整数倍。

非有限数、负速度/加速度/Jerk、零半径等参数在构造规划器时拒绝。当前边界速度继承
Phase 3 的约定，仅支持起止速度为零；传入非零边界速度必须得到明确的参数错误。

## 三维直线插补

给定起点 `P0` 和终点 `P1`，先计算路径长度和单位方向：

```text
d = P1 - P0
L = ||d||
u = d / L                 (L > 0)
P(t) = P0 + u * s(t)
V(t) = u * s_dot(t)
A(t) = u * s_ddot(t)
```

其中 `s(t)` 是由 Phase 3 `TrapezoidalProfile` 或 `SCurveProfile` 对标量距离
`L` 生成的唯一进度。三个坐标轴因此共享同一总时间和同一进度，不能由三个独立
单轴规划器拼接。`L=0` 时返回起点、零速度和零加速度的零时长轨迹。

梯形模式使用 `vmax`、`amax` 计算加速、匀速、减速三段；距离不足时自动成为三角形。
梯形加速度在边界跳变，理论 Jerk 是冲激，导出时应标记为不适用；需要有限 Jerk
时使用 S 曲线。

## XY 平面圆弧插补

输入包括圆心 `C=(cx,cy)`、起终点、CW/CCW 方向和运动约束。起点半径与终点半径
必须在浮点容差内相等且大于零。由起点和终点角度计算有向角差 `DeltaTheta`：

```text
x = cx + R cos(theta)
y = cy + R sin(theta)
z = z0
L = R |DeltaTheta|
theta(t) = theta0 + sign(direction) * s(t) / R
```

起点和终点相同默认表示零位移；整圆必须由调用方显式提供 `full_circle=true`，避免
把“已经到达终点”误判成一整圈。圆弧位置、速度和加速度由弧长解析求导，不能用
三个独立轴的结果近似。

曲率带来法向项。令 `T` 为切向单位向量，`N` 为从圆心指向轨迹点的径向单位向量，
切向速度、加速度和 Jerk 分别为 `v`、`a_t`、`j_t`，则：

```text
A = a_t T - (v²/R) N
J = (j_t - v³/R²) T - (3 v a_t/R) N
```

因此标量 `a_t`、`j_t` 满足限制并不代表合成向心加速度和 Jerk 也满足限制。规划器
需要在曲率约束下检查 `|A|`、`|J|`；若给定参数不可行，应降低标量速度后重规划
或拒绝指令并说明原因，不能在超限后仍报告为满足约束。

## 多轴伺服闭环

`MultiAxisServoSystem` 为 X、Y、Z 各创建独立 `DynamicAxisModel` 和串级控制器，
但每个 1 ms tick 使用同一份 `MultiAxisTrajectoryState`。控制器使用 Phase 3 的
轨迹跟踪接口：

```text
v_command = v_ref + Kp_position * (x_ref - x_actual)
```

随后由既有目标速度限幅、速度 PI、阻尼前馈和驱动力限幅完成动力学更新。一个轴提前
到达不会终止其它轴；轨迹结束后仍进入位置保持阶段，默认至少继续 2 s，记录各轴
最终位置、速度、驱动力和保持误差。

## 多段路径和拐角

路径队列由 `MotionSegment` 保存直线或圆弧的几何参数、约束和标量规划对象。
`CoordinatedTrajectoryPlanner` 顺序执行队列；当前对尖锐拐角采用保守策略：上一段
先减速到零，再从下一段重新加速。这样不在方向突变处产生理想无限加速度。路径前瞻、
连续曲率和非零拐角速度属于后续扩展。

## 误差和约束指标

规划器精度与闭环精度必须分开记录。常用指标定义如下：

```text
e_xyz(t) = ||P_actual(t) - P_ref(t)||
RMSE     = sqrt(mean(e_xyz²))
line_dev = distance(P_actual, target line)
arc_dev  = |sqrt((x-cx)² + (y-cy)²) - R|
sync_err = max_i |s_i/L_i - s_j/L_j|
```

直线场景按参与运动轴的归一化坐标进度计算最大差值；圆弧场景按实际角度弧长进度与
参考进度的归一化差值计算，因为圆弧的 X/Y 坐标不是线性轴向进度。

另记录最终三维定位误差、各轴最终速度、最大驱动力及逐轴饱和次数、最大实际速度/加速度
和圆弧合成约束。`line_dev` 使用点到有限线段的最近距离；圆弧同时记录有限圆弧最近
距离和 `radial_error_mm = |r_actual-R|`，不能把完整圆周的半径误差冒充有限圆弧距离。

## 运行和可视化

Phase 4 程序名为 `motioncore_phase4`。典型命令如下（参数名称以程序 `--help` 为准）：

```bash
./build/motioncore_phase4 --geometry line --profile scurve --mode planner \
  --start-mm 0,0,0 --target-mm 30,40,50 --vmax-mm-s 50 \
  --amax-mm-s2 100 --jmax-mm-s3 500 --dt-ms 1 \
  --output results/phase4/line_planner.csv

./build/motioncore_phase4 --geometry arc --profile scurve --mode closed-loop \
  --start-mm 20,0,0 --target-mm 0,20,0 --center-mm 0,0,0 \
  --direction ccw --vmax-mm-s 20 --amax-mm-s2 40 --jmax-mm-s3 200 \
  --hold-s 2 --output results/phase4/arc_closed_loop.csv
```

规划 CSV 至少包含 `time_s,x_ref_mm,y_ref_mm,z_ref_mm` 及参考速度/加速度列，参考 Jerk
拆分为 `jx_ref_mm_s3,jy_ref_mm_s3,jz_ref_mm_s3,jerk_ref_magnitude_mm_s3`；闭环
CSV 还包含实际三轴状态、`tracking_error_mm`、`path_deviation_mm`、驱动力和负载。
可视化脚本不依赖第三方库：

```bash
python3 scripts/plot_phase4.py --line results/phase4/line_closed_loop.csv \
  --arc results/phase4/arc_closed_loop.csv --output-dir results/phase4
```

它会生成 XYZ 位置和速度、三维等距投影、XY 圆弧、跟踪误差和同步误差 SVG。

## 已知限制

当前阶段只承诺零起止速度和 XY 平面圆弧；不包含五轴姿态、NURBS、在线重规划、
路径前瞻、EtherCAT/ROS2、真实编码器噪声和硬件功放。圆弧合成 Jerk 检查需要有效的
切向 Jerk 数据；对梯形规划的边界冲激只能标记为不适用，不能伪装成有限 Jerk。
