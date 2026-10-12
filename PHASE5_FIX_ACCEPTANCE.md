# MotionCore Phase 5.1 缺陷修复验收

本阶段先于 Qt 上位机开发执行，保留 Phase 1--5 的源代码、接口和原有
CTest 入口。修复针对独立审查中复现的三项问题，并补充真实数值回归测试。

## 历史问题与修复

| 问题 | 修复前的复现现象 | 修复后的行为 |
|---|---|---|
| 急停取消命令 | 急停时第二条命令为 `canceled`，复位并重新回零后仍进入 `Running` | 队列只返回 `Queued` 命令；终态命令永久不可再次运行 |
| 动态软限位 | X 行程 `[0,100] mm`、目标 `99.9 mm` 时实际位置达到 `100.282 mm`，状态仍为 `Ready`、故障为 `None` | 每个动力学步检查实际位置和预测制动距离；危险时锁存 `SoftLimitViolation`，保留实际越界数值 |
| Jerk CSV 单位 | `jx_ref_mm_s3=500` 而模长为 `0.5` | 三个分量和模长统一为 mm/s³，模长逐行验证为 `sqrt(jx²+jy²+jz²)` |

历史数值是修复前独立复现记录；当前版本未覆盖或删除这些数据，便于审计前后差异。

## 实现位置

- `src/motioncore/motion_manager/motion_command_queue.cpp`：终态命令从可执行视图中排除，`Queued -> Running -> terminal` 生命周期严格检查。
- `src/motioncore/motion_manager/motion_controller.cpp`：`monitor_dynamic_limits()` 在伺服步后检查实际位置、预测停止距离和故障快照；禁用时取消活动命令；Dwell 参考使用机床坐标；故障复位检查原因是否已消除。
- `apps/motioncore_phase4.cpp`：Jerk 模长按 mm/s³ 导出。
- `tests/phase5_1_queue_regression_test.cpp`：急停、取消、复位、重新回零回归。
- `tests/phase5_1_dynamic_limit_test.cpp`：正向和反向 99.9 mm 近限位运动的真实动力学回归。
- `tests/check_phase4_csv.py`：CSV 数值单位和模长关系检查。

运行中的限位检查采用实际模型返回的位置，不裁剪或覆盖位置。距离限位小于
`0.005 mm` 的静止数值抖动留给位置检查容差处理；预测停止距离仍按
`v*dt + v²/(2*a_brake)` 计算，超过可用裕量则立即锁存故障。

## 验收命令与真实输出

```bash
cd /mnt/d/运动控制项目
cmake -S . -B build_phase5 -DCMAKE_BUILD_TYPE=Debug
cmake --build build_phase5 -j2
ctest --test-dir build_phase5 --output-on-failure
```

当前 Debug 构建成功；CTest 输出为：

```text
100% tests passed, 0 tests failed out of 18
```

其中原有 15 个入口全部保留，新增入口为：

```text
phase5_1_queue_lifecycle_test
phase5_1_dynamic_limit_test
phase5_1_jerk_unit_test
```

缺陷回归可单独复现：

```bash
./build_phase5/phase5_1_queue_regression_test
./build_phase5/phase5_1_dynamic_limit_test
python3 tests/check_phase4_csv.py results/phase4/multi_demo_phase5_1.csv
```

实测动态限位回归分别在 `x=0.0899641 m`、`x=-0.0899634 m`，速度约
`±0.100162 m/s` 时报告预测制动距离约 `0.0101326 m`，两种方向均进入
`Fault/SoftLimitViolation`，没有把危险轨迹报告为 `Ready`。

Phase 4 多段示例重新运行：

```text
geometry=multi-demo, profile=scurve, mode=planner,
duration_s=3.63875621078,
final_reference_xyz_mm=40,20,0,
final_reference_path_mm=55.7079632679
validated 3639 Phase 4 CSV rows
```

CSV 首行 Jerk 数值实测为 `500,0,0,500`，Python 检查逐行确认模长关系。
多段示例的时间未因单位修复改变；曲率约束造成的保守降速仍由 Phase 4.1
规划器负责。

Phase 5 七个场景也在同一 Debug 构建中重新运行：

```text
homing          Ready, position_mm=0.0102972,0.0232082,0.0102966
nominal         Ready, final_position_mm=29.975,39.9666,49.9583,
                final_velocity_mm_s=0.0874933, following_error_mm=0.0589737
queue           Ready, final_position_mm=10.0258,10.0515,0,
                final_velocity_mm_s=0.0881859, fault=None
soft-limit      out_of_limit_accepted=0
controlled-stop Ready, final_velocity_mm_s=0.242721
emergency-stop  EmergencyStopped, fault=EmergencyStopTriggered
fault-recovery  Ready, final_position_mm=0.0101422,0.0228288,0.0101292
```

附加 sanitizer 构建使用 AddressSanitizer + UndefinedBehaviorSanitizer：

```text
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build_phase5_asan --output-on-failure
100% tests passed, 0 tests failed out of 18
```

## 复位和停止边界

Fault 复位会检查软限位位置、回零传感器故障和数值有限性；原因仍存在时返回
失败。EmergencyStop 会取消未执行命令、锁存故障并保持驱动禁能后的惯性/阻尼
动力学，复位后必须重新回零。Controlled Stop 取消活动命令并继续推进物理轴，
不会瞬间把位置或速度改成零。

## 当前结论

Phase 5.1 缺陷修复门槛已通过，保留 18 个核心/回归 CTest 入口。随后已安装 Qt 6.4.2，
完成 Phase 6 可选 Qt Widgets 上位机、独立仿真线程、XY 轨迹/遥测显示、事件日志和 CSV
导出。Phase 6 的实际构建与测试记录见 [PHASE6_ACCEPTANCE.md](PHASE6_ACCEPTANCE.md)。

GUI 仍是软件数字孪生：本机使用 `QT_QPA_PLATFORM=offscreen` 验证 Qt 对象、信号槽、
MotionController 数据交互和 CSV 导出，没有连接真实驱动器，也不宣称硬实时或安全认证。
可视化窗口目前是 XY 投影，Z 轴状态保存在快照和 CSV 中。

## 最终收尾回归补充

Phase 6 接入后又增加了 Fault 惯性连续性、核心队列重定基等最终回归入口。最新核心
Debug 运行保留原有 18 个入口并扩展为 19 个，实际结果为 `100% tests passed, 0 failed
out of 19`，总耗时 0.86 sec；Qt offscreen 构建包含 22 个测试，结果见
`PHASE6_ACCEPTANCE.md`。核心回归的实际数值为：Fault 后 1 s 位置
`0.0347233852301 → 0.356719010826` mm，速度 `1.0178876855 → 0.0504496516611`
mm/s；两条直线和 line→arc→line 均为 completed，终点分别为
`59.9458116347` mm 与 `(10.0258059715,10.0515265982,5e-13)` mm。
