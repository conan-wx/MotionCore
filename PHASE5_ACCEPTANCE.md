# MotionCore Phase 5 验收记录

## 功能实现

| 模块 | 实现文件 | 状态 |
|---|---|---|
| 状态机 | `include/src/motioncore/motion_manager/motion_state_machine.*` | 已实现并有非法转换断言 |
| FIFO/高优先级队列 | `motion_command_queue.*` | 已实现，命令 ID 和生命周期可查询 |
| 三轴回零 | `homing_controller.*` | 已实现 SearchFast、Backoff、SearchSlow、ApplyOffset、防抖和超时 |
| 软件限位 | `soft_limit_manager.*` | 已实现位置/有限线/有限圆弧极值检查和预测制动距离 |
| 故障管理 | `fault_manager.*` | 已实现故障码、锁存、历史和显式复位 |
| 统一调度器 | `motion_controller.*` | 已实现 1 ms 事件、规划、三轴伺服、Stop/EmergencyStop、日志状态 |
| 可执行仿真 | `apps/motioncore_phase5.cpp` | 七个确定性场景 |

状态转换遵循 `Disabled → NotHomed → Homing → Ready → Running`；Stop 进入
`ControlledStopping`，运动速度稳定后回到 Ready；故障和急停分别锁存到 Fault 和
EmergencyStopped，复位后都回到 NotHomed。未回零、急停和锁存故障均拒绝普通运动。

## 可复现命令

```bash
cd /mnt/d/运动控制项目
cmake -S . -B build_phase5 -DCMAKE_BUILD_TYPE=Debug
cmake --build build_phase5 --parallel 2
ctest --test-dir build_phase5 --output-on-failure
for s in homing nominal queue soft-limit controlled-stop emergency-stop fault-recovery; do
  ./build_phase5/motioncore_phase5 --scenario "$s" --output results/phase5
done
python3 scripts/plot_phase5.py results/phase5/nominal.csv --output-dir results/phase5
```

## 真实结果

以下数值由上述程序直接输出，CSV 保存在 `results/phase5/`，事件日志使用同一个仿真时间轴。

| 场景 | 结果 |
|---|---|
| homing | `Ready`，三轴机床坐标在传感器零点附近，速度由动力学减速后才报告完成 |
| nominal | `Ready`，终点约 `(29.975, 39.9666, 49.9583) mm`，最终速度约 `0.0875 mm/s`，跟踪误差约 `0.0590 mm` |
| queue | FIFO 依次启动直线、圆弧、Dwell 和末段直线，最终约 `(10.0258,10.0515,0) mm`；示例圆弧避开硬下限，尖角未使用非零速度切换 |
| soft-limit | 目标超出 `[0,100] mm` 时命令被拒绝，未修改实际轴状态 |
| controlled-stop | Stop 后约束减速，进入 Ready；最终速度约 `0.250 mm/s`，停止距离由真实动力学产生 |
| emergency-stop | 队列取消、状态锁存为 `EmergencyStopped`，`EmergencyStopTriggered` 已记录；驱动禁能后按惯性/阻尼滑行，示例最终 X 约 `7.93271 mm` |
| fault-recovery | FollowingError 注入后锁存 Fault；显式复位并重新回零，最终回零坐标约 `(0.01014,0.02283,0.01013) mm`，跟踪误差清零 |

## CTest 覆盖

保留 Phase 1–4 的 7 个测试（含 Phase 4.1 缺陷回归），新增：

`phase5_state_machine_test`、`phase5_command_queue_test`、`phase5_homing_test`、
`phase5_soft_limit_test`、`phase5_stop_test`、`phase5_fault_test`、
`phase5_integration_test`、`phase5_csv_test`。Phase 5.1 又增加了
`phase5_1_queue_lifecycle_test`、`phase5_1_dynamic_limit_test` 和
`phase5_1_jerk_unit_test`；当前实际结果为 `100% tests passed, 0 tests failed out of 18`。
测试包含状态非法转换、优先级队列、
回零、圆弧极值限位、停止距离、Stop/EmergencyStop 区别、故障复位和三轴终点数值断言。

Phase 4.1 回归在同一构建中保留并通过：原有 Phase 1–4 加 Phase 4.1 共 7 项测试全部通过。

在独立 AddressSanitizer/UndefinedBehaviorSanitizer 构建中执行：

```text
ASAN_OPTIONS=detect_leaks=0 ./build_phase5_asan/motioncore_phase5_tests all
phase5 all tests passed
```

## 限制与真实性边界

回零开关、急停和故障注入是确定性软件模型；没有真实编码器、驱动器、STO、安全 PLC、
EtherCAT 或实时调度保证。受控停止采用当前状态的零速度保持参考，支持本仿真模型的物理
减速；它不构成真实设备安全认证。当前仍使用零起止速度和拐角停车，未实现在线路径前瞻。
