# MotionCore v1.0.0 最终工程验收记录

日期：2026-10-12（Windows 主机 / WSL Ubuntu，C++17，CMake，Qt 6.4.2）。本文件只记录实际执行过的结果；offscreen 自动化不等同于人工图形验收，也不等同于真实硬件验证。

## 缺陷修复与回归

| 问题 | 修复位置 | 独立回归结果 |
|---|---|---|
| 急停后已取消命令可能再次执行 | `src/motioncore/motion_manager/motion_command_queue.cpp`、`motion_controller.cpp` | 队列生命周期测试及核心回归通过；Canceled/Rejected/Completed 不会启动 |
| 实际轴动态软限位未锁存 | `motion_controller.cpp`、`soft_limit_manager.cpp` | Phase 5.1 动态正/反向边界测试通过 |
| Jerk CSV 模长少乘 1000 | `apps/motioncore_phase4.cpp`、`tests/check_phase4_csv.py` | 5640 行最终多段 CSV、2115 行直线 CSV 均通过分量/模长校验 |
| Fault 状态冻结机械轴 | `motion_controller.cpp` | Fault 后 1 s：位置 0.0347233852301→0.356719010826 mm，速度 1.0178876855→0.0504496516611 mm/s；时钟继续推进 |
| Fault 运动中无条件复位 | `motion_controller.cpp` | 速度未降到阈值时复位被拒绝；故障原因消除且速度安全后才可复位 |
| 暂停时 Stop/EmergencyStop 不响应 | `motion_controller.cpp`、`gui/simulation_worker.cpp` | Qt 回归验证零时间处理和急停锁存 |
| 急停后惯性尾段不显示 | `gui/simulation_worker.cpp` | 急停后继续采样禁能驱动的动力学滑行 |
| GUI 连续命令起点过时 | `motion_command_queue.cpp`、`simulation_worker.cpp` | 两条直线均 completed；line→arc→line 最终 `(10.0258059715,10.0515265982,5.07e-13)` mm |
| GUI CSV 重复时间/历史截断 | `simulation_worker.cpp` | 只在仿真时间推进时写行；完整日志流式保存，20000 点仅为显示缓存 |
| CSV 离线回放缺少隔离 | `simulation_worker.cpp`、`mainwindow.cpp` | 真实导出 CSV 经表头、有限数值、严格递增时间校验后只读回放，不调用 MotionController |

## 实际构建与测试

| 配置 | 命令结果 |
|---|---|
| Debug 核心（Qt OFF） | `100% tests passed, 0 tests failed out of 19`，0.70 s |
| Release 核心（Qt OFF） | `100% tests passed, 0 tests failed out of 19`，0.74 s |
| Debug Qt/offscreen | `100% tests passed, 0 tests failed out of 22`，4.30 s |
| AddressSanitizer + UndefinedBehaviorSanitizer 核心 | `100% tests passed, 0 tests failed out of 19`，1.40 s；未见 sanitizer 报告 |

完整终端摘要保存在 `MotionCore_Test_Results/test_summary.txt`；GUI 回归独立输出为 `phase6_gui_regression_test passed rows=81 final_state=Ready`。CTest 入口覆盖 Phase 1–4、Phase 4.1、Phase 5、最终核心回归和三个 Qt 测试。

## 轨迹与性能实测

- Phase 3 标准 100 mm：梯形理论时间 2.5 s，S 曲线理论时间 2.7 s。
- Phase 4 直线 `(0,0,0)`→`(30,40,50)` mm：长度 `70.7106781187` mm，S 曲线实测规划时间 `2.11421356237` s，终点 `(30,40,50)` mm。
- Phase 4 多段闭环：最终误差 `3.14859882402e-05` mm，RMSE `0.254804099419` mm，最大跟踪误差 `0.581743215211` mm；参考合成加速度最大 `100.0 mm/s²`，参考合成 Jerk 最大 `500.0 mm/s³`。实际伺服响应峰值为 `117.73524543 mm/s²` 和 `708.998128429 mm/s³`，属于独立闭环响应指标，不能当作规划器约束证明。
- Phase 5 nominal：最终位置 `(29.975,39.9666,49.9583)` mm，following error `0.0589737` mm，最终速度 `0.0874933` mm/s。

对应 CSV 位于 `results/phase4/`、`results/phase5/`，绘图脚本为 `scripts/plot_phase4.py`、`scripts/plot_phase5.py`。本轮最终场景输出保存在 `MotionCore_Test_Results/final_scenarios.txt` 和 `final_phase4_scenarios.txt`。

## GUI 验收边界

已自动验证：Qt Widgets 构建、窗口 smoke、worker 快照、暂停期间控制事件、急停惯性、连续队列、CSV 导出与离线回放。界面轨迹图明确为 XY 平面二维投影，XYZ 数值仍来自真实三轴仿真；没有宣称三维渲染。

待用户在有显示环境的 Windows + WSLg 或原生 Qt 环境中人工操作：Enable → Home → MoveLine → Arc（需先将前一段终点设为 Arc Demo 要求的 `(50,30,10)` mm）→ Stop → EmergencyStop → Reset → Rehome，并检查按钮状态、曲线和轨迹视觉效果。本机没有可据以宣称通过的人工 GUI 记录，也没有真实电机、驱动器、STO、安全 PLC 或硬实时验证。

## Git/GitHub 发布状态

本地已加入 MIT `LICENSE`，创建了真实提交 `3c3a4d5`（源码发布）、`0e73bfb`（CI 依赖修复）和 `f4f8adf`（最终发布文档），并创建标签 `v1.0.0`。根据仓库所有者已确认的“MotionCore、公开、MIT”设置，公开仓库为 <https://github.com/conan-wx/MotionCore>。

GitHub CLI 的设备授权完成后，源码、标签和 CI 配置均已成功推送。补充 OpenGL 开发依赖后，GitHub Actions run [38207397936](https://github.com/conan-wx/MotionCore/actions/runs/38207397936) 的 Ubuntu Debug、Release、Qt offscreen 和 ASan/UBSan 四个 job 全部成功。GitHub Release [v1.0.0](https://github.com/conan-wx/MotionCore/releases/tag/v1.0.0) 已发布，并附带 `MotionCore_v1.0.0_Source.zip`（SHA-256：`e09acb14b7ec6ba9969dcee09258b364ab399237c486364f4ebe807b5f0e42b3`）。

发布状态：源码、标签、Release 和 CI 已独立验证；人工 GUI 操作仍待用户在有显示环境的 Windows + WSLg 或原生 Qt 环境中验收。






