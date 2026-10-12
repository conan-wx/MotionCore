# 测试与性能报告

## 环境

Windows 主机中的 WSL Ubuntu，GNU C++ 13.3、CMake 3.28、CTest、Qt 6.4.2。Qt GUI 使用 MOTIONCORE_BUILD_GUI=ON，核心构建不依赖 Qt。在线 GitHub Actions 尚未运行。

## 实际自动化结果

- Debug 核心：19/19 通过，0.70 s。
- Release 核心：19/19 通过，0.74 s。
- Qt Debug/offscreen：22/22 通过，4.30 s。
- ASan + UBSan 核心：19/19 通过，1.40 s，无 sanitizer 报告。

CTest 既保留 Phase 1–4 回归，也覆盖 Phase 4.1 CSV/曲率约束修复、Phase 5 状态机/队列/回零/限位/停止/故障、Fault 自由滑行和连续队列回归；Qt 测试覆盖窗口 smoke、快照/CSV、暂停 Stop/EmergencyStop、急停惯性、line→arc→line、完整 CSV 及离线回放。

终端摘要和场景输出位于 MotionCore_Test_Results/。

## 理论与实测结果

- Phase 3 100 mm：梯形 2.5 s，S 曲线 2.7 s。
- Phase 4 直线 (30,40,50) mm：路径长度 70.7106781187 mm，S 曲线时间 2.11421356237 s。
- Phase 4 多段闭环：最终误差 3.14859882402e-05 mm，RMSE 0.254804099419 mm，最大误差 0.581743215211 mm。
- 多段参考轨迹的合成加速度/Jerk 最大值由独立 CSV 复算为 100.0 mm/s²、500.0 mm/s³；实际闭环峰值分别为 117.73524543 mm/s²、708.998128429 mm/s³，两者口径分开。
- Phase 5 nominal：最终位置 (29.975,39.9666,49.9583) mm，following error 0.0589737 mm，最终速度 0.0874933 mm/s。
- Fault 自由滑行回归：1 s 内位置 0.0347233852301→0.356719010826 mm，速度 1.0178876855→0.0504496516611 mm/s。
- GUI 回归：phase6_gui_regression_test passed rows=81 final_state=Ready。

## 独立复算

~~~bash
python3 tests/check_phase4_csv.py results/phase4/final_multi.csv
python3 tests/check_phase4_csv.py results/phase4/final_line.csv
python3 tests/check_phase5_csv.py results/phase5/nominal.csv
python3 scripts/plot_phase4.py --line results/phase4/final_multi.csv \
  --arc results/phase4/arc_scurve_closed_loop.csv --output-dir results/phase4
python3 scripts/plot_phase5.py results/phase5/nominal.csv --output-dir results/phase5
~~~

以上命令的实际输出分别为 validated 5640 Phase 4 CSV rows、validated 2115 Phase 4 CSV rows 和 validated 11573 rows。

## 未覆盖项

没有真实硬件、Windows 硬实时、安全 PLC/STO 或工业认证测试；Qt 图形界面尚未在带显示环境的机器上人工操作验收；GitHub Actions 仅有配置文件，尚未在线执行；没有生成代码覆盖率百分比。






