# MotionCore_Demo

仓库包含 results/phase4 和 results/phase5 的 CSV、SVG 结果以及 scripts/plot_phase4.py、scripts/plot_phase5.py。Qt GUI 自动化测试在 offscreen 环境通过，但当前机器没有可据以宣称通过的人工显示验收，因此本目录不伪造截图或视频。

在有显示环境的 Windows + WSLg 或原生 Qt 环境中：

1. 开启 Qt 构建并运行 motioncore_gui。
2. Enable → Home。
3. 输入目标坐标和 Vmax/Amax/Jmax，执行 Move Line。
4. 先将前一段终点设为 Arc Demo 所需的 (50,30,10) mm，再执行 Arc Demo。
5. 测试 Stop、EmergencyStop、Reset、Rehome。
6. 使用 Load CSV Replay 载入项目导出的 CSV，确认回放不触发控制器。
7. 使用 Save Trajectory 保存 XY 投影图。

软件仿真没有真实硬件安全认证、硬实时保证或 STO 能力。
