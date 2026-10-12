# MotionCore 使用说明

## 命令行场景

先完成核心构建，再从 `build-*` 目录运行各阶段程序。Phase 5 的完整管理器演示为：

```bash
./motioncore_phase5 --scenario homing --output ../results/phase5
./motioncore_phase5 --scenario nominal --output ../results/phase5
./motioncore_phase5 --scenario queue --output ../results/phase5
./motioncore_phase5 --scenario controlled-stop --output ../results/phase5
./motioncore_phase5 --scenario emergency-stop --output ../results/phase5
```

程序会输出 CSV 和事件日志；`scripts/plot_phase5.py` 可生成位置、速度、误差和驱动力图。

## Qt 上位机

1. 启动 `motioncore_gui`。
2. 点击 Enable，状态从 Disabled 变为 NotHomed。
3. 点击 Home，等待三个轴进入 Ready。
4. 输入目标 X/Y/Z、速度、加速度和 Jerk，选择 Trapezoid 或 S-Curve，点击 Move Line。
5. 可使用 Arc Demo 查看固定 XY 圆弧；参考轨迹和实际轨迹用不同颜色绘制。
6. 运行中点击 Stop 进入受控减速，点击 EmergencyStop 进入锁存急停；机械轴按模型继续惯性/阻尼演化。
7. 故障原因消除后点击 Reset，再重新 Home；按钮状态由界面提示，核心仍执行权限检查。
8. 使用 Export CSV 保存当前 worker 采样，Save/Load Parameters 保存演示参数，Save Trajectory 保存 XY 投影图片。

当前界面将 X/Y 轨迹绘制为二维投影，Z 轴通过快照和遥测数值显示；它不是完整 3D 渲染器。

## CSV 检查和离线数据

用 Python 或文本工具检查表头、有限数值和单调 `time_s`。导出来自 worker 的完整流式历史，实时曲线只保留最近 20000 个显示点；当前版本支持导出、脚本分析和 GUI 的 Load CSV Replay 离线回放。回放会校验表头、有限数值和严格递增 time_s，只更新显示快照，不调用运动控制器，也不会把文件数据当作实时控制命令。

## 真实 GUI 验收

在带显示环境的 Windows + WSLg/Ubuntu 桌面中，按顺序执行 Enable → Home → MoveLine → Arc Demo → Stop → EmergencyStop → Reset → Rehome，并观察状态、事件和曲线。无显示环境只能执行 offscreen 自动化测试，人工操作项应记录为待验收。

