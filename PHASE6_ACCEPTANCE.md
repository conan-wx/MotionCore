# MotionCore Phase 6 验收记录

日期：2026-10-12（Windows 主机中的 WSL Ubuntu，Qt 6.4.2）。

## 实现内容

- MOTIONCORE_BUILD_GUI 可选；关闭时核心库、命令行程序和全部核心 CTest 不依赖 Qt。
- Qt Widgets 主窗口真实连接 MotionController：Enable、Home、Move Line、Arc Demo、Stop、EmergencyStop、Reset、暂停/继续、CSV 导出、参数保存/加载、轨迹图片保存。
- SimulationWorker 在独立 Qt 线程中运行，核心保持 1 ms 仿真步长，GUI 约 20 ms 刷新。
- GUI 展示 XYZ 状态、指令/故障、XY 参考与实际轨迹、位置/速度/加速度/误差/驱动力曲线和事件日志。
- Stop/EmergencyStop 通过零时间 process_control_events() 处理；急停锁存后仍按禁能驱动动力学显示惯性尾段。
- 绝对坐标命令由核心队列规划端点衔接，支持连续直线及 line→arc→line。
- CSV 运动时间只在仿真时间推进时写入；完整历史流式保存，20000 点仅为实时显示缓存。
- 新增离线 CSV 回放：加载时检查字段、有限数值和严格递增 time_s，回放只发送快照，不调用运动控制器，也不会触发真实运动指令。

## 实际构建与测试

核心 Debug：
~~~bash
cmake -S . -B build_final_debug -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DMOTIONCORE_BUILD_GUI=OFF
cmake --build build_final_debug -j2
ctest --test-dir build_final_debug --output-on-failure
~~~
结果：19/19 通过，0.70 s。

核心 Release：19/19 通过，0.74 s。ASan+UBSan 核心：19/19 通过，1.40 s，无 sanitizer 报告。

Qt Debug/offscreen：
~~~bash
cmake -S . -B build_final_gui -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DMOTIONCORE_BUILD_GUI=ON
cmake --build build_final_gui -j2
QT_QPA_PLATFORM=offscreen ctest --test-dir build_final_gui --output-on-failure
~~~
结果：22/22 通过，4.30 s。phase6_gui_regression_test 输出 rows=81 final_state=Ready，包含暂停 Stop/EmergencyStop、急停惯性、连续队列、严格递增 CSV 和离线回放。

## 验收边界

轨迹窗口是明确标注的 XY 二维投影，不是三维渲染。人工 GUI 操作（Enable → Home → MoveLine → Arc → Stop → EmergencyStop → Reset → Rehome）仍需用户在有显示环境的 Windows/WSLg 或原生 Qt 环境执行；offscreen 测试不能替代这项验收。项目不连接真实驱动器，不提供硬实时、STO、安全 PLC 或工业安全认证。





