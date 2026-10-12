# 简历项目描述

## 一句话

基于 C++17/CMake 的 XYZ 三轴工业运动控制与数字孪生仿真平台，覆盖动力学、PID 串级控制、S 曲线、直线/圆弧插补、状态管理、故障仿真和 Qt 上位机。

## 中文简历描述（约 200 字）

独立设计并实现 MotionCore 工业运动控制数字孪生平台，使用 C++17、CMake、CTest 和 Qt 6 构建模块化核心。建立质量—阻尼—负载动力学模型，实现 P/PI/PID 及位置 P+速度 PI 串级控制、梯形/七段 S 曲线轨迹规划、XYZ 直线和 XY 圆弧插补。基于统一 1 ms 仿真时钟实现三轴闭环跟踪、回零、软件限位、受控停止、急停和故障锁存，并通过 CSV、Python 图表、CTest 及 ASan/UBSan 验证边界和回归问题。Qt 上位机通过独立 worker 线程连接真实 MotionController，展示轨迹、遥测和事件日志。项目为软件仿真，未连接真实硬件或宣称硬实时安全能力。

## 技术成果

- 解析轨迹规划支持梯形/三角形、七段 S 曲线及短距离退化，终点和约束有数值测试。
- 三轴插补共用单一标量路径进度，圆弧检查曲率引起的合成加速度/Jerk。
- 状态机、命令 FIFO、回零、软限位、Fault/EmergencyStop 形成可测试管理层。
- 无 Qt 核心保持 19 个测试入口，Qt 构建扩展至 22 个测试；实际结果记录在验收文档。
- 修复急停取消命令重启、动态软限位和 Jerk CSV 单位问题，并保留回归测试。

## English version

Designed and implemented MotionCore, a C++17/CMake digital-twin platform for XYZ industrial motion control. The system models mass–viscous damping–load dynamics, cascaded position/velocity control, trapezoidal and jerk-limited S-curve profiles, coordinated line and XY arc interpolation, homing, software limits, controlled stop, emergency-stop simulation and latched faults. A Qt 6 worker-thread GUI visualizes reference/actual trajectories, telemetry and events. Numerical behavior is verified with CTest and sanitizer builds; the project is a software simulation and has no real drive, hard-real-time or safety certification.

## 面试介绍

一分钟：我把运动控制项目按模型、规划、插补、伺服和管理层拆开，先用解析公式和 1 ms 仿真验证单轴，再用一个标量路径同步 XYZ，最后把回零、限位、故障和 Qt worker 连接起来。重点是参考轨迹和实际动力学状态分离，并用真实数值测试捕获单位、队列和限位缺陷。

三分钟：可按动力学方程、控制回路、轨迹规划、三轴插补、状态机/故障和验证证据六部分展开，重点展示 `DynamicAxisModel`、`SCurveProfile`、`ArcInterpolator`、`MotionController` 和 `SimulationWorker` 的职责边界，以及 Phase 4/5 修复前后如何通过回归测试确认。

