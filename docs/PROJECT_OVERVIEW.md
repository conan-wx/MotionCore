# MotionCore 项目总览

## 项目定位

MotionCore 是一个使用 C++17、CMake、CTest、Qt 6 Widgets 和 Python 构建的工业运动控制数字孪生练习平台。它以三轴 XYZ 运动为对象，从单轴动力学和 PID 开始，逐步加入轨迹规划、直线/圆弧插补、状态管理、回零、软限位、故障和上位机界面。

项目的目的有两个：一是用可读的解析公式验证运动控制算法，二是展示一套可测试的 C++ 控制软件组织方式。项目不依赖实体电机、编码器、EtherCAT、ROS2 或 Gazebo。

## 功能范围

- Phase 1：基础单轴运动与限位示例。
- Phase 2：等效质量—阻尼动力学模型，P/PI/PID，位置 P + 速度 PI 串级控制。
- Phase 3：梯形/三角形速度规划和七段限 Jerk S 曲线。
- Phase 4：XYZ 直线插补、XY 圆弧插补、多段路径及三轴跟踪指标。
- Phase 5：控制状态机、FIFO 指令、回零、软件限位、故障锁存、受控停止和急停模拟。
- Phase 6：可选 Qt Widgets 上位机、独立仿真线程、实时快照、事件日志、CSV 导出和 XY 轨迹投影。

## 适用范围与边界

这是离线数字仿真和软件架构项目。1 ms 是数值积分步长，不代表 Windows/WSL 硬实时能力；EmergencyStop 是软件行为模拟，不是安全继电器、STO、PLC 或功能安全认证实现。Qt 界面当前以 XY 投影为主，不能称为完整三维渲染器。实际工程使用前仍需经过硬件、实时性、限位和安全认证验证。

## 技术栈和环境

- C++17，标准库优先。
- CMake >= 3.20，CTest，Ninja 或 Unix Makefiles。
- Qt 6 Widgets（GUI 可选）。
- Python 3，用于 CSV 分析和 SVG 图表。
- Windows + WSL2 Ubuntu、Linux Ubuntu 均可构建核心模块。

## 目录说明

`include/` 和 `src/` 保存可复用核心库；`apps/` 保存各阶段命令行程序；`gui/` 保存 Qt 界面和仿真 worker；`tests/` 保存数值与回归测试；`scripts/` 保存结果分析；`results/` 保存可复现实验数据；`docs/` 保存设计与验收资料。

## 可信性说明

验收文档只记录真实运行过的命令和结果。自动化 CTest 通过只说明覆盖到的测试通过；人工 GUI 操作、在线 GitHub Actions 结果和真实硬件性能需要单独标注，不能从 offscreen 测试推断。
