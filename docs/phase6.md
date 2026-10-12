# Phase 6：Qt 工业运动控制上位机

## 目标与边界

Phase 6 在保留 `motioncore` 核心库和 Phase 1--5 命令行程序的前提下，增加一个
Qt 6 Widgets 上位机。界面调用的是真实 `MotionController`，不是为了演示而复制一套
控制逻辑；所有轴仍由核心库以 1 ms 的统一仿真时钟更新。本阶段是数字孪生软件，
不连接驱动器、编码器、EtherCAT 或安全 PLC，也不宣称具备硬实时或急停安全认证。

## 工程结构

- `gui/simulation_worker.*`：位于独立 `QThread` 中的仿真 worker。worker 内部每个
  GUI 定时周期推进若干个 1 ms 步，GUI 刷新约 50 FPS；因此界面刷新频率不会改变
  数值仿真步长。
- `gui/mainwindow.*`：状态、参数、控制按钮、事件日志、CSV/参数/轨迹导出。
- `gui/trajectory_view.*`：参考轨迹（蓝色虚线）和实际 XY 投影（绿色实线）。
- `gui/telemetry_view.*`：位置、速度、加速度、跟踪误差和驱动力曲线切换。
- `apps/motioncore_gui.cpp`：Qt 应用入口。

界面事件通过 queued signal/slot 发送到 worker 线程，快照通过只读 `GuiSnapshot`
返回 GUI 线程。暂停只停止普通时间推进；Stop/EmergencyStop 通过核心的
`process_control_events()` 零时间路径立即处理。按钮禁用只是减少误操作，核心接口会再次
检查状态、回零、限位和故障条件。

## 构建与运行

WSL Ubuntu 已安装 Qt 6.4.2 Widgets 开发包。没有 Qt 时，默认配置仍只构建核心库、
命令行程序和 CTest；GUI 依赖由 `MOTIONCORE_BUILD_GUI` 隔离。

```bash
cd /mnt/d/运动控制项目
cmake -S . -B build_phase6 -DCMAKE_BUILD_TYPE=Debug -DMOTIONCORE_BUILD_GUI=ON
cmake --build build_phase6 -j2
QT_QPA_PLATFORM=offscreen ./build_phase6/motioncore_gui
ctest --test-dir build_phase6 --output-on-failure
```

在具有桌面显示的 Windows/WSL 图形环境中运行时可以去掉
`QT_QPA_PLATFORM=offscreen`。无显示环境下使用 offscreen 只验证 Qt 对象、信号槽和
控制器连接，不等价于人工视觉验收。

## 线程和数据流程

每个 worker tick 的顺序是：先处理已排队的高优先级控制事件，再按核心状态机推进一个或
多个 1 ms 步，读取 `MotionStatus`，生成 `GuiSnapshot`，最后把快照发送到 GUI。暂停只
停止普通步进，控制事件仍可在同一 worker 线程执行；急停后即使 `MotionController::step`
返回锁存状态，worker 仍继续调用禁能驱动的动力学滑行，直到速度低于阈值。GUI 只更新
控件和曲线，不直接修改动力学状态。窗口关闭时先停止 worker，再退出线程，避免后台对象
访问已销毁的界面。

实时绘图的 `samples_` 只保留最近 20000 点；每个严格递增的仿真时间样本同时流式写入
临时历史 CSV。暂停时重复的状态快照不会写入运动时间序列，状态事件通过事件日志区分。

## 已知限制

当前轨迹窗口绘制的是 XY 平面投影，Z 轴和三轴状态在数值快照与遥测中显示，尚未
引入 3D 渲染依赖。圆弧按钮提供固定的正象限演示输入；任意圆弧仍可由 Phase 4/5
命令行接口验证。CSV 导出来自完整的流式 worker 历史而不是 20000 点显示缓存，独立
CSV 可通过 GUI 的 Load CSV Replay 按钮加载；worker 校验字段、有限数值和严格递增 time_s，回放只读快照，不调用 MotionController。Qt smoke/data/regression 测试在 `QT_QPA_PLATFORM=offscreen`
下运行。

