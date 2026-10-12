# 开发过程与历史问题

## Phase 1

建立 CMake/CTest、单轴示例和基础 C++ 学习代码。`cpp_learning` 目录（如存在）不属于 MotionCore 核心改动。

## Phase 2

加入等效质量、阻尼、负载和驱动力饱和的动力学；实现 P、PI、PID 和位置—速度串级控制；用确定性扰动实验验证稳态误差和恢复。

## Phase 3

将运动时间从逐步积分中分离为解析轨迹，覆盖梯形、三角形、七段 S 曲线及退化轨迹。非零边界速度在接口未实现时明确拒绝。

## Phase 4

复用标量轨迹实现 XYZ 直线和 XY 圆弧插补。曾发现多段圆弧合成加速度/Jerk约束、Jerk CSV 模长单位、CLI 固定示例参数和有限路径误差口径问题，形成 `PHASE4_FIX_ACCEPTANCE.md` 并增加回归。

## Phase 5

加入状态机、FIFO 队列、回零、软限位、Fault 和 EmergencyStop 仿真。Phase 5.1 修复了急停后取消命令可能重新执行、实际位置运行中越界未锁存和 Phase 4 Jerk 单位错误，回归测试及真实输出见 `PHASE5_FIX_ACCEPTANCE.md`。

## Phase 6

加入 Qt 6 Widgets 上位机和独立仿真线程。GUI 使用真实 `MotionController`，offscreen smoke/data 测试通过；当前 XY 投影、参数化 Arc Demo 和导出已实现，CSV 只读回放已加入并与运动控制隔离；完整三维渲染仍为已知限制。

## 工程决策

- 先验证单位、公式和边界，再增加 UI。
- 核心库与 Qt 解耦，缺少 Qt 时仍可构建核心。
- 固定 1 ms 数值时钟与 GUI 刷新频率分离。
- 停止和急停不把位置/速度直接写为目标值；所有结果从模型演化。
- 文档和验收只引用实际终端输出，未验证项目明确标注。

