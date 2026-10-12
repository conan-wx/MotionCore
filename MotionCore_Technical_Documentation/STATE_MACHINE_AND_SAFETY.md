# 状态机与异常处理

## 状态

`Disabled → NotHomed → Homing → Ready → Running` 是正常路径。Running 可进入 `ControlledStopping`，异常可进入 `Fault`，急停事件进入锁存的 `EmergencyStopped`。Fault/EmergencyStopped 不能通过修改变量自行恢复，必须消除原因并调用复位；复位后保守地要求重新回零。

## 指令生命周期

命令具有唯一 ID、几何参数、约束、状态和执行时间。普通命令按 `Queued → Running → Completed`，非法命令为 `Rejected`，停止/急停/故障时未执行命令为 `Canceled`。终态命令不能再次进入 Running。Stop 和 EmergencyStop 走独立优先级通道。

## 回零

每轴依次执行 SearchFast、Backoff、SearchSlow、ApplyOffset；传感器有触发位置、方向、有效电平和防抖。回零前使用受限搜索范围，不假设机床坐标已经成立。传感器未触发、始终有效、抖动、超时和 Stop/EmergencyStop 都会记录结果。

## 限位和停止

运动前检查目标、整条有限直线/圆弧和曲率；运动中检查实际位置和裕量，并用 `v²/(2a_brake)` 预测停止距离。可控的风险进入受控停止，已经越界或无法保证停止时锁存 SoftLimitViolation。实际位置不能被直接裁剪。

## Fault 与 EmergencyStop

故障码至少包括 FollowingError、SoftLimitViolation、HomingTimeout、HomeSensorFault、InvalidMotionCommand、NumericalError、MotionTimeout 和 EmergencyStopTriggered。Fault 期间驱动力策略必须明确：若禁能，动力学仍按惯性、阻尼和负载继续演化；不能把位置或速度瞬间写为零。EmergencyStop 是软件仿真锁存，不是硬件安全回路。

## 安全局限

本项目不提供真实驱动器禁能、STO、安全 PLC、冗余传感器、实时调度或安全认证。任何现场设备接入都必须由具备资质的工程人员重新设计安全链路和验证流程。
