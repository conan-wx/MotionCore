# MotionCore v1.0.0（本地发布说明草案）

## 内容

MotionCore 提供 C++17 工业运动控制数字孪生核心、XYZ 直线/XY 圆弧插补、梯形与 S 曲线规划、三轴伺服管理器、回零/限位/故障仿真，以及可选 Qt 6 Widgets 上位机。Qt worker 支持 CSV 导出和只读离线回放，回放数据经过表头、有限数值和单调时间校验。

## 本机验证

- Debug 核心：19/19 CTest 通过。
- Release 核心：19/19 CTest 通过。
- Qt Debug/offscreen：22/22 CTest 通过。
- ASan+UBSan 核心：19/19 CTest 通过，无报告。

实际命令和输出保存在 FINAL_RELEASE_ACCEPTANCE.md、MotionCore_Test_Results/ 和各阶段验收文档。GitHub Actions run [38207397936](https://github.com/conan-wx/MotionCore/actions/runs/38207397936) 已验证 Ubuntu 核心 Debug/Release、Qt offscreen 和 ASan/UBSan。

## 已知限制

- 纯软件数字孪生，不连接真实驱动器、编码器、EtherCAT 或安全 PLC。
- 1 ms 是仿真时间步长，不提供 Windows/WSL 硬实时保证。
- Qt 当前展示明确标注的二维 XY 轨迹投影，不是完整三维渲染器。
- 人工图形环境验收和演示视频尚未由本机记录，需要用户在有显示环境时操作。
- 本地已采用 MIT License，提交 `3c3a4d5`、`9dbe74a` 并创建标签 `v1.0.0`。公开仓库为 <https://github.com/conan-wx/MotionCore>，Release 为 <https://github.com/conan-wx/MotionCore/releases/tag/v1.0.0>。

## 复现

请先阅读 docs/BUILD_AND_RUN.md，使用 CMakePresets.json 或显式 CMake 命令构建并执行 CTest。所有性能数字都应追溯到 results/ 中的 CSV 与脚本。
