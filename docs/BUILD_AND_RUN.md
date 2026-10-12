# 从零构建和运行

## Windows + WSL2

安装 WSL2 Ubuntu、VS Code 和 Remote Development/WSL 扩展。进入 Ubuntu 后安装基础工具：

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build git python3 qt6-base-dev libgl1-mesa-dev libglu1-mesa-dev
```

Qt 不是核心库的必需依赖；不安装 Qt 时保持 GUI 关闭即可。

## 核心 Debug/Release

```bash
cd /path/to/MotionCore
cmake -S . -B build-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMOTIONCORE_BUILD_GUI=OFF
cmake --build build-debug --parallel 2
ctest --test-dir build-debug --output-on-failure

cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release -DMOTIONCORE_BUILD_GUI=OFF
cmake --build build-release --parallel 2
ctest --test-dir build-release --output-on-failure
```

也可以使用 `cmake --preset core-debug`、`cmake --build --preset core-debug` 和 `ctest --preset core-debug`。

## Qt GUI

```bash
cmake -S . -B build-gui -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMOTIONCORE_BUILD_GUI=ON
cmake --build build-gui --parallel 2
QT_QPA_PLATFORM=offscreen ctest --test-dir build-gui --output-on-failure
QT_QPA_PLATFORM=offscreen ./build-gui/motioncore_gui
```

在 Windows/WSLg 或 Ubuntu 桌面环境中，确认 `DISPLAY`/Wayland 配置后可移除 `QT_QPA_PLATFORM=offscreen`。offscreen 只验证 Qt 对象、信号槽和核心连接，不能替代人工操作验收。

## Sanitizer

```bash
cmake -S . -B build-asan -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build build-asan --parallel 2
ctest --test-dir build-asan --output-on-failure
```

## Python 图表

Python 脚本只读取程序生成的 CSV：

```bash
python3 scripts/plot_phase4.py --line results/phase4/line_scurve_planner.csv --output-dir results/phase4
python3 scripts/plot_phase5.py results/phase5/nominal.csv --output-dir results/phase5
```

## 常见问题

- `Qt6Config.cmake not found`：安装 `qt6-base-dev`，或用 `MOTIONCORE_BUILD_GUI=OFF` 构建核心。
- `Could NOT find XKB`：WSL 中常见的可选开发库提示，不影响 offscreen Widgets 测试；桌面运行需按发行版安装 XKB/图形依赖。
- CTest 失败：使用 `--output-on-failure` 查看失败测试，不要忽略返回码。
- 运行目录没有写权限：把 `--output` 指向用户可写目录。
