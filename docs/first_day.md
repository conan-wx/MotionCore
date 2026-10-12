# 第一天：先跑通，再理解

今天的目标只有三个：能构建、能测试、能解释一个 PID 控制器。

## 1. 构建和运行

在 VS Code 的 Ubuntu 终端执行：

```bash
cd /mnt/d/运动控制项目
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/pid_demo
```

看到 `所有 PID 单元测试通过` 和 `PID 仿真完成`，说明第一个闭环已经跑起来了。

## 2. 按文件阅读

1. `include/motion_control/pid_controller.hpp`：看配置、成员变量和函数接口；
2. `src/pid_controller.cpp`：看误差、积分、微分、限幅如何计算；
3. `apps/pid_demo.cpp`：看控制器如何驱动电机模型；
4. `tests/test_pid.cpp`：看如何用小测试验证行为。

## 3. 做一个小实验

打开 `apps/pid_demo.cpp`，只修改一个参数，例如把 `pid_config.kp` 从 `24.0` 改成 `12.0`，重新构建并运行。比较两次日志中的最终位置和误差，然后回答：

- `kp` 变小后，响应变快还是变慢？
- 为什么输出命令要限制在 `[-12, 12]`？
- 为什么积分项需要限制？

这三个问题就是后面学习电机控制和轨迹跟踪的起点。不要先背公式，先通过实验观察参数和响应之间的关系。
