# MotionCore Phase 4.1 缺陷修复验收

本次修复先于 Phase 5 开发执行，保留 Phase 1～4 原有接口和测试。

## 修复内容

1. `ArcInterpolator` 在建立标量轨迹前检查曲率引入的法向加速度和 Frenet Jerk。有限合成限制下，按解析保守界限降低速度、切向加速度和 Jerk；无法留下正约束时明确抛出 `std::invalid_argument`。分段边界、加速度零点和合成加速度导数根作为候选极值，不再使用固定采样网格作为全局证明。
2. Phase 4 CSV 将单一 `jerk_ref_mm_s3` 拆成 `jx_ref_mm_s3`、`jy_ref_mm_s3`、`jz_ref_mm_s3` 和 `jerk_ref_magnitude_mm_s3`。四列统一采用 mm/s³；模长逐行满足 `sqrt(jx²+jy²+jz²)`。梯形轨迹 Jerk 分量仍以 NaN 表示理论不连续。
3. 固定多段示例改名为 `multi-demo`，向该圆弧显式传递 `{100 mm/s², 500 mm/s³}` 合成限制；显式给出 `start-mm`、`target-mm` 或 `center-mm` 时拒绝固定示例，避免悄悄忽略输入。
4. 直线偏差使用有限线段最近点；圆弧同时记录有限圆弧最近距离 `path_deviation_mm` 与完整圆周半径误差 `radial_error_mm`。
5. 驱动力饱和由控制器逐轴饱和标志统计，新增 `drive_saturated_x/y/z` CSV 列。

## 真实构建和测试

```bash
wsl -d Ubuntu -- bash -lc 'cd /mnt/d/运动控制项目 && \
  cmake -S . -B build_phase41 -DCMAKE_BUILD_TYPE=Debug && \
  cmake --build build_phase41 -j2 && \
  ctest --test-dir build_phase41 --output-on-failure'
```

历史 Phase 4.1 构建实测结果为 `100% tests passed, 0 tests failed out of 7`；当前 Phase 5.1 构建还通过了 Phase 4 CSV 的数值单位回归检查。

新增回归测试 `motioncore_phase4_fix_test` 验证半径 10 mm、原始 50/100/500 约束的圆弧会在规划前降速，解析合成加速度/Jerk 不超 `{100,500}`，以及零合成加速度限制会被拒绝。旧 Phase 1～4 测试仍全部通过。

## 标准场景复核

```text
geometry=multi-demo, profile=scurve, mode=planner,
duration_s=3.63875621078,
final_reference_xyz_mm=40,20,0,
final_reference_path_mm=55.7079632679
```

多段时间相比未考虑曲率时增加，是圆弧半径 10 mm 在 50 mm/s 下的法向加速度及 Jerk 超过参考限制后自动保守降速的结果。参考轨迹不再宣称以原始速度约束运行。

```text
geometry=multi-demo, profile=scurve, mode=closed-loop,
duration_s=3.63875621078, total_simulation_s=5.63875621078,
final_tracking_error_mm=3.14859882402e-05,
max_tracking_error_mm=0.581743215211,
max_path_deviation_mm=0.424188193803,
saturation_samples=0
```

实际伺服速度、加速度和 Jerk 是独立记录的响应指标；它们不被当作规划器参考约束的证明。

Phase 5.1 重新检查了 CSV 单位：标准多段 CSV 的首个有限 Jerk 行为
`jx_ref_mm_s3=500, jy_ref_mm_s3=0, jz_ref_mm_s3=0,
jerk_ref_magnitude_mm_s3=500`，不再出现旧的 `0.5` 模长。

## 可复现 CLI

```bash
./build_phase41/motioncore_phase4 --geometry multi-demo --profile scurve \
  --mode planner --output results/phase4/multi_demo_fix.csv
./build_phase41/motioncore_phase4 --geometry arc --profile scurve --mode planner \
  --start-mm 20,0,0 --target-mm 0,20,0 --center-mm 0,0,0 \
  --vmax-mm-s 20 --amax-mm-s2 40 --jmax-mm-s3 200 \
  --output results/phase4/arc_fix.csv
python3 scripts/plot_phase4.py --csv results/phase4/multi_demo_fix_closed.csv \
  --output-dir results/phase4
```

该阶段仍是数值仿真，不代表真实驱动器的硬件安全认证。Phase 5 模块在此验收通过后继续开发。
