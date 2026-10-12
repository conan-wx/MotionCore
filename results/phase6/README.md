# Phase 6 结果

Phase 6 的 GUI 自动化测试在 offscreen 模式下使用临时目录验证 CSV 导出和时间轴；回归测试实际输出 `phase6_gui_regression_test passed rows=81`。GUI 运行产生的完整历史 CSV 由 worker 流式写入临时文件，再通过 Export CSV 复制到用户指定位置，因此不会在仓库中伪造一份手工演示日志。

有显示环境的人工验收应将导出的 CSV、轨迹 PNG 和经过授权的截图放入本目录，并在 `PHASE6_ACCEPTANCE.md` 和 `FINAL_RELEASE_ACCEPTANCE.md` 中记录命令、参数和验证人。当前仓库只声明 offscreen 自动化结果。
