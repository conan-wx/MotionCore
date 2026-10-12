#!/usr/bin/env python3
"""MotionCore Phase 4 结果可视化（仅使用 Python 标准库）。

脚本读取 Phase 4 的 CSV，生成可重复打开的 SVG 图，不需要 matplotlib。CSV 的
最小规划列为 ``time_s,x_ref_mm,y_ref_mm,z_ref_mm``；闭环列使用对应的
``x_actual_mm``、``y_actual_mm``、``z_actual_mm``。速度、误差和同步误差列
缺失时会跳过相应曲线，而不是伪造数据。

示例：
    python3 scripts/plot_phase4.py --line results/phase4/line_closed_loop.csv \
        --arc results/phase4/arc_closed_loop.csv --output-dir results/phase4

脚本输出：
    xyz_position.svg、xyz_velocity.svg、trajectory_3d.svg、arc_xy.svg、
    tracking_error.svg、synchronization_error.svg。
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import sys
from pathlib import Path
from typing import Iterable, Sequence


def read_csv(path: Path) -> list[dict[str, float]]:
    """读取 CSV；空字段允许存在，实际绘图列由调用方检查。"""
    if not path.is_file():
        raise FileNotFoundError(f"找不到 CSV 文件: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or "time_s" not in reader.fieldnames:
            raise ValueError(f"{path} 必须包含 time_s 列")
        rows: list[dict[str, float]] = []
        for line, source in enumerate(reader, start=2):
            row: dict[str, float] = {}
            for key, value in source.items():
                if key is None or value is None or value.strip() == "":
                    continue
                try:
                    number = float(value)
                except ValueError as exc:
                    raise ValueError(f"{path}:{line} 列 {key} 不是数字") from exc
                # NaN 可用于明确表示不适用的 Jerk；绘图时会过滤掉该点。
                if not math.isfinite(number):
                    if math.isnan(number):
                        row[key] = number
                        continue
                    raise ValueError(f"{path}:{line} 列 {key} 不是有限数")
                row[key] = number
            rows.append(row)
    if not rows:
        raise ValueError(f"{path} 没有数据行")
    return rows


def _key(row: dict[str, float], *names: str) -> str | None:
    """返回第一个存在的列名，兼容不同实现的命名习惯。"""
    for name in names:
        if name in row:
            return name
    return None


def points(rows: Sequence[dict[str, float]], x_name: str, y_names: Sequence[str]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    for row in rows:
        name = _key(row, *y_names)
        if name is None or x_name not in row or not math.isfinite(row[name]):
            continue
        result.append((row[x_name], row[name]))
    return result


def _downsample(data: Sequence[tuple[float, float]], maximum: int = 3000) -> list[tuple[float, float]]:
    if len(data) <= maximum:
        return list(data)
    stride = (len(data) - 1) / float(maximum - 1)
    return [data[round(index * stride)] for index in range(maximum)]


def _range(low: float, high: float) -> tuple[float, float]:
    if math.isclose(low, high, rel_tol=0.0, abs_tol=1e-12):
        margin = max(1.0, abs(low) * 0.1)
        return low - margin, high + margin
    margin = (high - low) * 0.08
    return low - margin, high + margin


def _text(x: float, y: float, content: str, **attrs: object) -> str:
    values = " ".join(f'{key.replace("_", "-")}="{html.escape(str(value))}"' for key, value in attrs.items())
    return f'<text x="{x:.1f}" y="{y:.1f}" {values}>{html.escape(content)}</text>'


def render_chart(
    output: Path,
    title: str,
    x_label: str,
    y_label: str,
    series: Iterable[tuple[str, str, Sequence[tuple[float, float]]]],
) -> None:
    chart = [(name, color, _downsample(data)) for name, color, data in series if data]
    if not chart:
        return
    width, height = 1200, 700
    left, right, top, bottom = 105, 45, 70, 95
    plot_w, plot_h = width - left - right, height - top - bottom
    xs = [p[0] for _, _, data in chart for p in data]
    ys = [p[1] for _, _, data in chart for p in data]
    xmin, xmax = _range(min(xs), max(xs))
    ymin, ymax = _range(min(ys), max(ys))

    def px(x: float) -> float:
        return left + (x - xmin) / (xmax - xmin) * plot_w

    def py(y: float) -> float:
        return top + plot_h - (y - ymin) / (ymax - ymin) * plot_h

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        _text(width / 2, 32, title, text_anchor="middle", font_size=22, font_family="sans-serif", font_weight="bold"),
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#fbfbfb" stroke="#374151"/>',
    ]
    for i in range(6):
        r = i / 5.0
        x, y = left + r * plot_w, top + r * plot_h
        xv, yv = xmin + r * (xmax - xmin), ymax - r * (ymax - ymin)
        out.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_w}" y2="{y:.1f}" stroke="#e5e7eb"/>')
        out.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_h}" stroke="#e5e7eb"/>')
        out.append(_text(left - 10, y + 4, f"{yv:.6g}", text_anchor="end", font_size=12, font_family="sans-serif", fill="#4b5563"))
        out.append(_text(x, top + plot_h + 22, f"{xv:.6g}", text_anchor="middle", font_size=12, font_family="sans-serif", fill="#4b5563"))
    if ymin < 0 < ymax:
        y0 = py(0.0)
        out.append(f'<line x1="{left}" y1="{y0:.1f}" x2="{left + plot_w}" y2="{y0:.1f}" stroke="#9ca3af" stroke-dasharray="5,4"/>')
    for name, color, data in chart:
        poly = " ".join(f"{px(x):.1f},{py(y):.1f}" for x, y in data)
        out.append(f'<polyline fill="none" stroke="{html.escape(color)}" stroke-width="2" points="{poly}"/>')
    legend_x, legend_y = left + plot_w - 245, top + 22
    for i, (name, color, _) in enumerate(chart):
        y = legend_y + 22 * i
        out.append(f'<line x1="{legend_x}" y1="{y - 5:.1f}" x2="{legend_x + 28}" y2="{y - 5:.1f}" stroke="{html.escape(color)}" stroke-width="3"/>')
        out.append(_text(legend_x + 36, y, name, font_size=13, font_family="sans-serif"))
    out.extend([
        _text(left + plot_w / 2, height - 28, x_label, text_anchor="middle", font_size=15, font_family="sans-serif"),
        _text(24, top + plot_h / 2, y_label, text_anchor="middle", font_size=15, font_family="sans-serif", transform=f"rotate(-90 24 {top + plot_h / 2:.1f})"),
        "</svg>",
    ])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(out), encoding="utf-8")


def render_xy(output: Path, title: str, rows: Sequence[dict[str, float]], actual: bool = True) -> None:
    """绘制 XY 投影；这里使用同一坐标比例，适合检查圆弧几何误差。"""
    groups: list[tuple[str, str, list[tuple[float, float]]]] = []
    ref = []
    act = []
    for row in rows:
        xr = _key(row, "x_ref_mm", "reference_x_mm")
        yr = _key(row, "y_ref_mm", "reference_y_mm")
        xa = _key(row, "x_actual_mm", "actual_x_mm")
        ya = _key(row, "y_actual_mm", "actual_y_mm")
        if xr and yr and math.isfinite(row[xr]) and math.isfinite(row[yr]):
            ref.append((row[xr], row[yr]))
        if actual and xa and ya and math.isfinite(row[xa]) and math.isfinite(row[ya]):
            act.append((row[xa], row[ya]))
    if ref:
        groups.append(("参考 XY 轨迹", "#2563eb", ref))
    if act:
        groups.append(("实际 XY 轨迹", "#dc2626", act))
    if not groups:
        return
    # 将 XY 数据投影到普通图表坐标，复用 SVG 逻辑但轴名称不同。
    render_chart(output, title, "X (mm)", "Y (mm)", groups)


def render_3d(output: Path, title: str, datasets: Sequence[tuple[str, str, Sequence[tuple[float, float, float]]]]) -> None:
    """用等距投影绘制三维轨迹，保持脚本标准库依赖。"""
    projected: list[tuple[str, str, list[tuple[float, float]]]] = []
    for name, color, data in datasets:
        # 等距投影：水平轴同时受 x/y 影响，竖直轴由 z 和 x+y 共同决定。
        projected.append((name, color, [(x - 0.55 * y, z + 0.45 * (x + y)) for x, y, z in data]))
    render_chart(output, title + "（等距投影）", "投影 X/Y", "投影 Z/Y", projected)


def _xyz(rows: Sequence[dict[str, float]], actual: bool = False) -> list[tuple[float, float, float]]:
    suffix = "actual" if actual else "ref"
    aliases = [
        (_key(rows[0], f"x_{suffix}_mm", f"{suffix}_x_mm"),
         _key(rows[0], f"y_{suffix}_mm", f"{suffix}_y_mm"),
         _key(rows[0], f"z_{suffix}_mm", f"{suffix}_z_mm")),
    ] if rows else []
    keys = aliases[0] if aliases else (None, None, None)
    if any(key is None for key in keys):
        return []
    return [(row[keys[0]], row[keys[1]], row[keys[2]]) for row in rows if all(math.isfinite(row.get(key, math.nan)) for key in keys)]  # type: ignore[arg-type]


def generate_plots(inputs: Sequence[Path], output_dir: Path) -> list[Path]:
    if not inputs:
        raise ValueError("至少需要一份 Phase 4 CSV")
    datasets = [(path, read_csv(path)) for path in inputs]
    output_dir.mkdir(parents=True, exist_ok=True)
    primary = datasets[0][1]
    # 当同时传入 --line 和 --arc 时，第一份用于时间/闭环曲线，第二份用于
    # 圆弧几何图；这样规划器可以分别导出两份 CSV 而不需要拼接数据。
    arc_rows = datasets[1][1] if len(datasets) > 1 else primary
    generated: list[Path] = []

    # 位置图：第一份 CSV 为闭环/主场景，列缺失时只显示存在的参考曲线。
    pos_series: list[tuple[str, str, list[tuple[float, float]]]] = []
    colors = (("X 参考", "#2563eb", "x_ref_mm"), ("Y 参考", "#16a34a", "y_ref_mm"), ("Z 参考", "#9333ea", "z_ref_mm"))
    for label, color, key in colors:
        pos_series.append((label, color, points(primary, "time_s", (key, f"reference_{key[0]}_mm"))))
    for label, color, key in (("X 实际", "#dc2626", "x_actual_mm"), ("Y 实际", "#ea580c", "y_actual_mm"), ("Z 实际", "#7c3aed", "z_actual_mm")):
        pos_series.append((label, color, points(primary, "time_s", (key, f"actual_{key[0]}_mm"))))
    output = output_dir / "xyz_position.svg"
    render_chart(output, "XYZ 三轴位置", "时间 (s)", "位置 (mm)", pos_series)
    if output.is_file(): generated.append(output)

    vel_series: list[tuple[str, str, list[tuple[float, float]]]] = []
    for label, color, keys in (("X 参考", "#2563eb", ("vx_ref_mm_s", "x_ref_velocity_mm_s", "vx_ref")), ("Y 参考", "#16a34a", ("vy_ref_mm_s", "y_ref_velocity_mm_s", "vy_ref")), ("Z 参考", "#9333ea", ("vz_ref_mm_s", "z_ref_velocity_mm_s", "vz_ref")), ("X 实际", "#dc2626", ("vx_actual_mm_s", "x_actual_velocity_mm_s", "vx_actual")), ("Y 实际", "#ea580c", ("vy_actual_mm_s", "y_actual_velocity_mm_s", "vy_actual")), ("Z 实际", "#7c3aed", ("vz_actual_mm_s", "z_actual_velocity_mm_s", "vz_actual"))):
        vel_series.append((label, color, points(primary, "time_s", keys)))
    output = output_dir / "xyz_velocity.svg"
    render_chart(output, "XYZ 三轴速度", "时间 (s)", "速度 (mm/s)", vel_series)
    if output.is_file(): generated.append(output)

    output = output_dir / "trajectory_3d.svg"
    sets: list[tuple[str, str, Sequence[tuple[float, float, float]]]] = []
    ref_xyz = _xyz(primary)
    act_xyz = _xyz(primary, actual=True)
    if ref_xyz: sets.append(("参考轨迹", "#2563eb", ref_xyz))
    if act_xyz: sets.append(("实际轨迹", "#dc2626", act_xyz))
    if sets:
        render_3d(output, "XYZ 三维轨迹", sets)
        generated.append(output)

    output = output_dir / "arc_xy.svg"
    render_xy(output, "XY 圆弧规划与实际轨迹", arc_rows)
    if output.is_file(): generated.append(output)

    error_series = []
    for label, color, keys in (("三维位置误差", "#9333ea", ("tracking_error_mm", "position_error_mm", "tracking_error_3d_mm")), ("路径偏差", "#dc2626", ("path_deviation_mm", "line_deviation_mm", "radial_error_mm"))):
        error_series.append((label, color, points(primary, "time_s", keys)))
    output = output_dir / "tracking_error.svg"
    render_chart(output, "三维跟踪与路径误差", "时间 (s)", "误差 (mm)", error_series)
    if output.is_file(): generated.append(output)

    # Jerk 由三个参考分量显式组成；这里绘制其 CSV 中记录的模长。
    jerk = [
        ("参考 Jerk 模长", "#7c3aed",
         points(primary, "time_s", ("jerk_ref_magnitude_mm_s3",))),
    ]
    output = output_dir / "reference_jerk.svg"
    render_chart(output, "参考轨迹 Jerk 模长", "时间 (s)", "Jerk (mm/s³)", jerk)
    if output.is_file(): generated.append(output)

    sync = []
    for label, color, keys in (("归一化同步误差", "#0891b2", ("sync_error", "synchronization_error", "progress_sync_error")), ("X/Y/Z 进度误差", "#f59e0b", ("progress_error",))):
        sync.append((label, color, points(primary, "time_s", keys)))
    output = output_dir / "synchronization_error.svg"
    render_chart(output, "三轴归一化运动进度同步误差", "时间 (s)", "同步误差", sync)
    if output.is_file(): generated.append(output)

    return generated


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="生成 MotionCore Phase 4 XYZ 轨迹 SVG 图")
    parser.add_argument("--csv", action="append", type=Path, help="Phase 4 CSV，可重复指定")
    parser.add_argument("--line", type=Path, help="直线插补 CSV（兼容 --csv）")
    parser.add_argument("--arc", type=Path, help="圆弧插补 CSV（兼容 --csv）")
    parser.add_argument("--output-dir", type=Path, default=Path("results/phase4"), help="SVG 输出目录")
    args = parser.parse_args(argv)
    inputs: list[Path] = list(args.csv or [])
    for path in (args.line, args.arc):
        if path is not None and path not in inputs:
            inputs.append(path)
    if not inputs:
        # 方便在结果目录中直接重绘；优先闭环结果。
        inputs = sorted(args.output_dir.glob("*.csv"))
    try:
        generated = generate_plots(inputs, args.output_dir)
    except (OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1
    for path in generated:
        print(f"已生成: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
