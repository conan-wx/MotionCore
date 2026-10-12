#!/usr/bin/env python3
"""把 Phase 3 轨迹规划 CSV 绘制为可重复生成的 SVG 图。

脚本只使用 Python 标准库，不依赖 matplotlib。默认读取：

    results/phase3/trapezoid.csv
    results/phase3/scurve.csv
    results/phase3/closed_loop.csv

也可以显式传入文件：

    python3 scripts/plot_phase3.py \\
        --trapezoid results/phase3/trapezoid.csv \\
        --scurve results/phase3/scurve.csv \\
        --closed-loop results/phase3/closed_loop.csv \\
        --output-dir results/phase3

会生成五张图：梯形/S 曲线速度对比、加速度对比、S 曲线 Jerk、规划与
实际位置对比，以及位置跟踪误差。输入 CSV 使用 SI 仿真层导出的 mm、
mm/s、mm/s² 和 mm/s³ 列名，脚本不会改变原始数据。
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import sys
from pathlib import Path
from typing import Iterable, Sequence


REQUIRED_PLANNER_COLUMNS = (
    "time_s",
    "reference_position_mm",
    "reference_velocity_mm_s",
    "reference_acceleration_mm_s2",
    "reference_jerk_mm_s3",
)


def read_csv(path: Path) -> list[dict[str, float]]:
    """读取并校验一份轨迹 CSV，发现缺列或非有限数值时立即报错。"""

    if not path.is_file():
        raise FileNotFoundError(f"找不到 CSV 文件: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        missing = [name for name in REQUIRED_PLANNER_COLUMNS if name not in fieldnames]
        if missing:
            raise ValueError(f"{path} 缺少列: {', '.join(missing)}")

        rows: list[dict[str, float]] = []
        for line_number, source_row in enumerate(reader, start=2):
            row: dict[str, float] = {}
            for key, value in source_row.items():
                if key is None or value is None or value == "":
                    continue
                try:
                    number = float(value)
                except ValueError as exc:
                    raise ValueError(f"{path}:{line_number} 列 {key} 不是数字") from exc
                # 梯形轨迹的理论 Jerk 是加速度切换冲激，CSV 用 NaN
                # 明确表示“不适用”；其它状态必须保持有限，便于发现数值发散。
                if not math.isfinite(number):
                    if key == "reference_jerk_mm_s3" and math.isnan(number):
                        row[key] = number
                        continue
                    raise ValueError(f"{path}:{line_number} 列 {key} 不是有限数")
                row[key] = number
            rows.append(row)
    if not rows:
        raise ValueError(f"{path} 没有数据行")
    return rows


def _downsample(points: Sequence[tuple[float, float]], maximum: int = 3000) -> list[tuple[float, float]]:
    """限制 SVG 点数，同时保留首尾点和时间顺序。"""

    if len(points) <= maximum:
        return list(points)
    stride = (len(points) - 1) / float(maximum - 1)
    selected = [points[round(index * stride)] for index in range(maximum)]
    # round 在不同 Python 版本中都保持顺序；去掉重复索引以避免折线回跳。
    result: list[tuple[float, float]] = []
    for point in selected:
        if not result or point != result[-1]:
            result.append(point)
    return result


def _expand_range(low: float, high: float) -> tuple[float, float]:
    if math.isclose(low, high, rel_tol=0.0, abs_tol=1e-12):
        margin = max(1.0, abs(low) * 0.1)
        return low - margin, high + margin
    margin = (high - low) * 0.08
    return low - margin, high + margin


def _svg_text(x: float, y: float, content: str, **attributes: object) -> str:
    attrs = " ".join(f'{key.replace("_", "-")}="{html.escape(str(value))}"' for key, value in attributes.items())
    return f'<text x="{x:.1f}" y="{y:.1f}" {attrs}>{html.escape(content)}</text>'


def render_chart(
    output: Path,
    title: str,
    x_label: str,
    y_label: str,
    series: Iterable[tuple[str, str, Sequence[tuple[float, float]]]],
) -> None:
    """绘制一张带坐标轴、网格和图例的折线 SVG。"""

    chart_series = [(name, color, _downsample(points)) for name, color, points in series]
    chart_series = [(name, color, points) for name, color, points in chart_series if points]
    if not chart_series:
        raise ValueError(f"{title} 没有可绘制的数据")

    width, height = 1200, 700
    left, right, top, bottom = 105, 40, 70, 95
    plot_width, plot_height = width - left - right, height - top - bottom
    all_x = [point[0] for _, _, points in chart_series for point in points]
    all_y = [point[1] for _, _, points in chart_series for point in points]
    xmin, xmax = _expand_range(min(all_x), max(all_x))
    ymin, ymax = _expand_range(min(all_y), max(all_y))

    def px(value: float) -> float:
        return left + (value - xmin) / (xmax - xmin) * plot_width

    def py(value: float) -> float:
        return top + plot_height - (value - ymin) / (ymax - ymin) * plot_height

    chunks = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        _svg_text(width / 2, 32, title, text_anchor="middle", font_size=22, font_family="sans-serif", font_weight="bold"),
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" fill="#fbfbfb" stroke="#374151"/>',
    ]

    # 五条水平和垂直网格线既方便读数，也不依赖外部绘图库。
    for index in range(6):
        ratio = index / 5.0
        y_value = ymax - ratio * (ymax - ymin)
        y = top + ratio * plot_height
        chunks.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left + plot_width}" y2="{y:.1f}" stroke="#e5e7eb"/>')
        chunks.append(_svg_text(left - 10, y + 4, f"{y_value:.6g}", text_anchor="end", font_size=12, font_family="sans-serif", fill="#4b5563"))
        x_value = xmin + ratio * (xmax - xmin)
        x = left + ratio * plot_width
        chunks.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + plot_height}" stroke="#e5e7eb"/>')
        chunks.append(_svg_text(x, top + plot_height + 22, f"{x_value:.6g}", text_anchor="middle", font_size=12, font_family="sans-serif", fill="#4b5563"))

    # 零线在位置/误差/加速度图中很有帮助；超出图框时不绘制。
    if ymin < 0 < ymax:
        y_zero = py(0.0)
        chunks.append(f'<line x1="{left}" y1="{y_zero:.1f}" x2="{left + plot_width}" y2="{y_zero:.1f}" stroke="#9ca3af" stroke-dasharray="5,4"/>')

    for name, color, points in chart_series:
        coordinates = " ".join(f"{px(x):.1f},{py(y):.1f}" for x, y in points)
        chunks.append(f'<polyline fill="none" stroke="{html.escape(color)}" stroke-width="2" points="{coordinates}"/>')

    # 图例放在右上角，颜色与折线一致。
    legend_x = left + plot_width - 215
    legend_y = top + 22
    for index, (name, color, _) in enumerate(chart_series):
        y = legend_y + index * 22
        chunks.append(f'<line x1="{legend_x}" y1="{y - 5:.1f}" x2="{legend_x + 28}" y2="{y - 5:.1f}" stroke="{html.escape(color)}" stroke-width="3"/>')
        chunks.append(_svg_text(legend_x + 36, y, name, font_size=13, font_family="sans-serif"))

    chunks.extend([
        _svg_text(left + plot_width / 2, height - 28, x_label, text_anchor="middle", font_size=15, font_family="sans-serif"),
        _svg_text(24, top + plot_height / 2, y_label, text_anchor="middle", font_size=15, font_family="sans-serif", transform=f"rotate(-90 24 {top + plot_height / 2:.1f})"),
        "</svg>",
    ])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(chunks), encoding="utf-8")


def _points(rows: Sequence[dict[str, float]], x_key: str, y_key: str) -> list[tuple[float, float]]:
    if any(y_key not in row for row in rows):
        raise ValueError(f"CSV 缺少列: {y_key}")
    return [(row[x_key], row[y_key]) for row in rows]


def generate_plots(trapezoid: Path, scurve: Path, closed_loop: Path, output_dir: Path) -> list[Path]:
    trap = read_csv(trapezoid)
    scurve_rows = read_csv(scurve)
    closed = read_csv(closed_loop)
    output_dir.mkdir(parents=True, exist_ok=True)

    generated: list[Path] = []
    output = output_dir / "velocity_comparison.svg"
    render_chart(
        output,
        "梯形速度与 S 曲线速度对比",
        "时间 (s)",
        "参考速度 (mm/s)",
        [
            ("梯形规划", "#2563eb", _points(trap, "time_s", "reference_velocity_mm_s")),
            ("S 曲线规划", "#dc2626", _points(scurve_rows, "time_s", "reference_velocity_mm_s")),
        ],
    )
    generated.append(output)

    output = output_dir / "acceleration_comparison.svg"
    render_chart(
        output,
        "梯形加速度与 S 曲线加速度对比",
        "时间 (s)",
        "参考加速度 (mm/s²)",
        [
            ("梯形规划", "#2563eb", _points(trap, "time_s", "reference_acceleration_mm_s2")),
            ("S 曲线规划", "#dc2626", _points(scurve_rows, "time_s", "reference_acceleration_mm_s2")),
        ],
    )
    generated.append(output)

    output = output_dir / "scurve_jerk.svg"
    render_chart(
        output,
        "S 曲线 Jerk",
        "时间 (s)",
        "参考 Jerk (mm/s³)",
        [("S 曲线规划", "#16a34a", _points(scurve_rows, "time_s", "reference_jerk_mm_s3"))],
    )
    generated.append(output)

    output = output_dir / "closed_loop_position.svg"
    render_chart(
        output,
        "规划位置与闭环实际位置",
        "时间 (s)",
        "位置 (mm)",
        [
            ("参考位置", "#2563eb", _points(closed, "time_s", "reference_position_mm")),
            ("实际位置", "#dc2626", _points(closed, "time_s", "actual_position_mm")),
        ],
    )
    generated.append(output)

    output = output_dir / "tracking_error.svg"
    render_chart(
        output,
        "闭环位置跟踪误差",
        "时间 (s)",
        "位置误差 (mm)",
        [("参考位置 - 实际位置", "#9333ea", _points(closed, "time_s", "position_error_mm"))],
    )
    generated.append(output)
    return generated


def _default_input(output_dir: Path, names: Sequence[str]) -> Path | None:
    for name in names:
        candidate = output_dir / name
        if candidate.is_file():
            return candidate
    return None


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="生成 MotionCore Phase 3 轨迹规划 SVG 图")
    parser.add_argument("--trapezoid", type=Path, help="梯形规划 CSV")
    parser.add_argument("--scurve", type=Path, help="S 曲线规划 CSV")
    parser.add_argument("--closed-loop", type=Path, dest="closed_loop", help="闭环仿真 CSV")
    parser.add_argument("--output-dir", type=Path, default=Path("results/phase3"), help="SVG 输出目录")
    args = parser.parse_args(argv)

    trapezoid = args.trapezoid or _default_input(
        args.output_dir, ("trapezoid.csv", "trapezoid_planner.csv", "trapezoidal.csv")
    )
    scurve = args.scurve or _default_input(
        args.output_dir, ("scurve.csv", "scurve_planner.csv", "s_curve.csv")
    )
    closed_loop = args.closed_loop or _default_input(
        args.output_dir,
        (
            "closed_loop.csv",
            "scurve_closed_loop.csv",
            "scurve_closed-loop.csv",
            "trajectory_closed_loop.csv",
        ),
    )
    missing = [name for name, path in (("trapezoid", trapezoid), ("scurve", scurve), ("closed-loop", closed_loop)) if path is None]
    if missing:
        parser.error("缺少输入 CSV: " + ", ".join(missing) + "；请使用 --trapezoid、--scurve 和 --closed-loop 指定文件")
    try:
        generated = generate_plots(trapezoid, scurve, closed_loop, args.output_dir)
    except (OSError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1
    for path in generated:
        print(f"已生成: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
