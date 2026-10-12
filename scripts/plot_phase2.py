#!/usr/bin/env python3
"""用 Python 标准库把 Phase 2 CSV 转为四联 SVG 图。

不依赖 matplotlib，便于在刚安装好的 Ubuntu/WSL 环境中直接运行。
用法：python3 scripts/plot_phase2.py results/phase2/nominal.csv
"""
import csv
import math
import sys
from pathlib import Path


def read_csv(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("CSV 没有数据行")
    required = [
        "time_s", "target_position_mm", "actual_position_mm",
        "target_velocity_mm_s", "actual_velocity_mm_s",
        "actual_acceleration_mm_s2", "drive_force_N", "load_force_N", "position_error_mm",
    ]
    for key in required:
        if key not in rows[0]:
            raise ValueError(f"缺少列: {key}")
    return [{key: float(row[key]) for key in required} for row in rows]


def polyline(points, color, width, height, left, top, plot_width, plot_height,
             xmin, xmax, ymin, ymax):
    def px(x):
        return left + (x - xmin) / (xmax - xmin) * plot_width

    def py(y):
        return top + plot_height - (y - ymin) / (ymax - ymin) * plot_height

    coordinates = " ".join(f"{px(row[0]):.1f},{py(row[1]):.1f}" for row in points)
    return f'<polyline fill="none" stroke="{color}" stroke-width="1.5" points="{coordinates}"/>'


def make_svg(rows, output: Path, title: str):
    width, height = 1100, 760
    left, right, top, bottom = 80, 30, 55, 55
    panel_width, panel_height = 470, 280
    gap_x, gap_y = 40, 45
    x = [row["time_s"] for row in rows]
    panels = [
        ("位置 (mm)", "target_position_mm", "actual_position_mm", "#2563eb", "#dc2626"),
        ("速度 (mm/s)", "target_velocity_mm_s", "actual_velocity_mm_s", "#2563eb", "#dc2626"),
        ("位置误差 (mm)", "position_error_mm", None, "#16a34a", None),
        ("驱动力 (N)", "drive_force_N", None, "#9333ea", None),
    ]
    chunks = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
              '<rect width="100%" height="100%" fill="white"/>',
              f'<text x="{width/2:.0f}" y="28" text-anchor="middle" font-size="18" font-family="sans-serif">{title}</text>']
    for index, (label, key_a, key_b, color_a, color_b) in enumerate(panels):
        col, row = index % 2, index // 2
        px0 = left + col * (panel_width + gap_x)
        py0 = top + row * (panel_height + gap_y)
        values = [r[key_a] for r in rows]
        if key_b:
            values.extend(r[key_b] for r in rows)
        lo, hi = min(values), max(values)
        if math.isclose(lo, hi):
            lo -= 1.0
            hi += 1.0
        margin = (hi - lo) * 0.08
        lo -= margin
        hi += margin
        chunks.append(f'<rect x="{px0}" y="{py0}" width="{panel_width}" height="{panel_height}" fill="#fafafa" stroke="#9ca3af"/>')
        chunks.append(f'<text x="{px0+8}" y="{py0+20}" font-size="14" font-family="sans-serif">{label}</text>')
        def points(key):
            return [(r["time_s"], r[key]) for r in rows]
        chunks.append(polyline(points(key_a), color_a, width, height, px0 + 45, py0 + 35,
                               panel_width - 60, panel_height - 60, x[0], x[-1], lo, hi))
        if key_b:
            chunks.append(polyline(points(key_b), color_b, width, height, px0 + 45, py0 + 35,
                                   panel_width - 60, panel_height - 60, x[0], x[-1], lo, hi))
            chunks.append(f'<text x="{px0+panel_width-125}" y="{py0+20}" fill="{color_a}" font-size="11">target</text>')
            chunks.append(f'<text x="{px0+panel_width-65}" y="{py0+20}" fill="{color_b}" font-size="11">actual</text>')
    chunks.append("</svg>")
    output.write_text("\n".join(chunks), encoding="utf-8")


def main():
    if len(sys.argv) not in (2, 3):
        print("用法: python3 scripts/plot_phase2.py INPUT.csv [OUTPUT.svg]")
        return 2
    source = Path(sys.argv[1])
    output = Path(sys.argv[2]) if len(sys.argv) == 3 else source.with_suffix(".svg")
    rows = read_csv(source)
    make_svg(rows, output, source.stem)
    print(f"已生成: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
