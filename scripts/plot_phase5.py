#!/usr/bin/env python3
"""绘制 Phase 5 调度器 CSV 的可复现 SVG 图，不依赖 matplotlib。"""
from __future__ import annotations
import argparse, csv, html, math
from pathlib import Path

def read(path: Path):
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows or "time_s" not in rows[0]: raise ValueError("CSV 缺少 time_s 或没有数据")
    return rows

def series(rows, x, y):
    out=[]
    for r in rows:
        try: a=float(r[x]); b=float(r[y])
        except (KeyError, ValueError): continue
        if math.isfinite(a) and math.isfinite(b): out.append((a,b))
    return out

def svg(path: Path, title: str, curves: list[tuple[str,list[tuple[float,float]],str]], xlabel: str, ylabel: str):
    width,height=1000,560; left,right,top,bottom=80,25,50,70
    points=[p for _,s,_ in curves for p in s]
    if not points: return
    xmin=min(p[0] for p in points); xmax=max(p[0] for p in points)
    ymin=min(p[1] for p in points); ymax=max(p[1] for p in points)
    if math.isclose(xmin,xmax): xmax=xmin+1
    if math.isclose(ymin,ymax): ymin-=1; ymax+=1
    def px(x): return left+(x-xmin)/(xmax-xmin)*(width-left-right)
    def py(y): return height-bottom-(y-ymin)/(ymax-ymin)*(height-top-bottom)
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="{width/2}" y="28" text-anchor="middle" font-size="20">{html.escape(title)}</text>',
           f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="black"/>',
           f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="black"/>']
    for name,data,color in curves:
        pts=' '.join(f'{px(x):.2f},{py(y):.2f}' for x,y in data[::max(1,len(data)//2000)])
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1.5"/>')
    for i,(name,_,color) in enumerate(curves):
        y=top+20*i; parts.append(f'<line x1="{width-220}" y1="{y}" x2="{width-200}" y2="{y}" stroke="{color}"/>')
        parts.append(f'<text x="{width-190}" y="{y+5}" font-size="13">{html.escape(name)}</text>')
    parts += [f'<text x="{width/2}" y="{height-20}" text-anchor="middle">{html.escape(xlabel)}</text>',
              f'<text x="18" y="{height/2}" transform="rotate(-90 18 {height/2})" text-anchor="middle">{html.escape(ylabel)}</text>', '</svg>']
    path.write_text('\n'.join(parts), encoding='utf-8')

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('csv'); ap.add_argument('--output-dir',default='results/phase5')
    args=ap.parse_args(); rows=read(Path(args.csv)); out=Path(args.output_dir); out.mkdir(parents=True,exist_ok=True)
    t='time_s'; colors=['#1769aa','#d1495b','#2a9d8f']
    position_curves=[]
    for axis, color in zip('xyz', colors):
        position_curves.append((f'{axis} ref', series(rows,t,f'{axis}_ref_mm'), color))
        position_curves.append((f'{axis} actual', series(rows,t,f'{axis}_actual_mm'), '#555555'))
    svg(out/'xyz_position.svg','XYZ positions',position_curves, 'time (s)','position (mm)')
    svg(out/'xyz_velocity.svg','XYZ velocities',[("vx",series(rows,t,'vx_actual_mm_s'),colors[0]),("vy",series(rows,t,'vy_actual_mm_s'),colors[1]),("vz",series(rows,t,'vz_actual_mm_s'),colors[2])], 'time (s)','velocity (mm/s)')
    svg(out/'tracking_error.svg','Following error',[("error",series(rows,t,'following_error_mm'),colors[1])], 'time (s)','error (mm)')
    svg(out/'drive_force.svg','Drive force X/Y/Z',[("Fx",series(rows,t,'drive_force_x_n'),colors[0]),("Fy",series(rows,t,'drive_force_y_n'),colors[1]),("Fz",series(rows,t,'drive_force_z_n'),colors[2])], 'time (s)','force (N)')
    # SVG 的二维轨迹图用时间作为横轴的同一套缩放；它同时保留规划和实际 XY 点，
    # 便于初学者检查几何跟踪，而不引入绘图库依赖。
    svg(out/'trajectory_xy.svg','XY reference and actual',[("x ref",series(rows,t,'x_ref_mm'),colors[0]),("x actual",series(rows,t,'x_actual_mm'),'#555555'),("y ref",series(rows,t,'y_ref_mm'),colors[1]),("y actual",series(rows,t,'y_actual_mm'),'#777777')], 'time (s)','coordinate (mm)')
    print(f'generated SVG plots in {out}')
if __name__=='__main__': main()
