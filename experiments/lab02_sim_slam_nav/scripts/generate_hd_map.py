#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 SLAM 建图成果高清双联对比与几何特征标注图 (01_slam_map_hd.png)
"""

import os
from pathlib import Path
import yaml
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.font_manager as fm

# 配置中文字体，确保不产生方块乱码
font_paths = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
]

font_prop = None
font_prop_bold = None
for fp in font_paths:
    if os.path.exists(fp):
        font_prop = fm.FontProperties(fname=fp, size=11)
        font_prop_bold = fm.FontProperties(fname=fp, size=13, weight='bold')
        print(f"[Map Generator] Using Chinese font: {fp}")
        break

if font_prop is None:
    print("[Map Generator] Warning: No Chinese font found, falling back to sans-serif English labels.")

def get_prop(size=11, bold=False):
    if font_prop is None:
        return {'fontsize': size, 'fontweight': 'bold' if bold else 'normal'}
    fp = fm.FontProperties(fname=font_prop.get_file(), size=size, weight='bold' if bold else 'normal')
    return {'fontproperties': fp}

def main():
    SCRIPT_DIR = Path(__file__).resolve().parent
    WS_ROOT = SCRIPT_DIR.parent
    map_dir = str(WS_ROOT / 'src' / 'yunyan_nav' / 'maps')
    pgm_path = os.path.join(map_dir, 'slam_room_map.pgm')
    yaml_path = os.path.join(map_dir, 'slam_room_map.yaml')
    output_path = str(WS_ROOT / 'report_assets' / '01_slam_map_hd.png')
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    resolution = meta['resolution']
    origin = meta['origin']
    ox, oy = origin[0], origin[1]

    pgm_img = Image.open(pgm_path)
    img_data = np.array(pgm_img)
    height, width = img_data.shape

    extent_x = [ox, ox + width * resolution]
    extent_y = [oy, oy + height * resolution]

    print(f"Map size: {width}x{height}, res: {resolution} m/px")
    print(f"X bounds: {extent_x}, Y bounds: {extent_y}")

    # 创建双联对比图 18 x 9.5 英寸, 300 DPI
    fig, axes = plt.subplots(1, 2, figsize=(18, 9.2), dpi=300)
    plt.subplots_adjust(left=0.06, right=0.96, top=0.90, bottom=0.08, wspace=0.18)

    # ------------------ 左图: 原始占用栅格地图 ------------------
    ax1 = axes[0]
    ax1.set_facecolor('#dcdcdc')
    # pgm 中 254=free(白色), 0=occupied(黑色), 205=unknown(灰色)
    im1 = ax1.imshow(img_data, cmap='gray', extent=[0, width, 0, height], origin='upper')
    ax1.set_title("原始占用栅格地图 (Raw Occupancy Grid Map)", pad=14, **get_prop(14, bold=True))
    ax1.set_xlabel("栅格列索引 (Grid Column / Pixels)", labelpad=8, **get_prop(11))
    ax1.set_ylabel("栅格行索引 (Grid Row / Pixels)", labelpad=8, **get_prop(11))
    ax1.grid(color='#888888', linestyle=':', linewidth=0.6, alpha=0.7)

    # 左图标注参数卡片
    info_text = (
        "【地图元数据参数】\n"
        f"• 分辨率 (Resolution): {resolution:.3f} m/pixel\n"
        f"• 网格规模 (Dimensions): {width} × {height} pixels\n"
        f"• 原点坐标 (Origin): [{ox:.2f}, {oy:.2f}, 0.0] m\n"
        f"• 物理覆盖 (Physical Area): ~{width*resolution:.1f}m × {height*resolution:.1f}m\n"
        "• 建图算法: SLAM Toolbox 异步位姿图优化\n"
        "• 栅格编码: 0=障碍物, 254=自由空间, 205=未知"
    )
    ax1.text(0.04, 0.04, info_text, transform=ax1.transAxes,
             bbox=dict(boxstyle='round,pad=0.6', facecolor='#ffffff', edgecolor='#333333', alpha=0.92, linewidth=1.2),
             verticalalignment='bottom', **get_prop(10))

    # ------------------ 右图: 米制与几何语义标注图 ------------------
    ax2 = axes[1]
    ax2.set_facecolor('#ebebeb')
    # 转换为世界坐标显示: origin='lower' 保证 Y 轴向上增加
    # 注意: pgm 第 0 行为最上方 (Y_max), 最底层为 Y_min
    im2 = ax2.imshow(img_data, cmap='gray', extent=[extent_x[0], extent_x[1], extent_y[0], extent_y[1]], origin='upper')
    ax2.set_title("几何特征与语义标注图 (Annotated Metric Map)", pad=14, **get_prop(14, bold=True))
    ax2.set_xlabel("世界坐标 X (World X / Meters)", labelpad=8, **get_prop(11))
    ax2.set_ylabel("世界坐标 Y (World Y / Meters)", labelpad=8, **get_prop(11))
    ax2.set_xlim(-4.2, 4.2)
    ax2.set_ylim(-4.2, 4.2)
    ax2.set_xticks(np.arange(-4, 5, 1))
    ax2.set_yticks(np.arange(-4, 5, 1))
    ax2.grid(color='#4a90e2', linestyle='--', linewidth=0.7, alpha=0.5)

    # 1. 标注外围正交外墙
    wall_rect = patches.Rectangle((-4.0, -4.0), 8.0, 8.0, linewidth=2.0, edgecolor='#e74c3c', facecolor='none', linestyle='-')
    ax2.add_patch(wall_rect)
    ax2.text(0.0, 4.08, "90° 正交封闭外墙 (8.0m × 8.0m Outer Walls)", color='#c0392b', ha='center', va='bottom', **get_prop(10.5, bold=True))

    # 2. 标注内部 L 型走廊隔断
    # 纵向墙: x=1.0, y=0.0~2.5; 横向拐角: x=-1.5~1.0, y=2.5
    ax2.annotate("L型非对称走廊隔断\n(L-Shaped Partition)",
                 xy=(1.0, 1.25), xytext=(1.8, 1.4),
                 arrowprops=dict(facecolor='#2980b9', edgecolor='#1f618d', width=1.5, headwidth=7, shrink=0.08),
                 color='#1b4f72', bbox=dict(boxstyle='round,pad=0.4', facecolor='#eaf2f8', edgecolor='#2980b9', alpha=0.9),
                 **get_prop(10, bold=True))

    # 3. 标注左下方分流隔断墙
    ax2.annotate("左下分流隔断墙\n(x=-1.8, y=-1.5)",
                 xy=(-1.8, -1.5), xytext=(-3.6, -1.2),
                 arrowprops=dict(facecolor='#8e44ad', edgecolor='#6c3483', width=1.5, headwidth=7, shrink=0.08),
                 color='#512e5f', bbox=dict(boxstyle='round,pad=0.4', facecolor='#f4ecf7', edgecolor='#8e44ad', alpha=0.9),
                 **get_prop(9.5, bold=True))

    # 4. 标注 4 个典型障碍物
    # 障碍物 1: 右下圆柱 (2.4, -2.0)
    ax2.annotate("静态圆柱障碍物 1\n(Cylinder: R=0.35m)",
                 xy=(2.4, -2.0), xytext=(2.6, -3.4),
                 arrowprops=dict(facecolor='#27ae60', edgecolor='#1e8449', width=1.2, headwidth=6, shrink=0.1),
                 color='#145a32', bbox=dict(boxstyle='round,pad=0.3', facecolor='#eafaf1', edgecolor='#27ae60', alpha=0.9),
                 **get_prop(9, bold=True))

    # 障碍物 2: 右下方柱 (1.2, -2.8)
    ax2.annotate("静态方柱 2 (0.6m×0.6m)",
                 xy=(1.2, -2.8), xytext=(-0.5, -3.5),
                 arrowprops=dict(facecolor='#d35400', edgecolor='#a04000', width=1.2, headwidth=6, shrink=0.1),
                 color='#7e3204', bbox=dict(boxstyle='round,pad=0.3', facecolor='#fef5e7', edgecolor='#d35400', alpha=0.9),
                 **get_prop(9, bold=True))

    # 障碍物 3: 左上圆柱 (-2.6, 1.5)
    ax2.annotate("静态圆柱障碍物 3\n(Cylinder: R=0.40m)",
                 xy=(-2.6, 1.5), xytext=(-3.8, 2.7),
                 arrowprops=dict(facecolor='#27ae60', edgecolor='#1e8449', width=1.2, headwidth=6, shrink=0.1),
                 color='#145a32', bbox=dict(boxstyle='round,pad=0.3', facecolor='#eafaf1', edgecolor='#27ae60', alpha=0.9),
                 **get_prop(9, bold=True))

    # 障碍物 4: 右上方柱 (2.6, 2.2)
    ax2.annotate("静态方柱 4 (0.7m×0.5m)",
                 xy=(2.6, 2.2), xytext=(2.3, 3.3),
                 arrowprops=dict(facecolor='#d35400', edgecolor='#a04000', width=1.2, headwidth=6, shrink=0.1),
                 color='#7e3204', bbox=dict(boxstyle='round,pad=0.3', facecolor='#fef5e7', edgecolor='#d35400', alpha=0.9),
                 **get_prop(9, bold=True))

    # 5. 标注仿真出生点原点 (0, 0) 与朝向
    ax2.plot(0, 0, marker='o', markersize=9, color='#e74c3c', markeredgecolor='black', markeredgewidth=1.5, zorder=10)
    ax2.arrow(0, 0, 0.45, 0, head_width=0.15, head_length=0.15, fc='#e74c3c', ec='#922b21', linewidth=1.8, zorder=10)
    ax2.text(0.1, -0.42, "出生原点 Origin (0,0)\n朝向 Yaw=0°", color='#922b21', ha='center',
             bbox=dict(boxstyle='round,pad=0.3', facecolor='#fadbd8', edgecolor='#e74c3c', alpha=0.9),
             **get_prop(9, bold=True), zorder=10)

    # 6. 比例尺 (1.0 米)
    scale_x0 = -3.7
    scale_y0 = -3.7
    ax2.plot([scale_x0, scale_x0 + 1.0], [scale_y0, scale_y0], color='black', linewidth=4.0, zorder=10)
    ax2.plot([scale_x0, scale_x0], [scale_y0 - 0.08, scale_y0 + 0.08], color='black', linewidth=2.0, zorder=10)
    ax2.plot([scale_x0 + 1.0, scale_x0 + 1.0], [scale_y0 - 0.08, scale_y0 + 0.08], color='black', linewidth=2.0, zorder=10)
    ax2.text(scale_x0 + 0.5, scale_y0 + 0.12, "1.0 m 比例尺", ha='center', va='bottom', color='black', **get_prop(8.5, bold=True))

    # 7. 罗盘指示 (Compass)
    compass_x, compass_y = 3.6, -3.4
    ax2.annotate('N', xy=(compass_x, compass_y), xytext=(compass_x, compass_y + 0.45),
                 arrowprops=dict(facecolor='#c0392b', edgecolor='#922b21', width=2, headwidth=7),
                 ha='center', va='bottom', color='#c0392b', **get_prop(10, bold=True))

    # 总标题
    plt.suptitle("《蓝海机器人自动驾驶仿真》阶段 M2 高精栅格地图成果与几何特征解析", y=0.97, **get_prop(16, bold=True))

    plt.savefig(output_path, dpi=300, facecolor='#ffffff')
    plt.close()
    print(f"[Map Generator] Successfully saved HD map to {output_path}")

if __name__ == '__main__':
    main()
