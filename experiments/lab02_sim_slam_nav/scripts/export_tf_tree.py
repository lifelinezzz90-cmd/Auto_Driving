#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
后台启动 Gazebo 与 SLAM Toolbox，生成并导出 TF 坐标树高清拓扑图 (02_tf_tree.png)
"""

import os
import subprocess
import time
import shutil
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
REPO_ROOT = WS_ROOT.parent.parent

def main():
    assets_dir = str(WS_ROOT / 'report_assets')
    Path(assets_dir).mkdir(parents=True, exist_ok=True)
    out_png = os.path.join(assets_dir, '02_tf_tree.png')
    tmp_pdf = '/tmp/frames.pdf'
    tmp_png_prefix = '/tmp/tf_tree'

    # 1. 确保旧文件清理
    for f in [tmp_pdf, f"{tmp_png_prefix}-1.png"]:
        if os.path.exists(f):
            try:
                os.remove(f)
            except Exception:
                pass

    print("[TF Tree] 启动 Gazebo 仿真 (launch_rviz:=false)...")
    sim_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 launch yunyan_sim sim_bringup.launch.py launch_rviz:=false"
    ]
    sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 等待 Gazebo 与 robot_state_publisher 启动
    time.sleep(8)

    print("[TF Tree] 启动 SLAM Toolbox 异步建图以打通 map -> odom TF (launch_rviz:=false)...")
    slam_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 launch yunyan_nav slam_sim.launch.py launch_rviz:=false"
    ]
    slam_proc = subprocess.Popen(slam_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 等待 slam_toolbox 发布 map -> odom
    print("[TF Tree] 等待 TF 树稳定广播 (10s)...")
    time.sleep(10)

    # 运行 view_frames
    print("[TF Tree] 运行 ros2 run tf2_tools view_frames...")
    view_cmd = [
        "bash", "-c",
        "cd /tmp && source /opt/ros/humble/setup.bash && "
        "ros2 run tf2_tools view_frames -o /tmp/frames -t 4"
    ]
    res = subprocess.run(view_cmd, capture_output=True, text=True)
    print(res.stdout)

    # 关闭仿真与 SLAM
    print("[TF Tree] 清理临时后台节点...")
    slam_proc.terminate()
    sim_proc.terminate()
    subprocess.run(["pkill", "-9", "-f", "slam_toolbox"])
    subprocess.run(["pkill", "-9", "-f", "gzserver"])
    subprocess.run(["pkill", "-9", "-f", "gzclient"])
    subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"])
    time.sleep(2)

    # 检查 pdf 是否生成
    if not os.path.exists(tmp_pdf):
        print(f"[TF Tree] 错误: {tmp_pdf} 未生成!")
        return 1

    print(f"[TF Tree] 使用 pdftoppm 转换 {tmp_pdf} 为 300 DPI 高清 PNG...")
    conv_cmd = ["pdftoppm", "-png", "-r", "300", tmp_pdf, tmp_png_prefix]
    subprocess.run(conv_cmd, check=True)

    gen_png = f"{tmp_png_prefix}-1.png"
    if os.path.exists(gen_png):
        shutil.move(gen_png, out_png)
        print(f"[TF Tree] 成功生成高清 TF 拓扑图: {out_png}")
        sz = os.path.getsize(out_png)
        print(f"[TF Tree] 文件大小: {sz / 1024:.1f} KB")
        return 0
    else:
        print(f"[TF Tree] 错误: {gen_png} 未找到!")
        return 1

if __name__ == '__main__':
    exit(main())
