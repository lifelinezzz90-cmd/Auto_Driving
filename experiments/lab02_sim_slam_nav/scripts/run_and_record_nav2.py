#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键自动化运行 Gazebo + Nav2，采集静态场景截图与导航全过程 GIF
产出:
  - 03_gazebo_scene.png
  - 04_rviz_nav2_static.png
  - 05_nav2_planning_keyframe.png
  - 05_nav2_auto_navigation.gif
"""

import os
import sys
import time
import subprocess
import threading
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
REPO_ROOT = WS_ROOT.parent.parent

sys.path.append(str(SCRIPT_DIR))
from window_manager import find_window, raise_and_focus_window, get_capture_geometry

ASSETS_DIR = str(WS_ROOT / 'report_assets')
Path(ASSETS_DIR).mkdir(parents=True, exist_ok=True)

def cleanup():
    """清理所有仿真与可视化后台进程."""
    print("[Nav2 Pipeline] 清理后台 Gazebo 与 ROS2 进程...")
    for proc in ["nav2_", "rviz2", "gzserver", "gzclient", "robot_state_publisher", "slam_toolbox", "apf_avoidance"]:
        subprocess.run(["pkill", "-9", "-f", proc], stderr=subprocess.DEVNULL)
    time.sleep(2)

def main():
    cleanup()

    print("[Nav2 Pipeline] 1. 启动 Gazebo 仿真环境 (launch_rviz:=false)...")
    sim_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 launch yunyan_sim sim_bringup.launch.py launch_rviz:=false"
    ]
    sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(9)

    print("[Nav2 Pipeline] 2. 启动 Nav2 导航栈与 RViz2 (launch_rviz:=true)...")
    nav2_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 launch yunyan_nav nav2_sim.launch.py launch_rviz:=true"
    ]
    nav2_proc = subprocess.Popen(nav2_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("[Nav2 Pipeline] 等待 AMCL 粒子滤波自定位与代价地图膨胀层加载 (14s)...")
    time.sleep(14)

    # 3. 定位窗口
    print("[Nav2 Pipeline] 3. 查找 Gazebo 与 RViz2 窗口...")
    gz_win = find_window("Gazebo")
    rviz_win = find_window("RViz")

    if not gz_win:
        print("[Nav2 Pipeline] 警告: 未找到匹配 'Gazebo' 的窗口，尝试更宽泛匹配...")
        all_wins = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
        print("窗口列表部分片段:\n", "\n".join(all_wins.splitlines()[:20]))

    if not rviz_win:
        print("[Nav2 Pipeline] 警告: 未找到匹配 'RViz' 的窗口！")

    # 4. 采集 03_gazebo_scene.png
    out_gazebo = os.path.join(ASSETS_DIR, '03_gazebo_scene.png')
    if gz_win:
        print(f"[Nav2 Pipeline] 置顶 Gazebo 窗口 ({gz_win['id_hex']}) 并截图...")
        raise_and_focus_window(gz_win['id'])
        time.sleep(0.8)
        gx, gy, gw, gh = get_capture_geometry(gz_win)
        cap_cmd = [
            "ffmpeg", "-y", "-f", "x11grab", "-draw_mouse", "0",
            "-video_size", f"{gw}x{gh}", "-i", f":0.0+{gx},{gy}",
            "-vframes", "1", out_gazebo
        ]
        subprocess.run(cap_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[Nav2 Pipeline] 已生成: {out_gazebo} (大小: {os.path.getsize(out_gazebo)/1024:.1f} KB)")
    else:
        print("[Nav2 Pipeline] 回退: 全屏抓取仿真区域...")
        subprocess.run(["ffmpeg", "-y", "-f", "x11grab", "-video_size", "1920x1080", "-i", ":0.0+0,0", "-vframes", "1", out_gazebo])

    # 5. 采集 04_rviz_nav2_static.png
    out_rviz = os.path.join(ASSETS_DIR, '04_rviz_nav2_static.png')
    if rviz_win:
        print(f"[Nav2 Pipeline] 置顶 RViz2 窗口 ({rviz_win['id_hex']}) 并截取初始静态多图层...")
        raise_and_focus_window(rviz_win['id'])
        time.sleep(0.8)
        rx, ry, rw, rh = get_capture_geometry(rviz_win)
        cap_cmd = [
            "ffmpeg", "-y", "-f", "x11grab", "-draw_mouse", "0",
            "-video_size", f"{rw}x{rh}", "-i", f":0.0+{rx},{ry}",
            "-vframes", "1", out_rviz
        ]
        subprocess.run(cap_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[Nav2 Pipeline] 已生成: {out_rviz} (大小: {os.path.getsize(out_rviz)/1024:.1f} KB)")

    # 6. 录制 Nav2 动态导航全过程与关键帧
    print("[Nav2 Pipeline] 4. 开始录制 Nav2 动态导航过程与提取关键帧...")
    out_mp4 = "/tmp/nav2_trip.mp4"
    out_gif = os.path.join(ASSETS_DIR, '05_nav2_auto_navigation.gif')
    out_keyframe = os.path.join(ASSETS_DIR, '05_nav2_planning_keyframe.png')

    if rviz_win:
        rx, ry, rw, rh = get_capture_geometry(rviz_win)
        raise_and_focus_window(rviz_win['id'])
        time.sleep(0.5)

        # 启动 ffmpeg 录屏 (时长 13 秒)
        rec_cmd = [
            "ffmpeg", "-y", "-f", "x11grab", "-draw_mouse", "0",
            "-video_size", f"{rw}x{rh}", "-i", f":0.0+{rx},{ry}",
            "-r", "15", "-t", "13", out_mp4
        ]
        rec_proc = subprocess.Popen(rec_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 等待 1 秒使录制稳定启动，然后下发导航目标点 (目标点选在开阔自由区 x: 2.4, y: 0.3)
        time.sleep(1.0)
        print("[Nav2 Pipeline] 下发导航目标点 (x: 2.4, y: 0.3)...")
        goal_cmd = [
            "bash", "-c",
            "source /opt/ros/humble/setup.bash && "
            f"source {WS_ROOT / 'install' / 'setup.bash'} && "
            f"python3 {SCRIPT_DIR / 'send_nav2_goal.py'}"
        ]
        goal_proc = subprocess.Popen(goal_cmd)

        # 在小车行驶与避障中途 (第 4~5 秒) 抓取关键帧 05_nav2_planning_keyframe.png
        time.sleep(4.5)
        print(f"[Nav2 Pipeline] 抓取导航规划中途关键帧: {out_keyframe}...")
        keyframe_cmd = [
            "ffmpeg", "-y", "-f", "x11grab", "-draw_mouse", "0",
            "-video_size", f"{rw}x{rh}", "-i", f":0.0+{rx},{ry}",
            "-vframes", "1", out_keyframe
        ]
        subprocess.run(keyframe_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.exists(out_keyframe):
            print(f"[Nav2 Pipeline] 已生成关键帧: {out_keyframe} ({os.path.getsize(out_keyframe)/1024:.1f} KB)")

        # 等待录制完成
        rec_proc.wait()
        goal_proc.wait(timeout=10)

        # 转换为高品质优化调色板动图 GIF
        print(f"[Nav2 Pipeline] 将 {out_mp4} 转换为高质量 GIF: {out_gif}...")
        gif_filter = "fps=12,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen=stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3"
        conv_cmd = [
            "ffmpeg", "-y", "-i", out_mp4,
            "-vf", gif_filter,
            out_gif
        ]
        subprocess.run(conv_cmd, check=True)
        if os.path.exists(out_gif):
            sz = os.path.getsize(out_gif)
            print(f"[Nav2 Pipeline] 成功生成动图: {out_gif} ({sz/1024/1024:.2f} MB)")

    # 7. 清理
    cleanup()
    print("[Nav2 Pipeline] 阶段 3 & 阶段 4 视觉资产采集圆满完成！")
    return 0

if __name__ == '__main__':
    exit(main())
