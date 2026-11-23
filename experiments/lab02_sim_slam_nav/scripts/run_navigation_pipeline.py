#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键执行 Nav2 仿真、目标下发与高精多图层录制全流程
"""

import os
import time
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
REPO_ROOT = WS_ROOT.parent.parent

def cleanup():
    print("[Pipeline] 清理后台 Gazebo 与 ROS2 进程...")
    procs = [
        "controller_server", "planner_server", "bt_navigator", "costmap_2d",
        "amcl", "map_server", "lifecycle_manager", "rviz2", "gzserver",
        "gzclient", "robot_state_publisher", "slam_toolbox", "apf_avoidance"
    ]
    for proc in procs:
        subprocess.run(["pkill", "-9", "-f", proc], stderr=subprocess.DEVNULL)
    time.sleep(2)

def main():
    cleanup()

    print("[Pipeline] 1. 启动 Gazebo 仿真环境 (launch_rviz:=false)...")
    sim_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 launch yunyan_sim sim_bringup.launch.py launch_rviz:=false"
    ]
    sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(9)

    print("[Pipeline] 2. 启动 Nav2 导航栈 (launch_rviz:=false)...")
    nav2_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 launch yunyan_nav nav2_sim.launch.py launch_rviz:=false"
    ]
    nav2_proc = subprocess.Popen(nav2_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("[Pipeline] 等待 AMCL 粒子滤波与代价地图完全加载 (14s)...")
    time.sleep(14)

    # 3. 启动后台可视化录制节点
    print("[Pipeline] 3. 启动高精可视化渲染与录制节点...")
    vis_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        f"python3 {SCRIPT_DIR / 'nav2_visualizer.py'}"
    ]
    vis_proc = subprocess.Popen(vis_cmd)
    time.sleep(2.0)

    # 4. 下发导航目标点 (开阔走廊自由区 x: 2.4, y: 0.3)
    print("[Pipeline] 4. 下发导航目标点 (x: 2.4, y: 0.3)...")
    goal_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        f"python3 {SCRIPT_DIR / 'send_nav2_goal.py'}"
    ]
    goal_proc = subprocess.Popen(goal_cmd)

    # 等待录制与导航完成
    vis_proc.wait()
    try:
        goal_proc.wait(timeout=5)
    except Exception:
        pass

    cleanup()
    print("[Pipeline] Nav2 自动化执行与视觉生成完毕！")
    return 0

if __name__ == '__main__':
    exit(main())
