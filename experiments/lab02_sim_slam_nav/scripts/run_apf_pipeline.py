#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键启动 APF 人工势场避障与录制流程
生成:
  - 06_apf_avoidance.gif
"""

import os
import time
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
WS_ROOT = SCRIPT_DIR.parent
REPO_ROOT = WS_ROOT.parent.parent

def cleanup():
    print("[APF Pipeline] 清理后台 Gazebo 与 ROS2 进程...")
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

    print("[APF Pipeline] 1. 启动 Gazebo 仿真环境 (launch_rviz:=false)...")
    sim_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 launch yunyan_sim sim_bringup.launch.py launch_rviz:=false"
    ]
    sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(9)

    print("[APF Pipeline] 2. 启动 APF 反应式避障节点...")
    apf_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        "ros2 run yunyan_nav apf_avoidance"
    ]
    apf_proc = subprocess.Popen(apf_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

    print("[APF Pipeline] 3. 启动 APF 矢量场录制器 (录制 12 秒)...")
    vis_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT / 'install' / 'setup.bash'} && "
        f"python3 {SCRIPT_DIR / 'apf_visualizer.py'}"
    ]
    vis_proc = subprocess.Popen(vis_cmd)

    vis_proc.wait()
    cleanup()
    print("[APF Pipeline] APF 自动化避障与动图生成完毕！")
    return 0

if __name__ == '__main__':
    exit(main())
