#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
APF 人工势场避障实时矢量场渲染与动图生成器
实时订阅 /rpl_scan, /odom, /cmd_vel
可视化:
  - 8m x 8m 封闭房间与全部静态障碍物
  - 360° RPLIDAR 点云与探测射线 (斥力区黄色, 死锁区亮红)
  - 引力场向量 (Attraction Vector)、横向斥力场向量 (Repulsion Vector)、合成运动指令向量 (Command Vector)
  - 机器人体、朝向与动态避障航迹
  - 实时势场遥测仪表盘
生成:
  - 06_apf_avoidance.gif (<10MB)
"""

import os
import math
import time
import subprocess
from pathlib import Path
import numpy as np
import cv2
from PIL import Image

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

ASSETS_DIR = str(Path(__file__).resolve().parent.parent / 'report_assets')
Path(ASSETS_DIR).mkdir(parents=True, exist_ok=True)

class APFVisualizer(Node):
    def __init__(self):
        super().__init__('apf_visualizer')

        self.scan_msg = None
        self.odom_msg = None
        self.cmd_vel_msg = None

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.trail = []
        self.r_max = 0.80
        self.r_min = 0.15
        self.d_deadlock = 0.35

        self.min_front_dist = float('inf')
        self.lateral_force = 0.0
        self.left_repulsion = 0.0
        self.right_repulsion = 0.0
        self.repulsion_count = 0
        self.is_deadlock = False

        # 8m x 8m 房间范围 [-4.2, 4.2]
        self.min_x = -4.2
        self.max_x = 4.2
        self.min_y = -4.2
        self.max_y = 4.2

        self.img_w = 1200
        self.img_h = 1000

        self.create_subscription(LaserScan, '/rpl_scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.get_logger().info('APF 矢量场渲染器已就绪。')

    def odom_cb(self, msg):
        self.odom_msg = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        if len(self.trail) == 0 or math.hypot(self.robot_x - self.trail[-1][0], self.robot_y - self.trail[-1][1]) > 0.04:
            self.trail.append((self.robot_x, self.robot_y))

    def cmd_vel_cb(self, msg):
        self.cmd_vel_msg = msg
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z

    def scan_cb(self, msg):
        self.scan_msg = msg
        angle = msg.angle_min
        min_front = float('inf')
        lat_f = 0.0
        l_rep = 0.0
        r_rep = 0.0
        count = 0

        deadlock_angle = math.pi / 6.0
        sector_angle = math.pi / 2.0

        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r) or r < self.r_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            if -deadlock_angle <= angle <= deadlock_angle:
                if r < min_front:
                    min_front = r

            if -sector_angle <= angle <= sector_angle and r <= self.r_max:
                f_rep = (1.0 / r - 1.0 / self.r_max) * (1.0 / (r * r))
                lat_f += -f_rep * math.sin(angle)
                count += 1
                if angle > 0:
                    l_rep += f_rep
                else:
                    r_rep += f_rep

            angle += msg.angle_increment

        self.min_front_dist = min_front
        self.lateral_force = lat_f
        self.left_repulsion = l_rep
        self.right_repulsion = r_rep
        self.repulsion_count = count
        self.is_deadlock = (min_front < self.d_deadlock)

    def w2p(self, wx, wy):
        px = int((wx - self.min_x) / (self.max_x - self.min_x) * self.img_w)
        py = int((self.max_y - wy) / (self.max_y - self.min_y) * self.img_h)
        return px, py

    def render_frame(self):
        canvas = np.full((self.img_h, self.img_w, 3), 35, dtype=np.uint8)

        # 1. 网格背景
        for x in np.arange(-4.0, 4.1, 0.5):
            p1 = self.w2p(x, -4.0)
            p2 = self.w2p(x, 4.0)
            col = (55, 55, 55) if abs(x % 1.0) > 0.1 else (75, 75, 75)
            cv2.line(canvas, p1, p2, col, 1)

        for y in np.arange(-4.0, 4.1, 0.5):
            p1 = self.w2p(-4.0, y)
            p2 = self.w2p(4.0, y)
            col = (55, 55, 55) if abs(y % 1.0) > 0.1 else (75, 75, 75)
            cv2.line(canvas, p1, p2, col, 1)

        # 2. 绘制 8m x 8m 封闭房间几何轮廓与障碍物
        # 外墙
        p_nw = self.w2p(-4.0, 4.0)
        p_se = self.w2p(4.0, -4.0)
        cv2.rectangle(canvas, p_nw, p_se, (160, 160, 160), 4)

        # 内部隔断墙 1 (纵向 x=1.0, y=0.0~2.5)
        p_w1_a = self.w2p(1.0, 0.0)
        p_w1_b = self.w2p(1.0, 2.5)
        cv2.line(canvas, p_w1_a, p_w1_b, (140, 140, 140), 6)

        # 内部隔断墙 2 (横向拐角 x=-1.5~1.0, y=2.5)
        p_w2_a = self.w2p(-1.5, 2.5)
        p_w2_b = self.w2p(1.0, 2.5)
        cv2.line(canvas, p_w2_a, p_w2_b, (140, 140, 140), 6)

        # 内部分流墙 3 (x=-1.8, y=-2.8~-0.2)
        p_w3_a = self.w2p(-1.8, -2.8)
        p_w3_b = self.w2p(-1.8, -0.2)
        cv2.line(canvas, p_w3_a, p_w3_b, (140, 140, 140), 6)

        # 障碍物 1: 圆柱 (2.4, -2.0, R=0.35m)
        c1 = self.w2p(2.4, -2.0)
        r1 = int(0.35 / (self.max_x - self.min_x) * self.img_w)
        cv2.circle(canvas, c1, r1, (180, 100, 40), -1)
        cv2.circle(canvas, c1, r1, (220, 130, 60), 2)

        # 障碍物 2: 方柱 (1.2, -2.8, 0.6x0.6)
        b1_a = self.w2p(0.9, -2.5)
        b1_b = self.w2p(1.5, -3.1)
        cv2.rectangle(canvas, b1_a, b1_b, (40, 60, 200), -1)
        cv2.rectangle(canvas, b1_a, b1_b, (60, 90, 240), 2)

        # 障碍物 3: 圆柱 (-2.6, 1.5, R=0.40m)
        c2 = self.w2p(-2.6, 1.5)
        r2 = int(0.40 / (self.max_x - self.min_x) * self.img_w)
        cv2.circle(canvas, c2, r2, (40, 160, 80), -1)
        cv2.circle(canvas, c2, r2, (60, 200, 100), 2)

        # 障碍物 4: 方柱 (2.6, 2.2, 0.7x0.5)
        b2_a = self.w2p(2.25, 2.45)
        b2_b = self.w2p(2.95, 1.95)
        cv2.rectangle(canvas, b2_a, b2_b, (40, 180, 200), -1)
        cv2.rectangle(canvas, b2_a, b2_b, (60, 220, 240), 2)

        # 3. 历史轨迹线 (亮青色渐变)
        if len(self.trail) > 1:
            pts = [self.w2p(x, y) for (x, y) in self.trail]
            for i in range(len(pts) - 1):
                alpha = (i + 1) / len(pts)
                col = (int(255 * alpha), int(220 * alpha), 0)
                cv2.line(canvas, pts[i], pts[i+1], col, 2, cv2.LINE_AA)

        # 4. 绘制激光雷达扫描光束与势场作用点
        rx_p, ry_p = self.w2p(self.robot_x, self.robot_y)
        if self.scan_msg:
            angle = self.scan_msg.angle_min + self.robot_yaw
            for r in self.scan_msg.ranges:
                if not math.isnan(r) and not math.isinf(r) and self.scan_msg.range_min < r < self.scan_msg.range_max:
                    hit_x = self.robot_x + r * math.cos(angle)
                    hit_y = self.robot_y + r * math.sin(angle)
                    hp = self.w2p(hit_x, hit_y)

                    # 根据距离区分势场影响级别
                    if r < self.d_deadlock:
                        # 危险死锁区: 红色激光射线与亮点
                        cv2.line(canvas, (rx_p, ry_p), hp, (0, 0, 180), 1)
                        cv2.circle(canvas, hp, 4, (0, 0, 255), -1)
                    elif r <= self.r_max:
                        # 斥力作用区: 黄色射线与点
                        cv2.line(canvas, (rx_p, ry_p), hp, (0, 140, 180), 1)
                        cv2.circle(canvas, hp, 3, (0, 215, 255), -1)
                    else:
                        # 安全区域: 暗蓝细点
                        cv2.circle(canvas, hp, 1, (100, 100, 100), -1)
                angle += self.scan_msg.angle_increment

        # 5. 绘制势场影响安全圆环 (斥力边界 0.8m 与死锁边界 0.35m)
        r_rep_pix = int(self.r_max / (self.max_x - self.min_x) * self.img_w)
        r_dead_pix = int(self.d_deadlock / (self.max_x - self.min_x) * self.img_w)
        cv2.circle(canvas, (rx_p, ry_p), r_rep_pix, (0, 180, 220), 1, cv2.LINE_AA)
        cv2.circle(canvas, (rx_p, ry_p), r_dead_pix, (0, 0, 220), 1, cv2.LINE_AA)

        # 6. 绘制动态势场受力矢量箭头 (Attraction, Repulsion, Resultant)
        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)

        # (a) 引力矢量 F_att: 沿小车朝向 (蓝色箭头)
        att_len = 0.6
        att_end_w = (self.robot_x + att_len * cos_y, self.robot_y + att_len * sin_y)
        att_p = self.w2p(att_end_w[0], att_end_w[1])
        cv2.arrowedLine(canvas, (rx_p, ry_p), att_p, (255, 120, 0), 2, tipLength=0.25)

        # (b) 横向斥力矢量 F_rep: 垂直于车头方向 (亮红/橙色箭头)
        # 向左为正 (+pi/2), 向右为负 (-pi/2)
        if self.repulsion_count > 0:
            f_lat_val = self.lateral_force / self.repulsion_count
            rep_mag = min(abs(f_lat_val) * 0.45, 0.8)
            sign = 1.0 if f_lat_val > 0 else -1.0
            rep_angle = self.robot_yaw + sign * (math.pi / 2.0)
            rep_end_w = (self.robot_x + rep_mag * math.cos(rep_angle), self.robot_y + rep_mag * math.sin(rep_angle))
            rep_p = self.w2p(rep_end_w[0], rep_end_w[1])
            cv2.arrowedLine(canvas, (rx_p, ry_p), rep_p, (0, 69, 255), 3, tipLength=0.3)

        # (c) 合成速度控制向量 V_cmd (亮绿色粗箭头)
        if self.lin_vel > 0.02:
            res_mag = 0.85
            res_angle = self.robot_yaw + self.ang_vel * 0.4
            res_end_w = (self.robot_x + res_mag * math.cos(res_angle), self.robot_y + res_mag * math.sin(res_angle))
            res_p = self.w2p(res_end_w[0], res_end_w[1])
            cv2.arrowedLine(canvas, (rx_p, ry_p), res_p, (46, 204, 113), 3, tipLength=0.3)

        # 7. 绘制小车车体与轮系
        car_l, car_w = 0.28, 0.22
        corners = [
            (-car_l/2, -car_w/2),
            (car_l/2, -car_w/2),
            (car_l/2, car_w/2),
            (-car_l/2, car_w/2)
        ]
        poly_pts = []
        for cx, cy in corners:
            wx = self.robot_x + cx * cos_y - cy * sin_y
            wy = self.robot_y + cx * sin_y + cy * cos_y
            poly_pts.append(self.w2p(wx, wy))
        poly_np = np.array([poly_pts], dtype=np.int32)
        cv2.fillPoly(canvas, poly_np, (230, 230, 230))
        cv2.polylines(canvas, poly_np, True, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.circle(canvas, (rx_p, ry_p), 5, (0, 0, 255), -1)

        # 8. 顶部控制与遥测仪表盘
        cv2.rectangle(canvas, (10, 10), (self.img_w - 10, 85), (20, 20, 20), -1)
        cv2.rectangle(canvas, (10, 10), (self.img_w - 10, 85), (80, 80, 80), 1)

        cv2.putText(canvas, "APF REACTIVE POTENTIAL FIELD AVOIDANCE MONITOR", (25, 38),
                    cv2.FONT_HERSHEY_DUPLEX, 0.70, (255, 255, 255), 2, cv2.LINE_AA)

        if self.is_deadlock:
            state_text = f"DEADLOCK ESCAPE ROTATION (Front: {self.min_front_dist:.2f}m < 0.35m)"
            state_col = (0, 0, 255)
        elif self.repulsion_count > 0:
            state_text = f"SMOOTH REPULSION SWERVE (Active Rays: {self.repulsion_count})"
            state_col = (0, 215, 255)
        else:
            state_text = "FORWARD CRUISING (Free Space)"
            state_col = (46, 204, 113)

        cv2.putText(canvas, f"STATE: {state_text}", (25, 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, state_col, 2, cv2.LINE_AA)

        telemetry = f"Pos: ({self.robot_x:+.2f}, {self.robot_y:+.2f})m  Yaw: {math.degrees(self.robot_yaw):+5.1f}deg  V_lin: {self.lin_vel:.2f}m/s  W_ang: {self.ang_vel:+.2f}rad/s  Front: {self.min_front_dist:.2f}m"
        cv2.putText(canvas, telemetry, (430, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

        # 底部矢量图例
        cv2.rectangle(canvas, (15, self.img_h - 45), (self.img_w - 15, self.img_h - 12), (25, 25, 25), -1)
        cv2.rectangle(canvas, (15, self.img_h - 45), (self.img_w - 15, self.img_h - 12), (70, 70, 70), 1)

        legends = [
            ((255, 120, 0), "Attraction Vector F_att"),
            ((0, 69, 255), "Repulsion Vector F_rep"),
            ((46, 204, 113), "Resultant Command V_cmd"),
            ((0, 215, 255), "Repulsion Zone (R<=0.8m)"),
            ((0, 0, 255), "Deadlock Zone (R<=0.35m)"),
            ((0, 220, 255), "Traversed Path")
        ]
        x_leg = 30
        for col, label in legends:
            cv2.rectangle(canvas, (x_leg, self.img_h - 32), (x_leg + 18, self.img_h - 22), col, -1)
            cv2.putText(canvas, label, (x_leg + 25, self.img_h - 23), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
            x_leg += 185

        return canvas

def main():
    rclpy.init()
    node = APFVisualizer()

    frames = []
    out_gif = os.path.join(ASSETS_DIR, '06_apf_avoidance.gif')

    print("[APF Renderer] 等待雷达与里程计数据接入...")
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.scan_msg and node.odom_msg:
            break

    print("[APF Renderer] 开始录制 APF 避障过程 (12秒)...")
    start_time = time.time()
    while time.time() - start_time < 12.0:
        rclpy.spin_once(node, timeout_sec=0.08)
        frame = node.render_frame()
        frames.append(frame)

    print(f"[APF Renderer] 录制完成，共采集 {len(frames)} 帧。正在合成为优化 GIF...")
    tmp_mp4 = "/tmp/apf_rendered.mp4"
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_video = cv2.VideoWriter(tmp_mp4, fourcc, 14.0, (800, 666))

    step = max(1, len(frames) // 140)
    for i in range(0, len(frames), step):
        f_resized = cv2.resize(frames[i], (800, 666), interpolation=cv2.INTER_AREA)
        out_video.write(f_resized)
    out_video.release()

    gif_filter = "fps=12,split[s0][s1];[s0]palettegen=stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3"
    conv_cmd = [
        "ffmpeg", "-y", "-i", tmp_mp4,
        "-vf", gif_filter,
        out_gif
    ]
    subprocess.run(conv_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if os.path.exists(out_gif):
        sz_mb = os.path.getsize(out_gif) / (1024 * 1024)
        print(f"[APF Renderer] 成功生成动图 06_apf_avoidance.gif ({sz_mb:.2f} MB)")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
