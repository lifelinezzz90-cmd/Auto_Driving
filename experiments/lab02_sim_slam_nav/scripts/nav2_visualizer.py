#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav2 多图层全景可视化渲染与自动导航动图录制器
实时订阅 /map, /global_costmap/costmap, /local_costmap/costmap, /plan, /local_plan, /rpl_scan, /odom, /cmd_vel
生成:
  - 04_rviz_nav2_static.png (静态图层全景)
  - 05_nav2_planning_keyframe.png (避障规划中途关键帧)
  - 05_nav2_auto_navigation.gif (全自主导航流畅动图)
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
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

ASSETS_DIR = str(Path(__file__).resolve().parent.parent / 'report_assets')
Path(ASSETS_DIR).mkdir(parents=True, exist_ok=True)

class Nav2Visualizer(Node):
    def __init__(self):
        super().__init__('nav2_visualizer')

        self.map_msg = None
        self.global_costmap_msg = None
        self.local_costmap_msg = None
        self.plan_msg = None
        self.local_plan_msg = None
        self.scan_msg = None
        self.odom_msg = None
        self.cmd_vel_msg = None

        self.goal_x = 2.4
        self.goal_y = 0.3
        self.goal_yaw = 0.0

        self.trail = [] # 历史轨迹点 [(x, y), ...]
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        # 世界坐标范围 [-4.2, 4.2]
        self.min_x = -4.2
        self.max_x = 4.2
        self.min_y = -4.2
        self.max_y = 4.2

        # 画布尺寸 (偶数对齐)
        self.img_w = 1200
        self.img_h = 1000

        # 订阅各图层话题
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_cb, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_cb, 10)
        self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self.local_plan_cb, 10)
        self.create_subscription(LaserScan, '/rpl_scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.get_logger().info('Nav2 高清可视化渲染器已就绪。')

    def map_cb(self, msg):
        self.map_msg = msg

    def global_costmap_cb(self, msg):
        self.global_costmap_msg = msg

    def local_costmap_cb(self, msg):
        self.local_costmap_msg = msg

    def plan_cb(self, msg):
        self.plan_msg = msg

    def local_plan_cb(self, msg):
        self.local_plan_msg = msg

    def scan_cb(self, msg):
        self.scan_msg = msg

    def cmd_vel_cb(self, msg):
        self.cmd_vel_msg = msg
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z

    def odom_cb(self, msg):
        self.odom_msg = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # 提取四元数 yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 记录轨迹点
        if len(self.trail) == 0 or math.hypot(self.robot_x - self.trail[-1][0], self.robot_y - self.trail[-1][1]) > 0.05:
            self.trail.append((self.robot_x, self.robot_y))

    def w2p(self, wx, wy):
        """世界坐标 (米) 转画布像素坐标 (px, py)."""
        px = int((wx - self.min_x) / (self.max_x - self.min_x) * self.img_w)
        py = int((self.max_y - wy) / (self.max_y - self.min_y) * self.img_h)
        return px, py

    def render_frame(self, status_text="NAV2 AUTONOMOUS NAVIGATION"):
        """渲染一帧高精全景可视化画面 (BGR 格式)."""
        canvas = np.full((self.img_h, self.img_w, 3), 45, dtype=np.uint8)

        # 1. 绘制网格背景线与边界 (0.5m 细线, 1m 粗线)
        for x in np.arange(-4.0, 4.1, 0.5):
            p1 = self.w2p(x, -4.0)
            p2 = self.w2p(x, 4.0)
            col = (70, 70, 70) if abs(x % 1.0) > 0.1 else (95, 95, 95)
            cv2.line(canvas, p1, p2, col, 1)

        for y in np.arange(-4.0, 4.1, 0.5):
            p1 = self.w2p(-4.0, y)
            p2 = self.w2p(4.0, y)
            col = (70, 70, 70) if abs(y % 1.0) > 0.1 else (95, 95, 95)
            cv2.line(canvas, p1, p2, col, 1)

        # 2. 绘制静态地图 (/map)
        if self.map_msg:
            info = self.map_msg.info
            mw, mh = info.width, info.height
            m_res = info.resolution
            m_ox = info.origin.position.x
            m_oy = info.origin.position.y

            m_data = np.array(self.map_msg.data, dtype=np.int8).reshape((mh, mw))
            # 创建 RGBA 覆盖层
            map_rgba = np.zeros((mh, mw, 3), dtype=np.uint8)
            # -1 为未知区域 (40, 40, 40)
            map_rgba[m_data == -1] = [40, 40, 40]
            # 0 为可行走自由空间 (230, 230, 230)
            map_rgba[m_data == 0] = [230, 230, 230]
            # 100 为静态障碍物 (15, 15, 15)
            map_rgba[m_data == 100] = [15, 15, 15]

            # 缩放到画布对应区域
            p_bl = self.w2p(m_ox, m_oy)
            p_tr = self.w2p(m_ox + mw * m_res, m_oy + mh * m_res)
            x_min, x_max = min(p_bl[0], p_tr[0]), max(p_bl[0], p_tr[0])
            y_min, y_max = min(p_bl[1], p_tr[1]), max(p_bl[1], p_tr[1])

            # 翻转 Y 轴匹配图像坐标
            map_flipped = cv2.flip(map_rgba, 0)
            map_resized = cv2.resize(map_flipped, (x_max - x_min, y_max - y_min), interpolation=cv2.INTER_NEAREST)
            canvas[y_min:y_max, x_min:x_max] = map_resized

        # 3. 绘制全局代价地图膨胀层 (/global_costmap/costmap)
        if self.global_costmap_msg:
            c_info = self.global_costmap_msg.info
            cw, ch = c_info.width, c_info.height
            c_res = c_info.resolution
            c_ox = c_info.origin.position.x
            c_oy = c_info.origin.position.y
            c_data = np.array(self.global_costmap_msg.data, dtype=np.int8).reshape((ch, cw))

            # 仅处理有膨胀代价值的像素 (>0 且 <100)
            cost_mask = (c_data > 0) & (c_data <= 100)
            if np.any(cost_mask):
                # 颜色渐变: 1~99 映射为从青绿 -> 黄 -> 橙红
                cost_normalized = np.clip(c_data, 0, 100).astype(np.uint8)
                color_map = cv2.applyColorMap(cost_normalized * 2, cv2.COLORMAP_JET)

                p_bl = self.w2p(c_ox, c_oy)
                p_tr = self.w2p(c_ox + cw * c_res, c_oy + ch * c_res)
                x_min, x_max = min(p_bl[0], p_tr[0]), max(p_bl[0], p_tr[0])
                y_min, y_max = min(p_bl[1], p_tr[1]), max(p_bl[1], p_tr[1])

                color_map_flipped = cv2.flip(color_map, 0)
                mask_flipped = cv2.flip(cost_mask.astype(np.uint8), 0)
                color_map_resized = cv2.resize(color_map_flipped, (x_max - x_min, y_max - y_min), interpolation=cv2.INTER_NEAREST)
                mask_resized = cv2.resize(mask_flipped, (x_max - x_min, y_max - y_min), interpolation=cv2.INTER_NEAREST)

                sub_canvas = canvas[y_min:y_max, x_min:x_max]
                idx = mask_resized > 0
                sub_canvas[idx] = cv2.addWeighted(sub_canvas[idx], 0.45, color_map_resized[idx], 0.55, 0)

        # 4. 绘制局部代价地图窗口 (/local_costmap/costmap)
        if self.local_costmap_msg:
            l_info = self.local_costmap_msg.info
            lw, lh = l_info.width, l_info.height
            l_res = l_info.resolution
            l_ox = l_info.origin.position.x
            l_oy = l_info.origin.position.y
            p1 = self.w2p(l_ox, l_oy)
            p2 = self.w2p(l_ox + lw * l_res, l_oy + lh * l_res)
            # 绘制局部代价地图边界框 (亮蓝细虚线)
            cv2.rectangle(canvas, (min(p1[0], p2[0]), min(p1[1], p2[1])),
                          (max(p1[0], p2[0]), max(p1[1], p2[1])), (255, 190, 0), 2)

        # 5. 绘制历史轨迹线条 (渐变紫红)
        if len(self.trail) > 1:
            pts = [self.w2p(x, y) for (x, y) in self.trail]
            for i in range(len(pts) - 1):
                alpha = (i + 1) / len(pts)
                col = (int(220 * alpha), int(50 * alpha), int(200 * alpha))
                cv2.line(canvas, pts[i], pts[i+1], col, 2, cv2.LINE_AA)

        # 6. 绘制 Nav2 全局路径规划线 (/plan - 亮绿粗线)
        if self.plan_msg and len(self.plan_msg.poses) > 1:
            plan_pts = [self.w2p(p.pose.position.x, p.pose.position.y) for p in self.plan_msg.poses]
            for i in range(len(plan_pts) - 1):
                cv2.line(canvas, plan_pts[i], plan_pts[i+1], (46, 204, 113), 3, cv2.LINE_AA)
            # 绘制关键路径节点
            for i in range(0, len(plan_pts), 10):
                cv2.circle(canvas, plan_pts[i], 3, (39, 174, 96), -1)

        # 7. 绘制 DWB 局部轨迹线 (/local_plan - 亮青色)
        if self.local_plan_msg and len(self.local_plan_msg.poses) > 1:
            loc_pts = [self.w2p(p.pose.position.x, p.pose.position.y) for p in self.local_plan_msg.poses]
            for i in range(len(loc_pts) - 1):
                cv2.line(canvas, loc_pts[i], loc_pts[i+1], (0, 245, 255), 3, cv2.LINE_AA)

        # 8. 绘制激光雷达点云 (/rpl_scan - 鲜红点)
        if self.scan_msg:
            angle = self.scan_msg.angle_min + self.robot_yaw
            rp_center = self.w2p(self.robot_x, self.robot_y)
            for r in self.scan_msg.ranges:
                if not math.isnan(r) and not math.isinf(r) and self.scan_msg.range_min < r < self.scan_msg.range_max:
                    hit_x = self.robot_x + r * math.cos(angle)
                    hit_y = self.robot_y + r * math.sin(angle)
                    hp = self.w2p(hit_x, hit_y)
                    # 绘制微弱激光射线与高亮击中点
                    cv2.circle(canvas, hp, 2, (30, 30, 235), -1)
                angle += self.scan_msg.angle_increment

        # 9. 绘制导航目标点 (Goal Pose)
        gp = self.w2p(self.goal_x, self.goal_y)
        cv2.circle(canvas, gp, 12, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(canvas, gp, 6, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(canvas, f"Goal ({self.goal_x:.1f}, {self.goal_y:.1f})",
                    (gp[0] + 15, gp[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 240), 2, cv2.LINE_AA)

        # 10. 绘制机器人本体与朝向 (黄色底盘矩形与朝向箭头)
        rx, ry = self.w2p(self.robot_x, self.robot_y)
        # 车体外包轮廓 (0.28m x 0.22m)
        car_l, car_w = 0.28, 0.22
        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)
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
        cv2.fillPoly(canvas, poly_np, (0, 215, 255))
        cv2.polylines(canvas, poly_np, True, (0, 140, 255), 2, cv2.LINE_AA)

        # 朝向箭头 (Heading)
        arrow_end_w = (self.robot_x + 0.35 * cos_y, self.robot_y + 0.35 * sin_y)
        cv2.arrowedLine(canvas, (rx, ry), self.w2p(arrow_end_w[0], arrow_end_w[1]), (0, 0, 255), 3, tipLength=0.35)
        # 旋转中心激光雷达
        cv2.circle(canvas, (rx, ry), 5, (255, 0, 0), -1)

        # 11. 顶部专业仪表盘卡片与数据遥测
        cv2.rectangle(canvas, (10, 10), (self.img_w - 10, 85), (20, 20, 20), -1)
        cv2.rectangle(canvas, (10, 10), (self.img_w - 10, 85), (80, 80, 80), 1)

        # 标题与状态
        cv2.putText(canvas, "NAV2 AUTONOMOUS NAVIGATION MONITOR (LAB02 - M3)", (25, 38),
                    cv2.FONT_HERSHEY_DUPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
        
        status_color = (46, 204, 113) if "REACHED" in status_text else (0, 215, 255)
        cv2.putText(canvas, f"STATUS: {status_text}", (25, 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, status_color, 2, cv2.LINE_AA)

        dist_to_goal = math.hypot(self.robot_x - self.goal_x, self.robot_y - self.goal_y)
        telemetry = f"Pos: ({self.robot_x:+.2f}, {self.robot_y:+.2f})m  Yaw: {math.degrees(self.robot_yaw):+5.1f}deg  Dist: {dist_to_goal:.2f}m  Lin: {self.lin_vel:.2f}m/s  Ang: {self.ang_vel:+.2f}rad/s"
        cv2.putText(canvas, telemetry, (360, 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1, cv2.LINE_AA)

        # 图例框 (底部浮层)
        cv2.rectangle(canvas, (15, self.img_h - 45), (self.img_w - 15, self.img_h - 12), (25, 25, 25), -1)
        cv2.rectangle(canvas, (15, self.img_h - 45), (self.img_w - 15, self.img_h - 12), (70, 70, 70), 1)

        legend_items = [
            ((46, 204, 113), "Global Plan"),
            ((0, 245, 255), "Local DWB Plan"),
            ((30, 30, 235), "Laser Scan (/rpl_scan)"),
            ((255, 190, 0), "Local Costmap Window"),
            ((0, 215, 255), "Robot (Chassis & Odom)"),
            ((200, 50, 220), "Traversed Path")
        ]
        x_leg = 30
        for col, label in legend_items:
            cv2.rectangle(canvas, (x_leg, self.img_h - 32), (x_leg + 18, self.img_h - 22), col, -1)
            cv2.putText(canvas, label, (x_leg + 25, self.img_h - 23), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
            x_leg += 185

        return canvas

def main():
    rclpy.init()
    node = Nav2Visualizer()

    # 1. 循环等待直到接收到 /map 与 /global_costmap/costmap
    print("[Nav2 Renderer] 等待静态地图与代价地图初始化...")
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.map_msg and node.global_costmap_msg:
            break

    # 2. 导出 04_rviz_nav2_static.png
    out_static = os.path.join(ASSETS_DIR, '04_rviz_nav2_static.png')
    static_frame = node.render_frame(status_text="NAV2 INITIALIZED - STATIC LAYERS & COSTMAPS READY")
    cv2.imwrite(out_static, static_frame)
    print(f"[Nav2 Renderer] 成功生成 04_rviz_nav2_static.png ({os.path.getsize(out_static)/1024:.1f} KB)")

    # 3. 动态导航录制
    frames = []
    out_gif = os.path.join(ASSETS_DIR, '05_nav2_auto_navigation.gif')
    out_keyframe = os.path.join(ASSETS_DIR, '05_nav2_planning_keyframe.png')

    keyframe_saved = False
    print("[Nav2 Renderer] 开始动态录制 Nav2 避障导航全流程...")

    start_time = time.time()
    max_duration = 22.0 # 最长录制 22 秒

    while time.time() - start_time < max_duration:
        rclpy.spin_once(node, timeout_sec=0.08)
        elapsed = time.time() - start_time

        dist = math.hypot(node.robot_x - node.goal_x, node.robot_y - node.goal_y)
        if dist < 0.25:
            st = "GOAL REACHED! (SUCCESS)"
        elif node.plan_msg:
            st = f"ACTIVE TRACKING & OBSTACLE AVOIDING (T={elapsed:.1f}s)"
        else:
            st = f"GLOBAL PLANNING (T={elapsed:.1f}s)"

        frame = node.render_frame(status_text=st)
        frames.append(frame)

        # 在行驶中途 (第 4~5 秒，小车绕障时) 保存 05_nav2_planning_keyframe.png
        if elapsed >= 4.5 and not keyframe_saved:
            cv2.imwrite(out_keyframe, frame)
            keyframe_saved = True
            print(f"[Nav2 Renderer] 成功截取关键帧 05_nav2_planning_keyframe.png ({os.path.getsize(out_keyframe)/1024:.1f} KB)")

        if dist < 0.22 and elapsed > 8.0:
            print("[Nav2 Renderer] 机器人已到达目标点，稳定停留录制 1.5 秒后结束...")
            for _ in range(15):
                rclpy.spin_once(node, timeout_sec=0.1)
                frames.append(node.render_frame(status_text="GOAL REACHED! (SUCCESS)"))
            break

    # 4. 生成高质量 GIF
    print(f"[Nav2 Renderer] 录制完成，共采集 {len(frames)} 帧。正在合成为优化动图...")
    tmp_mp4 = "/tmp/nav2_rendered.mp4"
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
        print(f"[Nav2 Renderer] 成功生成动图 05_nav2_auto_navigation.gif ({sz_mb:.2f} MB)")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
