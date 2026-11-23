#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动化下发 Nav2 导航目标点脚本 (NavigateToPose Action)
目标点设置在经过空间验证的无障碍自由区 (x: 2.4, y: 0.3)
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2GoalSender(Node):
    def __init__(self, target_x=2.4, target_y=0.3, target_yaw=0.0):
        super().__init__('nav2_auto_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw
        self.goal_done = False
        self.success = False

    def send_goal(self, timeout_sec=25.0):
        self.get_logger().info('等待 Nav2 navigate_to_pose 动作服务器就绪...')
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('动作服务器连接超时！')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target_x)
        goal_msg.pose.pose.position.y = float(self.target_y)
        goal_msg.pose.pose.position.z = 0.0

        # 四元数转换 (绕 Z 轴旋转 yaw)
        half_yaw = self.target_yaw * 0.5
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        self.get_logger().info(f'发送导航目标点: x={self.target_x:.2f}, y={self.target_y:.2f}, yaw={self.target_yaw:.2f} rad')
        send_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标点被 Nav2 规划器拒绝！')
            self.goal_done = True
            return

        self.get_logger().info('导航目标点已被 Nav2 成功接收，开始全局路径规划与跟踪...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'距离目标剩余: {feedback.distance_remaining:.2f}m')

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Nav2 导航结束，状态码: {status}')
        self.success = (status == 4) # STATUS_SUCCEEDED = 4
        self.goal_done = True

def main():
    rclpy.init()
    sender = Nav2GoalSender(target_x=2.4, target_y=0.3, target_yaw=0.0)
    if not sender.send_goal():
        rclpy.shutdown()
        return 1

    start_time = sender.get_clock().now()
    while rclpy.ok() and not sender.goal_done:
        rclpy.spin_once(sender, timeout_sec=0.2)
        elapsed = (sender.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > 35.0:
            sender.get_logger().warn('导航执行超时(35s)，提前退出。')
            break

    sender.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()
