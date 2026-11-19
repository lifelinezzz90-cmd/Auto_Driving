"""基于人工势场法 (APF) 的独立反应式激光避障节点."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class APFAvoidance(Node):
    """基于人工势场法实现反应式避障与死锁脱困."""

    def __init__(self):
        """初始化订阅者、发布者与算法参数."""
        super().__init__('apf_avoidance')

        # 运动学与避障参数
        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('max_turn_speed', 1.0)
        self.declare_parameter('deadlock_turn_speed', 0.6)
        self.declare_parameter('repulsion_max_dist', 0.80)
        self.declare_parameter('repulsion_min_dist', 0.15)
        self.declare_parameter('deadlock_dist', 0.35)

        self.v_forward = self.get_parameter('forward_speed').value
        self.w_max = self.get_parameter('max_turn_speed').value
        self.w_deadlock = self.get_parameter('deadlock_turn_speed').value
        self.r_max = self.get_parameter('repulsion_max_dist').value
        self.r_min = self.get_parameter('repulsion_min_dist').value
        self.d_deadlock = self.get_parameter('deadlock_dist').value

        # 发布 /cmd_vel 与订阅 /rpl_scan
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'rpl_scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('APF 人工势场反应式避障节点已启动。')

    def scan_callback(self, msg: LaserScan):
        """处理激光雷达扫描数据并计算合成速度指令."""
        twist = Twist()

        angle = msg.angle_min
        min_front_dist = float('inf')
        left_repulsion = 0.0
        right_repulsion = 0.0
        lateral_force = 0.0
        repulsion_count = 0

        # 正前方扇区阈值: [-pi/6, +pi/6]
        deadlock_angle = math.pi / 6.0
        # 前方半球扇区阈值: [-pi/2, +pi/2]
        sector_angle = math.pi / 2.0

        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                angle += msg.angle_increment
                continue

            if r < self.r_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            # 1. 检测正前方扇区死锁与防撞距离
            if -deadlock_angle <= angle <= deadlock_angle:
                if r < min_front_dist:
                    min_front_dist = r

            # 2. 提取前方 [-pi/2, +pi/2] 扇区在影响距离内的有效点
            if -sector_angle <= angle <= sector_angle and r <= self.r_max:
                # 斥力场强度: F_rep = (1/r - 1/r_max) * (1 / r^2)
                f_rep = (1.0 / r - 1.0 / self.r_max) * (1.0 / (r * r))
                # 横向分量 (向左为正, 向右为负)
                lateral_force += -f_rep * math.sin(angle)
                repulsion_count += 1

                if angle > 0.0:
                    left_repulsion += f_rep
                else:
                    right_repulsion += f_rep

            angle += msg.angle_increment

        # 3. 决策逻辑：死锁脱困 vs 正常势场避障
        if min_front_dist < self.d_deadlock:
            # 触发死锁保护：原地旋转脱困
            twist.linear.x = 0.0
            # 若左侧斥力大于右侧，向右转脱困；反之向左转
            if left_repulsion > right_repulsion:
                twist.angular.z = -self.w_deadlock
            else:
                twist.angular.z = self.w_deadlock
            self.get_logger().debug(
                f'触发死锁脱困: 前方距离 {min_front_dist:.2f}m, 转向 {twist.angular.z:.2f}'
            )
        else:
            # 正常引力推进
            twist.linear.x = float(self.v_forward)
            if repulsion_count > 0:
                # 计算平均横向斥力并映射为角速度
                f_lat_avg = lateral_force / repulsion_count
                angular_z = 0.35 * f_lat_avg
                # 限幅在 [-max_turn_speed, max_turn_speed]
                twist.angular.z = float(
                    max(min(angular_z, self.w_max), -self.w_max)
                )
            else:
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def stop_car(self):
        """发布全零速度确保安全停车."""
        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):
    """启动 APF 避障节点."""
    rclpy.init(args=args)
    node = None

    try:
        node = APFAvoidance()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.stop_car()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
