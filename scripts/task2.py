#!/usr/bin/env python3
"""
Task 2: Obstacle Avoidance and Exploration
基于 Braitenberg Vehicle 3b 的避障 + 探索策略
"""
import math
import threading
import rclpy
import time

# 线程安全锁
lidar_lock = threading.Lock()
odom_lock = threading.Lock()
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from com2009_teamxx_2026_modules.tb3_tools import quaternion_to_euler
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # 订阅 LiDAR 和里程计
        self.lidar_sub = self.create_subscription(LaserScan, '/scan',
                                                  self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom',
                                                 self.odom_callback, 10)

        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # 避障参数
        self.safe_distance = 0.6  # 安全距离 [m]
        self.linear_vel = 0.3  # 前进速度 [m/s]
        self.angular_vel = 0.5  # 旋转速度 [rad/s]
        self.explore_angular = 0.05  # 探索时的轻微旋转 [rad/s]

        # 状态跟踪
        self.last_lidar_time = 0.0
        self.has_lidar_data = False
        self.start_time = None
        self.running = True

        self.yaw: float = 0.0
        self.x: float = 0.0
        self.y: float = 0.0

        self.front: float = self.safe_distance
        self.left: float = self.safe_distance
        self.right: float = self.safe_distance
        self.front_left: float = self.safe_distance
        self.front_right: float = self.safe_distance
        self.back: float = self.safe_distance

        self.get_logger().info('Obstacle Avoidance node initialized')

    def lidar_callback(self, msg: LaserScan):
        """处理 LiDAR 数据"""
        self.last_lidar_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.has_lidar_data = True

        # 关键方向索引 (270 度扫描，0-359 度)
        front_idx = 0  # 0 度 - 正前方
        left_idx = 90  # 90 度 - 左侧
        right_idx = 270  # 270 度 - 右侧
        back_idx = 180  # 180 度 - 后方
        # 对角线方向
        front_left_idx = 315  # -45 度
        front_right_idx = 45  # +45 度

        # 线程安全地更新 LiDAR 数据
        with lidar_lock:
            self.front = float(msg.ranges[front_idx])
            self.left = float(msg.ranges[left_idx])
            self.right = float(msg.ranges[right_idx])
            self.back = float(msg.ranges[back_idx])
            self.front_left = float(msg.ranges[front_left_idx])
            self.front_right = float(msg.ranges[front_right_idx])

        # self.get_logger().info(
        #     f'LiDAR - Front: {self.front:.2f}m, Left: {self.left:.2f}m, '
        #     f'Right: {self.right:.2f}m, Back: {self.back:.2f}m')

    def odom_callback(self, msg: Odometry):
        """处理里程计数据"""
        if self.start_time is None:
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation

        # 获取位置
        with odom_lock:
            self.x = position.x
            self.y = position.y
            _, _, self.yaw = quaternion_to_euler(orientation)

        # 计算运行时间
        elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

    def rotate_by(self, direction: str, ang_vel: float, duration: float):
        """控制小车转动方向以及旋转时间

        Args:
            direction: 'left' 或 'right'
            ang_vel: 旋转角速度 [rad/s]
            duration: 旋转持续时间 [s]
        """
        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        while (self.get_clock().now().seconds_nanoseconds()[0] -
               start_time) < duration:
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = self.get_clock().now().to_msg()
            vel_cmd.header.frame_id = 'base_footprint'
            vel_cmd.twist.linear.x = 0.0

            if direction == 'left':
                vel_cmd.twist.angular.z = ang_vel
            elif direction == 'right':
                vel_cmd.twist.angular.z = -ang_vel
            else:
                self.get_logger().warn(
                    f'Invalid direction: {direction}, stopping')
                vel_cmd.twist.angular.z = 0.0

            self.cmd_vel_pub.publish(vel_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 旋转结束后停止
        stop_cmd = TwistStamped()
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    def go_by(self, direction: str, lin_vel: float, duration: float):
        """控制小车前进方向以及时间

        Args:
            direction: 'front' 或 'back'
            ang_vel: 旋转角速度 [rad/s]
            duration: 旋转持续时间 [s]
        """
        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        while (self.get_clock().now().seconds_nanoseconds()[0] -
               start_time) < duration:
            # 检查前方是否有障碍物 (线程安全)
            with lidar_lock:
                front_dist = self.front

            # 前方有障碍物则停止
            if front_dist < self.safe_distance:
                self.get_logger().warn(
                    f'Obstacle detected! Front: {front_dist:.2f}m < {self.safe_distance:.2f}m'
                )
                break

            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = self.get_clock().now().to_msg()
            vel_cmd.header.frame_id = 'base_footprint'
            vel_cmd.twist.angular.z = 0.0

            if direction == 'front':
                vel_cmd.twist.linear.x = lin_vel
            elif direction == 'back':
                vel_cmd.twist.linear.x = -lin_vel
            else:
                self.get_logger().warn(
                    f'Invalid direction: {direction}, stopping')
                vel_cmd.twist.linear.x = 0.0

            self.cmd_vel_pub.publish(vel_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 结束后停止
        stop_cmd = TwistStamped()
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)


    def go_until(self, distance: float, err: float):
        """控制小车前进直到达到指定距离

        Args:
            distance: 期望前进的距离 [m]
            err: 允许的误差范围 [m]
        """
        # 线程安全地获取初始位置
        with odom_lock:
            init_x = self.x
            init_y = self.y

        self.get_logger().info(
            f'Starting go_until: distance={distance:.2f}m, err={err:.2f}m, '
            f'from ({init_x:.2f}, {init_y:.2f})'
        )

        while True:
            # 线程安全地读取当前位置
            with odom_lock:
                current_x = self.x
                current_y = self.y

            # 计算已走过的距离 (欧几里得距离)
            traveled = math.sqrt(
                (current_x - init_x) ** 2 + (current_y - init_y) ** 2
            )

            # 检查是否达到目标距离 (distance ± err)
            if traveled >= distance - err:
                self.get_logger().info(
                    f'Goal reached! Traveled: {traveled:.2f}m, '
                    f'Target: {distance:.2f}m ± {err:.2f}m'
                )
                break

            # 检查前方是否有障碍物 (线程安全)
            with lidar_lock:
                front_dist = self.front

            # 前方有障碍物则停止
            if front_dist < self.safe_distance:
                self.get_logger().warn(
                    f'Obstacle detected! Front: {front_dist:.2f}m < {self.safe_distance:.2f}m, '
                    f'Traveled: {traveled:.2f}m'
                )
                break

            # 发布前进命令
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = self.get_clock().now().to_msg()
            vel_cmd.header.frame_id = 'base_footprint'
            vel_cmd.twist.linear.x = self.linear_vel
            vel_cmd.twist.angular.z = 0.0

            self.cmd_vel_pub.publish(vel_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 停止机器人
        stop_cmd = TwistStamped()
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    def rotate_until(self, direction: str, deg: float, err: float):
        """控制小车旋转到指定角度

        Args:
            direction: 'left' 或 'right'
            deg: 期望旋转的角度 [度]
            err: 允许的误差范围 [度]
        """
        # 线程安全地获取初始 yaw
        with odom_lock:
            init_yaw = self.yaw

        self.get_logger().warn(
            f'Starting rotate_until: direction={direction}, '
            f'deg={deg:.1f}deg, err={err:.1f}deg, '
            f'from {math.degrees(init_yaw):.1f}deg'
        )

        while True:
            # 线程安全地读取当前 yaw
            with odom_lock:
                current_yaw = self.yaw

            # 计算旋转角度差 (考虑角度 wrap-around)
            angle_diff = current_yaw - init_yaw
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            rotated_deg = abs(math.degrees(angle_diff))

            # 检查是否达到目标角度 (deg ± err)
            if rotated_deg >= deg - err:
                self.get_logger().info(
                    f'Rotation complete! Rotated: {rotated_deg:.1f}deg, '
                    f'Target: {deg:.1f}deg ± {err:.1f}deg'
                )
                break

            # 发布旋转命令
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = self.get_clock().now().to_msg()
            vel_cmd.header.frame_id = 'base_footprint'
            vel_cmd.twist.linear.x = 0.0

            if direction == 'left':
                vel_cmd.twist.angular.z = self.angular_vel
            elif direction == 'right':
                vel_cmd.twist.angular.z = -self.angular_vel
            else:
                self.get_logger().warn(f'Invalid direction: {direction}, stopping')
                break

            self.cmd_vel_pub.publish(vel_cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 停止机器人
        stop_cmd = TwistStamped()
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    def main_loop(self):
        """主循环"""
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        # 创建并启动 run 线程
        run_thread = threading.Thread(target=self.run, daemon=True)
        run_thread.start()

        try:
            while rclpy.ok() and self.running:
                # 检查 LiDAR 数据是否更新
                current_time = self.get_clock().now().seconds_nanoseconds()[0]
                # if current_time - self.last_lidar_time > 1.0:
                # self.get_logger().warn('No LiDAR data for >1s!')

                # 检查是否超过 90 秒
                if self.start_time:
                    elapsed = current_time - self.start_time
                    if elapsed >= 90:
                        self.get_logger().info('90 seconds elapsed, stopping')
                        self.running = False

                executor.spin_once(timeout_sec=0.1)

        except KeyboardInterrupt:
            self.get_logger().info('Received shutdown request')
            self.running = False

        finally:
            # 等待 run 线程结束
            run_thread.join(timeout=1.0)

            # 停止机器人
            stop_cmd = TwistStamped()
            stop_cmd.twist.linear.x = 0.0
            stop_cmd.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)

            executor.shutdown()
            self.destroy_node()

    def run(self):
        time.sleep(0.5)
        # self.rotate_until("left", 45.0, 1.0)
        # self.go_until(math.sqrt(2.0)*1.5, 0.1)
        # self.rotate_until("left", 45.0, 1.0)
        # self.go_until(2, 0.1)
        # self.go_until(0.2, 0.1)
        # self.rotate_until("right", 90.0, 1.0)
        # self.go_until(0.7, 0.1)
        # self.rotate_until("left", 45.0, 1.0)
        # self.go_until(0.2, 0.1)
        self.rotate_until("left", 45.0, 1.0)
        self.go_until(2, 0.1)


def main():
    rclpy.init()
    node = ObstacleAvoidance()
    node.main_loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
