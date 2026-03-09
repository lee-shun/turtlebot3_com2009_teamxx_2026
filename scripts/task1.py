#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# 执行器
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from com2009_teamxx_2026_modules.tb3_tools import quaternion_to_euler
from math import sqrt, pow, pi
import time

class Fig8(Node):
    def callback_function(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation
        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y
        self.x = pos_x
        self.y = pos_y
        # theta_z = orientation.y

        # convert the quaternion to roll pitch yaw
        (roll, pitch, yaw) = quaternion_to_euler(orientation)

        if self.startup:
            self.starttime = self.get_clock().now().seconds_nanoseconds()[0]
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.yaw0 = yaw
            self.get_logger().info("x=0.00 [m], y=0.00 [m], yaw=0.0 [degrees].")
        if self.counter > 10:
            self.counter = 0
            print(f"x = {(abs(self.x0) - abs(pos_x)):.2f} [m], y = {(abs(self.y0) - abs(pos_y)):.2f} [m], theta_z = {abs(abs(self.yaw0)*(180/pi)) - abs(abs(yaw)*(180/pi)):.1f} [degrees].")
        else:
            self.counter += 1

        if(abs(self.x - self.x0) < 0.01) and (abs(self.y - self.y0) < 0.01) and (self.get_clock().now().seconds_nanoseconds()[0] > self.starttime + 20):
            if self.reverse:
                #has already completed one loop, should stop moving
                # print("need stop!")
                self.need_stop = True
            else:
                self.starttime = self.get_clock().now().seconds_nanoseconds()[0]
                # print("need reverse!")
                self.reverse = True

    def __init__(self):
        super().__init__("circleof8")
        topic_name = "cmd_vel"
        self.startup = True
        self.reverse = False
        self.counter = 0
        self.need_stop = False
        self.pub = self.create_publisher(TwistStamped, topic_name, 10)
        self.sub = self.create_subscription(Odometry, "odom", self.callback_function, 10)
        self.get_logger().info(f"The '{self.get_name()}' node is active...")

    def main_loop(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            while rclpy.ok() and not self.need_stop:
                vel_cmd = TwistStamped()
                vel_cmd.header.stamp = self.get_clock().now().to_msg()
                vel_cmd.header.frame_id = "ctrl"
                vel_cmd.twist.linear.x = 0.11  # m/s
                vel_cmd.twist.linear.y = 0.0  # m/s
                vel_cmd.twist.linear.z = 0.0  # m/s
                if self.reverse: # NOTE: TRUE
                    vel_cmd.twist.angular.z = -0.22  # rad/s
                    vel_cmd.twist.angular.y = 0.0  # rad/s
                    vel_cmd.twist.angular.x = 0.0  # rad/s
                else:
                    vel_cmd.twist.angular.z = 0.22  # rad/s
                    vel_cmd.twist.angular.y = 0.0  # rad/s
                    vel_cmd.twist.angular.x = 0.0  # rad/s
                # print("publish!")
                self.pub.publish(vel_cmd)
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            if self.need_stop or not rclpy.ok():
                vel_cmd = TwistStamped()
                vel_cmd.twist.linear.x = 0.0  # m/s
                vel_cmd.twist.angular.z = 0.0  # rad/s
                self.pub.publish(vel_cmd)
            print(f"Stopping the '{self.get_name()}' node at: {self.get_clock().now().nanoseconds / 1e9}")
            executor.shutdown()
            self.destroy_node()

if __name__ == '__main__':
    rclpy.init()
    node = Fig8()
    node.main_loop()
    rclpy.shutdown()
