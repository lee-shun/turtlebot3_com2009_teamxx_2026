#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import tf_transformations
import math
import time



class FigOfEightNode(Node):
    def __init__(self):
        super().__init__('fig_of_right_node')
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        #-- odom calibration
        self.odom_calibrated = False
        self.initial_x = 0.0
        self.intial_y = 0.0
        self.initial_yaw = 0.0

        self.current_x_rel = 0.0
        self.current_y_rel = 0.0
        self.current_yaw_rel_deg = 0.0

        #-- figure-8 parameters
        self.radius = 0.5
        self.linear_vel = 0.105
        self.angular_vel = self.linear_vel / self.radius

        self.loop_duration = 76.0
        self.state = "CALIBRATE"
        
        self.timestamp_ns = self.get_clock().now().nanoseconds
        self.ctrl_timer = self.create_timer(0.1, self.control_step)

        # spin
        self.log_timer = self.create_timer(1.0, self.print_odom_log)
        

        self._shutdown_done = False
        self.get_logger().info("Fig-of-eight started.Waiting for...")

    def odom_callback(self,msg:Odometry):
        q= (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        euler = tf_transformations.euler_from_quaternion(q)
        current_yaw = euler[2]
        current_x= msg.pose.pose.position.x
        current_y= msg.pose.pose.position.y

        if not self.odom_calibrated:
            self.initial_x = current_x
            self.intial_y = current_y
            self.initial_yaw = current_yaw
            self.odom_calibrated = True

            self.state = "Loop1"
            self.timestamp_ns = self.get_clock().now().nanoseconds
            self.get_logger().info("Odom calibrated. Starting Loop1...")
            return
        
        self.current_x_rel = current_x - self.initial_x
        self.current_y_rel = current_y - self.intial_y
        self.current_yaw_rel = current_yaw - self.initial_yaw
        self.current_yaw_rel_deg = (math.degrees(self.current_yaw_rel)) % 360

    def print_odom_log(self):
        if not self.odom_calibrated:
            self.get_logger().info("x=0.00 [m], y=0.00 [m], yaw=0.0 [degrees].")
            return
        self.get_logger().info(
            f"x={self.current_x_rel:.2f} [m], y={self.current_y_rel:.2f} [m], yaw={self.current_yaw_rel_deg:.1f} [degrees]"
        )

    def publish_cmd(self, lin_x: float, ang_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(lin_x)
        msg.twist.angular.z = float(ang_z)
        self.cmd_vel_pub.publish(msg)

    def control_step(self):
        if not self.odom_calibrated:
            self.publish_cmd(0.0, 0.0)
            return
        
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.timestamp_ns) * 1e-9
        print("elapsed:", elapsed)

        if self.state == "Loop1":
            self.publish_cmd(self.linear_vel, +self.angular_vel)

            if elapsed >= self.loop_duration:
                self.state = "Loop2"
                self.timestamp_ns = self.get_clock().now().nanoseconds
                self.get_logger().info("Loop1 done. Starting Loop2...")

        elif self.state == "Loop2":
            self.publish_cmd(self.linear_vel, -self.angular_vel)

            if elapsed >= self.loop_duration:
                self.state = "STOP"
                self.get_logger().info("Loop2 done. Stoping")

        elif self.state == "STOP":
            self.publish_cmd(0.0, 0.0)

    def on_shutdown(self):
        self.get_logger().info("Stopping robot... ")
        for _ in range(4):
            self.publish_cmd(0.0, 0.0)
        self._shutdown_done = True

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    fig8 = FigOfEightNode()

    try:
        rclpy.spin(fig8)
    except KeyboardInterrupt:
        print(f"{fig8.get_name()} received a shutdown request (Ctrl+C)")
    finally:
        fig8.on_shutdown()
        fig8.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
