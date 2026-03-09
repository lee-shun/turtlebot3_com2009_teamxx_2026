#!/usr/bin/env python3
# A simple ROS2 Publisher

import rclpy 
import math

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.signals import SignalHandlerOptions

from example_interfaces.msg import String 
from nav_msgs.msg import Odometry
import tf_transformations

class Circle(Node): 
    # def __init__(self):
    #     self.shutdown = False
    #     super().__init__("move_circle") 

    #     self.my_publisher = self.create_publisher(
    #         msg_type=TwistStamped,
    #         topic="/cmd_vel",
    #         qos_profile=10,
    #     ) 

    #     publish_rate = 10 # Hz
    #     self.timer = self.create_timer(
    #         timer_period_sec=1/publish_rate, 
    #         callback=self.timer_callback
    #     ) 

    #     self.get_logger().info(
    #         f"The '{self.get_name()}' node is initialised." 
    #     )



    def __init__(self):
        self.shutdown = False
        super().__init__('fig_of_eight_node')

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        #-- odom calibration
        self.odom_calibrated = False
        self.prev_x = 0.0
        self.prev_y = 0.0
        # self.prev_yaw = 0.0
        self.total_distance_travel = 0.0


        self.initial_yaw = 0.0
        self.current_yaw_rel_deg = 0.0


        self.current_x = 0.0
        self.current_y = 0.0
        # self.current_yaw = 0.0
        # self.current_yaw_rel_deg = 0.0

        #-- figure-8 parameters
        self.radius = 0.5
        # BUG
        self.linear_vel = 0.26
        # self.linear_vel = 0.5
        self.angular_vel = self.linear_vel / self.radius
        self.circumference = 2 * math.pi * self.radius 

        self.state = "LOOP1"

        publish_rate = 1
        
        self.timestamp_ns = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(1/publish_rate, self.timer_callback)
        # self.log_timer = self.create_timer(1.0, self.print_odom_log)
        

        self._shutdown_done = False
        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised. Starting Loop 1" 
        )


    def odom_callback(self,msg:Odometry):
            current_x= msg.pose.pose.position.x
            current_y= msg.pose.pose.position.y

            q= (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
            euler = tf_transformations.euler_from_quaternion(q)
            current_yaw = euler[2]

            #  find the curved line of the circle
            self.total_distance_travel += math.sqrt((current_x - self.prev_x) ** 2 +
                                                    (current_y - self.prev_y) ** 2)

            self.prev_x = current_x
            self.prev_y = current_y

            self.current_yaw_rel = current_yaw - self.initial_yaw
            self.current_yaw_rel_deg = (math.degrees(self.current_yaw_rel)) % 360
            

            # if not self.odom_calibrated:
            #     self.initial_x = current_x
            #     self.intial_y = current_y
            #     self.odom_calibrated = True

            #     self.state = "Loop1"
            #     self.timestamp_ns = self.get_clock().now().nanoseconds
            #     self.get_logger().info("Odom calibrated. Starting Loop1...")
            #     return
            
            # self.current_x_rel = current_x - self.initial_x
            # self.current_y_rel = current_y - self.intial_y
            # self.current_yaw_rel = current_yaw - self.initial_yaw
            # self.current_yaw_rel_deg = (math.degrees(self.current_yaw_rel)) % 360

            if self.total_distance_travel >= self.circumference: 
                # reset all distances for next loop
                self.total_distance_travel = 0.0
                self.current_x = self.current_y = 0.0

                if self.state == "LOOP1":
                    self.state = "LOOP2"
                    self.prev_x = current_x
                    self.prev_y = current_y
                    self.get_logger().info("Loop 1 done. Loop 2 starting")
                elif self.state == "LOOP2":
                    self.state = "STOP"
                    self.get_logger().info("Loop 2 done. Stopping")



    def timer_callback(self): 
        # if not self.odom_calibrated:
        #     return
            
        msg = TwistStamped()
        
        if self.state == "LOOP1":
            msg.twist.linear.x = self.linear_vel
            msg.twist.angular.z = -self.angular_vel  # anticlockwise

        elif self.state == "LOOP2":
            msg.twist.linear.x = self.linear_vel
            msg.twist.angular.z = +self.angular_vel  # clockwise

        elif self.state == "STOP":
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            
        self.cmd_vel_pub.publish(msg)

        self.print_odom_log()
        
    #     radius = 0.5 # meters
        # linear_velocity = 0.1 # meters per second [m/s]
    #     angular_velocity = linear_velocity / radius 

    #     topic_msg = TwistStamped() 
    #     topic_msg.twist.linear.x = linear_velocity
    #     topic_msg.twist.angular.z = angular_velocity
    #     self.my_publisher.publish(topic_msg) 

    #     self.get_logger().info( 
    #         f"Linear Velocity: {topic_msg.twist.linear.x:.2f} [m/s], "
    #         f"Angular Velocity: {topic_msg.twist.angular.z:.2f} [rad/s].",
    #         throttle_duration_sec=1, 
    # )
        
    def print_odom_log(self):
        # if not self.odom_calibrated:
        #     self.get_logger().info("x=0.00 [m], y=0.00 [m], yaw=0.0 [degrees].")
        #     return
        self.get_logger().info(
            f"x={self.prev_x:.2f} [m], y={self.prev_y:.2f} [m], yaw={self.current_yaw_rel_deg:.1f} [degrees]"
        )




    def on_shutdown(self):
        self.get_logger().info(
            "Stopping the robot..."
        )
        self.cmd_vel_pub.publish(TwistStamped()) 
        self.shutdown = True

def main(args=None): 
    rclpy.init(
        args=args,
        signal_handler_options = SignalHandlerOptions.NO
    )
    node = Circle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        print(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.on_shutdown() 
        while not node.shutdown: 
            continue
        node.destroy_node() 
        rclpy.shutdown()
    

if __name__ == '__main__': 
    main()
