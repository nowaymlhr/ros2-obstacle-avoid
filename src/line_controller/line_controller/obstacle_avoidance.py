#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class ObstacleAvoidanceNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("obstacle_avoidance") # MODIFY NAME

        self.declare_parameter('start', False)

        self.subscriber_ = self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start = self.get_parameter('start').value
        self.line_error = 0.0
        self.line_subscriber_ = self.create_subscription(Float64, "/track_detection", self.line_error_callback, 10)

        self.add_post_set_parameters_callback(self.parameter_callback)

    def line_error_callback(self, msg):
        self.line_error = msg.data

    def laser_scan_callback(self, msg):

        if self.start:
            ranges = list(msg.ranges)
            ranges = [r if not math.isinf(r) else msg.range_max for r in ranges]
            front = min(ranges[160:200])
            min_left = min(ranges[160:180]) #Checking the left side of the robot
            min_right = min(ranges[180:200]) #Checking the right side of the robot

            twist = Twist()

            Kp = 0.3


    

            if front < 1.2:  # obstacle detected with hysteresis
                if min_left < min_right:
                    twist.angular.z = 0.9  # keep turning until clear
                else:
                    twist.angular.z = -0.9  # keep turning until clear
                twist.linear.x = 0.2  # slow forward to help turn
            else:
                twist.linear.x = 0.7
                twist.angular.z = -Kp * self.line_error  # proportional control for smoother turns
            
            
            self.get_logger().info(f"left: {min_left}, right: {min_right}, front: {min(ranges[160:200])}")
            self.publisher_.publish(twist)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start' and param.type_ == rclpy.Parameter.Type.BOOL:
                self.start = param.value
                self.get_logger().info(f"Start parameter set to: {self.start}")

        
        


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
