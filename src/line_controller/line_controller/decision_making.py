#!/usr/bin/env python3
import math

import rclpy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node


class DecisionMakingNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("decision_making") # MODIFY NAME

        self.declare_parameter('start', False)

        self.obstacle_ = self.create_subscription(LaserScan, "/scan", self.obstacle_callback, 10)
        self.line_error_subscriber_ = self.create_subscription(Float64, "/track_detection", self.line_error_callback, 10)
        self.decision_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.front_ = float('inf')
        self.min_right = 0.0
        self.min_left = 0.0
        self.line_error_ = 0.0
        self.last_error = 0.0
        self.start_ = self.get_parameter('start').value
        self.avoid_direction = 0 #1 - Left, -1 - right
        self.avoid_state = 0
        self.timer_ = 0
        self.state_ = 1

        self.add_post_set_parameters_callback(self.parameter_callback)

        self.create_timer(0.1, self.make_decision) # Call make_decision every 100ms to ensure timely responses to sensor updates

    def obstacle_callback(self, msg):
        ranges = list(msg.ranges)
        ranges = [r if not math.isinf(r) else msg.range_max for r in ranges]
        self.front_ = min(ranges[160:200])
        self.min_right = min(ranges[200:240])
        self.min_left = min(ranges[120:160])

    def line_error_callback(self, msg):
        self.line_error_ = msg.data

    def make_decision(self):
        twist = Twist()
        if self.start_:

            if self.front_ < 1.2 and self.avoid_state == 0 and self.state_ == 1: # If an obstacle is detected in front and we are not currently avoiding
                self.state_ = 0
                self.avoid_state = 1
                self.avoid_direction = 1 if self.min_left < self.min_right else -1 # Decide which direction to turn based on which side has more space
                self.avoid_timer = 0
            
            if self.avoid_state == 1: # If we are currently avoiding
                Kp = 0.8
                twist.angular.z = 0.3 * self.avoid_direction # Turn in the chosen direction
                twist.linear.x = 0.2 # Slow forward to help turn
                self.avoid_timer += 1
                if self.avoid_timer > 30: # After 2 seconds of avoiding, check if we can stop avoiding
                    self.avoid_state = 2
                    self.avoid_timer = 0
            
            elif self.avoid_state == 2: # After the initial turn, continue forward for a short time to clear the obstacle
                twist.linear.x = 0.7
                self.avoid_timer += 1
                if self.avoid_timer > 15: # After 1 second of moving forward, check if we can stop avoiding
                    self.avoid_state = 0
                    self.avoid_timer = 0
                    self.state_ = 1 # Return to line following state
                
            elif self.state_ == 1 and self.front_ >= 1.2: # If no obstacle is detected, we can use the line error to follow the line
                error = self.line_error_
                sum_errors = error + self.last_error

                #Using a simple proportional controller (for turning)
                Kp = 0.005
                Ki = 0.0
                Kd = 0.01

                twist = Twist()
                twist.linear.x = max(0.1, 0.3 - 0.001 *abs(error)) # Constant forward speed
                twist.angular.z = -(Kp * error + Ki * sum_errors + Kd * (error - self.last_error)) # Turn based on the error (negative sign to turn in the correct direction)
                self.last_error = error # Update the last error for the next iteration

            self.get_logger().info(f"front: {self.front_}, line error: {self.line_error_}")
            self.decision_publisher_.publish(twist)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start':
                self.start_ = param.value
                self.get_logger().info(f"Start parameter set to: {self.start_}")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakingNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
