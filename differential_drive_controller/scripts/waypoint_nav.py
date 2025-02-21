#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')
        
        # Declare parameters
        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', -2.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', -4.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        
        # Get parameters
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_waypoint = 0
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    
    def pid_control(self, error):
        self.integral += error * 0.1
        derivative = (error - self.prev_error) / 0.1
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("Reached all waypoints!")
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint]
        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        distance = math.sqrt(error_x**2 + error_y**2)
        
        if distance < 0.2:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint+1}")
            self.current_waypoint += 1
            return
        
        target_angle = math.atan2(error_y, error_x)
        angle_error = target_angle - self.current_yaw
        
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        angular_velocity = self.pid_control(angle_error)
        linear_velocity = min(0.5, distance * 0.5)  # Cap speed at 0.5 m/s
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
