#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        # Parameters for desired wall distance
        self.declare_parameter('wall_distance', 0.5)
        self.wall_distance = self.get_parameter('wall_distance').get_parameter_value().double_value

        # Subscribe to laser scan topic
        self.laser_scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        # Publish velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def laser_callback(self, msg):
        # Find the closest laser reading on the left side
        min_distance_left = min(msg.ranges[0:int(len(msg.ranges) / 2)])

        # Calculate control signal based on distance to wall
        error = self.wall_distance - min_distance_left
        control_signal = error * 0.5  # Proportional control

        # Create Twist message for robot movement
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  # Forward velocity
        twist_msg.angular.z = control_signal  # Angular velocity

        # Publish velocity command
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    try:
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
