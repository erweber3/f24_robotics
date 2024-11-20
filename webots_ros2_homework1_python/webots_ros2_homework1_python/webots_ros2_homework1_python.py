import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
from apriltag_msgs.msg import AprilTagDetectionArray
import math
import random

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 1.2
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 90
RIGHT_FRONT_INDEX = 45 ############################### CHANGE HERE
LEFT_FRONT_INDEX = 315
LEFT_SIDE_INDEX = 270

class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber3 = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif reading == 0.0:
                self.scan_cleaned.append(10) ############################### CHANGE HERE
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
            	self.scan_cleaned.append(reading)



    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        # self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        self.pose_saved=position
        return None

    def apriltag_callback(self, msg):
        if msg.detections:
            self.get_logger().info('AprilTag detected: "%s"' % msg.detections)
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:359] + self.scan_cleaned[0:RIGHT_FRONT_INDEX]) ######CHANGE HERE

        if front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                self.cmd.linear.x = 0.0 
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('Stopping')
                return
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.07 
            if (right_lidar_min > left_lidar_min):
                self.cmd.angular.z = -0.3
            else:
                self.cmd.angular.z = 0.3
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning')
            self.turtlebot_moving = True
        else:
            self.cmd.linear.x = LINEAR_VEL
            self.cmd.linear.z = 0.1
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
