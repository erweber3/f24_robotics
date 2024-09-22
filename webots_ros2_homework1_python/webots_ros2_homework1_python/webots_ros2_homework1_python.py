import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

LINEAR_VEL = 0.2
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
WALL_FOLLOW_DISTANCE = 0.5  # Target distance from the wall
STALL_THRESHOLD = 0.1  # Threshold to determine a stall

RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower_node')
        self.scan_cleaned = []
        self.stall = False
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
        
        self.pose_saved = None
        self.cmd = Twist()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def listener_callback1(self, msg):
        scan = msg.ranges
        self.scan_cleaned = []
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    def listener_callback2(self, msg):
        position = msg.pose.pose.position
        self.pose_saved = position

    def timer_callback(self):
        if len(self.scan_cleaned) == 0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        # Check for stall
        if self.pose_saved is not None:
            diff_x = abs(self.pose_saved.x - self.scan_cleaned[0])  # Assuming the robot moves mostly in x
            if diff_x < STALL_THRESHOLD:
                self.stall = True
            else:
                self.stall = False

        if self.stall:
            self.get_logger().info('Recovering from stall')
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3  # Turn to escape
        else:
            # Wall-following logic
            if right_lidar_min < WALL_FOLLOW_DISTANCE:
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = 0.3  # Turn left
            elif left_lidar_min < WALL_FOLLOW_DISTANCE:
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = -0.3  # Turn right
            else:
                self.cmd.linear.x = LINEAR_VEL
                self.cmd.angular.z = 0.0  # Move straight

        self.publisher_.publish(self.cmd)
        self.get_logger().info('Publishing: "%s"' % self.cmd)

def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollower()
    rclpy.spin(wall_follower_node)
    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
