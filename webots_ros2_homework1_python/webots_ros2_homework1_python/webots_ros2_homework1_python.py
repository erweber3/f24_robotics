class RandomWalk(Node):

    def __init__(self):
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
        timer_period = 0.5  # Timer for movement callback
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.spin_cycle_interval = 20.0  # Spin every 20 seconds
        self.spin_duration = 3.0        # Spin for 5 seconds
        self.spin_cycle_timer = self.create_timer(self.spin_cycle_interval, self.spin_cycle_callback)
        self.is_spinning = False
        self.spin_start_time = None

    def spin_cycle_callback(self):
        """Initiates the spin cycle."""
        if not self.is_spinning:
            self.get_logger().info('Starting spin cycle')
            self.is_spinning = True
            self.spin_start_time = self.get_clock().now()
            self.cmd.angular.z = 2.0  # Angular velocity for spinning
            self.cmd.linear.x = 0.0   # Ensure no forward/backward movement
            self.publisher_.publish(self.cmd)

    def timer_callback(self):
        """Handles movement and logic during normal operation."""
        if self.is_spinning:
            # Check spin duration
            elapsed_time = (self.get_clock().now() - self.spin_start_time).nanoseconds * 1e-9
            if elapsed_time >= self.spin_duration:
                self.get_logger().info('Spin cycle complete')
                self.is_spinning = False
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)
            return  # Skip normal operations while spinning

        # Normal operation logic below
        if not self.scan_cleaned:
            self.turtlebot_moving = False
            return

        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:359] + self.scan_cleaned[0:RIGHT_FRONT_INDEX])

        if front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.2
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('Stopping')
                return
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.3
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning')
            self.turtlebot_moving = True
        else:
            self.cmd.linear.x = LINEAR_VEL
            self.cmd.angular.z = -0.2
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Moving forward')

        self.get_logger().info('Distance of the obstacle: %f' % front_lidar_min)

