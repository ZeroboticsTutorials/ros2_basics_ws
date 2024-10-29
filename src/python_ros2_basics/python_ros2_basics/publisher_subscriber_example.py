#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ZeroDiffDriveExample(Node):
    def __init__(self):
        super().__init__('zero_diff_drive_example')

        # Publisher to control robot velocity (cmd_vel topic)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to scan topic (LaserScan data)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Timer to periodically check distance and decide movement
        self.timer = self.create_timer(0.2, self.control_loop)  # Every 0.2 second
        self.front_distance = None
        self.front_distance_threshold = 0.5 #meters
        self.get_logger().info('Zero Diff Drive Controller Node has been started.')

    def scan_callback(self, scan_msg):
        """Callback function that processes laser scan data."""
        # Scan data is an array, we can check the values of angle_min
        # angle_max and increment to know which range we want
        # We consider the middle of the array to get the front value
        front_angle_index = len(scan_msg.ranges) // 2
        self.front_distance = scan_msg.ranges[front_angle_index]
        self.get_logger().info(f'Front distance: {self.front_distance:.2f} meters')

    def control_loop(self):
        """Control loop that moves the robot based on sensor readings."""
        msg = Twist()
        # Check if the first message has been received before moving the robot
        if self.front_distance is not None:
            # If the front distance is less than 0.5 meters, stop
            if self.front_distance < self.front_distance_threshold:
                self.get_logger().info('Obstacle detected! Stop the robot.')
                msg.linear.x = 0.0
            else:
                self.get_logger().info('Path is clear, moving forward.')
                msg.linear.x = 0.2  # Move forward with linear velocity

            # Publish the velocity command to the /cmd_vel topic
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZeroDiffDriveExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
