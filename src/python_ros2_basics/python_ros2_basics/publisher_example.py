#!/usr/bin/env python3

# ros2 python library
import rclpy
# The node class from rclpy
from rclpy.node import Node
# The message that we are using
from std_msgs.msg import String

# Create a class that inherit from ros2 rclpy Node
class TextPublisher(Node):
    def __init__(self):
        # Use the init of the Node class
        super().__init__('text_publisher')
        # Declare a publisher that will publish a String message on the topic
        # example_string_topic, with a queue size of 10
        self.publisher_ = self.create_publisher(String, "example_string_topic", 10)
        
        timer_period = 1.0
        # Create a timer that will call the function self.timer_callback every second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        # logger are a way to print in ROS2 node
        self.get_logger().info('Publisher node has been started.')
    
    # The callback called by the timer
    def timer_callback(self):
        self.counter += 1
        #Create a String message
        msg = String()
        # Set the data field of the string message with our string
        msg.data = f"{self.counter} x " + 'Welcome to Zerobotics Tutorials!'
        # Use the method publish of our the publisher_ to publish message on the topic
        self.publisher_.publish(msg)
        # Print a log message
        self.get_logger().info(f'Published: {msg.data}')

# The main function executed when we are launching the node
def main(args=None):
    # Init rclpy ROS2
    rclpy.init(args=args)
    # Create the node
    node = TextPublisher()
    # Keep the node running
    rclpy.spin(node)
    # Destroy the node on stop
    node.destroy_node()
    # Shutdown the rclpy ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
