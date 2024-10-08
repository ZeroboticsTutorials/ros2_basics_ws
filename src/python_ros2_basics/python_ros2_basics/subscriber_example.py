#!/usr/bin/env python3

# ros2 python library
import rclpy
# The node class from rclpy
from rclpy.node import Node
# The message that we are using
from std_msgs.msg import String

# Create a class that inherit from ros2 rclpy Node
class TextSubscriber(Node):
    def __init__(self):
        # Use the init of the Node class
        super().__init__('text_subscriber')
        # Declare a subscriber that will subscribe to the topic with String message type
        # example_string_topic, with the callbach self.listener_callback and a queue size of 10
        self.subscription = self.create_subscription(
            String, 
            "example_string_topic", 
            self.listener_callback, 
            10)
        # Create a class attribute to store the message from topic
        self.last_message = String()
        # Print the init log of the node
        self.get_logger().info('Subscriber node has been started.')

    
    # The callback associtated to the subscriber
    def listener_callback(self, msg):
        self.last_message.data = msg.data
        self.get_logger().info(f'Received: {self.last_message.data}')


# The main function executed when we are launching the node
def main(args=None):
    # Init rclpy ROS2
    rclpy.init(args=args)
    # Create the node
    node = TextSubscriber()
    # Keep the node running
    rclpy.spin(node)
    # Destroy the node on stop
    node.destroy_node()
    # Shutdown the rclpy ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
