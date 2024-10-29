// Include necessary C++ headers
#include <memory>
// ROS2 C++ Client library
#include "rclcpp/rclcpp.hpp"
// Include the message type that we will subscribe
#include "std_msgs/msg/string.hpp"

// Define the TextSubscriber class, inheriting from rclcpp::Node
class TextSubscriber : public rclcpp::Node {
public:
    TextSubscriber() : Node("text_subscriber") {
        // Initialize the subscriber, subscribing to example_string_topic with a queue size of 10
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "example_string_topic", 
            10, 
            std::bind(&TextSubscriber::listener_callback, this, std::placeholders::_1)
        );

        // Log a message indicating the subscriber node has started
        RCLCPP_INFO(this->get_logger(), "Subscriber node has been started.");
    }

private:
    // Callback function called when a message is received
    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
        last_message.data = msg->data;
        // Log the received message
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", last_message.data.c_str());
    }

    // Member variable for the subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std_msgs::msg::String last_message;
};

// Main function, similar to Python main
int main(int argc, char * argv[]) {
    // Initialize rclcpp
    rclcpp::init(argc, argv);

    // Create a shared pointer for the TextSubscriber node
    auto node = std::make_shared<TextSubscriber>();

    // Keep the node spinning to process incoming messages
    rclcpp::spin(node);

    // Shutdown and cleanup
    rclcpp::shutdown();
    return 0;
}
