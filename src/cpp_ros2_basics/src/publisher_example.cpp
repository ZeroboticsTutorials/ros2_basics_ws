// Include the necessary C++ headers
#include <memory>
// ROS2 C++ Client library
#include "rclcpp/rclcpp.hpp"
// Include the message type that we will publish
#include "std_msgs/msg/string.hpp"

// Define the TextPublisher class, inheriting from rclcpp::Node
class TextPublisher : public rclcpp::Node {
public:
    TextPublisher() : Node("text_publisher"), counter_(0) {
        // Initialize the publisher, publishing to example_string_topic with a queue size of 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("example_string_topic", 10);

        // Initialize the timer with a 1-second interval
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TextPublisher::timer_callback, this)
        );

        // Log a message indicating the node has started
        RCLCPP_INFO(this->get_logger(), "Publisher node has been started.");
    }

private:
    // Callback function for the timer
    void timer_callback() {
        counter_++;
        
        // Create a String message
        auto message = std_msgs::msg::String();
        // Fill the field data of our message with a string
        message.data = std::to_string(counter_) + " x Welcome to Zerobotics Tutorials!";

        // Publish the message
        publisher_->publish(message);

        // Log the published message
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }

    // Member variables declaration
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

// The main function, equivalent to the Python main
int main(int argc, char * argv[]) {
    // Initialize rclcpp
    rclcpp::init(argc, argv);

    // Create a shared pointer for the TextPublisher node
    auto node = std::make_shared<TextPublisher>();

    // Keep the node spinning
    rclcpp::spin(node);

    // Shutdown and cleanup
    rclcpp::shutdown();
    return 0;
}