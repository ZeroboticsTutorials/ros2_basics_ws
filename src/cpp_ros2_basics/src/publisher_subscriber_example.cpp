// Include necessary ROS2 C++ headers
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Define the ZeroDiffDriveExample class, inheriting from rclcpp::Node
class ZeroDiffDriveExample : public rclcpp::Node {
public:
    ZeroDiffDriveExample() 
    : Node("zero_diff_drive_example"), front_distance_(std::numeric_limits<float>::infinity()) {
        // Publisher to control robot velocity on the /cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber to LaserScan messages on the /scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&ZeroDiffDriveExample::scan_callback, this, std::placeholders::_1)
        );

        // Timer to periodically check distance and decide movement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&ZeroDiffDriveExample::control_loop, this)
        );

        // Log the initialization message
        RCLCPP_INFO(this->get_logger(), "Zero Diff Drive Controller Node has been started.");
    }

private:
    // Callback function for processing laser scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // Get the front angle index in the LaserScan ranges
        size_t front_angle_index = scan_msg->ranges.size() / 2;
        front_distance_ = scan_msg->ranges[front_angle_index];
        
        RCLCPP_INFO(this->get_logger(), "Front distance: %.2f meters", front_distance_);
    }

    // Control loop to decide robot movement based on sensor readings
    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();

        // Check if the front distance is within the threshold
        if (front_distance_ < front_distance_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected! Stop the robot.");
            msg.linear.x = 0.0;  // Stop the robot
        } else {
            RCLCPP_INFO(this->get_logger(), "Path is clear, moving forward.");
            msg.linear.x = 0.2;  // Move forward with a linear velocity
        }

        // Publish the velocity command to the /cmd_vel topic
        publisher_->publish(msg);
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    float front_distance_;
    const float front_distance_threshold_ = 0.5;
};

// Main function, similar to Python main
int main(int argc, char * argv[]) {
    // Initialize rclcpp
    rclcpp::init(argc, argv);

    // Create a shared pointer for the ZeroDiffDriveExample node
    auto node = std::make_shared<ZeroDiffDriveExample>();

    // Keep the node spinning to process incoming messages
    rclcpp::spin(node);

    // Shutdown and cleanup
    rclcpp::shutdown();
    return 0;
}
