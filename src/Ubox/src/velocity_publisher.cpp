#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>

class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher()
        : Node("velocity_publisher") {
        // Create a publisher for the "cmd_vel" topic with message type Twist
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Set up a timer to publish messages at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&VelocityPublisher::publish_velocity, this));
    }

private:
    void publish_velocity() {
        // Create a new Twist message
        auto msg = geometry_msgs::msg::Twist();

        // Simulate target velocities
        
        msg.linear.x = 1.0;  // Linear velocity in meters per second
        msg.angular.z = 0.5; // Angular velocity in radians per second

        // Publish the message
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %.2f, angular.z = %.2f", msg.linear.x, msg.angular.z);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create and spin the VelocityPublisher node
    rclcpp::spin(std::make_shared<VelocityPublisher>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
