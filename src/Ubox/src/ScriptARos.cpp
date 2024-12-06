

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "shared_data.hpp"
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

constexpr double WHEEL_DISTANCE = 0.443; 
constexpr double WHEEL_DIAMETER = 0.181; 
constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

class VelocityProcessor : public rclcpp::Node {
public:
    VelocityProcessor(SharedData* shared_data)
        : Node("velocity_processor"), shared_data_(shared_data) {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&VelocityProcessor::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_velocity = msg->linear.x; 
        double angular_velocity = msg->angular.z; 

        // Calculate wheel velocities
        double left_wheel_velocity = linear_velocity - (angular_velocity * WHEEL_DISTANCE / 2.0);
        double right_wheel_velocity = linear_velocity + (angular_velocity * WHEEL_DISTANCE / 2.0);

        // Convert to RPM
        double left_wheel_rpm = (left_wheel_velocity / WHEEL_CIRCUMFERENCE) * 60.0;
        double right_wheel_rpm = (right_wheel_velocity / WHEEL_CIRCUMFERENCE) * 60.0;

        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        auto epoch = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch();
        std::int64_t timestamp_ms = epoch.count();

        // Update shared memory data
        shared_data_->target_left_wheel_rpm = left_wheel_rpm;
        shared_data_->target_right_wheel_rpm = right_wheel_rpm;
        shared_data_->input_linear_velocity = linear_velocity;
        shared_data_->input_angular_velocity = angular_velocity;
        shared_data_->timestamp_ms = timestamp_ms;

        RCLCPP_INFO(this->get_logger(), "Updated wheel velocities: Left=%.2f RPM, Right=%.2f RPM", left_wheel_rpm, right_wheel_rpm);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    SharedData* shared_data_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    
    int shm_fd = shm_open(SHARED_MEMORY_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("Failed to open shared memory");
        return -1;
    }

    
    if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
        perror("Failed to resize shared memory");
        return -1;
    }

    
    SharedData* shared_data = static_cast<SharedData*>(mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (shared_data == MAP_FAILED) {
        perror("Failed to map shared memory");
        return -1;
    }

    
    auto node = std::make_shared<VelocityProcessor>(shared_data);
    rclcpp::spin(node);
    rclcpp::shutdown();

    
    munmap(shared_data, SHARED_MEMORY_SIZE);
    close(shm_fd);

    return 0;
}


