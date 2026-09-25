#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <cmath>
#include <algorithm>

class RacerNode : public rclcpp::Node {
public:
    RacerNode() : Node("racer_node") {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan", 10,
            std::bind(&RacerNode::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "🚀 Forward-Looking Racer Started!");
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    // --- Tuning Parameters ---
    double kp_angle_ = 1.2;      // How aggressively to steer
    double max_steering_ = 0.45; // Physical limit of the wheels
    double max_speed_ = 0.9;     // Straightaway speed
    double min_speed_ = 0.05;     // Cornering speed

    // Helper function to safely grab a specific laser ray
    double get_range(const sensor_msgs::msg::LaserScan::SharedPtr& msg, double desired_angle_rad) {
        int index = (desired_angle_rad - msg->angle_min) / msg->angle_increment;
        
        if (index >= 0 && index < static_cast<int>(msg->ranges.size())) {
            double r = msg->ranges[index];
            // If the ray hits nothing, assume open space (5.0 meters)
            if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
                return r;
            }
        }
        return 5.0; 
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. Look Diagonally (63 degrees = ~1.1 radians)
        double left_diagonal = get_range(msg, 1.1);
        double right_diagonal = get_range(msg, -1.1);

        // 2. Calculate the error
        double error = left_diagonal - right_diagonal;

        // 3. Proportional Steering
        double steering = kp_angle_ * error;
        steering = std::max(-max_steering_, std::min(steering, max_steering_));

        // 4. Dynamic speed math (slow down in corners!)
        double speed = max_speed_ - 1.8 * std::abs(steering);
        speed = std::max(min_speed_, std::min(speed, max_speed_));

        // 5. Publish the command
        ackermann_msgs::msg::AckermannDrive drive_msg;
        drive_msg.speed = speed;
        drive_msg.steering_angle = steering;
        publisher_->publish(drive_msg);

        // Debug print to watch the magic happen
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 250,
            "L: %.2fm | R: %.2fm | Err: %.2f | Steer: %.2f | Spd: %.2f",
            left_diagonal, right_diagonal, error, steering, speed);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacerNode>());
    rclcpp::shutdown();
    return 0;
}