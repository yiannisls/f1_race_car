#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/bool.hpp"

class DriveMuxNode : public rclcpp::Node {
public:
    DriveMuxNode() : Node("drive_mux_node") {
        
        // ── 1. The Trigger Input ──────────────────────────────────────────
        mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/yellow_car/overtake_mode", 10,
            std::bind(&DriveMuxNode::mode_callback, this, std::placeholders::_1));

        // ── 2. The Driving Inputs ─────────────────────────────────────────
        disparity_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_disparity", 10,
            std::bind(&DriveMuxNode::disparity_callback, this, std::placeholders::_1));

        reactive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_reactive", 10,
            std::bind(&DriveMuxNode::reactive_callback, this, std::placeholders::_1));

        // ── 3. The Final Output to the Wheels ─────────────────────────────
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10);

        RCLCPP_INFO(this->get_logger(), "🚦 Drive Mux Initialized. Defaulting to Hot Lap Mode.");
    }

private:
    // ── Callbacks ─────────────────────────────────────────────────────────
    
    void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data != is_overtake_mode_) {
            is_overtake_mode_ = msg->data;
            if (is_overtake_mode_) {
                RCLCPP_INFO(this->get_logger(), "🚦 SWITCHING TO OVERTAKE MODE (Reactive Gap)");
            } else {
                RCLCPP_INFO(this->get_logger(), "🚦 SWITCHING TO HOT LAP MODE (Disparity Extender)");
            }
        }
    }

    void disparity_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        // 🏎️ Only pass Disparity commands if the track is CLEAR
        if (!is_overtake_mode_) {
            drive_pub_->publish(*msg);
        }
    }

    void reactive_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        // 🫧 Only pass Reactive commands if we are OVERTAKING
        if (is_overtake_mode_) {
            drive_pub_->publish(*msg);
        }
    }

    // ── Variables ─────────────────────────────────────────────────────────
    bool is_overtake_mode_ = false; 

    // ── ROS 2 Interfaces ──────────────────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr disparity_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr reactive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveMuxNode>());
    rclcpp::shutdown();
    return 0;
}