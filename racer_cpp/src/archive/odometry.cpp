#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class LocaliserNode : public rclcpp::Node {
public:
    LocaliserNode() : Node("localiser_node") {

        // Inputs
        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/yellow_car/current_speed", 10,
            std::bind(&LocaliserNode::speed_callback, this, std::placeholders::_1));

        // Listen to the final command going to the car (from FSM output)
        cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10,
            std::bind(&LocaliserNode::cmd_callback, this, std::placeholders::_1));

        // Output: fake odometry for SMPPI
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/yellow_car/odom", 10);

        // Integrate at 50 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LocaliserNode::update, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Localiser node started.");
    }

private:
    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        v_ = (float)msg->data;
    }

    void cmd_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        delta_ = msg->steering_angle; // track commanded steering
    }

    void update() {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 0.5) return; // skip bad dt

        // Kinematic bicycle integration
        // (ego-centric: x,y,yaw drift from 0 over time)
        yaw_ += (v_ / wheelbase_) * std::tan(delta_) * dt;
        yaw_  = std::atan2(std::sin(yaw_), std::cos(yaw_)); // wrap

        x_ += v_ * std::cos(yaw_) * dt;
        y_ += v_ * std::sin(yaw_) * dt;

        // Build and publish odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        // Yaw → quaternion
        odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);

        odom.twist.twist.linear.x = v_;

        odom_pub_->publish(odom);
    }

    // Vehicle
    const float wheelbase_ = 0.257f;

    // State
    float x_   = 0.0f;
    float y_   = 0.0f;
    float yaw_ = 0.0f;
    float v_   = 0.0f;
    float delta_ = 0.0f;

    rclcpp::Time last_time_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           speed_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             odom_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocaliserNode>());
    rclcpp::shutdown();
    return 0;
}