#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/Point.hpp"
#include <algorithm>

// Define our states
enum class State {
    HOT_LAP,
    TRAIL,
    OVERTAKE
};

class RacingFSMNode : public rclcpp::Node {
public:
    RacingFSMNode() : Node("racing_fsm_node"), current_state_(State::HOT_LAP) {
        
        // ── 1. The Opponent Data Input ────────────────────────────────────
        opponent_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/yellow_car/closest_opponent", 10,
            std::bind(&RacingFSMNode::opponent_callback, this, std::placeholders::_1));

        // ── 2. The Driving Inputs ─────────────────────────────────────────
        disparity_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_disparity", 10,
            std::bind(&RacingFSMNode::disparity_callback, this, std::placeholders::_1));

        reactive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_mppi", 10,
            std::bind(&RacingFSMNode::reactive_callback, this, std::placeholders::_1));

        // ── 3. The Final Output to the Wheels ─────────────────────────────
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10);

        RCLCPP_INFO(this->get_logger(), "🧠 Racing FSM Initialized. Default: HOT_LAP");
    }

private:
    void opponent_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        float opp_x = msg->x;
        // float opp_y = msg->y; // Will be used later for lateral gap checks

        State new_state = current_state_;

        // ── FSM Transition Logic ──
        if (opp_x < 0.0f) {
            new_state = State::HOT_LAP; // No valid opponent
        } 
        else if (opp_x > 1.5f && opp_x <= 4.5f) {
            new_state = State::TRAIL; // Opponent ahead, hold position
        } 
        else if (opp_x > 0.0f && opp_x <= 1.5f) {
            // TODO: Add lateral clearance check here using Lidar data or opponent Y
            new_state = State::OVERTAKE; // Dive for the pass!
        }

        // Handle State Change Logging
        if (new_state != current_state_) {
            current_state_ = new_state;
            if (current_state_ == State::HOT_LAP) RCLCPP_INFO(this->get_logger(), "🟢 STATE: HOT_LAP");
            if (current_state_ == State::TRAIL)   RCLCPP_INFO(this->get_logger(), "🟡 STATE: TRAIL");
            if (current_state_ == State::OVERTAKE) RCLCPP_INFO(this->get_logger(), "🔴 STATE: OVERTAKE");
        }
    }

    void disparity_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        ackermann_msgs::msg::AckermannDrive out_msg = *msg;

        if (current_state_ == State::HOT_LAP) {
            // Pass straight through
            drive_pub_->publish(out_msg);
        } 
        else if (current_state_ == State::TRAIL) {
            // Keep Disparity steering (racing line), but CRIPPLE the speed
            // You can make this proportional to the opponent's distance later
            out_msg.speed = std::min(out_msg.speed, max_trail_speed_);
            drive_pub_->publish(out_msg);
        }
    }

    void reactive_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        if (current_state_ == State::OVERTAKE) {
            // Reactive gap takes the wheel to swerve around the opponent
            drive_pub_->publish(*msg);
        }
    }

    State current_state_;
    float max_trail_speed_ = 2.0f; // Safe following speed

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr opponent_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr disparity_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr reactive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacingFSMNode>());
    rclcpp::shutdown();
    return 0;
}