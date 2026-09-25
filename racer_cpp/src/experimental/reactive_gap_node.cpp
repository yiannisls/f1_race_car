#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

class ReactiveGapNode : public rclcpp::Node {
public:
    ReactiveGapNode() : Node("reactive_gap_node"), last_time_(this->now()) {
        
        // Using rclcpp::SensorDataQoS() is best practice for high-speed LiDAR
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan", rclcpp::SensorDataQoS(),
            std::bind(&ReactiveGapNode::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10);

        RCLCPP_INFO(this->get_logger(), "🚀 Reactive Gap + Anti-Scrape Initialized!");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        
        // ── 1. Fix: Use the Node's Clock, not the Simulator's Message Stamp ──
        rclcpp::Time current_time = this->now();
        if (is_first_scan_) {
            last_time_ = current_time;
            prev_error_ = 0.0;
            is_first_scan_ = false;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Fallback to a tiny dt if the loop runs miraculously fast, instead of returning!
        if (dt <= 0.001) dt = 0.05; 

        // ── 2. Define Forward FOV (+/- 80 degrees) ─────────────────────
        int center_idx = msg->ranges.size() / 2;
        int offset_80_deg = (80.0 * M_PI / 180.0) / std::abs(msg->angle_increment);

        int start_idx = std::max(0, center_idx - offset_80_deg);
        int end_idx = std::min((int)msg->ranges.size() - 1, center_idx + offset_80_deg);

        // ── 3. Apply Safety Bubble to Closest Threat ───────────────────
        float min_range = std::numeric_limits<float>::infinity();
        int min_idx = center_idx; 

        for (int i = start_idx; i <= end_idx; ++i) {
            if (msg->ranges[i] < min_range) {
                min_range = msg->ranges[i];
                min_idx = i;
            }
        }

        int bubble_indices = 0;
        if (min_range > 0.0f && min_range < 2.0f) {
            bubble_indices = std::ceil(safety_radius_ / (min_range * std::abs(msg->angle_increment)));
        }

        std::vector<float> modified_ranges = msg->ranges;
        int bubble_start = std::max(start_idx, min_idx - bubble_indices);
        int bubble_end   = std::min(end_idx, min_idx + bubble_indices);

        for (int i = bubble_start; i <= bubble_end; ++i) {
            modified_ranges[i] = 0.0f; // Mark as undrivable
        }

        // ── 4. Find the Maximum Contiguous Gap ─────────────────────────
        int max_gap_start = start_idx;
        int max_gap_length = 0;
        int current_gap_start = start_idx;
        int current_gap_length = 0;
        float safe_threshold = 0.1f; 

        for (int i = start_idx; i <= end_idx; ++i) {
            // Treat infinite ranges (open space) as a massive number so they count as safe
            float r = std::isinf(modified_ranges[i]) ? 10.0f : modified_ranges[i];
            
            if (r > safe_threshold) {
                if (current_gap_length == 0) current_gap_start = i; 
                current_gap_length++;
            } else {
                if (current_gap_length > max_gap_length) {
                    max_gap_length = current_gap_length;
                    max_gap_start = current_gap_start;
                }
                current_gap_length = 0; 
            }
        }
        if (current_gap_length > max_gap_length) {
            max_gap_length = current_gap_length;
            max_gap_start = current_gap_start;
        }

        // ── 5. Find the Best Aim Point & PD Control ────────────────────
        float max_depth = 0.0f;
        for (int i = max_gap_start; i < max_gap_start + max_gap_length; ++i) {
            float r = std::isinf(modified_ranges[i]) ? 10.0f : modified_ranges[i];
            if (r > max_depth) {
                max_depth = r;
            }
        }

        int deep_start = -1;
        int deep_end = -1;
        for (int i = max_gap_start; i < max_gap_start + max_gap_length; ++i) {
            float r = std::isinf(modified_ranges[i]) ? 10.0f : modified_ranges[i];
            if (r >= max_depth - 0.5f) {
                if (deep_start == -1) deep_start = i;
                deep_end = i;
            }
        }

        int best_idx = max_gap_start + (max_gap_length / 2);
        if (deep_start != -1) {
            best_idx = deep_start + ((deep_end - deep_start) / 2);
        }

        float theta_steer = msg->angle_min + (best_idx * msg->angle_increment);

        float error = theta_steer;
        float d_error = (error - prev_error_) / dt;
        float steering_command = (kp_ * error) + (kd_ * d_error);

        steering_command = std::max(-max_steering_rad_, std::min(steering_command, max_steering_rad_));
        prev_error_ = error;

        // ── 6. Dynamic Power-Curve Speed Profiling ─────────────────────
        float target_dist = msg->ranges[best_idx];
        if (!std::isfinite(target_dist)) target_dist = safe_brake_dist_; 

        float dist_ratio = (target_dist - panic_brake_dist_) / (safe_brake_dist_ - panic_brake_dist_);
        dist_ratio = std::max(0.0f, std::min(1.0f, dist_ratio)); 
        float dist_speed = min_speed_mps_ + (max_speed_mps_ - min_speed_mps_) * dist_ratio;

        float steer_ratio = std::abs(steering_command) / max_steering_rad_;
        float grip_speed = max_speed_mps_ - (max_speed_mps_ - min_speed_mps_) * std::pow(steer_ratio, 4.0f);

        float speed_command = std::min(dist_speed, grip_speed);

        // ── 7. Active Anti-Scrape Side Check ───────────────────────────
        bool scrape_right = false;
        bool scrape_left = false;
        
        int offset_70  = (70.0 * M_PI / 180.0) / std::abs(msg->angle_increment);
        int offset_110 = (110.0 * M_PI / 180.0) / std::abs(msg->angle_increment);

        int right_start = std::max(0, center_idx - offset_110);
        int right_end   = std::max(0, center_idx - offset_70);
        for (int i = right_start; i <= right_end; ++i) {
            if (msg->ranges[i] < side_clearance_ && steering_command < -0.1f) {
                scrape_right = true; break;
            }
        }

        int left_start = std::min((int)msg->ranges.size() - 1, center_idx + offset_70);
        int left_end   = std::min((int)msg->ranges.size() - 1, center_idx + offset_110);
        for (int i = left_start; i <= left_end; ++i) {
            if (msg->ranges[i] < side_clearance_ && steering_command > 0.1f) {
                scrape_left = true; break;
            }
        }

        if (scrape_right) {
            steering_command = max_steering_rad_; 
            speed_command = min_speed_mps_; 
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "ANTI-SCRAPE: Right wall!");
        } else if (scrape_left) {
            steering_command = -max_steering_rad_; 
            speed_command = min_speed_mps_; 
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "ANTI-SCRAPE: Left wall!");
        }

        speed_command = std::max(min_speed_mps_, std::min(speed_command, max_speed_mps_));

        // ── 8. Publish Command & Telemetry ─────────────────────────────
        ackermann_msgs::msg::AckermannDrive drive_msg;
        drive_msg.steering_angle = steering_command;
        drive_msg.speed = speed_command;
        drive_pub_->publish(drive_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, 
            "GAP ▶ steer: %5.2f rad | speed: %4.2f m/s | dt: %.3f", 
            steering_command, speed_command, dt);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub_;

    bool is_first_scan_ = true;
    rclcpp::Time last_time_;
    float prev_error_ = 0.0;

    // ── Tuning Parameters ──────────────────────────────────────────────
    float max_steering_rad_ = 0.35f;  // Matched to URDF
    float max_speed_mps_    = 5.0f;  
    float min_speed_mps_    = 1.5f;
    float safe_brake_dist_  = 3.0f;  
    float panic_brake_dist_ = 1.0f;  

    float kp_ = 1.1f;
    float kd_ = 0.4f; 

    float safety_radius_    = 0.40f; 
    float side_clearance_   = 0.28f; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveGapNode>());
    rclcpp::shutdown();
    return 0;
}