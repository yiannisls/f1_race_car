#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

class RacerNode : public rclcpp::Node {
public:
    RacerNode() : Node("disparity_extender_node") {
        
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10);

        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/yellow_car/target_yaw", 10);

        opp_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/yellow_car/closest_opponent", 10,
            std::bind(&RacerNode::opponent_callback, this, std::placeholders::_1));

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan_injected", 10,
            std::bind(&RacerNode::scan_callback, this, std::placeholders::_1));

        latest_opponent_.x = -1.0;

        RCLCPP_INFO(this->get_logger(), "🏁 Hybrid FSM Racer (Mirage & Bifurcation Fixed) Activated!");
    }

    void opponent_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_opponent_ = *msg;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        
        // ==========================================
        // 1. MICRO-FSM: State Evaluation 
        // ==========================================
        float opp_x = latest_opponent_.x;
        float opp_y = latest_opponent_.y;

        if (opp_x < 0.0) {
            current_state_ = CLEAR_TRACK;
            max_speed_ = 6.0;  // Pushed to the absolute limit for the straights!
            side_clearance_ = 0.22; 
        } else {
            // DANGER: We are overtaking. Shrink the scrape box so we can squeeze by!
            side_clearance_ = 0.10; 

            if (opp_x <= 1.5 && std::abs(opp_y) > 0.4) {
                current_state_ = COMMITTED;
                max_speed_ = 4.5;   // Gun it out of the pass to secure the overtake!
            } else {
                current_state_ = OVERTAKING;
                
                if (opp_x < 1.5 && std::abs(opp_y) < 0.3) {
                    max_speed_ = 1.0; // Wait behind them
                } else {
                    max_speed_ = 3.5; // Gap is open, hit it
                }
            }
        }

        // ==========================================
        // PHASE 1: Clean Your Glasses (THE MIRAGE FIX)
        // ==========================================
        std::vector<float> ranges = msg->ranges;
        int center_idx = ranges.size() / 2; 
        int offset_110_deg = (110.0 * M_PI / 180.0) / std::abs(msg->angle_increment);
        
        int start_idx = std::max(0, center_idx - offset_110_deg);
        int end_idx = std::min((int)ranges.size() - 1, center_idx + offset_110_deg);

        // Define a safe "Horizon" distance. 
        // INCREASED to 7.0m so the car can look further down the straight and hold top speed!
        double max_vision_dist = 7.0; 

        for (int i = start_idx; i <= end_idx; ++i) {
            if (!std::isfinite(ranges[i]) || ranges[i] <= 0.01 || ranges[i] > max_vision_dist) {
                ranges[i] = max_vision_dist;
            }
        }

        // ==========================================
        // PHASE 2 & 3: Find Edges & Inflate Obstacles
        // ==========================================
        std::vector<int> disparity_indices;
        for (int i = start_idx; i < end_idx; ++i) { 
            double difference = std::abs(ranges[i] - ranges[i+1]);
            if (difference > disparity_threshold_) {
                disparity_indices.push_back(i);
            }
        }

        for (int idx : disparity_indices) {
            double distance_i = ranges[idx];
            double distance_i_plus_1 = ranges[idx+1];
            double closer_dist = std::min(distance_i, distance_i_plus_1);
            
            int extend_indices = std::ceil((car_width_ / 2.0) / (closer_dist * std::abs(msg->angle_increment)));

            if (distance_i < distance_i_plus_1) {
                for (int j = 1; j <= extend_indices; ++j) {
                    if (idx + j < (int)ranges.size()) {
                        ranges[idx + j] = std::min(ranges[idx + j], (float)closer_dist);
                    }
                }
            } else {
                for (int j = 0; j < extend_indices; ++j) {
                    if (idx - j >= 0) {
                        ranges[idx - j] = std::min(ranges[idx - j], (float)closer_dist);
                    }
                }
            }
        }

        // ==========================================
        // PHASE 4: Gap Selection (BIFURCATION FIX)
        // ==========================================
        double max_dist = 0.0;
        for (int i = start_idx; i <= end_idx; ++i) {
            if (ranges[i] > max_dist) { max_dist = ranges[i]; }
        }

        // 1. Find all valid "deep" rays
        std::vector<int> valid_indices;
        for (int i = start_idx; i <= end_idx; ++i) {
            // Because max_dist is capped at 4.0, we select any ray that is deep
            if (ranges[i] >= max_dist - 0.5) { 
                valid_indices.push_back(i);
            }
        }

        // 2. Group them into contiguous gaps
        std::vector<std::vector<int>> gaps;
        std::vector<int> current_gap;
        
        for (size_t i = 0; i < valid_indices.size(); ++i) {
            if (current_gap.empty()) {
                current_gap.push_back(valid_indices[i]);
            } else {
                if (valid_indices[i] - current_gap.back() <= 3) {
                    current_gap.push_back(valid_indices[i]);
                } else {
                    gaps.push_back(current_gap);
                    current_gap.clear();
                    current_gap.push_back(valid_indices[i]);
                }
            }
        }
        if (!current_gap.empty()) {
            gaps.push_back(current_gap);
        }

        // 3. Choose the best gap using Hysteresis (Memory)
        std::vector<int> best_gap;
        double best_score = -1e9;
        
        for (const auto& gap : gaps) {
            int mid_idx = gap[gap.size() / 2];
            double gap_angle = msg->angle_min + mid_idx * msg->angle_increment;
            
            // Score = Size of the gap
            double size_score = (double)gap.size();
            
            // Penalty = Compare raw angles so units match! 
            // 1 Radian difference penalizes the score by the equivalent of 40 rays.
            double stability_penalty = 40.0 * std::abs(gap_angle - previous_target_angle_); 
            double score = size_score - stability_penalty;
            
            if (score > best_score) {
                best_score = score;
                best_gap = gap;
            }
        }

        // 4. Aim for the center of the SPECIFIC chosen gap
        double target_angle = 0.0;
        if (!best_gap.empty()) {
            double sum_angles = 0.0;
            for (int idx : best_gap) {
                sum_angles += (msg->angle_min + idx * msg->angle_increment);
            }
            target_angle = sum_angles / best_gap.size();
        } else {
            target_angle = msg->angle_min + center_idx * msg->angle_increment;
        }

        // Save the raw angle for the hysteresis penalty on the next frame!
        previous_target_angle_ = target_angle; 

        std_msgs::msg::Float32 yaw_msg;
        yaw_msg.data = target_angle;
        yaw_pub_->publish(yaw_msg);

        double raw_steering = kp_angle_ * target_angle; 
        raw_steering = std::max(-max_steering_, std::min(raw_steering, max_steering_));

        // --- Light EMA Filter ---
        double steering = (ema_alpha_ * raw_steering) + ((1.0 - ema_alpha_) * previous_steering_);
        previous_steering_ = steering; 

        // Speed Limits
        double dist_ratio = (max_dist - panic_brake_dist_) / (safe_brake_dist_ - panic_brake_dist_);
        dist_ratio = std::max(0.0, std::min(1.0, dist_ratio)); 
        double dist_speed = min_speed_ + (max_speed_ - min_speed_) * dist_ratio;

        double steer_ratio = std::abs(raw_steering) / max_steering_;
        double grip_speed = max_speed_ - (max_speed_ - min_speed_) * std::pow(steer_ratio, 4.0);

        double speed = std::min(dist_speed, grip_speed);
        
        // ==========================================
        // PHASE 5: ANTI-SCRAPE SIDE CHECK
        // ==========================================
        bool scrape_override = false;

        for (int i = start_idx; i <= end_idx; ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double dist = ranges[i];

            if (dist < side_clearance_) {
                if (angle > (70.0 * M_PI / 180.0) && angle < (110.0 * M_PI / 180.0)) {
                    if (steering > 0.15) { scrape_override = true; }
                }
                else if (angle < (-70.0 * M_PI / 180.0) && angle > (-110.0 * M_PI / 180.0)) {
                    if (steering < -0.15) { scrape_override = true; }
                }
            }
        }

        if (scrape_override) {
            steering = 0.0;     
            speed = min_speed_; 
        }

        speed = std::max(min_speed_, std::min(speed, max_speed_));
        
        ackermann_msgs::msg::AckermannDrive drive_msg;
        drive_msg.speed = speed;
        drive_msg.steering_angle = steering;
        publisher_->publish(drive_msg);

        // ==========================================
        // TELEMETRY
        // ==========================================
        std::string state_str = "CLEAR";
        if (current_state_ == OVERTAKING) state_str = "OVERTAKE";
        else if (current_state_ == COMMITTED) state_str = "COMMIT";

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 250,
            "[%s] Spd: %.2f | Steer: %.2f | Gap: %.2fm | Opp_Y: %.2fm | Scrape: %s",
            state_str.c_str(), speed, steering, max_dist, opp_y, scrape_override ? "YES" : "NO");
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_; 
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr opp_sub_;
    
    geometry_msgs::msg::Point latest_opponent_;

    enum RaceState { CLEAR_TRACK, OVERTAKING, COMMITTED };
    RaceState current_state_ = CLEAR_TRACK;
    
    double previous_steering_ = 0.0;   
    double previous_target_angle_ = 0.0; // Added memory for raw angle hysteresis

    // --- YOUR AGGRESSIVE RACING TUNE ---
    double kp_angle_ = 1.4;            
    double max_steering_ = 0.7;       
    double ema_alpha_ = 0.5;          
    
    double max_speed_ = 1.0;           // (Was 4.5) Push the Tamiya motor on the straights!
    double min_speed_ = 0.2;           // (Was 0.6) Carry much more speed through the corners!
    double safe_brake_dist_ = 4.5;     // (Was 5.0) Brake much later before entering a turn.
    double panic_brake_dist_ = 1.6;    // (Was 1.8) Dive deep into the apex before hitting min_speed.
    
    double disparity_threshold_ = 0.5; 
    double car_width_ = 0.50;           
    double side_clearance_ = 0.3;      
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacerNode>());
    rclcpp::shutdown();
    return 0;
}