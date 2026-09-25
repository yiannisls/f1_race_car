#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include "geometry_msgs/msg/point.hpp"

class RacerNode : public rclcpp::Node {
public:
    RacerNode() : Node("disparity_extender_node") {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/cmd_ackermann", 10);

        // 1. Subscribe to the injected scan (contains the dynamic bubbles)
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&RacerNode::scan_callback, this, std::placeholders::_1));

        // 2. Subscribe to the tracker for FSM logic
        opp_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/closest_opponent", 10,
            std::bind(&RacerNode::opponent_callback, this, std::placeholders::_1));

        latest_opponent_.x = -1.0; // Default to clear track

        RCLCPP_INFO(this->get_logger(), "🏁 Disparity Extender Activated (Anti-Scrape & Smooth Speed)!");
    }

    void opponent_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_opponent_ = *msg;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Scan received!");
        // ==========================================
        // MICRO-FSM: State Evaluation & Parameter Tuning
        // ==========================================
        float opp_x = latest_opponent_.x;
        float opp_y = latest_opponent_.y;

        if (opp_x < 0.0) {
            // No opponent detected
            current_state_ = CLEAR_TRACK;
           // max_speed_ = max_speed;
            side_clearance_ = 0.28;
        } else {
            // Opponent is close AND we are shifting alongside them
            if (opp_x <= 1.5 && std::abs(opp_y) > 0.4) {
                current_state_ = COMMITTED;
               // max_speed_ = max_speed;       // Gun it to finish the pass
                side_clearance_ = 0.15; // Shrink anti-scrape to avoid panic braking
            } else {
                current_state_ = OVERTAKING;
                //max_speed_ = 3.5;       // Cap speed while tracking the bubble
                side_clearance_ = 0.28; // Keep normal safety margins
            }
        }
        // ==========================================
        // PHASE 1: Clean Your Glasses (Expanded to 220 degrees for side vision!)
        // ==========================================
        std::vector<float> ranges = msg->ranges;

        int center_idx = ranges.size() / 2;
        int offset_110_deg = (110.0 * M_PI / 180.0) / std::abs(msg->angle_increment);

        int start_idx = std::max(0, center_idx - offset_110_deg);
        int end_idx = std::min((int)ranges.size() - 1, center_idx + offset_110_deg);

        // FIX 1: reject phantom near-returns. RPLIDAR emits tiny/zero values on
        // dropped returns; anything below the lidar's real minimum range is invalid.
        float floor = std::max(0.12f, msg->range_min);
        for (int i = start_idx; i <= end_idx; ++i) {
            if (!std::isfinite(ranges[i]) || ranges[i] < floor) {
                ranges[i] = 10.0; // Pushed horizon to 10 meters
            }
        }

        // ==========================================
        // PHASE 2: Hunt for the Edges
        // ==========================================
        std::vector<int> disparity_indices;
        for (int i = start_idx; i < end_idx; ++i) {
            double difference = std::abs(ranges[i] - ranges[i+1]);
            if (difference > disparity_threshold_) {
                disparity_indices.push_back(i);
            }
        }

        // ==========================================
        // PHASE 3: Inflate the Obstacles (The Extender)
        // ==========================================
        for (int idx : disparity_indices) {
            double distance_i = ranges[idx];
            double distance_i_plus_1 = ranges[idx+1];
            double closer_dist = std::min(distance_i, distance_i_plus_1);

            int extend_indices = std::ceil((car_width_ / 2.0) / (closer_dist * std::abs(msg->angle_increment)));
            // FIX 2: cap the extender so a single close point can't flood the window.
            extend_indices = std::min(extend_indices, 80);

            if (distance_i < distance_i_plus_1) { // Extend Right
                for (int j = 1; j <= extend_indices; ++j) {
                    if (idx + j < (int)ranges.size()) {
                        ranges[idx + j] = std::min(ranges[idx + j], (float)closer_dist);
                    }
                }
            } else { // Extend Left
                for (int j = 0; j < extend_indices; ++j) {
                    if (idx - j >= 0) {
                        ranges[idx - j] = std::min(ranges[idx - j], (float)closer_dist);
                    }
                }
            }
        }

        // ==========================================
        // PHASE 4: Gun It (With Aggressive Speed Control)
        // ==========================================
        double max_dist = 0.0;

        for (int i = start_idx; i <= end_idx; ++i) {
            if (ranges[i] > max_dist) { max_dist = ranges[i]; }
        }

        std::vector<int> best_indices;
        for (int i = start_idx; i <= end_idx; ++i) {
            if (ranges[i] >= max_dist - 0.1) {
                best_indices.push_back(i);
            }
        }

        // --- SPATIAL SMOOTHING (Average Angle) ---
        double target_angle = 0.0;

        if (!best_indices.empty()) {
            double sum_angles = 0.0;
            for (int idx : best_indices) {
                // Calculate the exact angle for each valid LiDAR ray
                sum_angles += (msg->angle_min + idx * msg->angle_increment);
            }
            // The new target is the true mathematical center of the gap
            target_angle = sum_angles / best_indices.size();
        } else {
            // Fallback safety: aim straight ahead if no valid gaps exist
            target_angle = msg->angle_min + center_idx * msg->angle_increment;
        }

        // Calculate Steering
        double steering = kp_angle_ * target_angle;
        steering = std::max(-max_steering_, std::min(steering, max_steering_));

        // --- SMOOTH HIGH-PERFORMANCE SPEED CONTROLLER ---

        // 1. The Braking Limit (Distance)
        double dist_ratio = (max_dist - panic_brake_dist_) / (safe_brake_dist_ - panic_brake_dist_);
        dist_ratio = std::max(0.0, std::min(1.0, dist_ratio)); // Clamp between 0 and 1
        double dist_speed = min_speed_ + (max_speed_ - min_speed_) * dist_ratio;


        // 2. The Grip Limit (Steering) - TUNED FOR FAST CORNER EXITS
        // We use a higher exponent (^4.0). This creates a "deadzone" where minor
        // counter-steering on corner exits doesn't penalize your speed at all!

        //
        double steer_ratio = std::abs(steering) / max_steering_;
        double grip_speed = max_speed_ - (max_speed_ - min_speed_) * std::pow(steer_ratio, 4.0);

        // 3. The Final Decision
        double speed = std::min(dist_speed, grip_speed);

        // ==========================================
        // PHASE 5: ANTI-SCRAPE SIDE CHECK
        // ==========================================
        bool scrape_override = false;

        for (int i = start_idx; i <= end_idx; ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double dist = ranges[i];

            if (dist < side_clearance_) {
                // Check LEFT side (between +70 and +110 degrees)
                if (angle > (70.0 * M_PI / 180.0) && angle < (110.0 * M_PI / 180.0)) {
                    if (steering > 0.15) { scrape_override = true; }
                }
                // Check RIGHT side (between -110 and -70 degrees)
                else if (angle < (-70.0 * M_PI / 180.0) && angle > (-110.0 * M_PI / 180.0)) {
                    if (steering < -0.15) { scrape_override = true; }
                }
            }
        }

        if (scrape_override) {
            steering = 0.0;
            speed = min_speed_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "ANTI-SCRAPE TRIGGERED! Straightening out.");
        }

        // Final safety clamp
        speed = std::max(min_speed_, std::min(speed, max_speed_));
        RCLCPP_INFO_ONCE(this->get_logger(), "Scan received!");
        // Publish the command
        ackermann_msgs::msg::AckermannDrive drive_msg;
        drive_msg.speed = speed;
        drive_msg.steering_angle = steering;
        publisher_->publish(drive_msg);

        // Telemetry
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 250,
            "Gap: %.2fm | Steer: %.2f | Spd: %.2f | Edges: %zu | Scrape: %s",
            max_dist, steering, speed, disparity_indices.size(), scrape_override ? "YES" : "NO");
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr opp_sub_;
    geometry_msgs::msg::Point latest_opponent_;

    // FSM States
    enum RaceState { CLEAR_TRACK, OVERTAKING, COMMITTED };
    RaceState current_state_ = CLEAR_TRACK;

    // --- Tuning Parameters ---

    // Core Control
    double kp_angle_ = 1.3;            // How aggressively to steer
    double max_steering_ = 0.4;        // radians (~23 deg), physical limit of the wheels

    // Speed Limits
    double max_speed_ = 4.5;           // Straightaway speed
    double min_speed_ = 0.8;           // Cornering speed
    double safe_brake_dist_ = 5.0;     // Distance to start braking (meters)
    double panic_brake_dist_ = 2.1;    // Distance to hit min_speed (meters)

    // Disparity & Geometry
    double disparity_threshold_ = 0.22; // Minimum depth difference to be an obstacle edge (meters)
    double car_width_ = 0.4;           // Width of the car to inflate obstacles (meters)
    double side_clearance_ = 0.30;      // Min safe distance to the sides (half car width + margin)
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacerNode>());
    rclcpp::shutdown();
    return 0;
}