#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

struct CellOffset { int dx, dy; };

class LocalGridNode : public rclcpp::Node {
public:
    LocalGridNode() : Node("local_occupancy_grid") {
        // Grid parameters
        x_min_ = -1.0f;  x_max_ = 9.0f;
        y_min_ = -3.0f;  y_max_ = 3.0f;
        res_   = 0.05f;

        grid_w_ = static_cast<int>((x_max_ - x_min_) / res_);
        grid_h_ = static_cast<int>((y_max_ - y_min_) / res_);
        
        // Use int8_t directly to match ROS 2 OccupancyGrid standard (0-100)
        grid_.assign(grid_w_ * grid_h_, 0); 

        // --- OPTIMIZATION: Precompute Wall Inflation Stencil ---
        float wall_radius = 0.2f;
        int cells = static_cast<int>(std::ceil(wall_radius / res_));
        for (int dx = -cells; dx <= cells; ++dx) {
            for (int dy = -cells; dy <= cells; ++dy) {
                if (std::hypot(dx * res_, dy * res_) <= wall_radius) {
                    wall_stencil_.push_back({dx, dy});
                }
            }
        }

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan", 10, 
            std::bind(&LocalGridNode::scan_callback, this, std::placeholders::_1));

        cluster_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/yellow_car/closest_opponent_base", 10,
            std::bind(&LocalGridNode::cluster_callback, this, std::placeholders::_1));

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/yellow_car/local_grid", 10);

        RCLCPP_INFO(this->get_logger(), "⚡ Optimized Local Occupancy Grid Active!");
    }

private:
    void cluster_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // FIX: Just store the state. Do not draw yet!
        opp_x_ = msg->point.x;
        opp_y_ = msg->point.y;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1) Fast clear of the 1D array
        std::fill(grid_.begin(), grid_.end(), 0);

        // 2) Draw the LiDAR Walls
        const float angle_min = msg->angle_min;
        const float angle_inc = msg->angle_increment;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (!std::isfinite(r) || r < 0.05f || r > msg->range_max) continue;

            float theta = angle_min + static_cast<float>(i) * angle_inc;
            
            // Transform to base_link
            float x_base = 0.21f + (r * std::cos(theta)); 
            float y_base = 0.0f  + (r * std::sin(theta));

            apply_stencil(x_base, y_base, wall_stencil_);
        }

        // 3) Draw the Opponent (if valid)
        if (opp_x_ > 0.0f) {
            // Quick inline inflation for opponent (e.g., 40cm box)
            int ix_center = static_cast<int>(std::floor((opp_x_ - x_min_) / res_));
            int iy_center = static_cast<int>(std::floor((opp_y_ - y_min_) / res_));
            int opp_cells = static_cast<int>(std::ceil(0.4f / res_));
            
            for (int dx = -opp_cells; dx <= opp_cells; ++dx) {
                for (int dy = -opp_cells; dy <= opp_cells; ++dy) {
                    int ix = ix_center + dx;
                    int iy = iy_center + dy;
                    if (ix >= 0 && ix < grid_w_ && iy >= 0 && iy < grid_h_) {
                        grid_[iy * grid_w_ + ix] = 100;
                    }
                }
            }
        }

        publish_grid(msg->header.stamp);
    }

    void apply_stencil(float x, float y, const std::vector<CellOffset>& stencil) {
        int ix_center = static_cast<int>(std::floor((x - x_min_) / res_));
        int iy_center = static_cast<int>(std::floor((y - y_min_) / res_));
        
        // Skip if center is out of bounds
        if (ix_center < 0 || ix_center >= grid_w_ || iy_center < 0 || iy_center >= grid_h_) return;

        for (const auto& offset : stencil) {
            int ix = ix_center + offset.dx;
            int iy = iy_center + offset.dy;
            if (ix >= 0 && ix < grid_w_ && iy >= 0 && iy < grid_h_) {
                grid_[iy * grid_w_ + ix] = 100; // 100 = definitely occupied in ROS 2
            }
        }
    }

    void publish_grid(const rclcpp::Time& stamp) {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";

        msg.info.resolution = res_;
        msg.info.width  = grid_w_;
        msg.info.height = grid_h_;
        msg.info.origin.position.x = x_min_;
        msg.info.origin.position.y = y_min_;
        
        msg.data = grid_; // Direct copy, no conversion loop needed!

        grid_pub_->publish(msg);
    }

    // Parameters
    float x_min_, x_max_, y_min_, y_max_, res_;
    int grid_w_, grid_h_;
    std::vector<int8_t> grid_;
    std::vector<CellOffset> wall_stencil_;
    
    // Opponent Memory
    float opp_x_ = -1.0f;
    float opp_y_ = 0.0f;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cluster_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalGridNode>());
    rclcpp::shutdown();
    return 0;
}