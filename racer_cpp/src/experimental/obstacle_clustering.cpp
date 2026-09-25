// obstacle_tracker_node.cpp
// Clusters LiDAR points → tracks opponent cars → injects phantom obstacles

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

// ── 1. A single LiDAR dot converted to a flat grid ───────────
struct Point2D {
    float x, y;
    int   scan_idx;  // which LiDAR index this came from (e.g., laser beam #450)
};

// ── 2. A detected obstacle (DBSCAN output) ───────────────────
struct Cluster {
    float cx, cy;        // centroid (center point) in robot frame (meters)
    float width;         // bounding box width (meters)
    float range;         // straight-line distance from robot center
    float angle;         // angle from robot forward
    int   point_count;   // how many LiDAR dots hit this object
};

// ── 3. A Kalman-tracked opponent car over time ───────────────
struct TrackedObstacle {
    int   id;            // Unique name tag for this car (e.g., Car #1)
    float x,  y;         // position estimate
    float vx, vy;        // velocity estimate (meters/second)
    float px, py;        // Kalman error (how "sure" we are about its position)
    int   age;           // frames seen (is this a real car or a 1-frame glitch?)
    int   lost;          // frames not matched (did it drive out of view?)
    bool  is_active;     // true if we are currently tracking it
};

class ObstacleTrackerNode : public rclcpp::Node {
public:
    ObstacleTrackerNode() : Node("obstacle_tracker_node"), next_id_(0) {

        // Raw scan subscriber for injection
        raw_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan", 10,
            std::bind(&ObstacleTrackerNode::raw_scan_callback, this, std::placeholders::_1));

        // Input: the fused scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan_filtered", 10,
            std::bind(&ObstacleTrackerNode::scan_callback, this, std::placeholders::_1));

        // Output: the tricked scan (for ReactiveGapNode)
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan_injected", 10);

        // cluster visualization for debugging
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/yellow_car/cluster_markers", 10);

        // OVERTAKE MODE ON
        mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/yellow_car/overtake_mode", 10);

        // Closest opponent outputs
        closest_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/yellow_car/closest_opponent", 10);

        closest_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/yellow_car/closest_opponent_base", 10);

        RCLCPP_INFO(this->get_logger(), "ObstacleTracker ready.");
    }

private:
  
    // ─────────────────────────────────────────────────────────────────────

    void raw_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_raw_scan_ = msg;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        // ── Calculate Dynamic Time Step (dt) ────────────────────────
        if (is_first_scan_) {
            last_time_ = msg->header.stamp;
            is_first_scan_ = false;
            return;
        }
        rclcpp::Time current_time = msg->header.stamp;
        dt_ = (current_time - last_time_).seconds();
        last_time_ = current_time;
        
        if (dt_ <= 0.0) return; // Prevent division by zero

        // ── Step 1: Convert scan to Cartesian points ──────────────────────
        std::vector<Point2D> points;
        int n = msg->ranges.size();

        // Front FOV: -90° to +90°
        const float window_min = -90.0f * M_PI / 180.0f;
        const float window_max =  90.0f * M_PI / 180.0f;

        for (int i = 0; i < n; ++i) {
            float r = msg->ranges[i];
            
            // Filter 1: Distance limits
            if (!std::isfinite(r) || r < 0.05f || r > 5.0f) continue;

            float angle = msg->angle_min + i * msg->angle_increment;

            // Filter 2: Angle limits (keep only -90° .. +90°)
            if (angle < window_min || angle > window_max) {
                continue;
            }

            float px = r * std::cos(angle);
            float py = r * std::sin(angle);

            // 🛑 GHOST FIX 1: THE EGO MASK 
            // Increased to 0.25m wide and 0.50m forward to completely blindfold the car's own turning wheels
            if (px > -0.30f && px < 0.50f && std::abs(py) < 0.25f) {
                continue;
            }

            points.push_back({
                px,   // x: forward
                py,   // y: left
                i     // original index
            });
        }


        // ── STEP 2: Adaptive Breakpoint Clustering ────────────────────────
        std::vector<Cluster> clusters;
        
        if (points.empty()) return; // Nothing to process

        std::vector<Point2D> current_cluster_pts;
        current_cluster_pts.push_back(points[0]);

        auto evaluate_cluster = [&]() {
            if ((int)current_cluster_pts.size() >= min_cluster_pts_ && 
                (int)current_cluster_pts.size() <= max_cluster_pts_) {
                
                float sum_x = 0, sum_y = 0;
                float min_x = 1e6, max_x = -1e6;
                float min_y = 1e6, max_y = -1e6;

                for (auto& p : current_cluster_pts) {
                    sum_x += p.x; sum_y += p.y;
                    min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
                    min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
                }

                float cx = sum_x / current_cluster_pts.size();
                float cy = sum_y / current_cluster_pts.size();
                float forward_span = std::abs(max_x - min_x);
                float lateral_span = std::abs(max_y - min_y);
                float width = std::sqrt(forward_span*forward_span + lateral_span*lateral_span);

                float range = std::sqrt(cx*cx + cy*cy);
                float angle = std::atan2(cy, cx);

                // Re-apply logic filters
                if (std::abs(angle) <= (100.0f * M_PI / 180.0f) && 
                    range >= 0.2f && range <= 3.0f &&
                    forward_span <= 0.8f && lateral_span <= 0.6f) {
                    
                    // Reject tiny dots (noise masquerading as cars)
                    if (!(forward_span < 0.05f && lateral_span < 0.05f)) {
                        if (width >= min_cluster_width_ && width <= max_cluster_width_) {
                            clusters.push_back({
                                cx, cy, width, range, angle, 
                                (int)current_cluster_pts.size()
                            });
                        }
                    }
                }
            }
            // Clear the list to start the next object
            current_cluster_pts.clear();
        };

        // Loop through points sequentially
        for (size_t i = 1; i < points.size(); ++i) {
            
            float dx = points[i].x - points[i-1].x;
            float dy = points[i].y - points[i-1].y;
            float dist_to_prev = std::sqrt(dx*dx + dy*dy);
            
            // 1) The Adaptive Gap
            // Objects further away have wider beam spacing. 
            // Base threshold of 10cm + 5% of the distance.
            float current_range = std::sqrt(points[i].x*points[i].x + points[i].y*points[i].y);
            float adaptive_gap = 0.10f + (current_range * 0.05f); 

            // 2) Index Jump Check
            // If we filtered out a bunch of points in Step 1 (e.g. ego mask or inf values), 
            // the scan indices will jump. A large jump means a physical gap.
            int index_jump = points[i].scan_idx - points[i-1].scan_idx;

            // --- BREAKPOINT DETECTED ---
            if (dist_to_prev > adaptive_gap || index_jump > 3) {
                evaluate_cluster();
            }
            // Add the current point to the growing cluster
            current_cluster_pts.push_back(points[i]);
        }
        
        // Evaluate the final cluster after the loop finishes
        if (!current_cluster_pts.empty()) {
            evaluate_cluster();
        }


        // ── Step 3: Kalman Filter — Predict ──────────────────────────────
        for (auto& t : trackers_) {
            if (!t.is_active) continue;
            t.x += t.vx * dt_;
            t.y += t.vy * dt_;
            t.px += Q_;
            t.py += Q_;
            t.lost++;
        }

        // ── Step 4: Data association — match clusters to trackers ─────────
        std::vector<bool> cluster_matched(clusters.size(), false);

        for (auto& t : trackers_) {
            if (!t.is_active) continue;

            float best_dist = max_track_dist_m_;
            int   best_ci   = -1;

            for (int ci = 0; ci < (int)clusters.size(); ++ci) {
                if (cluster_matched[ci]) continue;
                float dx = clusters[ci].cx - t.x;
                float dy = clusters[ci].cy - t.y;
                float d  = std::sqrt(dx*dx + dy*dy);
                if (d < best_dist) {
                    best_dist = d;
                    best_ci   = ci;
                }
            }

            if (best_ci >= 0) {
                // ── Kalman Update ──────────────────────────────────────
                float Kx = t.px / (t.px + R_);
                float Ky = t.py / (t.py + R_);
                float innov_x = clusters[best_ci].cx - t.x;
                float innov_y = clusters[best_ci].cy - t.y;
                t.vx += Kx * (innov_x / dt_ - t.vx) * 0.3f;  
                t.vy += Ky * (innov_y / dt_ - t.vy) * 0.3f;
                t.x = t.x + Kx * innov_x;
                t.y = t.y + Ky * innov_y;
                t.px = (1.0f - Kx) * t.px;
                t.py = (1.0f - Ky) * t.py;

                t.lost = 0;
                t.age++;
                cluster_matched[best_ci] = true;
            }

            if (t.lost > max_lost_frames_) t.is_active = false;
        }

        // ── Step 5: Spawn new trackers for unmatched clusters ─────────────
        for (int ci = 0; ci < (int)clusters.size(); ++ci) {
            if (cluster_matched[ci]) continue;
            
            TrackedObstacle t;
            t.id         = next_id_++;
            t.x          = clusters[ci].cx;
            t.y          = clusters[ci].cy;
            t.vx         = 0.0f;
            t.vy         = 0.0f;
            t.px         = 1.0f;
            t.py         = 1.0f;
            t.age        = 1;
            t.lost       = 0;
            t.is_active  = true;
            trackers_.push_back(t);
        }

        // ── Step 6: Inject phantom obstacle points into the scan ──────────
        if (!latest_raw_scan_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for raw /scan...");
            return;
        }
        
        auto output_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*latest_raw_scan_);

        for (auto& t : trackers_) {
            if (!t.is_active) continue;
            if (t.age < min_age_to_trust_) continue;  

            float pred_x = t.x + t.vx * dt_;
            float pred_y = t.y + t.vy * dt_;
            float pred_range = std::sqrt(pred_x*pred_x + pred_y*pred_y);
            float pred_angle = std::atan2(pred_y, pred_x);

            float bubble = phantom_radius_m_ + phantom_inflate_m_;
            float angular_half = std::atan2(bubble, pred_range);

            int steps = std::ceil(angular_half / std::abs(output_scan->angle_increment));
            int center_phantom_idx = static_cast<int>(
                std::round((pred_angle - output_scan->angle_min) /
                           std::abs(output_scan->angle_increment)));

            for (int di = -steps; di <= steps; ++di) {
                int idx = center_phantom_idx + di;
                if (idx < 0 || idx >= (int)output_scan->ranges.size()) continue;

                float phantom_range = pred_range - bubble * 0.5f;  
                phantom_range = std::max(0.1f, phantom_range);

                if (!std::isfinite(output_scan->ranges[idx]) ||
                    output_scan->ranges[idx] > phantom_range) {
                    output_scan->ranges[idx] = phantom_range;
                }
           }//
        }


        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < trackers_.size(); ++i) {
            if (!trackers_[i].is_active) continue; // Only draw active Kalman tracks

            visualization_msgs::msg::Marker marker;
            marker.header = msg->header; 
            marker.id = trackers_[i].id; // Use the tracker ID!
            marker.type = visualization_msgs::msg::Marker::CYLINDER; 
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = trackers_[i].x; // Use Kalman smoothed X
            marker.pose.position.y = trackers_[i].y; // Use Kalman smoothed Y
            marker.pose.position.z = 0.0; 
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.3; // Give it a fixed visible size
            marker.scale.y = 0.3; 
            marker.scale.z = 0.2;               

            marker.color.r = 0.0f;
            marker.color.g = 1.0f; // Make trackers green to distinguish them
            marker.color.b = 0.0f;
            marker.color.a = 0.8f; 

            // Make the marker last slightly longer than your drop-out threshold
            marker.lifetime = rclcpp::Duration::from_seconds(0.3); 
            marker_array.markers.push_back(marker);
        }
        
        marker_pub_->publish(marker_array);
        scan_pub_->publish(*output_scan);

        // Telemetry
        int active = std::count_if(trackers_.begin(), trackers_.end(),
                                   [](const TrackedObstacle& t){ return t.is_active; });
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Clusters: %zu | Trackers active: %d", clusters.size(), active);

        // ── 8. Trigger Overtake Mode (Trackers instead of Clusters) ──────────
        bool car_in_overtake_zone = false;

        for (const auto& t : trackers_) {
            if (!t.is_active) continue;
            if (t.age < min_age_to_trust_) continue; 

            if (t.x > 0.1f && t.x < 3.5f && std::abs(t.y) < 0.8f) {
                car_in_overtake_zone = true;
                break; 
            }
        }

        if (car_in_overtake_zone) {
            overtake_cooldown_ = 1.0f; 
        } else {
            overtake_cooldown_ -= dt_;
        }

        std_msgs::msg::Bool mode_msg;
        mode_msg.data = (overtake_cooldown_ > 0.0f); 
        mode_pub_->publish(mode_msg);

        // ── 9. Publish closest opponent for FSM / planners ───────────────
        publish_closest_opponent();

        // ── 10. Cleanup dead trackers ─────────────────────────────────────
        trackers_.erase(
            std::remove_if(trackers_.begin(), trackers_.end(),
                [](const TrackedObstacle& t) { return !t.is_active; }),
            trackers_.end());
    }

    void publish_closest_opponent()
    {
        // Parameters: what we consider "in front" and "in our lane"
        const float max_lateral = 0.8f; // meters left/right from centerline
        const int   min_age     = min_age_to_trust_; // already defined above

        float best_x = -1.0f; // sentinel: no opponent
        float best_y =  0.0f;

        for (const auto& t : trackers_) {
            if (!t.is_active) continue;
            if (t.age < min_age) continue; // require a few frames to trust

            float x = t.x; // forward (laser frame ≈ base_link)
            float y = t.y; // lateral

            // Only interested in obstacles in front
            if (x <= 0.0f) continue;

            // Only consider ones roughly in our lane
            if (std::abs(y) > max_lateral) continue;

            if (best_x < 0.0f || x < best_x) {
                best_x = x;
                best_y = y;
            }
        }

        // Bare point for FSM (your existing RacingFSMNode)
        geometry_msgs::msg::Point p;
        p.x = best_x; // < 0 means "no opponent"
        p.y = best_y;
        p.z = 0.0;
        closest_pub_->publish(p);

        // Stamped version for debugging / RViz / future planners
        geometry_msgs::msg::PointStamped ps;
        ps.header.stamp = this->now();
        ps.header.frame_id = "base_link"; // laser_link is fixed to base_link
        ps.point = p;
        closest_stamped_pub_->publish(ps);
    }

    int next_id_;
    double dt_ = 0.1;   
    bool is_first_scan_ = true;
    rclcpp::Time last_time_;
    float overtake_cooldown_ = 0.0f;

    // 🛑 GHOST FIX 3: TUNING PARAMETERS
    float cluster_gap_m_     = 0.15f;  // Increased to bridge small gaps in walls better
    int   min_cluster_pts_   = 4;     // Increased to reject random sensor noise
    int   max_cluster_pts_   = 60;     
    float max_cluster_width_ = 0.50f;  
    float min_cluster_width_ = 0.20f;  // Increased to 20cm! Tiny bumps won't trigger anymore.

    float max_track_dist_m_  = 0.70f;  
    int   max_lost_frames_   = 5;      
    int   min_age_to_trust_  = 3;      

    float phantom_radius_m_  = 0.30f;  
    float phantom_inflate_m_ = 0.20f;  

    float Q_ = 0.5f;   
    float R_ = 0.2f;   
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_scan_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr                       latest_raw_scan_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr        closest_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_stamped_pub_;

    std::vector<TrackedObstacle> trackers_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleTrackerNode>());
    rclcpp::shutdown();
    return 0;
}