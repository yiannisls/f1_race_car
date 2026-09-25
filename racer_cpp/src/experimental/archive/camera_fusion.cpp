#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>
#include <cmath>
#include <algorithm>

class ScanFusionNode : public rclcpp::Node {
public:
    using DepthMsg = sensor_msgs::msg::Image;
    using RGBMsg   = sensor_msgs::msg::Image;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<DepthMsg, RGBMsg>;

    ScanFusionNode() : Node("scan_fusion_node") {

        // LiDAR subscriber (always running, independent)
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan", 10,
            std::bind(&ScanFusionNode::lidar_callback, this, std::placeholders::_1));

        


        // Camera subscribers (depth + RGB, approximately synchronized)
        depth_sub_.subscribe(this, "/yellow_car/camera/depth/image_rect_raw");
        rgb_sub_.subscribe(this, "/yellow_car/camera/color/image_raw");

        // sync the 2 cause they might not arrive at the same time
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), depth_sub_, rgb_sub_);
        sync_->registerCallback(
            std::bind(&ScanFusionNode::camera_callback, this, 
                std::placeholders::_1, std::placeholders::_2));
        
            // Publishers
        fused_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan_fused", 10);

        // Published either CAR or WALL
        obstacle_type_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/yellow_car/obstacle_type", 10);


        // Camera intrinsics for D435i (horizontal FOV = 69° for RGB, 87° for depth)
        // These come from /camera/depth/camera_info — hardcoding typical D435i values
        // You should replace fx_ with the value from your camera_info topic
        depth_hfov_rad_ = 87.0 * M_PI / 180.0;  // D435i depth horizontal FOV
        depth_width_px_ = 640;
        depth_height_px_ = 480;
        // fx = (width/2) / tan(hfov/2)
        fx_ = (depth_width_px_ / 2.0) / std::tan(depth_hfov_rad_ / 2.0);

        RCLCPP_INFO(this->get_logger(), "ScanFusionNode ready. fx=%.1f", fx_);
    }

private:    

// ── Callbacks ────────────────────────────────────────────────────────────

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(lidar_mutex_);
            latest_lidar_ = msg;
        }
        publish_fused();
    }

    void camera_callback(
        const DepthMsg::ConstSharedPtr& depth_msg,
        const RGBMsg::ConstSharedPtr&   rgb_msg)
    {
        // ── Convert depth image ──────────────────────────────────────────────
        cv_bridge::CvImagePtr depth_cv;
        try {
            // D435i publishes depth as 16UC1 in millimetres
            depth_cv = cv_bridge::toCvCopy(depth_msg, "16UC1");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "depth cv_bridge: %s", e.what());
            return;
        }

        // ── Convert RGB image and make HSV mask ──────────────────────────────
        // HSV looks at an image and blacks out the unecessary pixels
        // (Like Greyscale)
        cv_bridge::CvImagePtr rgb_cv;
        try {
            rgb_cv = cv_bridge::toCvCopy(rgb_msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "rgb cv_bridge: %s", e.what());
            return;
        }

        // Resize RGB to match depth (1920x1080 → 640x480)
        cv::Mat rgb_resized;
        cv::resize(rgb_cv->image, rgb_resized,
                   cv::Size(depth_width_px_, depth_height_px_));

        cv::Mat hsv;
        cv::cvtColor(rgb_resized, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask_green, mask_red1, mask_red2, mask_red;
        cv::inRange(hsv, green_lo_, green_hi_, mask_green);
        cv::inRange(hsv, red_lo1_,  red_hi1_,  mask_red1);
        cv::inRange(hsv, red_lo2_,  red_hi2_,  mask_red2);
        cv::bitwise_or(mask_red1, mask_red2, mask_red);

        // ── Sample the depth row and classify each pixel ─────────────────────
        int row = std::min(depth_row_, depth_cv->image.rows - 1);

        std::vector<float> new_depth_ranges(depth_width_px_, 0.0f);
        std::vector<bool>  new_is_car(depth_width_px_, false);

        for (int col = 0; col < depth_width_px_; ++col) {
            uint16_t raw_mm = depth_cv->image.at<uint16_t>(row, col);
            float range_m = raw_mm * 0.001f;  // mm → metres

            if (range_m < min_depth_m_ || range_m > max_depth_m_) {
                new_depth_ranges[col] = 0.0f;  // invalid
                continue;
            }

            new_depth_ranges[col] = range_m;

            // Color classification: if pixel is neither red nor green → it's a car
            bool is_green = (mask_green.at<uint8_t>(row, col) > 0);
            bool is_red   = (mask_red.at<uint8_t>(row, col) > 0);
            new_is_car[col] = (!is_green && !is_red);
        }

        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            depth_ranges_    = new_depth_ranges;
            is_car_obstacle_ = new_is_car;
        }
    }
    // ── Core fusion logic ────────────────────────────────────────────────────

    void publish_fused() {
        sensor_msgs::msg::LaserScan::SharedPtr lidar;
        {
            std::lock_guard<std::mutex> lock(lidar_mutex_);
            if (!latest_lidar_) return;
            lidar = latest_lidar_;
        }

        // Start with a copy of the LiDAR scan
        auto fused = std::make_shared<sensor_msgs::msg::LaserScan>(*lidar);

        std::vector<float> depth_snapshot;
        std::vector<bool>  car_snapshot;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            depth_snapshot = depth_ranges_;
            car_snapshot   = is_car_obstacle_;
        }

        if (depth_snapshot.empty()) {
            // No camera data yet — just republish LiDAR as-is
            fused_pub_->publish(*fused);
            return;
        }

        // ── Map each depth pixel to a LiDAR angle index ─────────────────────
        // Depth pixel col → angle from camera center:
        //   angle = atan2(col - cx, fx)   where cx = width/2
        // The D435i depth sensor is mounted forward-facing (0° = straight ahead).
        // In the LiDAR frame, straight ahead = angle 0 (center_idx).

        int n = fused->ranges.size();
        int center_idx = n / 2;
        double cx = depth_width_px_ / 2.0;

        bool car_detected_forward = false;

        for (int col = 0; col < depth_width_px_; ++col) {
            float depth_range = depth_snapshot[col];
            if (depth_range <= 0.0f) continue;

            // Angle of this pixel relative to camera forward axis.
            // Flipped to match standard ROS LiDAR convention (positive angle = left).
            double angle = std::atan2((cx - col), fx_);

            // Map to LiDAR index
            int lidar_idx = center_idx + static_cast<int>(
                std::round(angle / std::abs(lidar->angle_increment)));

            if (lidar_idx < 0 || lidar_idx >= n) continue;

            // Take the MINIMUM (closer detection wins — conservative/safe)
            float lidar_range = fused->ranges[lidar_idx];
            if (!std::isfinite(lidar_range) || lidar_range <= 0.0f) {
                fused->ranges[lidar_idx] = depth_range;
            } else {
                fused->ranges[lidar_idx] = std::min(lidar_range, depth_range);
            }

            // Track if a non-wall obstacle is in the forward 40°
            if (!car_snapshot.empty() && car_snapshot[col]) {
                if (std::abs(angle) < (40.0 * M_PI / 180.0)) {
                    car_detected_forward = true;
                }
            }
        }


    fused_pub_->publish(*fused);

    // Publish obstacle type hint for higher-level nodes
    std_msgs::msg::String type_msg;
    type_msg.data = car_detected_forward ? "CAR" : "WALL";
    obstacle_type_pub_->publish(type_msg);
    }
    
private:
    // ── Data stores ──────────────────────────────────────────────────────────
    sensor_msgs::msg::LaserScan::SharedPtr latest_lidar_;
    std::vector<float> depth_ranges_;      // ranges derived from depth image
    std::vector<bool>  is_car_obstacle_;   // true = not a colored wall → treat as car
    std::mutex lidar_mutex_;
    std::mutex camera_mutex_;

    // ── Tuning ───────────────────────────────────────────────────────────────
    // We only sample one horizontal row of the 480-row depth image. 
    // Row 240 is the vertical centre — which, depending on how the camera is 
    // mounted, roughly corresponds to "car height" objects. 
    // You can tune this up or down based on where opponent cars appear in the 
    // image in simulation.

    // Depth image row to sample (0 = top). Center row = car-level horizon.
    int depth_row_ = 240;           // middle row of 480px image
    float max_depth_m_ = 4.0f;      // ignore depth returns beyond this
    float min_depth_m_ = 0.15f;     // D435i reliable min range

    // HSV color thresholds for wall classification
    // Green wall (right) — Hue 40–80
    cv::Scalar green_lo_{40, 60, 60}, green_hi_{80, 255, 255};
    // Red wall (left)   — Hue wraps: 0–10 and 170–180
    cv::Scalar red_lo1_{0, 60, 60},  red_hi1_{10, 255, 255};
    cv::Scalar red_lo2_{170, 60, 60}, red_hi2_{180, 255, 255};

    double depth_hfov_rad_;
    int depth_width_px_, depth_height_px_;
    double fx_;

    // ── ROS interfaces ───────────────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_type_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    message_filters::Subscriber<DepthMsg> depth_sub_;
    message_filters::Subscriber<RGBMsg>   rgb_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFusionNode>());
    rclcpp::shutdown();
    return 0;
}