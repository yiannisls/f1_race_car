#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <cmath>
#include <vector>
#include <limits>

class CameraRGBWallFilter : public rclcpp::Node {
public:
    CameraRGBWallFilter() : Node("camera_rgb_wall_filter") {

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/yellow_car/image_raw/camera_info",
            rclcpp::SensorDataQoS(),
            std::bind(&CameraRGBWallFilter::cam_info_callback, this, std::placeholders::_1));
            
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/yellow_car/image_raw/image_color", 10,
            std::bind(&CameraRGBWallFilter::image_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan", 10,
            std::bind(&CameraRGBWallFilter::scan_callback, this, std::placeholders::_1));
            
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan_filtered", 10);

        mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/yellow_car/wall_mask_debug", 10);
            
        RCLCPP_INFO(this->get_logger(), "📷 Camera RGB Wall Filter (Cone of Vision) Ready!");
    }

private:
    void cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (cam_info_received_) return; 

        fx_ = msg->k[0];   
        fy_ = msg->k[4];   
        cx_ = msg->k[2];   
        cy_ = msg->k[5];   
        img_width_  = (int)msg->width;
        img_height_ = (int)msg->height;

        cam_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info locked in! %dx%d", img_width_, img_height_);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!cam_info_received_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat small_bgr;
        int small_w = img_width_ / downsample_factor_;
        int small_h = img_height_ / downsample_factor_;
        cv::resize(cv_ptr->image, small_bgr, cv::Size(small_w, small_h), 0, 0, cv::INTER_NEAREST);

        cv::Mat hsv;
        cv::cvtColor(small_bgr, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask_green, mask_red1, mask_red2, wall_mask_small;
        cv::inRange(hsv, green_lo_, green_hi_, mask_green);
        cv::inRange(hsv, red_lo1_,  red_hi1_,  mask_red1);
        cv::inRange(hsv, red_lo2_,  red_hi2_,  mask_red2);
        wall_mask_small = mask_green | mask_red1 | mask_red2;

        if (dilate_mask_) {
            // Increased dilation size slightly to ensure edge-cases on curved walls are caught
            cv::dilate(wall_mask_small, wall_mask_small,
                       cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        }

        {
            std::lock_guard<std::mutex> lock(mask_mutex_);
            wall_mask_small_ = wall_mask_small.clone();
        }

        auto mask_full = wall_mask_small.clone();
        cv::Mat mask_display;
        cv::resize(mask_full, mask_display, cv::Size(img_width_, img_height_), 0, 0, cv::INTER_NEAREST);

        std_msgs::msg::Header h;
        h.stamp = msg->header.stamp;
        h.frame_id = msg->header.frame_id;
        auto mask_msg = cv_bridge::CvImage(h, "mono8", mask_display).toImageMsg();
        mask_pub_->publish(*mask_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!cam_info_received_) {
            scan_pub_->publish(*msg);
            return;
        }

        cv::Mat local_mask;
        {
            std::lock_guard<std::mutex> lock(mask_mutex_);
            if (wall_mask_small_.empty()) {
                scan_pub_->publish(*msg);
                return;
            }
            local_mask = wall_mask_small_.clone();
        }

        auto filtered = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        const int   n         = static_cast<int>(msg->ranges.size());
        const float angle_min = msg->angle_min;
        const float angle_inc = msg->angle_increment;

        const int small_w = img_width_  / downsample_factor_;
        const int small_h = img_height_ / downsample_factor_;

        int suppressed = 0;

        for (int i = 0; i < n; ++i) {
            float r = msg->ranges[i];
            if (!std::isfinite(r) || r < 0.05f) continue;

            float theta = angle_min + i * angle_inc;

            float x_base = lidar_x_base_ + r * std::cos(theta);  
            float y_base =              r * std::sin(theta);     
            float z_base = lidar_z_base_;

            float X_cam = -y_base;                              
            float Y_cam = -(z_base - cam_z_base_);              
            float Z_cam =  x_base - cam_x_base_;                

            // If it's behind the camera, the camera can't verify it. Blind the tracker!
            if (Z_cam <= 0.05f) {
                filtered->ranges[i] = std::numeric_limits<float>::infinity();
                continue;  
            }

            float u = fx_ * X_cam / Z_cam + cx_;
            float v = fy_ * Y_cam / Z_cam + cy_;

            int u_small = static_cast<int>(std::round(u / downsample_factor_));
            int v_small = static_cast<int>(std::round(v / downsample_factor_));

            // --- THE FIX ---
            // If the LiDAR point is outside the camera's field of view, the camera 
            // cannot guarantee it isn't a wall. We MUST hide it from the tracker!
            if (u_small < 0 || u_small >= small_w || v_small < 0 || v_small >= small_h) {
                filtered->ranges[i] = std::numeric_limits<float>::infinity();
                continue;  
            }

            // If mask says "wall" → remove this beam
            if (local_mask.at<uint8_t>(v_small, u_small) > 0) {
                filtered->ranges[i] = std::numeric_limits<float>::infinity();
                ++suppressed;
            }
        }

        scan_pub_->publish(*filtered);
    }

    // ── Variables ──────────────────────────────────────────────────────────
    bool cam_info_received_ = false;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    int img_width_ = 0, img_height_ = 0;

    cv::Mat wall_mask_small_;
    std::mutex mask_mutex_;

    const int downsample_factor_ = 4;  
    bool dilate_mask_ = true;

    // HSV thresholds (Tuned for Webots Simulator)
    cv::Scalar green_lo_{35,  50, 50};
    cv::Scalar green_hi_{90, 255, 255};
    cv::Scalar red_lo1_{  0,  50, 50};
    cv::Scalar red_hi1_{ 15, 255, 255};
    cv::Scalar red_lo2_{160,  50, 50};
    cv::Scalar red_hi2_{180, 255, 255};

    double lidar_x_base_ = 0.0;
    double lidar_z_base_ = 0.0;
    double cam_x_base_ = 0.0;
    double cam_z_base_ = 0.1;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr     scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         mask_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRGBWallFilter>());
    rclcpp::shutdown();
    return 0;
}