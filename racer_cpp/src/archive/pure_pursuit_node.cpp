// pure_pursuit_node.cpp
// Follow a precomputed raceline (raceline.csv) using pure pursuit.
//
// FRAME / SETUP (Option A):
//   * Raceline CSV is in the MAP frame (same frame your SLAM map / centerline
//     were built in).
//   * Car pose is read from the map->base_link TF, which slam_toolbox publishes
//     when run in LOCALIZATION mode against your saved track_map. So you MUST be
//     running localization for this node to work.
//   * Output: /yellow_car/cmd_ackermann (ackermann_msgs/AckermannDrive), matching
//     your disparity racer + twist_to_ackermann.
//
// HOW PURE PURSUIT WORKS (one paragraph):
//   Find the raceline point nearest the car. From there, walk forward along the
//   raceline by a "lookahead distance" L_d to pick a target point. Steer so the
//   car drives a circular arc from its current position to that target. The
//   steering angle for a bicycle model is:  delta = atan2(2 * L * y_t, L_d^2),
//   where L is the wheelbase and y_t is the target's lateral offset in the car
//   frame. Speed is taken from the raceline's vx at the nearest point (scaled).
//
// CSV FORMAT expected (header line starts with '#', comma separated):
//   s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps
//
// PARAMS (override with --ros-args -p name:=value)
//   raceline_csv     path to raceline.csv  (REQUIRED, set it)
//   lookahead        [m]   base lookahead distance              (default 0.6)
//   lookahead_k      [s]   speed-scaled lookahead: L_d = lookahead + k*v
//   wheelbase        [m]   default 0.257
//   max_steer        [rad] hard clamp, default 0.35 (from URDF)
//   speed_scale      [-]   multiply raceline vx (start LOW, e.g. 0.5, ramp up)
//   min_speed        [m/s] floor so it never fully stops mid-lap (default 0.5)
//   map_frame        default "map"
//   base_frame       default "base_link"
//   control_rate_hz  default 30.0
//
// RUN
//   ros2 run racer_cpp pure_pursuit_node --ros-args 
//     -p raceline_csv:=/home/turtle/maps/raceline.csv -p speed_scale:=0.5

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

struct RPoint { double x, y, psi, kappa, vx; };

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit() : Node("pure_pursuit_node") {
        csv_path_     = declare_parameter<std::string>("raceline_csv", "");
        lookahead_    = declare_parameter<double>("lookahead", 0.6);
        lookahead_k_  = declare_parameter<double>("lookahead_k", 0.15);
        wheelbase_    = declare_parameter<double>("wheelbase", 0.257);
        max_steer_    = declare_parameter<double>("max_steer", 0.35);
        speed_scale_  = declare_parameter<double>("speed_scale", 0.5);
        min_speed_    = declare_parameter<double>("min_speed", 0.5);
        map_frame_    = declare_parameter<std::string>("map_frame", "map");
        base_frame_   = declare_parameter<std::string>("base_frame", "base_link");
        double rate   = declare_parameter<double>("control_rate_hz", 30.0);

        if (csv_path_.empty()) {
            RCLCPP_FATAL(get_logger(),
                "raceline_csv param is empty. Set -p raceline_csv:=/path/raceline.csv");
            throw std::runtime_error("no raceline csv");
        }
        load_raceline(csv_path_);

        pub_ = create_publisher<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10);
        target_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "/yellow_car/pp_target", 10);   // lookahead point, for RViz debug

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PurePursuit::control_step, this));

        RCLCPP_INFO(get_logger(),
            "pure_pursuit up. %zu raceline points, L=%.3f, max_steer=%.2f, "
            "speed_scale=%.2f", path_.size(), wheelbase_, max_steer_, speed_scale_);
    }

private:
    void load_raceline(const std::string &path) {
        std::ifstream f(path);
        if (!f.is_open()) {
            RCLCPP_FATAL(get_logger(), "cannot open raceline csv: %s", path.c_str());
            throw std::runtime_error("csv open failed");
        }
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string tok;
            std::vector<double> v;
            while (std::getline(ss, tok, ',')) {
                try { v.push_back(std::stod(tok)); } catch (...) { v.clear(); break; }
            }
            // columns: s, x, y, psi, kappa, vx
            if (v.size() >= 6) {
                path_.push_back({v[1], v[2], v[3], v[4], v[5]});
            }
        }
        if (path_.size() < 3) {
            RCLCPP_FATAL(get_logger(), "raceline too short (%zu pts)", path_.size());
            throw std::runtime_error("bad csv");
        }
    }

    // nearest raceline index to (x,y)
    size_t nearest_index(double x, double y) {
        size_t best = 0;
        double bd = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_.size(); ++i) {
            double dx = path_[i].x - x, dy = path_[i].y - y;
            double d = dx*dx + dy*dy;
            if (d < bd) { bd = d; best = i; }
        }
        return best;
    }

    // walk forward from 'start' until cumulative distance >= L_d; return that index
    size_t lookahead_index(size_t start, double Ld) {
        double acc = 0.0;
        size_t n = path_.size();
        size_t i = start;
        for (size_t step = 0; step < n; ++step) {
            size_t j = (i + 1) % n;
            double dx = path_[j].x - path_[i].x, dy = path_[j].y - path_[i].y;
            acc += std::hypot(dx, dy);
            i = j;
            if (acc >= Ld) return i;
        }
        return start;   // degenerate
    }

    void control_step() {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                map_frame_, base_frame_, tf2::TimePointZero);
        } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "no %s->%s TF yet (is slam_toolbox localization running?): %s",
                map_frame_.c_str(), base_frame_.c_str(), e.what());
            return;
        }

        double cx = tf.transform.translation.x;
        double cy = tf.transform.translation.y;
        double cyaw = tf2::getYaw(tf.transform.rotation);

        // 1. nearest raceline point -> its speed target
        size_t ni = nearest_index(cx, cy);
        double v_ref = std::max(min_speed_, path_[ni].vx * speed_scale_);

        // 2. speed-scaled lookahead, then pick target point
        double Ld = lookahead_ + lookahead_k_ * v_ref;
        size_t ti = lookahead_index(ni, Ld);
        double tx = path_[ti].x, ty = path_[ti].y;

        // 3. transform target into the car (base_link) frame
        double dx = tx - cx, dy = ty - cy;
        double x_car =  std::cos(cyaw) * dx + std::sin(cyaw) * dy;   // forward
        double y_car = -std::sin(cyaw) * dx + std::cos(cyaw) * dy;   // left

        // 4. pure pursuit steering for a bicycle model
        double Ld_actual = std::hypot(x_car, y_car);
        double steer = 0.0;
        if (Ld_actual > 1e-3) {
            steer = std::atan2(2.0 * wheelbase_ * y_car, Ld_actual * Ld_actual);
        }
        bool clamped = std::abs(steer) > max_steer_;
        steer = std::max(-max_steer_, std::min(steer, max_steer_));

        // 5. publish command
        ackermann_msgs::msg::AckermannDrive cmd;
        cmd.speed = v_ref;
        cmd.steering_angle = steer;
        pub_->publish(cmd);

        // debug marker at the lookahead target
        publish_target_marker(tx, ty);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
            "pos(%.2f,%.2f) -> tgt(%.2f,%.2f) | steer %.2f%s | v %.2f (ref idx %zu)",
            cx, cy, tx, ty, steer, clamped ? " [CLAMPED]" : "", v_ref, ni);
    }

    void publish_target_marker(double x, double y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.header.stamp = now();
        m.ns = "pp_target";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
        m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.2; m.color.a = 1.0;
        target_pub_->publish(m);
    }

    // params
    std::string csv_path_, map_frame_, base_frame_;
    double lookahead_, lookahead_k_, wheelbase_, max_steer_, speed_scale_, min_speed_;

    std::vector<RPoint> path_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}