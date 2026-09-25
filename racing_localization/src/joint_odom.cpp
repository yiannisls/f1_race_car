// joint_odometry_node.cpp
// Wheel + steering odometry from Webots /joint_states for the F1/10 car.
//
// WHY THIS EXISTS
//   /yellow_car/current_speed is dead (always 0.0), but /joint_states publishes
//   real joint POSITIONS (velocity[] is empty in this sim). So we:
//     * differentiate the REAR wheel joint angle to get rolling speed
//     * read the measured FRONT steering angle directly (a real upgrade over the
//       old odometry_node.cpp, which had to fake steering from commands)
//     * run the kinematic bicycle model -> forward speed vx + yaw rate vyaw
//
// OUTPUT
//   /yellow_car/odom_joint   nav_msgs/Odometry   (twist-only, NO TF)
//     twist.linear.x  = measured forward speed   [m/s]
//     twist.angular.z = v / L * tan(steer)        [rad/s]
//     pose.*          = locally integrated dead-reckoning, for RViz/debug ONLY
//
// DESIGN
//   * Publishes NO TF. The EKF stays the single owner of odom->base_link, just
//     like your rf2o + odometry_node setup. This is purely a TWIST source you
//     can later wire into the EKF as odom2.
//   * Covariances: vx trusted (small), vyaw less trusted (steering geometry +
//     differentiation noise), everything else huge so the EKF ignores it.
//LaserScan
// PARAMS (declare so they can be overridden from launch/yaml/CLI)
//   wheel_radius     [m]   default 0.04   (confirmed from URDF cylinder radius)
//   wheelbase        [m]   default 0.257  (front axle x-offset in URDF)
//   use_rear_wheels  bool  default true   (rear = driven; flip if rear slips)
//   publish_pose     bool  default true   (integrate x,y,yaw for RViz debug)
//   speed_filter_alpha     default 0.5    (0..1, higher = snappier/noisier)
//
// CHECK
//   ros2 topic echo /yellow_car/odom_joint --field twist.twist
//   # drive forward -> linear.x positive ~ your speed
//   # turn          -> angular.z changes sign with steering

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

class JointOdometryNode : public rclcpp::Node {
public:
    JointOdometryNode() : Node("joint_odometry_node") {
        // --- parameters ---
        wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.04);
        wheelbase_    = this->declare_parameter<double>("wheelbase", 0.257);
        use_rear_     = this->declare_parameter<bool>("use_rear_wheels", true);
        publish_pose_ = this->declare_parameter<bool>("publish_pose", true);
        alpha_        = this->declare_parameter<double>("speed_filter_alpha", 0.5);
        odom_frame_   = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_   = this->declare_parameter<std::string>("base_frame", "base_link");
        var_vx_       = this->declare_parameter<double>("var_vx", 0.05);
        var_vyaw_     = this->declare_parameter<double>("var_vyaw", 0.30);

        steer_names_ = {"front_left_steering_joint", "front_right_steering_joint"};
        if (use_rear_) {
            wheel_names_ = {"rear_left_wheel_joint", "rear_right_wheel_joint"};
        } else {
            wheel_names_ = {"front_left_wheel_joint", "front_right_wheel_joint"};
        }

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 50,
            std::bind(&JointOdometryNode::cb, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/yellow_car/odom_joint", 10);

        std::string wn;
        for (auto &n : wheel_names_) wn += n + " ";
        RCLCPP_INFO(this->get_logger(),
            "joint_odometry up. wheels=[ %s] r=%.3f L=%.3f (twist-only, no TF)",
            wn.c_str(), wheel_radius_, wheelbase_);
    }

private:
    // Average a named set of joint positions. ok=false if any name is missing.
    bool avg_position(const sensor_msgs::msg::JointState::SharedPtr &msg,
                      const std::vector<std::string> &names,
                      double &out) {
        double sum = 0.0;
        int found = 0;
        for (const auto &n : names) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == n) {
                    if (i >= msg->position.size()) return false;
                    sum += msg->position[i];
                    ++found;
                    break;
                }
            }
        }
        if (found != (int)names.size()) return false;
        out = sum / names.size();
        return true;
    }

    void cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.empty()) return;  // this sim has no velocity[]

        // gather current wheel positions per name
        std::unordered_map<std::string, double> wheel_pos;
        for (const auto &n : wheel_names_) {
            bool ok = false;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == n) {
                    if (i >= msg->position.size()) return;
                    wheel_pos[n] = msg->position[i];
                    ok = true;
                    break;
                }
            }
            if (!ok) {
                RCLCPP_WARN_ONCE(this->get_logger(),
                    "joint '%s' not in /joint_states", n.c_str());
                return;
            }
        }

        double steer = 0.0;
        avg_position(msg, steer_names_, steer);   // 0 if absent; harmless

        rclcpp::Time stamp(msg->header.stamp);

        // first message: store and bail (need a delta to differentiate)
        if (!have_last_) {
            last_stamp_ = stamp;
            last_wheel_pos_ = wheel_pos;
            have_last_ = true;
            return;
        }

        double dt = (stamp - last_stamp_).seconds();
        if (dt <= 0.0 || dt > 0.5) {
            // stalled / out-of-order: resync, skip this frame
            last_stamp_ = stamp;
            last_wheel_pos_ = wheel_pos;
            return;
        }

        // forward speed = mean wheel angular velocity * radius
        double mean_dpos = 0.0;
        for (const auto &n : wheel_names_) {
            mean_dpos += (wheel_pos[n] - last_wheel_pos_[n]);
        }
        mean_dpos /= wheel_names_.size();         // [rad]
        double omega = mean_dpos / dt;            // [rad/s]
        double v_raw = omega * wheel_radius_;     // [m/s]

        // low-pass filter the speed
        v_filt_ = alpha_ * v_raw + (1.0 - alpha_) * v_filt_;
        double v = v_filt_;

        // bicycle-model yaw rate from MEASURED steering
        double yaw_rate = (v / wheelbase_) * std::tan(steer);

        // local dead-reckoning (debug pose only)
        if (publish_pose_) {
            yaw_ += yaw_rate * dt;
            yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
            x_ += v * std::cos(yaw_) * dt;
            y_ += v * std::sin(yaw_) * dt;
        }

        // --- build Odometry ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);

        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = yaw_rate;

        for (auto &c : odom.pose.covariance)  c = 0.0;
        for (auto &c : odom.twist.covariance) c = 0.0;

        const double big = 1e6;
        odom.pose.covariance[0]  = big;   // x
        odom.pose.covariance[7]  = big;   // y
        odom.pose.covariance[14] = big;   // z
        odom.pose.covariance[21] = big;   // roll
        odom.pose.covariance[28] = big;   // pitch
        odom.pose.covariance[35] = big;   // yaw

        odom.twist.covariance[0]  = var_vx_;   // vx   (trusted)
        odom.twist.covariance[7]  = big;       // vy   (Ackermann ~0)
        odom.twist.covariance[14] = big;       // vz
        odom.twist.covariance[21] = big;       // vroll
        odom.twist.covariance[28] = big;       // vpitch
        odom.twist.covariance[35] = var_vyaw_; // vyaw

        pub_->publish(odom);

        last_stamp_ = stamp;
        last_wheel_pos_ = wheel_pos;
    }

    // --- parameters ---
    double wheel_radius_, wheelbase_, alpha_, var_vx_, var_vyaw_;
    bool use_rear_, publish_pose_;
    std::string odom_frame_, base_frame_;
    std::vector<std::string> steer_names_, wheel_names_;

    // --- state ---
    bool have_last_ = false;
    rclcpp::Time last_stamp_;
    std::unordered_map<std::string, double> last_wheel_pos_;
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    double v_filt_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
