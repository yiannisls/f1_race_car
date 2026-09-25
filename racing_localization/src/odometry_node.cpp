// odometry_node.cpp
// Wheel/kinematic odometry source for robot_localization EKF.
//
// IMPORTANT DESIGN NOTES (read before tuning):
//  * This node is a TWIST source for the EKF. It deliberately does NOT try to be
//    an accurate global pose estimator on its own.
//  * It uses MEASURED forward speed (/yellow_car/current_speed) and COMMANDED
//    steering (/yellow_car/cmd_ackermann) because the sim exposes no measured
//    steering yet. Because steering is only commanded, its yaw-rate estimate is
//    unreliable -> we publish a LARGE covariance on yaw so the EKF trusts rf2o
//    (laser odometry) for heading instead.
//  * This node does NOT broadcast any TF. The EKF is the single owner of the
//    odom->base_link transform. Publishing TF here would create a conflict.
//
// Published: /yellow_car/odom_wheel (nav_msgs/Odometry)
//   - twist.linear.x  = measured forward speed         (LOW covariance, trusted)
//   - twist.angular.z = v/L * tan(delta_cmd)           (HIGH covariance, distrusted)
//   - pose is integrated locally for debugging/RViz only; EKF ignores it (config).

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <array>

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : Node("odometry_node") {
        // --- Parameters (declare so they can be overridden from launch/yaml) ---
        wheelbase_      = this->declare_parameter<double>("wheelbase", 0.257);
        publish_rate_   = this->declare_parameter<double>("publish_rate_hz", 50.0);
        odom_frame_     = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_     = this->declare_parameter<std::string>("base_frame", "base_link");

        // Covariances: SMALL = trusted, LARGE = distrusted by the EKF.
        var_vx_         = this->declare_parameter<double>("var_vx", 0.02);    // measured speed: trusted
        var_vyaw_       = this->declare_parameter<double>("var_vyaw", 4.0);   // commanded steer: distrusted
        // Pose covariance is set huge because EKF is configured to ignore pose here.
        var_pose_       = this->declare_parameter<double>("var_pose", 1e6);

        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/yellow_car/current_speed", 10,
            std::bind(&OdometryNode::speed_callback, this, std::placeholders::_1));

        cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/yellow_car/cmd_ackermann", 10,
            std::bind(&OdometryNode::cmd_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/yellow_car/odom_wheel", 10);

        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&OdometryNode::update, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(),
            "Wheel odometry source started (twist-only, no TF). L=%.3f m @ %.0f Hz",
            wheelbase_, publish_rate_);
    }

private:
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        v_ = msg->data;
    }

    void cmd_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        delta_ = msg->steering_angle;   // commanded steering (no measured topic yet)
    }

    void update() {
        const auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 0.5) return;   // skip startup / stalls

        // Kinematic bicycle: yaw rate from commanded steering.
        const double yaw_rate = (v_ / wheelbase_) * std::tan(delta_);

        // Local dead-reckoning ONLY for debug visualization. EKF ignores pose.
        yaw_ += yaw_rate * dt;
        yaw_  = std::atan2(std::sin(yaw_), std::cos(yaw_));
        x_   += v_ * std::cos(yaw_) * dt;
        y_   += v_ * std::sin(yaw_) * dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;

        // Pose (debug only) -----------------------------------------------------
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);

        // Twist (what the EKF actually consumes) --------------------------------
        odom.twist.twist.linear.x  = v_;
        odom.twist.twist.angular.z = yaw_rate;

        // Covariance matrices (row-major 6x6: x,y,z,roll,pitch,yaw) -------------
        // Pose: huge -> EKF told to ignore. Twist: vx trusted, vyaw distrusted.
        for (auto &c : odom.pose.covariance)  c = 0.0;
        for (auto &c : odom.twist.covariance) c = 0.0;

        odom.pose.covariance[0]   = var_pose_;   // x
        odom.pose.covariance[7]   = var_pose_;   // y
        odom.pose.covariance[14]  = var_pose_;   // z
        odom.pose.covariance[21]  = var_pose_;   // roll
        odom.pose.covariance[28]  = var_pose_;   // pitch
        odom.pose.covariance[35]  = var_pose_;   // yaw

        odom.twist.covariance[0]  = var_vx_;     // vx   (trusted)
        odom.twist.covariance[7]  = 1e6;         // vy   (Ackermann: ~0, distrust)
        odom.twist.covariance[14] = 1e6;         // vz
        odom.twist.covariance[21] = 1e6;         // vroll
        odom.twist.covariance[28] = 1e6;         // vpitch
        odom.twist.covariance[35] = var_vyaw_;   // vyaw (distrusted; rf2o handles heading)

        odom_pub_->publish(odom);
    }

    // Parameters
    double wheelbase_;
    double publish_rate_;
    std::string odom_frame_, base_frame_;
    double var_vx_, var_vyaw_, var_pose_;

    // State
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    double v_ = 0.0, delta_ = 0.0;
    rclcpp::Time last_time_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
