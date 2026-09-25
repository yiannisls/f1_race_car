#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

class TwistToAckermann : public rclcpp::Node
{
public:
  TwistToAckermann() : Node("twist_to_ackermann")
  {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TwistToAckermann::twistCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(
      "/cmd_ackermann", 10);
    RCLCPP_INFO(this->get_logger(), "twist_to_ackermann started");
  }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        ackermann_msgs::msg::AckermannDrive out;
        out.speed = msg->linear.x;
        out.steering_angle = msg->angular.z;
        pub_->publish(out);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToAckermann>());
  rclcpp::shutdown();
  return 0;
}