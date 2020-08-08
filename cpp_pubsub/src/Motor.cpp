#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Motor : public rclcpp::Node
{
public:
  Motor()
  : Node("Motor")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "Output_Controller", 10, std::bind(&Motor::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->linear.x).c_str());
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->linear.y).c_str());
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->linear.z).c_str());
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->angular.x).c_str());
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->angular.y).c_str());
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->angular.z).c_str());

  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Motor>());
  rclcpp::shutdown();
  return 0;
}