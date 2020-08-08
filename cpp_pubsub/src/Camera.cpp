#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point32.hpp"

using namespace std::chrono_literals;


class Camera : public rclcpp::Node
{
public:
  Camera()
  : Node("Camera")
 {
      publisher_ = this->create_publisher<geometry_msgs::msg::Point32>("Ball_Position", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&Camera::timer_callback, this));
    }
 private:
    void timer_callback()
    {
      geometry_msgs::msg::Point32 message;
      message.x =  1.11; // Camerainfo
      message.y =  2.22; // Camerainfo
      message.z =  0; 
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", std::to_string(message.x).c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", std::to_string(message.y).c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", std::to_string(message.z).c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr publisher_; 
};

int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Camera>());
    rclcpp::shutdown();
    return 0;
  }
