
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>

#include <eigen3/Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;
#define PI 3.14159265358979323846


class ImuAnalyse : public rclcpp::Node  // 2 means ros2
{
  public:
    ImuAnalyse()
    : Node("ImuAnalyse")
    {
     publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/stewart/norm_platform_twist", 10);
     subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "Stewart_actual_Imu", 10, std::bind(&ImuAnalyse::callback, this, _1));
    }

  private:
    void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {   
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float q0 = msg->orientation.w;
        float q1 = msg->orientation.x;
        float q2 = msg->orientation.y;
        float q3 = msg->orientation.z;
        double Pitch = (float)(asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3); // pitch
	      double Roll = (float)(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3); // roll
	      //double Yaw = (float)(atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3); //yaw
        double Yaw = 0.0;
        geometry_msgs::msg::Twist pubmsg;

        pubmsg.linear.x = x;
        pubmsg.linear.y = y;
        pubmsg.linear.z = z;
        pubmsg.angular.x = -Roll;
        pubmsg.angular.y = -Pitch;
        pubmsg.angular.z = Yaw;
        //pubmsg.angular.z = -Yaw;
        

        publisher_->publish(pubmsg);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

 int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuAnalyse>());
    rclcpp::shutdown();
    return 0;
  }
