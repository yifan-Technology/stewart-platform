
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
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

class gazeboBillotController2 : public rclcpp::Node  // 2 means ros2
{
  public:
    gazeboBillotController2()
    : Node("gazeboBillotController2")
    {
     publisher_togazebo = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/stewart/set_trajectory_stewart", 10);
     subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/stewart/norm_length", 10, std::bind(&gazeboBillotController2::callback, this, _1));

}
  private:
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr f32ma_msg)
    {

      std::cout << "f32ma_msg:" << "\n";
      std::cout << f32ma_msg->data[0] << "\n";
      std::cout << f32ma_msg->data[1] << "\n";
      std::cout << f32ma_msg->data[2] << "\n";
      std::cout << f32ma_msg->data[3] << "\n";
      std::cout << f32ma_msg->data[4] << "\n";
      std::cout << f32ma_msg->data[5] << "\n";

      trajectory_msgs::msg::JointTrajectory traj1;
      traj1.joint_names.resize(1);
      traj1.points.resize(1);
      traj1.points[0].positions.resize(1);
      traj1.joint_names[0] ="piston1_prismatic_joint";
      //traj1.header.stamp = this->now();
      traj1.header.frame_id = "piston1_cylinder_link";
      traj1.points[0].positions[0] = f32ma_msg->data[0];
      traj1.points[0].time_from_start = rclcpp::Duration(1,0);

            trajectory_msgs::msg::JointTrajectory traj2;
      traj2.joint_names.resize(1);
      traj2.points.resize(1);
      traj2.points[0].positions.resize(1);
      traj2.joint_names[0] ="piston2_prismatic_joint";
      //traj2.header.stamp = this->now();
      traj2.header.frame_id = "piston2_cylinder_link";
      traj2.points[0].positions[0] = f32ma_msg->data[1];
      traj2.points[0].time_from_start = rclcpp::Duration(1,0);

            trajectory_msgs::msg::JointTrajectory traj3;
      traj3.joint_names.resize(1);
      traj3.points.resize(1);
      traj3.points[0].positions.resize(1);
      traj3.joint_names[0] ="piston3_prismatic_joint";
      //traj3.header.stamp = this->now();
      traj3.header.frame_id = "piston3_cylinder_link";
      traj3.points[0].positions[0] = f32ma_msg->data[2];
      traj3.points[0].time_from_start = rclcpp::Duration(1,0);

            trajectory_msgs::msg::JointTrajectory traj4;
      traj4.joint_names.resize(1);
      traj4.points.resize(1);
      traj4.points[0].positions.resize(1);
      traj4.joint_names[0] ="piston4_prismatic_joint";
      //traj4.header.stamp = this->now();
      traj4.header.frame_id = "piston4_cylinder_link";
      traj4.points[0].positions[0] = f32ma_msg->data[3];
      traj4.points[0].time_from_start = rclcpp::Duration(1,0);

            trajectory_msgs::msg::JointTrajectory traj5;
      traj5.joint_names.resize(1);
      traj5.points.resize(1);
      traj5.points[0].positions.resize(1);
      traj5.joint_names[0] ="piston5_prismatic_joint";
      //traj5.header.stamp = this->now();
      traj5.header.frame_id = "piston5_cylinder_link";
      traj5.points[0].positions[0] = f32ma_msg->data[4];
      traj5.points[0].time_from_start = rclcpp::Duration(1,0);

            trajectory_msgs::msg::JointTrajectory traj6;
      traj6.joint_names.resize(1);
      traj6.points.resize(1);
      traj6.points[0].positions.resize(1);
      traj6.joint_names[0] ="piston6_prismatic_joint";
      //traj6.header.stamp = this->now();
      traj6.header.frame_id = "piston6_cylinder_link";
      traj6.points[0].positions[0] = f32ma_msg->data[5];
      traj6.points[0].time_from_start = rclcpp::Duration(1,0);


      publisher_togazebo->publish(traj1);
      /*

      publisher_togazebo->publish(traj2);

      publisher_togazebo->publish(traj3);

      publisher_togazebo->publish(traj4);

      publisher_togazebo->publish(traj5);

      publisher_togazebo->publish(traj6);*/
    } 

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_togazebo;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  
  };

 int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gazeboBillotController2>());
    rclcpp::shutdown();
    return 0;
  }
