
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
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ForwardKinematicsStewart2 : public rclcpp::Node  // 2 means ros2
{
  public:
    ForwardKinematicsStewart2()
    : Node("ForwardKinematicsStewart2")
    {

        radius_p = 84;  
        radius_b = 150; 
        // wb = 30;  //  Grad
        // wp =30;
        height = 160;
        b <<  radius_b*0.70710678,    radius_b*0.70710678,   0, 1,
              radius_b*0.96592582,    radius_b*0.25881904,   0, 1,
              radius_b*0.25881904,    radius_b*-0.96592582,  0, 1,
              radius_b*-0.25881904,   radius_b*-0.96592582,  0, 1,
              radius_b*-0.96592582,   radius_b*0.25881904,   0, 1,
              radius_b*-0.70710678,   radius_b*0.70710678,   0, 1;

        p <<  radius_p*0.25881904,    radius_p*0.96592582,   0, 1,
              radius_p*0.96592582,    radius_p*-0.25881904,  0, 1,
              radius_p*0.70710678,    radius_p*-0.70710678,  0, 1,
              radius_p*-0.70710678,   radius_p*-0.70710678,  0, 1, 
              radius_p*-0.96592582,   radius_p*-0.25881904,  0, 1,
              radius_p*-0.25881904,   radius_p*0.96592582,   0, 1;
        // b <<  radius_b*cos(Deg2Rad(30+wb/2)),    radius_b*sin(Deg2Rad(30+wb/2)),  0, 1,
        //       radius_b*cos(Deg2Rad(30-wb/2)),    radius_b*sin(Deg2Rad(30-wb/2)),  0, 1;
        //       radius_b*cos(Deg2Rad(270+wb/2)),   radius_b*sin(Deg2Rad(270+wb/2)), 0, 1,
        //       radius_b*cos(Deg2Rad(270-wb/2)),   radius_b*sin(Deg2Rad(270-wb/2)), 0, 1,
        //       radius_b*cos(Deg2Rad(150+wb/2)),   radius_b*sin(Deg2Rad(150+wb/2)), 0, 1,
        //       radius_b*cos(Deg2Rad(150-wb/2)),   radius_b*sin(Deg2Rad(150-wb/2)), 0, 1,
              
        // p <<  radius_p*cos(Deg2Rad(90-wp/2)),    radius_p*sin(Deg2Rad(90-wp/2)),  0, 1,
        //       radius_p*cos(Deg2Rad(330+wp/2)),   radius_p*sin(Deg2Rad(330+wp/2)), 0, 1;
        //       radius_p*cos(Deg2Rad(330-wp/2)),   radius_p*sin(Deg2Rad(330-wp/2)), 0, 1,
        //       radius_p*cos(Deg2Rad(210+wp/2)),   radius_p*sin(Deg2Rad(210+wp/2)), 0, 1, 
        //       radius_p*cos(Deg2Rad(210-wp/2)),   radius_p*sin(Deg2Rad(210-wp/2)), 0, 1,
        //       radius_p*cos(Deg2Rad(90+wp/2)),    radius_p*sin(Deg2Rad(90+wp/2)),  0, 1,
              

     publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/stewart/actual_platform_twist", 10);
     
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/Stewart_actual_JointState", 10, std::bind(&ForwardKinematicsStewart2::callback, this, _1));

      subscription_forInit = this->create_subscription<geometry_msgs::msg::Twist>(
    "/stewart/norm_platform_twist", 10, std::bind(&ForwardKinematicsStewart2::callbackForInit, this, _1));
 

}
  private:

      void callbackForInit(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        q_init << msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z;
    }
    

    void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Eigen::Matrix<double, 6, 1> l;
        l << msg->position[0], msg->position[1], msg->position[2], msg->position[3],msg->position[4],msg->position[5];

        Eigen::Matrix<double, 6, 1> q = Iteraltion(l);
        geometry_msgs::msg::Twist pubmsg;

        pubmsg.linear.x = q(0,0);
        pubmsg.linear.y = q(1,0);
        pubmsg.linear.z = q(2,0);
        pubmsg.angular.x = q(3,0);
        pubmsg.angular.y = q(4,0);
        pubmsg.angular.z = q(5,0);

        publisher_->publish(pubmsg);
    }
    
    Eigen::Matrix<double, 4, 4> transformation_matrix(float x, float y, float z, float r, float p, float yaw)
    {
        Eigen::Matrix<double, 4, 4> T;
        T << cos(yaw)*cos(p), -sin(yaw)*cos(r) + cos(yaw)*sin(p)*sin(r),  sin(yaw)*sin(r)+cos(yaw)*sin(p)*cos(r), x,
             sin(yaw)*cos(p),  cos(yaw)*cos(r) + sin(yaw)*sin(p)*sin(r), -cos(yaw)*sin(r)+sin(yaw)*sin(p)*cos(r), y,
                     -sin(p),                             cos(p)*sin(r),                         cos(p)*cos(yaw), z,
                           0,                                         0,                                       0, 1;
        return T;
    }

    Eigen::Matrix<double, 6, 1> caculatelength(Eigen::Matrix<double, 6, 1> q)
    {
        float x = q(0,0);
        float y = q(1,0);
        float z = q(2,0);
        float Wx = q(3,0);
        float Wy = q(4,0);
        float Wz = q(5,0);

        Eigen::Matrix<double, 4, 4> T = transformation_matrix(x, y, z + height, Wx, Wy, Wz);
        double result[6];
        for (size_t i = 0; i < 6; i++)
        {
            Eigen::Matrix<double, 4, 1> length = T*p.row(i).transpose() - b.row(i).transpose();
            result[i] = sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2));
        }
        Eigen::Matrix<double, 6, 1> l;
        l << result[0], result[1],  result[2], result[3],result[4], result[5];
        return l;
    }

    float Deg2Rad(float angular)
    {
          float rad = angular * 3.1415926 / 180;
          return rad;
    }
     Eigen::Matrix<double, 6, 6> jacobi_Matrix(Eigen::Matrix<double, 6, 1> q)
    {
        float x = q(0,0);
        float y = q(1,0);
        float z = q(2,0);
        float Wx = q(3,0);
        float Wy = q(4,0);
        float Wz = q(5,0);

    double cg0 = fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) * fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) / (0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) * pow(pow(fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1);
		double cg1 = fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * pow(pow(fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1);
		double cg2 = fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)* fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * pow(pow(fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1);
		double cg3 = pow(pow(fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) * (0.81144e2 * sin(Wz) * sin(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * sin(Wz) * cos(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * sin(Wx)) * fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) / (0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) + 0.2e1 * fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * (-0.81144e2 * cos(Wz) * sin(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * cos(Wx) - 0.160e3 * cos(Wz) * cos(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * sin(Wx)) * fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) + 0.2e1 * fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (0.81144e2 * cos(Wy) * cos(Wx) - 0.160e3 * cos(Wy) * sin(Wx)) * fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg4 = pow(pow(fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) * (-0.21756e2 * cos(Wz) * sin(Wy) + 0.81144e2 * cos(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * cos(Wy) * cos(Wx)) * fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) / (0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) + 0.2e1 * fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * (-0.21756e2 * sin(Wz) * sin(Wy) + 0.81144e2 * sin(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * cos(Wy) * cos(Wx)) * fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) + 0.2e1 * fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (-0.21756e2 * cos(Wy) - 0.81144e2 * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wy) * cos(Wx)) * fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg5 = pow(pow(fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) * (-0.21756e2 * sin(Wz) * cos(Wy) - 0.81144e2 * cos(Wz) * cos(Wx) - 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx)) * fabs(0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) / (0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.106050e3) + 0.2e1 * fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * (0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx)) * fabs(0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3)) / 0.2e1;
		double cg6 = pow(pow(fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) / (0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3);
		double cg7 = -pow(pow(fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2);
		double cg8 = pow(pow(fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z);
		double cg9 = pow(pow(fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) * (-0.21756e2 * sin(Wz) * sin(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * sin(Wz) * cos(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * sin(Wx)) * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) / (0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) + 0.2e1 * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * (-0.21756e2 * cos(Wz) * sin(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * cos(Wz) * cos(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * sin(Wx)) * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) + 0.2e1 * fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (-0.21756e2 * cos(Wy) * cos(Wx) - 0.160e3 * cos(Wy) * sin(Wx)) * fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg10 = pow(pow(fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) * (-0.81144e2 * cos(Wz) * sin(Wy) - 0.21756e2 * cos(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * cos(Wy) * cos(Wx)) * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) / (0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) + 0.2e1 * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * (0.81144e2 * sin(Wz) * sin(Wy) + 0.21756e2 * sin(Wz) * cos(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * cos(Wy) * cos(Wx)) * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) + 0.2e1 * fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (-0.81144e2 * cos(Wy) + 0.21756e2 * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wy) * cos(Wx)) * fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg11 = pow(pow(fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(-0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) * (-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx)) * fabs(0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) / (0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.144900e3) + 0.2e1 * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * (-0.81144e2 * cos(Wz) * cos(Wy) - 0.21756e2 * sin(Wz) * cos(Wx) + 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx)) * fabs(-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (-0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2)) / 0.2e1;
		double cg12 = pow(pow(fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) / (0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2);
		double cg13 = -pow(pow(fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3);
		double cg14 = pow(pow(fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z);
		double cg15 = pow(pow(fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) * (-0.59388e2 * sin(Wz) * sin(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * sin(Wz) * cos(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * sin(Wx)) * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) / (0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) + 0.2e1 * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * (-0.59388e2 * cos(Wz) * sin(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * cos(Wz) * cos(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * sin(Wx)) * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) + 0.2e1 * fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (-0.59388e2 * cos(Wy) * cos(Wx) - 0.160e3 * cos(Wy) * sin(Wx)) * fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg16 = pow(pow(fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) * (-0.59388e2 * cos(Wz) * sin(Wy) - 0.59388e2 * cos(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * cos(Wy) * cos(Wx)) * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) / (0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) + 0.2e1 * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * (0.59388e2 * sin(Wz) * sin(Wy) + 0.59388e2 * sin(Wz) * cos(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * cos(Wy) * cos(Wx)) * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) + 0.2e1 * fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (-0.59388e2 * cos(Wy) + 0.59388e2 * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wy) * cos(Wx)) * fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg17 = pow(pow(fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(-0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) * (-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx)) * fabs(0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) / (0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x - 0.38850e2) + 0.2e1 * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * (-0.59388e2 * cos(Wz) * cos(Wy) - 0.59388e2 * sin(Wz) * cos(Wx) + 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx)) * fabs(-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (-0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3)) / 0.2e1;
		double cg18 = pow(pow(fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2), 0.2e1) + pow(fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) / (-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2);
		double cg19 = -pow(pow(fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2), 0.2e1) + pow(fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3);
		double cg20 = fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * pow(pow(fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2), 0.2e1) + pow(fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1);
		double cg21 = pow(pow(fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2), 0.2e1) + pow(fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) * (-0.59388e2 * sin(Wz) * sin(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * sin(Wz) * cos(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * sin(Wx)) * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) / (-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) + 0.2e1 * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * (-0.59388e2 * cos(Wz) * sin(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * cos(Wz) * cos(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * sin(Wx)) * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) + 0.2e1 * fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (-0.59388e2 * cos(Wy) * cos(Wx) - 0.160e3 * cos(Wy) * sin(Wx)) * fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg22 = pow(pow(fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2), 0.2e1) + pow(fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) * (0.59388e2 * cos(Wz) * sin(Wy) - 0.59388e2 * cos(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * cos(Wy) * cos(Wx)) * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) / (-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) + 0.2e1 * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * (-0.59388e2 * sin(Wz) * sin(Wy) + 0.59388e2 * sin(Wz) * cos(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * cos(Wy) * cos(Wx)) * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) + 0.2e1 * fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (0.59388e2 * cos(Wy) + 0.59388e2 * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wy) * cos(Wx)) * fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg23 = pow(pow(fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2), 0.2e1) + pow(fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3), 0.2e1) + pow(fabs(0.59388e2 * sin(Wy) - 0.59388e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) * (0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx)) * fabs(-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) / (-0.59388e2 * cos(Wz) * cos(Wy) + 0.59388e2 * sin(Wz) * cos(Wx) - 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.38850e2) + 0.2e1 * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) * (0.59388e2 * cos(Wz) * cos(Wy) - 0.59388e2 * sin(Wz) * cos(Wx) + 0.59388e2 * cos(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx)) * fabs(0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3) / (0.59388e2 * sin(Wz) * cos(Wy) + 0.59388e2 * cos(Wz) * cos(Wx) + 0.59388e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y - 0.144900e3)) / 0.2e1;
		double cg24 = pow(pow(fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3), 0.2e1) + pow(fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) / (-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3);
		double cg25 = -pow(pow(fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3), 0.2e1) + pow(fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2);
		double cg26 = pow(pow(fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3), 0.2e1) + pow(fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z);
		double cg27 = pow(pow(fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3), 0.2e1) + pow(fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) * (-0.21756e2 * sin(Wz) * sin(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * sin(Wz) * cos(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * sin(Wx)) * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) / (-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) + 0.2e1 * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * (-0.21756e2 * cos(Wz) * sin(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * cos(Wz) * cos(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * sin(Wx)) * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) + 0.2e1 * fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (-0.21756e2 * cos(Wy) * cos(Wx) - 0.160e3 * cos(Wy) * sin(Wx)) * fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg28 = pow(pow(fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3), 0.2e1) + pow(fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) * (0.81144e2 * cos(Wz) * sin(Wy) - 0.21756e2 * cos(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * cos(Wy) * cos(Wx)) * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) / (-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) + 0.2e1 * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * (-0.81144e2 * sin(Wz) * sin(Wy) + 0.21756e2 * sin(Wz) * cos(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * cos(Wy) * cos(Wx)) * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) + 0.2e1 * fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (0.81144e2 * cos(Wy) + 0.21756e2 * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wy) * cos(Wx)) * fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg29 = pow(pow(fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3), 0.2e1) + pow(fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2), 0.2e1) + pow(fabs(0.81144e2 * sin(Wy) - 0.21756e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) * (0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx)) * fabs(-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) / (-0.81144e2 * cos(Wz) * cos(Wy) + 0.21756e2 * sin(Wz) * cos(Wx) - 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.144900e3) + 0.2e1 * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) * (0.81144e2 * cos(Wz) * cos(Wy) - 0.21756e2 * sin(Wz) * cos(Wx) + 0.21756e2 * cos(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx)) * fabs(0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2) / (0.81144e2 * sin(Wz) * cos(Wy) + 0.21756e2 * cos(Wz) * cos(Wx) + 0.21756e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) - y + 0.38850e2)) / 0.2e1;
		double cg30 = pow(pow(fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) / (-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3);
		double cg31 = pow(pow(fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3);
		double cg32 = pow(pow(fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z);
		double cg33 = pow(pow(fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) * (0.81144e2 * sin(Wz) * sin(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * cos(Wx) + 0.160e3 * sin(Wz) * cos(Wx) - 0.160e3 * cos(Wz) * sin(Wy) * sin(Wx)) * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) / (-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) + 0.2e1 * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * (-0.81144e2 * cos(Wz) * sin(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * cos(Wx) - 0.160e3 * cos(Wz) * cos(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * sin(Wx)) * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) + 0.2e1 * fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (0.81144e2 * cos(Wy) * cos(Wx) - 0.160e3 * cos(Wy) * sin(Wx)) * fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg34 = pow(pow(fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) * (0.21756e2 * cos(Wz) * sin(Wy) + 0.81144e2 * cos(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * cos(Wy) * cos(Wx)) * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) / (-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) + 0.2e1 * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * (0.21756e2 * sin(Wz) * sin(Wy) + 0.81144e2 * sin(Wz) * cos(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * cos(Wy) * cos(Wx)) * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) + 0.2e1 * fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) * (0.21756e2 * cos(Wy) - 0.81144e2 * sin(Wy) * sin(Wx) - 0.160e3 * sin(Wy) * cos(Wx)) * fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z) / (0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z)) / 0.2e1;
		double cg35 = pow(pow(fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3), 0.2e1) + pow(fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3), 0.2e1) + pow(fabs(0.21756e2 * sin(Wy) + 0.81144e2 * cos(Wy) * sin(Wx) + 0.160e3 * cos(Wy) * cos(Wx) + z), 0.2e1), -0.1e1 / 0.2e1) * (0.2e1 * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) * (0.21756e2 * sin(Wz) * cos(Wy) - 0.81144e2 * cos(Wz) * cos(Wx) - 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wx) - 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx)) * fabs(-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) / (-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx) + x + 0.106050e3) + 0.2e1 * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) * (-0.21756e2 * cos(Wz) * cos(Wy) - 0.81144e2 * sin(Wz) * cos(Wx) + 0.81144e2 * cos(Wz) * sin(Wy) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wx) + 0.160e3 * cos(Wz) * sin(Wy) * cos(Wx)) * fabs(-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3) / (-0.21756e2 * sin(Wz) * cos(Wy) + 0.81144e2 * cos(Wz) * cos(Wx) + 0.81144e2 * sin(Wz) * sin(Wy) * sin(Wx) - 0.160e3 * cos(Wz) * sin(Wx) + 0.160e3 * sin(Wz) * sin(Wy) * cos(Wx) + y - 0.106050e3)) / 0.2e1;

        Eigen::Matrix<double, 6, 6> J;
        Eigen::Matrix<double, 6, 6> J_inverse;

        J << cg0,   cg1,  cg2,  cg3,  cg4,  cg5,
             cg6,   cg7,  cg8,  cg9,  cg10, cg11,
             cg12,  cg13, cg14, cg15, cg16, cg17,
             cg18,  cg19, cg20, cg21, cg22, cg23,
             cg24,  cg25, cg26, cg27, cg28, cg29,
             cg30,  cg31, cg32, cg33, cg34, cg35;
/*
        std::cout << "jacobi_Matrix:" << "\n";
        std::cout << J << "\n";*/

        double J_det = cg17 * cg18 * cg26 * cg33 * cg4 * cg7 - cg17 * cg2 * cg21 *
cg24 * cg34 * cg7 + cg12 * cg23 * cg27 * cg32 * cg4 * cg7 - cg12 *
cg23 * cg28 * cg3 * cg32 * cg7 - cg15 * cg18 * cg28 * cg32 * cg5 *
cg7 + cg15 * cg18 * cg29 * cg32 * cg4 * cg7 + cg15 * cg2 * cg22 *
cg29 * cg30 * cg7 - cg15 * cg2 * cg23 * cg28 * cg30 * cg7 + cg15 *
cg22 * cg24 * cg32 * cg5 * cg7 - cg15 * cg23 * cg24 * cg32 * cg4 *
cg7 + cg16 * cg18 * cg27 * cg32 * cg5 * cg7 - cg16 * cg18 * cg29 *
cg3 * cg32 * cg7 - cg16 * cg2 * cg21 * cg29 * cg30 * cg7 + cg16 *
cg2 * cg23 * cg27 * cg30 * cg7 - cg16 * cg21 * cg24 * cg32 * cg5 *
cg7 + cg16 * cg23 * cg24 * cg3 * cg32 * cg7 - cg17 * cg18 * cg27 *
cg32 * cg4 * cg7 + cg17 * cg18 * cg28 * cg3 * cg32 * cg7 + cg17 *
cg2 * cg21 * cg28 * cg30 * cg7 - cg17 * cg2 * cg22 * cg27 * cg30 *
cg7 + cg17 * cg21 * cg24 * cg32 * cg4 * cg7 - cg17 * cg22 * cg24 *
cg3 * cg32 * cg7 + cg14 * cg23 * cg24 * cg33 * cg4 * cg7 - cg14 *
cg23 * cg27 * cg30 * cg4 * cg7 + cg14 * cg23 * cg28 * cg3 * cg30 *
cg7 + cg12 * cg20 * cg27 * cg34 * cg5 * cg7 - cg12 * cg20 * cg27 *
cg35 * cg4 * cg7 + cg12 * cg20 * cg28 * cg3 * cg35 * cg7 - cg12 *
cg20 * cg28 * cg33 * cg5 * cg7 + cg16 * cg18 * cg26 * cg3 * cg35 *
cg7 - cg16 * cg18 * cg26 * cg33 * cg5 * cg7 + cg16 * cg2 * cg21 *
cg24 * cg35 * cg7 - cg16 * cg2 * cg23 * cg24 * cg33 * cg7 + cg16 *
cg21 * cg26 * cg30 * cg5 * cg7 - cg16 * cg23 * cg26 * cg3 * cg30 *
cg7 - cg17 * cg18 * cg26 * cg3 * cg34 * cg7 - cg15 * cg20 * cg29 *
cg30 * cg4 * cg7 - cg16 * cg18 * cg2 * cg27 * cg35 * cg7 + cg16 *
cg18 * cg2 * cg29 * cg33 * cg7 - cg16 * cg20 * cg24 * cg3 * cg35 *
cg7 + cg16 * cg20 * cg24 * cg33 * cg5 * cg7 - cg16 * cg20 * cg27 *
cg30 * cg5 * cg7 + cg16 * cg20 * cg29 * cg3 * cg30 * cg7 + cg17 *
cg18 * cg2 * cg27 * cg34 * cg7 - cg17 * cg18 * cg2 * cg28 * cg33 *
cg7 + cg17 * cg20 * cg24 * cg3 * cg34 * cg7 - cg17 * cg20 * cg24 *
cg33 * cg4 * cg7 + cg17 * cg20 * cg27 * cg30 * cg4 * cg7 - cg17 *
cg20 * cg28 * cg3 * cg30 * cg7 - cg12 * cg21 * cg26 * cg34 * cg5 *
cg7 + cg12 * cg21 * cg26 * cg35 * cg4 * cg7 - cg12 * cg22 * cg26 *
cg3 * cg35 * cg7 + cg12 * cg22 * cg26 * cg33 * cg5 * cg7 + cg12 *
cg23 * cg26 * cg3 * cg34 * cg7 - cg12 * cg23 * cg26 * cg33 * cg4 *
cg7 + cg15 * cg18 * cg26 * cg34 * cg5 * cg7 - cg15 * cg18 * cg26 *
cg35 * cg4 * cg7 - cg15 * cg2 * cg22 * cg24 * cg35 * cg7 + cg15 *
cg2 * cg23 * cg24 * cg34 * cg7 - cg15 * cg22 * cg26 * cg30 * cg5 *
cg7 + cg15 * cg23 * cg26 * cg30 * cg4 * cg7 - cg12 * cg2 * cg21 *
cg28 * cg35 * cg7 + cg12 * cg2 * cg21 * cg29 * cg34 * cg7 + cg17 *
cg2 * cg22 * cg24 * cg33 * cg7 - cg17 * cg21 * cg26 * cg30 * cg4 *
cg7 + cg17 * cg22 * cg26 * cg3 * cg30 * cg7 + cg12 * cg21 * cg28 *
cg32 * cg5 * cg7 - cg12 * cg21 * cg29 * cg32 * cg4 * cg7 - cg12 *
cg22 * cg27 * cg32 * cg5 * cg7 + cg12 * cg22 * cg29 * cg3 * cg32 *
cg7 + cg14 * cg18 * cg28 * cg33 * cg5 * cg7 + cg14 * cg18 * cg29 *
cg3 * cg34 * cg7 - cg14 * cg18 * cg29 * cg33 * cg4 * cg7 + cg14 *
cg21 * cg24 * cg34 * cg5 * cg7 - cg14 * cg21 * cg24 * cg35 * cg4 *
cg7 - cg14 * cg21 * cg28 * cg30 * cg5 * cg7 + cg14 * cg21 * cg29 *
cg30 * cg4 * cg7 + cg14 * cg22 * cg24 * cg3 * cg35 * cg7 - cg14 *
cg22 * cg24 * cg33 * cg5 * cg7 + cg14 * cg22 * cg27 * cg30 * cg5 *
cg7 - cg14 * cg22 * cg29 * cg3 * cg30 * cg7 - cg14 * cg23 * cg24 *
cg3 * cg34 * cg7 - cg1 * cg14 * cg21 * cg28 * cg35 * cg6 + cg1 *
cg14 * cg21 * cg29 * cg34 * cg6 + cg1 * cg14 * cg22 * cg27 * cg35 *
cg6 - cg1 * cg14 * cg22 * cg29 * cg33 * cg6 - cg1 * cg14 * cg23 *
cg27 * cg34 * cg6 + cg1 * cg14 * cg23 * cg28 * cg33 * cg6 + cg1 *
cg15 * cg20 * cg28 * cg35 * cg6 - cg1 * cg15 * cg20 * cg29 * cg34 *
cg6 - cg1 * cg16 * cg20 * cg27 * cg35 * cg6 + cg1 * cg16 * cg20 *
cg29 * cg33 * cg6 + cg1 * cg17 * cg20 * cg27 * cg34 * cg6 - cg1 *
cg17 * cg20 * cg28 * cg33 * cg6 - cg1 * cg15 * cg22 * cg26 * cg35 *
cg6 + cg1 * cg15 * cg23 * cg26 * cg34 * cg6 + cg1 * cg16 * cg21 *
cg26 * cg35 * cg6 - cg12 * cg20 * cg29 * cg3 * cg34 * cg7 + cg12 *
cg20 * cg29 * cg33 * cg4 * cg7 + cg15 * cg18 * cg2 * cg28 * cg35 *
cg7 - cg15 * cg18 * cg2 * cg29 * cg34 * cg7 - cg15 * cg20 * cg24 *
cg34 * cg5 * cg7 + cg15 * cg20 * cg24 * cg35 * cg4 * cg7 + cg15 *
cg20 * cg28 * cg30 * cg5 * cg7 + cg1 * cg17 * cg21 * cg28 * cg32 *
cg6 - cg1 * cg17 * cg22 * cg27 * cg32 * cg6 + cg11 * cg13 * cg2 *
cg22 * cg27 * cg30 - cg11 * cg13 * cg21 * cg24 * cg32 * cg4 + cg11
* cg13 * cg22 * cg24 * cg3 * cg32 + cg13 * cg18 * cg28 * cg32 * cg5
* cg9 - cg13 * cg18 * cg29 * cg32 * cg4 * cg9 - cg13 * cg2 * cg22 *
cg29 * cg30 * cg9 + cg13 * cg2 * cg23 * cg28 * cg30 * cg9 - cg13 *
cg21 * cg28 * cg32 * cg5 * cg6 + cg13 * cg21 * cg29 * cg32 * cg4 *
cg6 - cg13 * cg22 * cg24 * cg32 * cg5 * cg9 + cg13 * cg22 * cg27 *
cg32 * cg5 * cg6 - cg13 * cg22 * cg29 * cg3 * cg32 * cg6 + cg13 *
cg23 * cg24 * cg32 * cg4 * cg9 - cg13 * cg23 * cg27 * cg32 * cg4 *
cg6 + cg13 * cg23 * cg28 * cg3 * cg32 * cg6 - cg13 * cg20 * cg28 *
cg3 * cg35 * cg6 - cg13 * cg20 * cg28 * cg30 * cg5 * cg9 + cg13 *
cg20 * cg28 * cg33 * cg5 * cg6 + cg13 * cg20 * cg29 * cg3 * cg34 *
cg6 + cg13 * cg20 * cg29 * cg30 * cg4 * cg9 - cg13 * cg20 * cg29 *
cg33 * cg4 * cg6 - cg10 * cg13 * cg18 * cg26 * cg3 * cg35 + cg10 *
cg13 * cg18 * cg26 * cg33 * cg5 - cg10 * cg13 * cg2 * cg21 * cg24 *
cg35 + cg10 * cg13 * cg2 * cg23 * cg24 * cg33 + cg12 * cg2 * cg22 *
cg27 * cg35 * cg7 - cg12 * cg2 * cg22 * cg29 * cg33 * cg7 - cg12 *
cg2 * cg23 * cg27 * cg34 * cg7 + cg12 * cg2 * cg23 * cg28 * cg33 *
cg7 - cg14 * cg18 * cg27 * cg34 * cg5 * cg7 + cg14 * cg18 * cg27 *
cg35 * cg4 * cg7 - cg14 * cg18 * cg28 * cg3 * cg35 * cg7 - cg11 *
cg13 * cg22 * cg26 * cg3 * cg30 - cg13 * cg18 * cg26 * cg34 * cg5 *
cg9 + cg13 * cg18 * cg26 * cg35 * cg4 * cg9 + cg13 * cg2 * cg22 *
cg24 * cg35 * cg9 - cg13 * cg2 * cg23 * cg24 * cg34 * cg9 + cg13 *
cg21 * cg26 * cg34 * cg5 * cg6 - cg13 * cg21 * cg26 * cg35 * cg4 *
cg6 + cg13 * cg22 * cg26 * cg3 * cg35 * cg6 + cg13 * cg22 * cg26 *
cg30 * cg5 * cg9 - cg13 * cg22 * cg26 * cg33 * cg5 * cg6 - cg13 *
cg23 * cg26 * cg3 * cg34 * cg6 - cg13 * cg23 * cg26 * cg30 * cg4 *
cg9 + cg13 * cg23 * cg26 * cg33 * cg4 * cg6 - cg10 * cg13 * cg18 *
cg27 * cg32 * cg5 + cg10 * cg13 * cg18 * cg29 * cg3 * cg32 + cg10
* cg13 * cg2 * cg21 * cg29 * cg30 - cg10 * cg13 * cg2 * cg23 * cg27
* cg30 + cg10 * cg13 * cg21 * cg24 * cg32 * cg5 - cg10 * cg13 *
cg23 * cg24 * cg3 * cg32 + cg11 * cg13 * cg18 * cg27 * cg32 * cg4 -
cg11 * cg13 * cg18 * cg28 * cg3 * cg32 - cg11 * cg13 * cg2 * cg21 *
cg28 * cg30 - cg13 * cg18 * cg28 * cg33 * cg5 * cg8 - cg13 * cg18 *
cg29 * cg3 * cg34 * cg8 + cg13 * cg18 * cg29 * cg33 * cg4 * cg8 +
cg13 * cg2 * cg21 * cg28 * cg35 * cg6 - cg13 * cg2 * cg21 * cg29 *
cg34 * cg6 - cg1 * cg16 * cg23 * cg26 * cg33 * cg6 - cg1 * cg17 *
cg21 * cg26 * cg34 * cg6 + cg1 * cg17 * cg22 * cg26 * cg33 * cg6 +
cg1 * cg15 * cg22 * cg29 * cg32 * cg6 - cg1 * cg15 * cg23 * cg28 *
cg32 * cg6 - cg1 * cg16 * cg21 * cg29 * cg32 * cg6 + cg1 * cg16 *
cg23 * cg27 * cg32 * cg6 - cg13 * cg21 * cg29 * cg30 * cg4 * cg8 -
cg13 * cg22 * cg24 * cg3 * cg35 * cg8 + cg13 * cg22 * cg24 * cg33 *
cg5 * cg8 - cg13 * cg22 * cg27 * cg30 * cg5 * cg8 + cg13 * cg22 *
cg29 * cg3 * cg30 * cg8 + cg13 * cg23 * cg24 * cg3 * cg34 * cg8 -
cg13 * cg23 * cg24 * cg33 * cg4 * cg8 + cg13 * cg23 * cg27 * cg30 *
cg4 * cg8 - cg13 * cg23 * cg28 * cg3 * cg30 * cg8 + cg10 * cg13 *
cg18 * cg2 * cg27 * cg35 - cg10 * cg13 * cg18 * cg2 * cg29 * cg33 +
cg10 * cg13 * cg20 * cg24 * cg3 * cg35 - cg10 * cg13 * cg20 * cg24
* cg33 * cg5 + cg10 * cg13 * cg20 * cg27 * cg30 * cg5 - cg10 * cg13
* cg20 * cg29 * cg3 * cg30 - cg11 * cg13 * cg18 * cg2 * cg27 * cg34
+ cg11 * cg13 * cg18 * cg2 * cg28 * cg33 - cg11 * cg13 * cg20 *
cg24 * cg3 * cg34 + cg11 * cg13 * cg20 * cg24 * cg33 * cg4 - cg11 *
cg13 * cg20 * cg27 * cg30 * cg4 + cg11 * cg13 * cg20 * cg28 * cg3 *
cg30 - cg13 * cg18 * cg2 * cg28 * cg35 * cg9 + cg13 * cg18 * cg2 *
cg29 * cg34 * cg9 + cg13 * cg20 * cg24 * cg34 * cg5 * cg9 - cg13 *
cg20 * cg24 * cg35 * cg4 * cg9 - cg13 * cg20 * cg27 * cg34 * cg5 *
cg6 + cg13 * cg20 * cg27 * cg35 * cg4 * cg6 - cg10 * cg13 * cg21 *
cg26 * cg30 * cg5 + cg10 * cg13 * cg23 * cg26 * cg3 * cg30 + cg11 *
cg13 * cg18 * cg26 * cg3 * cg34 - cg11 * cg13 * cg18 * cg26 * cg33
* cg4 + cg11 * cg13 * cg2 * cg21 * cg24 * cg34 - cg11 * cg13 * cg2
* cg22 * cg24 * cg33 + cg11 * cg13 * cg21 * cg26 * cg30 * cg4 - cg1
* cg11 * cg12 * cg20 * cg27 * cg34 + cg1 * cg11 * cg12 * cg20 *
cg28 * cg33 - cg1 * cg12 * cg20 * cg28 * cg35 * cg9 + cg1 * cg12 *
cg20 * cg29 * cg34 * cg9 - cg1 * cg10 * cg12 * cg21 * cg26 * cg35 +
cg1 * cg10 * cg12 * cg23 * cg26 * cg33 + cg1 * cg11 * cg12 * cg21 *
cg26 * cg34 - cg1 * cg11 * cg12 * cg22 * cg26 * cg33 + cg1 * cg12 *
cg22 * cg26 * cg35 * cg9 - cg1 * cg12 * cg23 * cg26 * cg34 * cg9 +
cg1 * cg10 * cg12 * cg21 * cg29 * cg32 - cg1 * cg10 * cg12 * cg23 *
cg27 * cg32 - cg1 * cg11 * cg12 * cg21 * cg28 * cg32 + cg1 * cg11 *
cg12 * cg22 * cg27 * cg32 - cg1 * cg12 * cg22 * cg29 * cg32 * cg9 +
cg1 * cg12 * cg23 * cg28 * cg32 * cg9 + cg1 * cg12 * cg21 * cg28 *
cg35 * cg8 - cg1 * cg12 * cg21 * cg29 * cg34 * cg8 - cg1 * cg12 *
cg22 * cg27 * cg35 * cg8 + cg1 * cg12 * cg22 * cg29 * cg33 * cg8 +
cg15 * cg19 * cg26 * cg35 * cg4 * cg6 - cg16 * cg19 * cg2 * cg24 *
cg35 * cg9 - cg16 * cg19 * cg26 * cg3 * cg35 * cg6 - cg16 * cg19 *
cg26 * cg30 * cg5 * cg9 + cg16 * cg19 * cg26 * cg33 * cg5 * cg6 +
cg17 * cg19 * cg2 * cg24 * cg34 * cg9 + cg17 * cg19 * cg26 * cg3 *
cg34 * cg6 - cg13 * cg2 * cg22 * cg27 * cg35 * cg6 + cg13 * cg2 *
cg22 * cg29 * cg33 * cg6 + cg13 * cg2 * cg23 * cg27 * cg34 * cg6 -
cg13 * cg2 * cg23 * cg28 * cg33 * cg6 - cg13 * cg21 * cg24 * cg34 *
cg5 * cg8 + cg13 * cg21 * cg24 * cg35 * cg4 * cg8 + cg13 * cg21 *
cg28 * cg30 * cg5 * cg8 + cg10 * cg17 * cg19 * cg24 * cg3 * cg32 -
cg11 * cg12 * cg19 * cg27 * cg32 * cg4 + cg11 * cg12 * cg19 * cg28
* cg3 * cg32 + cg11 * cg15 * cg19 * cg2 * cg28 * cg30 + cg11 * cg15
* cg19 * cg24 * cg32 * cg4 - cg11 * cg16 * cg19 * cg2 * cg27 * cg30
- cg11 * cg16 * cg19 * cg24 * cg3 * cg32 - cg12 * cg19 * cg28 *
cg32 * cg5 * cg9 + cg12 * cg19 * cg29 * cg32 * cg4 * cg9 + cg15 *
cg19 * cg28 * cg32 * cg5 * cg6 - cg15 * cg19 * cg29 * cg32 * cg4 *
cg6 + cg16 * cg19 * cg2 * cg29 * cg30 * cg9 + cg16 * cg19 * cg24 *
cg32 * cg5 * cg9 - cg16 * cg19 * cg27 * cg32 * cg5 * cg6 + cg16 *
cg19 * cg29 * cg3 * cg32 * cg6 - cg17 * cg19 * cg2 * cg28 * cg30 *
cg9 - cg17 * cg19 * cg24 * cg32 * cg4 * cg9 + cg17 * cg19 * cg27 *
cg32 * cg4 * cg6 - cg17 * cg19 * cg28 * cg3 * cg32 * cg6 - cg10 *
cg12 * cg19 * cg2 * cg27 * cg35 + cg10 * cg12 * cg19 * cg2 * cg29 *
cg33 - cg10 * cg14 * cg19 * cg24 * cg3 * cg35 + cg10 * cg14 * cg19
* cg24 * cg33 * cg5 - cg10 * cg14 * cg19 * cg27 * cg30 * cg5 + cg10
* cg14 * cg19 * cg29 * cg3 * cg30 + cg11 * cg12 * cg19 * cg2 * cg27
* cg34 - cg11 * cg12 * cg19 * cg2 * cg28 * cg33 + cg13 * cg18 *
cg27 * cg34 * cg5 * cg8 - cg13 * cg18 * cg27 * cg35 * cg4 * cg8 +
cg13 * cg18 * cg28 * cg3 * cg35 * cg8 + cg1 * cg12 * cg23 * cg27 *
cg34 * cg8 - cg1 * cg12 * cg23 * cg28 * cg33 * cg8 + cg1 * cg10 *
cg12 * cg20 * cg27 * cg35 - cg1 * cg10 * cg12 * cg20 * cg29 * cg33
+ cg14 * cg19 * cg24 * cg35 * cg4 * cg9 + cg14 * cg19 * cg27 * cg34
* cg5 * cg6 - cg14 * cg19 * cg27 * cg35 * cg4 * cg6 + cg14 * cg19 *
cg28 * cg3 * cg35 * cg6 + cg14 * cg19 * cg28 * cg30 * cg5 * cg9 -
cg14 * cg19 * cg28 * cg33 * cg5 * cg6 - cg14 * cg19 * cg29 * cg3 *
cg34 * cg6 - cg14 * cg19 * cg29 * cg30 * cg4 * cg9 + cg14 * cg19 *
cg29 * cg33 * cg4 * cg6 + cg10 * cg12 * cg19 * cg26 * cg3 * cg35 -
cg10 * cg12 * cg19 * cg26 * cg33 * cg5 + cg10 * cg15 * cg19 * cg2 *
cg24 * cg35 + cg10 * cg15 * cg19 * cg26 * cg30 * cg5 - cg10 * cg17
* cg19 * cg2 * cg24 * cg33 - cg10 * cg17 * cg19 * cg26 * cg3 * cg30
- cg11 * cg12 * cg19 * cg26 * cg3 * cg34 + cg11 * cg12 * cg19 *
cg26 * cg33 * cg4 - cg11 * cg15 * cg19 * cg2 * cg24 * cg34 - cg11 *
cg15 * cg19 * cg26 * cg30 * cg4 + cg11 * cg16 * cg19 * cg2 * cg24 *
cg33 + cg11 * cg16 * cg19 * cg26 * cg3 * cg30 + cg12 * cg19 * cg26
* cg34 * cg5 * cg9 - cg12 * cg19 * cg26 * cg35 * cg4 * cg9 - cg15 *
cg19 * cg26 * cg34 * cg5 * cg6 - cg12 * cg19 * cg27 * cg34 * cg5 *
cg8 + cg12 * cg19 * cg27 * cg35 * cg4 * cg8 - cg12 * cg19 * cg28 *
cg3 * cg35 * cg8 + cg17 * cg19 * cg26 * cg30 * cg4 * cg9 - cg17 *
cg19 * cg26 * cg33 * cg4 * cg6 + cg10 * cg12 * cg19 * cg27 * cg32 *
cg5 - cg10 * cg12 * cg19 * cg29 * cg3 * cg32 - cg10 * cg15 * cg19 *
cg2 * cg29 * cg30 - cg10 * cg15 * cg19 * cg24 * cg32 * cg5 + cg10 *
cg17 * cg19 * cg2 * cg27 * cg30 - cg15 * cg19 * cg28 * cg30 * cg5 *
cg8 + cg15 * cg19 * cg29 * cg30 * cg4 * cg8 + cg16 * cg19 * cg2 *
cg27 * cg35 * cg6 - cg16 * cg19 * cg2 * cg29 * cg33 * cg6 + cg16 *
cg19 * cg24 * cg3 * cg35 * cg8 - cg16 * cg19 * cg24 * cg33 * cg5 *
cg8 + cg16 * cg19 * cg27 * cg30 * cg5 * cg8 - cg16 * cg19 * cg29 *
cg3 * cg30 * cg8 - cg17 * cg19 * cg2 * cg27 * cg34 * cg6 + cg17 *
cg19 * cg2 * cg28 * cg33 * cg6 - cg17 * cg19 * cg24 * cg3 * cg34 *
cg8 + cg17 * cg19 * cg24 * cg33 * cg4 * cg8 - cg17 * cg19 * cg27 *
cg30 * cg4 * cg8 + cg17 * cg19 * cg28 * cg3 * cg30 * cg8 - cg1 *
cg15 * cg18 * cg28 * cg35 * cg8 + cg1 * cg15 * cg18 * cg29 * cg34 *
cg8 + cg1 * cg16 * cg18 * cg27 * cg35 * cg8 - cg1 * cg16 * cg18 *
cg29 * cg33 * cg8 - cg1 * cg17 * cg18 * cg27 * cg34 * cg8 + cg1 *
cg17 * cg18 * cg28 * cg33 * cg8 - cg1 * cg10 * cg14 * cg18 * cg27 *
cg35 + cg1 * cg10 * cg14 * cg18 * cg29 * cg33 + cg1 * cg11 * cg14 *
cg18 * cg27 * cg34 - cg1 * cg11 * cg14 * cg18 * cg28 * cg33 + cg1 *
cg14 * cg18 * cg28 * cg35 * cg9 - cg1 * cg14 * cg18 * cg29 * cg34 *
cg9 + cg1 * cg10 * cg15 * cg18 * cg26 * cg35 + cg11 * cg14 * cg19 *
cg24 * cg3 * cg34 - cg11 * cg14 * cg19 * cg24 * cg33 * cg4 + cg11 *
cg14 * cg19 * cg27 * cg30 * cg4 - cg11 * cg14 * cg19 * cg28 * cg3 *
cg30 + cg12 * cg19 * cg2 * cg28 * cg35 * cg9 - cg12 * cg19 * cg2 *
cg29 * cg34 * cg9 - cg14 * cg19 * cg24 * cg34 * cg5 * cg9 + cg1 *
cg11 * cg15 * cg18 * cg28 * cg32 - cg1 * cg11 * cg16 * cg18 * cg27
* cg32 + cg1 * cg16 * cg18 * cg29 * cg32 * cg9 - cg1 * cg17 * cg18
* cg28 * cg32 * cg9 + cg12 * cg22 * cg25 * cg32 * cg5 * cg9 - cg12
* cg23 * cg25 * cg32 * cg4 * cg9 - cg15 * cg22 * cg25 * cg32 * cg5
* cg6 + cg15 * cg23 * cg25 * cg32 * cg4 * cg6 - cg16 * cg18 * cg25
* cg32 * cg5 * cg9 - cg16 * cg2 * cg23 * cg25 * cg30 * cg9 + cg16 *
cg21 * cg25 * cg32 * cg5 * cg6 - cg16 * cg23 * cg25 * cg3 * cg32 *
cg6 + cg17 * cg18 * cg25 * cg32 * cg4 * cg9 + cg17 * cg2 * cg22 *
cg25 * cg30 * cg9 - cg17 * cg21 * cg25 * cg32 * cg4 * cg6 + cg17 *
cg22 * cg25 * cg3 * cg32 * cg6 + cg14 * cg23 * cg25 * cg3 * cg34 *
cg6 + cg14 * cg23 * cg25 * cg30 * cg4 * cg9 - cg14 * cg23 * cg25 *
cg33 * cg4 * cg6 - cg10 * cg12 * cg20 * cg25 * cg3 * cg35 + cg10 *
cg12 * cg20 * cg25 * cg33 * cg5 - cg10 * cg15 * cg18 * cg2 * cg25 *
cg35 - cg10 * cg15 * cg20 * cg25 * cg30 * cg5 + cg10 * cg17 * cg18
* cg2 * cg25 * cg33 + cg10 * cg17 * cg20 * cg25 * cg3 * cg30 + cg11
* cg12 * cg20 * cg25 * cg3 * cg34 - cg11 * cg12 * cg20 * cg25 *
cg33 * cg4 + cg12 * cg19 * cg28 * cg33 * cg5 * cg8 + cg12 * cg19 *
cg29 * cg3 * cg34 * cg8 - cg12 * cg19 * cg29 * cg33 * cg4 * cg8 -
cg15 * cg19 * cg2 * cg28 * cg35 * cg6 + cg15 * cg19 * cg2 * cg29 *
cg34 * cg6 + cg15 * cg19 * cg24 * cg34 * cg5 * cg8 - cg15 * cg19 *
cg24 * cg35 * cg4 * cg8 - cg15 * cg20 * cg25 * cg35 * cg4 * cg6 +
cg16 * cg18 * cg2 * cg25 * cg35 * cg9 + cg16 * cg20 * cg25 * cg3 *
cg35 * cg6 + cg16 * cg20 * cg25 * cg30 * cg5 * cg9 - cg16 * cg20 *
cg25 * cg33 * cg5 * cg6 - cg17 * cg18 * cg2 * cg25 * cg34 * cg9 -
cg17 * cg20 * cg25 * cg3 * cg34 * cg6 - cg17 * cg20 * cg25 * cg30 *
cg4 * cg9 + cg17 * cg20 * cg25 * cg33 * cg4 * cg6 - cg10 * cg12 *
cg21 * cg25 * cg32 * cg5 + cg10 * cg12 * cg23 * cg25 * cg3 * cg32 +
cg10 * cg15 * cg18 * cg25 * cg32 * cg5 + cg10 * cg15 * cg2 * cg23 *
cg25 * cg30 - cg10 * cg17 * cg18 * cg25 * cg3 * cg32 - cg10 * cg17
* cg2 * cg21 * cg25 * cg30 + cg11 * cg12 * cg21 * cg25 * cg32 * cg4
- cg11 * cg12 * cg22 * cg25 * cg3 * cg32 - cg11 * cg15 * cg18 *
cg25 * cg32 * cg4 - cg11 * cg15 * cg2 * cg22 * cg25 * cg30 + cg11 *
cg16 * cg18 * cg25 * cg3 * cg32 + cg11 * cg16 * cg2 * cg21 * cg25 *
cg30 - cg15 * cg18 * cg25 * cg34 * cg5 * cg8 + cg15 * cg18 * cg25 *
cg35 * cg4 * cg8 + cg15 * cg2 * cg22 * cg25 * cg35 * cg6 - cg15 *
cg2 * cg23 * cg25 * cg34 * cg6 + cg15 * cg22 * cg25 * cg30 * cg5 *
cg8 - cg15 * cg23 * cg25 * cg30 * cg4 * cg8 - cg1 * cg10 * cg17 *
cg18 * cg26 * cg33 - cg1 * cg11 * cg15 * cg18 * cg26 * cg34 + cg1 *
cg11 * cg16 * cg18 * cg26 * cg33 - cg1 * cg16 * cg18 * cg26 * cg35
* cg9 + cg1 * cg17 * cg18 * cg26 * cg34 * cg9 - cg1 * cg10 * cg15 *
cg18 * cg29 * cg32 + cg1 * cg10 * cg17 * cg18 * cg27 * cg32 - cg17
* cg18 * cg25 * cg33 * cg4 * cg8 + cg17 * cg2 * cg21 * cg25 * cg34
* cg6 - cg17 * cg2 * cg22 * cg25 * cg33 * cg6 + cg17 * cg21 * cg25
* cg30 * cg4 * cg8 - cg17 * cg22 * cg25 * cg3 * cg30 * cg8 + cg10 *
cg12 * cg2 * cg21 * cg25 * cg35 - cg10 * cg12 * cg2 * cg23 * cg25 *
cg33 + cg10 * cg14 * cg18 * cg25 * cg3 * cg35 - cg10 * cg14 * cg18
* cg25 * cg33 * cg5 + cg10 * cg14 * cg21 * cg25 * cg30 * cg5 - cg10
* cg14 * cg23 * cg25 * cg3 * cg30 - cg11 * cg12 * cg2 * cg21 * cg25
* cg34 + cg11 * cg12 * cg2 * cg22 * cg25 * cg33 - cg11 * cg14 *
cg18 * cg25 * cg3 * cg34 + cg11 * cg14 * cg18 * cg25 * cg33 * cg4 -
cg11 * cg14 * cg21 * cg25 * cg30 * cg4 + cg11 * cg14 * cg22 * cg25
* cg3 * cg30 - cg12 * cg2 * cg22 * cg25 * cg35 * cg9 + cg12 * cg2 *
cg23 * cg25 * cg34 * cg9 + cg14 * cg18 * cg25 * cg34 * cg5 * cg9 -
cg14 * cg18 * cg25 * cg35 * cg4 * cg9 - cg14 * cg21 * cg25 * cg34 *
cg5 * cg6 + cg14 * cg21 * cg25 * cg35 * cg4 * cg6 - cg14 * cg22 *
cg25 * cg3 * cg35 * cg6 - cg14 * cg22 * cg25 * cg30 * cg5 * cg9 +
cg14 * cg22 * cg25 * cg33 * cg5 * cg6 + cg12 * cg21 * cg25 * cg34 *
cg5 * cg8 + cg11 * cg15 * cg18 * cg2 * cg25 * cg34 + cg11 * cg15 *
cg20 * cg25 * cg30 * cg4 - cg11 * cg16 * cg18 * cg2 * cg25 * cg33 -
cg11 * cg16 * cg20 * cg25 * cg3 * cg30 - cg12 * cg20 * cg25 * cg34
* cg5 * cg9 + cg12 * cg20 * cg25 * cg35 * cg4 * cg9 + cg15 * cg20 *
cg25 * cg34 * cg5 * cg6 + cg1 * cg17 * cg21 * cg24 * cg34 * cg8 -
cg1 * cg17 * cg22 * cg24 * cg33 * cg8 + cg1 * cg10 * cg14 * cg21 *
cg24 * cg35 - cg1 * cg10 * cg14 * cg23 * cg24 * cg33 - cg1 * cg11 *
cg14 * cg21 * cg24 * cg34 + cg1 * cg11 * cg14 * cg22 * cg24 * cg33
- cg1 * cg14 * cg22 * cg24 * cg35 * cg9 + cg1 * cg14 * cg23 * cg24
* cg34 * cg9 - cg1 * cg10 * cg15 * cg20 * cg24 * cg35 + cg1 * cg10
* cg17 * cg20 * cg24 * cg33 + cg1 * cg11 * cg15 * cg20 * cg24 *
cg34 - cg1 * cg11 * cg16 * cg20 * cg24 * cg33 + cg1 * cg16 * cg20 *
cg24 * cg35 * cg9 - cg1 * cg17 * cg20 * cg24 * cg34 * cg9 + cg1 *
cg10 * cg15 * cg23 * cg24 * cg32 - cg1 * cg10 * cg17 * cg21 * cg24
* cg32 - cg1 * cg11 * cg15 * cg22 * cg24 * cg32 + cg1 * cg11 * cg16
* cg21 * cg24 * cg32 - cg1 * cg16 * cg23 * cg24 * cg32 * cg9 + cg1
* cg17 * cg22 * cg24 * cg32 * cg9 + cg1 * cg15 * cg22 * cg24 * cg35
* cg8 - cg1 * cg15 * cg23 * cg24 * cg34 * cg8 + cg16 * cg20 * cg27
* cg31 * cg5 * cg6 - cg16 * cg20 * cg29 * cg3 * cg31 * cg6 + cg17 *
cg18 * cg2 * cg28 * cg31 * cg9 + cg17 * cg20 * cg24 * cg31 * cg4 *
cg9 - cg17 * cg20 * cg27 * cg31 * cg4 * cg6 - cg16 * cg18 * cg25 *
cg3 * cg35 * cg8 + cg16 * cg18 * cg25 * cg33 * cg5 * cg8 - cg16 *
cg2 * cg21 * cg25 * cg35 * cg6 + cg16 * cg2 * cg23 * cg25 * cg33 *
cg6 - cg16 * cg21 * cg25 * cg30 * cg5 * cg8 + cg16 * cg23 * cg25 *
cg3 * cg30 * cg8 + cg17 * cg18 * cg25 * cg3 * cg34 * cg8 - cg11 *
cg12 * cg21 * cg26 * cg31 * cg4 + cg11 * cg12 * cg22 * cg26 * cg3 *
cg31 + cg11 * cg15 * cg18 * cg26 * cg31 * cg4 + cg11 * cg15 * cg2 *
cg22 * cg24 * cg31 - cg11 * cg16 * cg18 * cg26 * cg3 * cg31 - cg11
* cg16 * cg2 * cg21 * cg24 * cg31 - cg12 * cg22 * cg26 * cg31 * cg5
* cg9 + cg12 * cg23 * cg26 * cg31 * cg4 * cg9 + cg15 * cg22 * cg26
* cg31 * cg5 * cg6 - cg15 * cg23 * cg26 * cg31 * cg4 * cg6 + cg16 *
cg18 * cg26 * cg31 * cg5 * cg9 + cg16 * cg2 * cg23 * cg24 * cg31 *
cg9 - cg16 * cg21 * cg26 * cg31 * cg5 * cg6 + cg16 * cg23 * cg26 *
cg3 * cg31 * cg6 - cg17 * cg18 * cg26 * cg31 * cg4 * cg9 - cg17 *
cg2 * cg22 * cg24 * cg31 * cg9 + cg17 * cg21 * cg26 * cg31 * cg4 *
cg6 - cg17 * cg22 * cg26 * cg3 * cg31 * cg6 - cg10 * cg14 * cg18 *
cg29 * cg3 * cg31 - cg10 * cg14 * cg21 * cg24 * cg31 * cg5 + cg10 *
cg14 * cg23 * cg24 * cg3 * cg31 + cg11 * cg12 * cg2 * cg21 * cg28 *
cg31 - cg11 * cg12 * cg2 * cg22 * cg27 * cg31 - cg11 * cg14 * cg18
* cg27 * cg31 * cg4 + cg11 * cg14 * cg18 * cg28 * cg3 * cg31 + cg11
* cg14 * cg21 * cg24 * cg31 * cg4 - cg11 * cg14 * cg22 * cg24 * cg3
* cg31 - cg12 * cg21 * cg25 * cg35 * cg4 * cg8 + cg12 * cg22 * cg25
* cg3 * cg35 * cg8 - cg12 * cg22 * cg25 * cg33 * cg5 * cg8 - cg12 *
cg23 * cg25 * cg3 * cg34 * cg8 + cg12 * cg23 * cg25 * cg33 * cg4 *
cg8 - cg1 * cg16 * cg21 * cg24 * cg35 * cg8 + cg1 * cg16 * cg23 *
cg24 * cg33 * cg8 - cg14 * cg22 * cg27 * cg31 * cg5 * cg6 + cg14 *
cg22 * cg29 * cg3 * cg31 * cg6 - cg14 * cg23 * cg24 * cg31 * cg4 *
cg9 + cg14 * cg23 * cg27 * cg31 * cg4 * cg6 - cg14 * cg23 * cg28 *
cg3 * cg31 * cg6 - cg10 * cg12 * cg20 * cg27 * cg31 * cg5 + cg10 *
cg12 * cg20 * cg29 * cg3 * cg31 + cg10 * cg15 * cg18 * cg2 * cg29 *
cg31 + cg10 * cg15 * cg20 * cg24 * cg31 * cg5 - cg10 * cg17 * cg18
* cg2 * cg27 * cg31 - cg10 * cg17 * cg20 * cg24 * cg3 * cg31 + cg11
* cg12 * cg20 * cg27 * cg31 * cg4 - cg11 * cg12 * cg20 * cg28 * cg3
* cg31 - cg11 * cg15 * cg18 * cg2 * cg28 * cg31 - cg11 * cg15 *
cg20 * cg24 * cg31 * cg4 + cg11 * cg16 * cg18 * cg2 * cg27 * cg31 +
cg11 * cg16 * cg20 * cg24 * cg3 * cg31 + cg12 * cg20 * cg28 * cg31
* cg5 * cg9 - cg12 * cg20 * cg29 * cg31 * cg4 * cg9 - cg15 * cg20 *
cg28 * cg31 * cg5 * cg6 + cg15 * cg20 * cg29 * cg31 * cg4 * cg6 -
cg16 * cg18 * cg2 * cg29 * cg31 * cg9 - cg16 * cg20 * cg24 * cg31 *
cg5 * cg9 - cg12 * cg21 * cg28 * cg31 * cg5 * cg8 + cg12 * cg21 *
cg29 * cg31 * cg4 * cg8 + cg12 * cg22 * cg27 * cg31 * cg5 * cg8 -
cg12 * cg22 * cg29 * cg3 * cg31 * cg8 + cg17 * cg20 * cg28 * cg3 *
cg31 * cg6 + cg10 * cg12 * cg21 * cg26 * cg31 * cg5 - cg10 * cg12 *
cg23 * cg26 * cg3 * cg31 - cg10 * cg15 * cg18 * cg26 * cg31 * cg5 -
cg10 * cg15 * cg2 * cg23 * cg24 * cg31 + cg10 * cg17 * cg18 * cg26
* cg3 * cg31 + cg10 * cg17 * cg2 * cg21 * cg24 * cg31 + cg15 * cg23
* cg24 * cg31 * cg4 * cg8 - cg16 * cg18 * cg27 * cg31 * cg5 * cg8 +
cg16 * cg18 * cg29 * cg3 * cg31 * cg8 + cg16 * cg2 * cg21 * cg29 *
cg31 * cg6 - cg16 * cg2 * cg23 * cg27 * cg31 * cg6 + cg16 * cg21 *
cg24 * cg31 * cg5 * cg8 - cg16 * cg23 * cg24 * cg3 * cg31 * cg8 +
cg17 * cg18 * cg27 * cg31 * cg4 * cg8 - cg17 * cg18 * cg28 * cg3 *
cg31 * cg8 - cg17 * cg2 * cg21 * cg28 * cg31 * cg6 + cg17 * cg2 *
cg22 * cg27 * cg31 * cg6 - cg17 * cg21 * cg24 * cg31 * cg4 * cg8 +
cg17 * cg22 * cg24 * cg3 * cg31 * cg8 - cg10 * cg12 * cg2 * cg21 *
cg29 * cg31 + cg10 * cg12 * cg2 * cg23 * cg27 * cg31 + cg10 * cg14
* cg18 * cg27 * cg31 * cg5 - cg1 * cg15 * cg22 * cg29 * cg30 * cg8
+ cg1 * cg15 * cg23 * cg28 * cg30 * cg8 + cg1 * cg16 * cg21 * cg29
* cg30 * cg8 - cg1 * cg16 * cg23 * cg27 * cg30 * cg8 - cg1 * cg17 *
cg21 * cg28 * cg30 * cg8 + cg1 * cg17 * cg22 * cg27 * cg30 * cg8 -
cg1 * cg10 * cg14 * cg21 * cg29 * cg30 + cg1 * cg10 * cg14 * cg23 *
cg27 * cg30 + cg1 * cg11 * cg14 * cg21 * cg28 * cg30 - cg1 * cg11 *
cg14 * cg22 * cg27 * cg30 + cg1 * cg14 * cg22 * cg29 * cg30 * cg9 +
cg12 * cg2 * cg22 * cg29 * cg31 * cg9 - cg12 * cg2 * cg23 * cg28 *
cg31 * cg9 - cg14 * cg18 * cg28 * cg31 * cg5 * cg9 + cg14 * cg18 *
cg29 * cg31 * cg4 * cg9 + cg14 * cg21 * cg28 * cg31 * cg5 * cg6 -
cg14 * cg21 * cg29 * cg31 * cg4 * cg6 + cg14 * cg22 * cg24 * cg31 *
cg5 * cg9 - cg1 * cg14 * cg23 * cg28 * cg30 * cg9 + cg1 * cg10 *
cg15 * cg20 * cg29 * cg30 - cg1 * cg10 * cg17 * cg20 * cg27 * cg30
- cg1 * cg11 * cg15 * cg20 * cg28 * cg30 + cg1 * cg11 * cg16 * cg20
* cg27 * cg30 - cg1 * cg16 * cg20 * cg29 * cg30 * cg9 + cg1 * cg17
* cg20 * cg28 * cg30 * cg9 - cg1 * cg10 * cg15 * cg23 * cg26 * cg30
+ cg1 * cg10 * cg17 * cg21 * cg26 * cg30 + cg1 * cg11 * cg15 * cg22
* cg26 * cg30 - cg1 * cg11 * cg16 * cg21 * cg26 * cg30 + cg1 * cg16
* cg23 * cg26 * cg30 * cg9 - cg1 * cg17 * cg22 * cg26 * cg30 * cg9
- cg12 * cg23 * cg27 * cg31 * cg4 * cg8 + cg12 * cg23 * cg28 * cg3
* cg31 * cg8 + cg15 * cg18 * cg28 * cg31 * cg5 * cg8 - cg15 * cg18
* cg29 * cg31 * cg4 * cg8 - cg15 * cg2 * cg22 * cg29 * cg31 * cg6 +
cg15 * cg2 * cg23 * cg28 * cg31 * cg6 - cg15 * cg22 * cg24 * cg31 *
cg5 * cg8 - cg0 * cg16 * cg23 * cg27 * cg32 * cg7 - cg0 * cg17 *
cg21 * cg28 * cg32 * cg7 + cg0 * cg17 * cg22 * cg27 * cg32 * cg7 -
cg0 * cg13 * cg23 * cg27 * cg34 * cg8 + cg0 * cg13 * cg23 * cg28 *
cg33 * cg8 - cg0 * cg10 * cg13 * cg20 * cg27 * cg35 + cg0 * cg10 *
cg13 * cg20 * cg29 * cg33 + cg0 * cg11 * cg13 * cg20 * cg27 * cg34
- cg0 * cg11 * cg13 * cg20 * cg28 * cg33 + cg0 * cg13 * cg20 * cg28
* cg35 * cg9 - cg0 * cg13 * cg20 * cg29 * cg34 * cg9 + cg0 * cg10 *
cg13 * cg21 * cg26 * cg35 - cg0 * cg10 * cg13 * cg23 * cg26 * cg33
- cg0 * cg11 * cg13 * cg21 * cg26 * cg34 + cg0 * cg11 * cg13 * cg22
* cg26 * cg33 - cg0 * cg13 * cg22 * cg26 * cg35 * cg9 + cg0 * cg13
* cg23 * cg26 * cg34 * cg9 - cg0 * cg10 * cg13 * cg21 * cg29 * cg32
+ cg0 * cg10 * cg13 * cg23 * cg27 * cg32 + cg0 * cg11 * cg13 * cg21
* cg28 * cg32 - cg0 * cg11 * cg13 * cg22 * cg27 * cg32 + cg0 * cg13
* cg22 * cg29 * cg32 * cg9 - cg0 * cg13 * cg23 * cg28 * cg32 * cg9
- cg0 * cg13 * cg21 * cg28 * cg35 * cg8 + cg0 * cg13 * cg21 * cg29
* cg34 * cg8 + cg0 * cg13 * cg22 * cg27 * cg35 * cg8 - cg0 * cg13 *
cg22 * cg29 * cg33 * cg8 + cg0 * cg14 * cg21 * cg28 * cg35 * cg7 -
cg0 * cg14 * cg21 * cg29 * cg34 * cg7 - cg0 * cg14 * cg22 * cg27 *
cg35 * cg7 + cg0 * cg14 * cg22 * cg29 * cg33 * cg7 + cg0 * cg14 *
cg23 * cg27 * cg34 * cg7 - cg0 * cg14 * cg23 * cg28 * cg33 * cg7 -
cg0 * cg15 * cg20 * cg28 * cg35 * cg7 + cg0 * cg15 * cg20 * cg29 *
cg34 * cg7 + cg0 * cg16 * cg20 * cg27 * cg35 * cg7 - cg0 * cg10 *
cg15 * cg19 * cg26 * cg35 + cg0 * cg10 * cg17 * cg19 * cg26 * cg33
+ cg0 * cg11 * cg15 * cg19 * cg26 * cg34 - cg0 * cg11 * cg16 * cg19
* cg26 * cg33 + cg0 * cg16 * cg19 * cg26 * cg35 * cg9 - cg0 * cg17
* cg19 * cg26 * cg34 * cg9 + cg0 * cg10 * cg15 * cg19 * cg29 * cg32
- cg0 * cg10 * cg17 * cg19 * cg27 * cg32 - cg0 * cg11 * cg15 * cg19
* cg28 * cg32 + cg0 * cg11 * cg16 * cg19 * cg27 * cg32 - cg0 * cg16
* cg19 * cg29 * cg32 * cg9 + cg0 * cg17 * cg19 * cg28 * cg32 * cg9
+ cg0 * cg16 * cg21 * cg25 * cg35 * cg8 - cg0 * cg16 * cg23 * cg25
* cg33 * cg8 - cg0 * cg17 * cg21 * cg25 * cg34 * cg8 + cg0 * cg17 *
cg22 * cg25 * cg33 * cg8 - cg0 * cg10 * cg14 * cg21 * cg25 * cg35 +
cg0 * cg10 * cg14 * cg23 * cg25 * cg33 + cg0 * cg11 * cg14 * cg21 *
cg25 * cg34 - cg0 * cg11 * cg14 * cg22 * cg25 * cg33 + cg0 * cg14 *
cg22 * cg25 * cg35 * cg9 - cg0 * cg14 * cg23 * cg25 * cg34 * cg9 +
cg0 * cg10 * cg15 * cg20 * cg25 * cg35 - cg0 * cg10 * cg17 * cg20 *
cg25 * cg33 - cg0 * cg11 * cg15 * cg20 * cg25 * cg34 + cg0 * cg11 *
cg16 * cg20 * cg25 * cg33 - cg0 * cg16 * cg20 * cg25 * cg35 * cg9 -
cg0 * cg16 * cg20 * cg29 * cg33 * cg7 - cg0 * cg17 * cg20 * cg27 *
cg34 * cg7 + cg0 * cg17 * cg20 * cg28 * cg33 * cg7 + cg0 * cg15 *
cg22 * cg26 * cg35 * cg7 - cg0 * cg15 * cg23 * cg26 * cg34 * cg7 -
cg0 * cg16 * cg21 * cg26 * cg35 * cg7 + cg0 * cg16 * cg23 * cg26 *
cg33 * cg7 + cg0 * cg17 * cg21 * cg26 * cg34 * cg7 - cg0 * cg17 *
cg22 * cg26 * cg33 * cg7 - cg0 * cg15 * cg22 * cg29 * cg32 * cg7 +
cg0 * cg15 * cg23 * cg28 * cg32 * cg7 + cg0 * cg16 * cg21 * cg29 *
cg32 * cg7 - cg0 * cg17 * cg22 * cg25 * cg32 * cg9 - cg0 * cg15 *
cg22 * cg25 * cg35 * cg8 + cg0 * cg15 * cg23 * cg25 * cg34 * cg8 +
cg0 * cg15 * cg22 * cg29 * cg31 * cg8 - cg0 * cg15 * cg23 * cg28 *
cg31 * cg8 - cg0 * cg16 * cg21 * cg29 * cg31 * cg8 + cg0 * cg16 *
cg23 * cg27 * cg31 * cg8 + cg0 * cg17 * cg21 * cg28 * cg31 * cg8 -
cg0 * cg17 * cg22 * cg27 * cg31 * cg8 + cg0 * cg10 * cg14 * cg21 *
cg29 * cg31 - cg0 * cg10 * cg14 * cg23 * cg27 * cg31 - cg0 * cg11 *
cg14 * cg21 * cg28 * cg31 + cg0 * cg11 * cg14 * cg22 * cg27 * cg31
- cg0 * cg14 * cg22 * cg29 * cg31 * cg9 + cg0 * cg14 * cg23 * cg28
* cg31 * cg9 - cg0 * cg10 * cg15 * cg20 * cg29 * cg31 + cg0 * cg10
* cg17 * cg20 * cg27 * cg31 + cg0 * cg11 * cg15 * cg20 * cg28 *
cg31 - cg0 * cg11 * cg16 * cg20 * cg27 * cg31 + cg0 * cg16 * cg20 *
cg29 * cg31 * cg9 - cg0 * cg17 * cg20 * cg28 * cg31 * cg9 + cg0 *
cg10 * cg15 * cg23 * cg26 * cg31 - cg0 * cg10 * cg17 * cg21 * cg26
* cg31 - cg0 * cg11 * cg15 * cg22 * cg26 * cg31 + cg0 * cg11 * cg16
* cg21 * cg26 * cg31 - cg0 * cg16 * cg23 * cg26 * cg31 * cg9 + cg0
* cg17 * cg22 * cg26 * cg31 * cg9 + cg0 * cg15 * cg19 * cg28 * cg35
* cg8 - cg0 * cg15 * cg19 * cg29 * cg34 * cg8 - cg0 * cg16 * cg19 *
cg27 * cg35 * cg8 + cg0 * cg16 * cg19 * cg29 * cg33 * cg8 + cg0 *
cg17 * cg19 * cg27 * cg34 * cg8 - cg0 * cg17 * cg19 * cg28 * cg33 *
cg8 + cg0 * cg10 * cg14 * cg19 * cg27 * cg35 - cg0 * cg10 * cg14 *
cg19 * cg29 * cg33 - cg0 * cg11 * cg14 * cg19 * cg27 * cg34 + cg0 *
cg11 * cg14 * cg19 * cg28 * cg33 - cg0 * cg14 * cg19 * cg28 * cg35
* cg9 + cg0 * cg14 * cg19 * cg29 * cg34 * cg9 + cg0 * cg17 * cg20 *
cg25 * cg34 * cg9 - cg0 * cg10 * cg15 * cg23 * cg25 * cg32 + cg0 *
cg10 * cg17 * cg21 * cg25 * cg32 + cg0 * cg11 * cg15 * cg22 * cg25
* cg32 - cg0 * cg11 * cg16 * cg21 * cg25 * cg32 + cg0 * cg16 * cg23
* cg25 * cg32 * cg9;

        double J_inverse_array[6][6];

        J_inverse_array[0][0] = -cg10 * cg13 * cg20 * cg27 * cg35 + cg10 * cg13 * cg20
* cg29 * cg33 + cg10 * cg13 * cg21 * cg26 * cg35 - cg10 * cg13 *
cg21 * cg29 * cg32 - cg10 * cg13 * cg23 * cg26 * cg33 + cg10 * cg13
* cg23 * cg27 * cg32 + cg10 * cg14 * cg19 * cg27 * cg35 - cg10 *
cg14 * cg19 * cg29 * cg33 - cg10 * cg14 * cg21 * cg25 * cg35 + cg10
* cg14 * cg21 * cg29 * cg31 + cg10 * cg14 * cg23 * cg25 * cg33 -
cg10 * cg14 * cg23 * cg27 * cg31 - cg10 * cg15 * cg19 * cg26 * cg35
+ cg10 * cg15 * cg19 * cg29 * cg32 + cg10 * cg15 * cg20 * cg25 *
cg35 - cg10 * cg15 * cg20 * cg29 * cg31 - cg10 * cg15 * cg23 * cg25
* cg32 + cg10 * cg15 * cg23 * cg26 * cg31 + cg10 * cg17 * cg19 *
cg26 * cg33 - cg10 * cg17 * cg19 * cg27 * cg32 - cg10 * cg17 * cg20
* cg25 * cg33 + cg10 * cg17 * cg20 * cg27 * cg31 + cg10 * cg17 *
cg21 * cg25 * cg32 - cg10 * cg17 * cg21 * cg26 * cg31 + cg11 * cg13
* cg20 * cg27 * cg34 - cg11 * cg13 * cg20 * cg28 * cg33 - cg11 *
cg13 * cg21 * cg26 * cg34 + cg11 * cg13 * cg21 * cg28 * cg32 + cg11
* cg13 * cg22 * cg26 * cg33 - cg11 * cg13 * cg22 * cg27 * cg32 -
cg11 * cg14 * cg19 * cg27 * cg34 + cg11 * cg14 * cg19 * cg28 * cg33
+ cg11 * cg14 * cg21 * cg25 * cg34 - cg11 * cg14 * cg21 * cg28 *
cg31 - cg11 * cg14 * cg22 * cg25 * cg33 + cg11 * cg14 * cg22 * cg27
* cg31 + cg11 * cg15 * cg19 * cg26 * cg34 - cg11 * cg15 * cg19 *
cg28 * cg32 - cg11 * cg15 * cg20 * cg25 * cg34 + cg11 * cg15 * cg20
* cg28 * cg31 + cg11 * cg15 * cg22 * cg25 * cg32 - cg11 * cg15 *
cg22 * cg26 * cg31 - cg11 * cg16 * cg19 * cg26 * cg33 + cg11 * cg16
* cg19 * cg27 * cg32 + cg11 * cg16 * cg20 * cg25 * cg33 - cg11 *
cg16 * cg20 * cg27 * cg31 - cg11 * cg16 * cg21 * cg25 * cg32 + cg11
* cg16 * cg21 * cg26 * cg31 + cg13 * cg20 * cg28 * cg35 * cg9 -
cg13 * cg20 * cg29 * cg34 * cg9 - cg13 * cg21 * cg28 * cg35 * cg8 +
cg13 * cg21 * cg29 * cg34 * cg8 - cg13 * cg22 * cg26 * cg35 * cg9 +
cg13 * cg22 * cg27 * cg35 * cg8 + cg13 * cg22 * cg29 * cg32 * cg9 -
cg13 * cg22 * cg29 * cg33 * cg8 + cg13 * cg23 * cg26 * cg34 * cg9 -
cg13 * cg23 * cg27 * cg34 * cg8 - cg13 * cg23 * cg28 * cg32 * cg9 +
cg13 * cg23 * cg28 * cg33 * cg8 - cg14 * cg19 * cg28 * cg35 * cg9 +
cg14 * cg19 * cg29 * cg34 * cg9 + cg14 * cg21 * cg28 * cg35 * cg7 -
cg14 * cg21 * cg29 * cg34 * cg7 + cg14 * cg22 * cg25 * cg35 * cg9 -
cg14 * cg22 * cg27 * cg35 * cg7 - cg14 * cg22 * cg29 * cg31 * cg9 +
cg14 * cg22 * cg29 * cg33 * cg7 - cg14 * cg23 * cg25 * cg34 * cg9 +
cg14 * cg23 * cg27 * cg34 * cg7 + cg14 * cg23 * cg28 * cg31 * cg9 -
cg14 * cg23 * cg28 * cg33 * cg7 + cg15 * cg19 * cg28 * cg35 * cg8 -
cg15 * cg19 * cg29 * cg34 * cg8 - cg15 * cg20 * cg28 * cg35 * cg7 +
cg15 * cg20 * cg29 * cg34 * cg7 - cg15 * cg22 * cg25 * cg35 * cg8 +
cg15 * cg22 * cg26 * cg35 * cg7 + cg15 * cg22 * cg29 * cg31 * cg8 -
cg15 * cg22 * cg29 * cg32 * cg7 + cg15 * cg23 * cg25 * cg34 * cg8 -
cg15 * cg23 * cg26 * cg34 * cg7 - cg15 * cg23 * cg28 * cg31 * cg8 +
cg15 * cg23 * cg28 * cg32 * cg7 + cg16 * cg19 * cg26 * cg35 * cg9 -
cg16 * cg19 * cg27 * cg35 * cg8 - cg16 * cg19 * cg29 * cg32 * cg9 +
cg16 * cg19 * cg29 * cg33 * cg8 - cg16 * cg20 * cg25 * cg35 * cg9 +
cg16 * cg20 * cg27 * cg35 * cg7 + cg16 * cg20 * cg29 * cg31 * cg9 -
cg16 * cg20 * cg29 * cg33 * cg7 + cg16 * cg21 * cg25 * cg35 * cg8 -
cg16 * cg21 * cg26 * cg35 * cg7 - cg16 * cg21 * cg29 * cg31 * cg8 +
cg16 * cg21 * cg29 * cg32 * cg7 + cg16 * cg23 * cg25 * cg32 * cg9 -
cg16 * cg23 * cg25 * cg33 * cg8 - cg16 * cg23 * cg26 * cg31 * cg9 +
cg16 * cg23 * cg26 * cg33 * cg7 + cg16 * cg23 * cg27 * cg31 * cg8 -
cg16 * cg23 * cg27 * cg32 * cg7 - cg17 * cg19 * cg26 * cg34 * cg9 +
cg17 * cg19 * cg27 * cg34 * cg8 + cg17 * cg19 * cg28 * cg32 * cg9 -
cg17 * cg19 * cg28 * cg33 * cg8 + cg17 * cg20 * cg25 * cg34 * cg9 -
cg17 * cg20 * cg27 * cg34 * cg7 - cg17 * cg20 * cg28 * cg31 * cg9 +
cg17 * cg20 * cg28 * cg33 * cg7 - cg17 * cg21 * cg25 * cg34 * cg8 +
cg17 * cg21 * cg26 * cg34 * cg7 + cg17 * cg21 * cg28 * cg31 * cg8 -
cg17 * cg21 * cg28 * cg32 * cg7 - cg17 * cg22 * cg25 * cg32 * cg9 +
cg17 * cg22 * cg25 * cg33 * cg8 + cg17 * cg22 * cg26 * cg31 * cg9 -
cg17 * cg22 * cg26 * cg33 * cg7 - cg17 * cg22 * cg27 * cg31 * cg8 +
cg17 * cg22 * cg27 * cg32 * cg7;
J_inverse_array[0][1] = -cg1 * cg14 * cg21 * cg28 * cg35 + cg1 * cg14 * cg21 *
cg29 * cg34 + cg1 * cg14 * cg22 * cg27 * cg35 - cg1 * cg14 * cg22 *
cg29 * cg33 - cg1 * cg14 * cg23 * cg27 * cg34 + cg1 * cg14 * cg23 *
cg28 * cg33 + cg1 * cg15 * cg20 * cg28 * cg35 - cg1 * cg15 * cg20 *
cg29 * cg34 - cg1 * cg15 * cg22 * cg26 * cg35 + cg1 * cg15 * cg22 *
cg29 * cg32 + cg1 * cg15 * cg23 * cg26 * cg34 - cg1 * cg15 * cg23 *
cg28 * cg32 - cg1 * cg16 * cg20 * cg27 * cg35 + cg1 * cg16 * cg20 *
cg29 * cg33 + cg1 * cg16 * cg21 * cg26 * cg35 - cg1 * cg16 * cg21 *
cg29 * cg32 - cg1 * cg16 * cg23 * cg26 * cg33 + cg1 * cg16 * cg23 *
cg27 * cg32 + cg1 * cg17 * cg20 * cg27 * cg34 - cg1 * cg17 * cg20 *
cg28 * cg33 - cg1 * cg17 * cg21 * cg26 * cg34 + cg1 * cg17 * cg21 *
cg28 * cg32 + cg1 * cg17 * cg22 * cg26 * cg33 - cg1 * cg17 * cg22 *
cg27 * cg32 + cg13 * cg2 * cg21 * cg28 * cg35 - cg13 * cg2 * cg21 *
cg29 * cg34 - cg13 * cg2 * cg22 * cg27 * cg35 + cg13 * cg2 * cg22 *
cg29 * cg33 + cg13 * cg2 * cg23 * cg27 * cg34 - cg13 * cg2 * cg23 *
cg28 * cg33 - cg13 * cg20 * cg27 * cg34 * cg5 + cg13 * cg20 * cg27
* cg35 * cg4 - cg13 * cg20 * cg28 * cg3 * cg35 + cg13 * cg20 * cg28
* cg33 * cg5 + cg13 * cg20 * cg29 * cg3 * cg34 - cg13 * cg20 * cg29
* cg33 * cg4 + cg13 * cg21 * cg26 * cg34 * cg5 - cg13 * cg21 * cg26
* cg35 * cg4 - cg13 * cg21 * cg28 * cg32 * cg5 + cg13 * cg21 * cg29
* cg32 * cg4 + cg13 * cg22 * cg26 * cg3 * cg35 - cg13 * cg22 * cg26
* cg33 * cg5 + cg13 * cg22 * cg27 * cg32 * cg5 - cg13 * cg22 * cg29
* cg3 * cg32 - cg13 * cg23 * cg26 * cg3 * cg34 + cg13 * cg23 * cg26
* cg33 * cg4 - cg13 * cg23 * cg27 * cg32 * cg4 + cg13 * cg23 * cg28
* cg3 * cg32 + cg14 * cg19 * cg27 * cg34 * cg5 - cg14 * cg19 * cg27
* cg35 * cg4 + cg14 * cg19 * cg28 * cg3 * cg35 - cg14 * cg19 * cg28
* cg33 * cg5 - cg14 * cg19 * cg29 * cg3 * cg34 + cg14 * cg19 * cg29
* cg33 * cg4 - cg14 * cg21 * cg25 * cg34 * cg5 + cg14 * cg21 * cg25
* cg35 * cg4 + cg14 * cg21 * cg28 * cg31 * cg5 - cg14 * cg21 * cg29
* cg31 * cg4 - cg14 * cg22 * cg25 * cg3 * cg35 + cg14 * cg22 * cg25
* cg33 * cg5 - cg14 * cg22 * cg27 * cg31 * cg5 + cg14 * cg22 * cg29
* cg3 * cg31 + cg14 * cg23 * cg25 * cg3 * cg34 - cg14 * cg23 * cg25
* cg33 * cg4 + cg14 * cg23 * cg27 * cg31 * cg4 - cg14 * cg23 * cg28
* cg3 * cg31 - cg15 * cg19 * cg2 * cg28 * cg35 + cg15 * cg19 * cg2
* cg29 * cg34 - cg15 * cg19 * cg26 * cg34 * cg5 + cg15 * cg19 *
cg26 * cg35 * cg4 + cg15 * cg19 * cg28 * cg32 * cg5 - cg15 * cg19 *
cg29 * cg32 * cg4 + cg15 * cg2 * cg22 * cg25 * cg35 - cg15 * cg2 *
cg22 * cg29 * cg31 - cg15 * cg2 * cg23 * cg25 * cg34 + cg15 * cg2 *
cg23 * cg28 * cg31 + cg15 * cg20 * cg25 * cg34 * cg5 - cg15 * cg20
* cg25 * cg35 * cg4 - cg15 * cg20 * cg28 * cg31 * cg5 + cg15 * cg20
* cg29 * cg31 * cg4 - cg15 * cg22 * cg25 * cg32 * cg5 + cg15 * cg22
* cg26 * cg31 * cg5 + cg15 * cg23 * cg25 * cg32 * cg4 - cg15 * cg23
* cg26 * cg31 * cg4 + cg16 * cg19 * cg2 * cg27 * cg35 - cg16 * cg19
* cg2 * cg29 * cg33 - cg16 * cg19 * cg26 * cg3 * cg35 + cg16 * cg19
* cg26 * cg33 * cg5 - cg16 * cg19 * cg27 * cg32 * cg5 + cg16 * cg19
* cg29 * cg3 * cg32 - cg16 * cg2 * cg21 * cg25 * cg35 + cg16 * cg2
* cg21 * cg29 * cg31 + cg16 * cg2 * cg23 * cg25 * cg33 - cg16 * cg2
* cg23 * cg27 * cg31 + cg16 * cg20 * cg25 * cg3 * cg35 - cg16 *
cg20 * cg25 * cg33 * cg5 + cg16 * cg20 * cg27 * cg31 * cg5 - cg16 *
cg20 * cg29 * cg3 * cg31 + cg16 * cg21 * cg25 * cg32 * cg5 - cg16 *
cg21 * cg26 * cg31 * cg5 - cg16 * cg23 * cg25 * cg3 * cg32 + cg16 *
cg23 * cg26 * cg3 * cg31 - cg17 * cg19 * cg2 * cg27 * cg34 + cg17 *
cg19 * cg2 * cg28 * cg33 + cg17 * cg19 * cg26 * cg3 * cg34 - cg17 *
cg19 * cg26 * cg33 * cg4 + cg17 * cg19 * cg27 * cg32 * cg4 - cg17 *
cg19 * cg28 * cg3 * cg32 + cg17 * cg2 * cg21 * cg25 * cg34 - cg17 *
cg2 * cg21 * cg28 * cg31 - cg17 * cg2 * cg22 * cg25 * cg33 + cg17 *
cg2 * cg22 * cg27 * cg31 - cg17 * cg20 * cg25 * cg3 * cg34 + cg17 *
cg20 * cg25 * cg33 * cg4 - cg17 * cg20 * cg27 * cg31 * cg4 + cg17 *
cg20 * cg28 * cg3 * cg31 - cg17 * cg21 * cg25 * cg32 * cg4 + cg17 *
cg21 * cg26 * cg31 * cg4 + cg17 * cg22 * cg25 * cg3 * cg32 - cg17 *
cg22 * cg26 * cg3 * cg31;
J_inverse_array[0][2] = cg1 * cg10 * cg20 * cg27 * cg35 - cg1 * cg10 * cg20 *
cg29 * cg33 - cg1 * cg10 * cg21 * cg26 * cg35 + cg1 * cg10 * cg21 *
cg29 * cg32 + cg1 * cg10 * cg23 * cg26 * cg33 - cg1 * cg10 * cg23 *
cg27 * cg32 - cg1 * cg11 * cg20 * cg27 * cg34 + cg1 * cg11 * cg20 *
cg28 * cg33 + cg1 * cg11 * cg21 * cg26 * cg34 - cg1 * cg11 * cg21 *
cg28 * cg32 - cg1 * cg11 * cg22 * cg26 * cg33 + cg1 * cg11 * cg22 *
cg27 * cg32 - cg1 * cg20 * cg28 * cg35 * cg9 + cg1 * cg20 * cg29 *
cg34 * cg9 + cg1 * cg21 * cg28 * cg35 * cg8 - cg1 * cg21 * cg29 *
cg34 * cg8 + cg1 * cg22 * cg26 * cg35 * cg9 - cg1 * cg22 * cg27 *
cg35 * cg8 - cg1 * cg22 * cg29 * cg32 * cg9 + cg1 * cg22 * cg29 *
cg33 * cg8 - cg1 * cg23 * cg26 * cg34 * cg9 + cg1 * cg23 * cg27 *
cg34 * cg8 + cg1 * cg23 * cg28 * cg32 * cg9 - cg1 * cg23 * cg28 *
cg33 * cg8 - cg10 * cg19 * cg2 * cg27 * cg35 + cg10 * cg19 * cg2 *
cg29 * cg33 + cg10 * cg19 * cg26 * cg3 * cg35 - cg10 * cg19 * cg26
* cg33 * cg5 + cg10 * cg19 * cg27 * cg32 * cg5 - cg10 * cg19 * cg29
* cg3 * cg32 + cg10 * cg2 * cg21 * cg25 * cg35 - cg10 * cg2 * cg21
* cg29 * cg31 - cg10 * cg2 * cg23 * cg25 * cg33 + cg10 * cg2 * cg23
* cg27 * cg31 - cg10 * cg20 * cg25 * cg3 * cg35 + cg10 * cg20 *
cg25 * cg33 * cg5 - cg10 * cg20 * cg27 * cg31 * cg5 + cg10 * cg20 *
cg29 * cg3 * cg31 - cg10 * cg21 * cg25 * cg32 * cg5 + cg10 * cg21 *
cg26 * cg31 * cg5 + cg10 * cg23 * cg25 * cg3 * cg32 - cg10 * cg23 *
cg26 * cg3 * cg31 + cg11 * cg19 * cg2 * cg27 * cg34 - cg11 * cg19 *
cg2 * cg28 * cg33 - cg11 * cg19 * cg26 * cg3 * cg34 + cg11 * cg19 *
cg26 * cg33 * cg4 - cg11 * cg19 * cg27 * cg32 * cg4 + cg11 * cg19 *
cg28 * cg3 * cg32 - cg11 * cg2 * cg21 * cg25 * cg34 + cg11 * cg2 *
cg21 * cg28 * cg31 + cg11 * cg2 * cg22 * cg25 * cg33 - cg11 * cg2 *
cg22 * cg27 * cg31 + cg11 * cg20 * cg25 * cg3 * cg34 - cg11 * cg20
* cg25 * cg33 * cg4 + cg11 * cg20 * cg27 * cg31 * cg4 - cg11 * cg20
* cg28 * cg3 * cg31 + cg11 * cg21 * cg25 * cg32 * cg4 - cg11 * cg21
* cg26 * cg31 * cg4 - cg11 * cg22 * cg25 * cg3 * cg32 + cg11 * cg22
* cg26 * cg3 * cg31 + cg19 * cg2 * cg28 * cg35 * cg9 - cg19 * cg2 *
cg29 * cg34 * cg9 + cg19 * cg26 * cg34 * cg5 * cg9 - cg19 * cg26 *
cg35 * cg4 * cg9 - cg19 * cg27 * cg34 * cg5 * cg8 + cg19 * cg27 *
cg35 * cg4 * cg8 - cg19 * cg28 * cg3 * cg35 * cg8 - cg19 * cg28 *
cg32 * cg5 * cg9 + cg19 * cg28 * cg33 * cg5 * cg8 + cg19 * cg29 *
cg3 * cg34 * cg8 + cg19 * cg29 * cg32 * cg4 * cg9 - cg19 * cg29 *
cg33 * cg4 * cg8 - cg2 * cg21 * cg28 * cg35 * cg7 + cg2 * cg21 *
cg29 * cg34 * cg7 - cg2 * cg22 * cg25 * cg35 * cg9 + cg2 * cg22 *
cg27 * cg35 * cg7 + cg2 * cg22 * cg29 * cg31 * cg9 - cg2 * cg22 *
cg29 * cg33 * cg7 + cg2 * cg23 * cg25 * cg34 * cg9 - cg2 * cg23 *
cg27 * cg34 * cg7 - cg2 * cg23 * cg28 * cg31 * cg9 + cg2 * cg23 *
cg28 * cg33 * cg7 - cg20 * cg25 * cg34 * cg5 * cg9 + cg20 * cg25 *
cg35 * cg4 * cg9 + cg20 * cg27 * cg34 * cg5 * cg7 - cg20 * cg27 *
cg35 * cg4 * cg7 + cg20 * cg28 * cg3 * cg35 * cg7 + cg20 * cg28 *
cg31 * cg5 * cg9 - cg20 * cg28 * cg33 * cg5 * cg7 - cg20 * cg29 *
cg3 * cg34 * cg7 - cg20 * cg29 * cg31 * cg4 * cg9 + cg20 * cg29 *
cg33 * cg4 * cg7 + cg21 * cg25 * cg34 * cg5 * cg8 - cg21 * cg25 *
cg35 * cg4 * cg8 - cg21 * cg26 * cg34 * cg5 * cg7 + cg21 * cg26 *
cg35 * cg4 * cg7 - cg21 * cg28 * cg31 * cg5 * cg8 + cg21 * cg28 *
cg32 * cg5 * cg7 + cg21 * cg29 * cg31 * cg4 * cg8 - cg21 * cg29 *
cg32 * cg4 * cg7 + cg22 * cg25 * cg3 * cg35 * cg8 + cg22 * cg25 *
cg32 * cg5 * cg9 - cg22 * cg25 * cg33 * cg5 * cg8 - cg22 * cg26 *
cg3 * cg35 * cg7 - cg22 * cg26 * cg31 * cg5 * cg9 + cg22 * cg26 *
cg33 * cg5 * cg7 + cg22 * cg27 * cg31 * cg5 * cg8 - cg22 * cg27 *
cg32 * cg5 * cg7 - cg22 * cg29 * cg3 * cg31 * cg8 + cg22 * cg29 *
cg3 * cg32 * cg7 - cg23 * cg25 * cg3 * cg34 * cg8 - cg23 * cg25 *
cg32 * cg4 * cg9 + cg23 * cg25 * cg33 * cg4 * cg8 + cg23 * cg26 *
cg3 * cg34 * cg7 + cg23 * cg26 * cg31 * cg4 * cg9 - cg23 * cg26 *
cg33 * cg4 * cg7 - cg23 * cg27 * cg31 * cg4 * cg8 + cg23 * cg27 *
cg32 * cg4 * cg7 + cg23 * cg28 * cg3 * cg31 * cg8 - cg23 * cg28 *
cg3 * cg32 * cg7;
J_inverse_array[0][3] = -cg1 * cg10 * cg14 * cg27 * cg35 + cg1 * cg10 * cg14 *
cg29 * cg33 + cg1 * cg10 * cg15 * cg26 * cg35 - cg1 * cg10 * cg15 *
cg29 * cg32 - cg1 * cg10 * cg17 * cg26 * cg33 + cg1 * cg10 * cg17 *
cg27 * cg32 + cg1 * cg11 * cg14 * cg27 * cg34 - cg1 * cg11 * cg14 *
cg28 * cg33 - cg1 * cg11 * cg15 * cg26 * cg34 + cg1 * cg11 * cg15 *
cg28 * cg32 + cg1 * cg11 * cg16 * cg26 * cg33 - cg1 * cg11 * cg16 *
cg27 * cg32 + cg1 * cg14 * cg28 * cg35 * cg9 - cg1 * cg14 * cg29 *
cg34 * cg9 - cg1 * cg15 * cg28 * cg35 * cg8 + cg1 * cg15 * cg29 *
cg34 * cg8 - cg1 * cg16 * cg26 * cg35 * cg9 + cg1 * cg16 * cg27 *
cg35 * cg8 + cg1 * cg16 * cg29 * cg32 * cg9 - cg1 * cg16 * cg29 *
cg33 * cg8 + cg1 * cg17 * cg26 * cg34 * cg9 - cg1 * cg17 * cg27 *
cg34 * cg8 - cg1 * cg17 * cg28 * cg32 * cg9 + cg1 * cg17 * cg28 *
cg33 * cg8 + cg10 * cg13 * cg2 * cg27 * cg35 - cg10 * cg13 * cg2 *
cg29 * cg33 - cg10 * cg13 * cg26 * cg3 * cg35 + cg10 * cg13 * cg26
* cg33 * cg5 - cg10 * cg13 * cg27 * cg32 * cg5 + cg10 * cg13 * cg29
* cg3 * cg32 + cg10 * cg14 * cg25 * cg3 * cg35 - cg10 * cg14 * cg25
* cg33 * cg5 + cg10 * cg14 * cg27 * cg31 * cg5 - cg10 * cg14 * cg29
* cg3 * cg31 - cg10 * cg15 * cg2 * cg25 * cg35 + cg10 * cg15 * cg2
* cg29 * cg31 + cg10 * cg15 * cg25 * cg32 * cg5 - cg10 * cg15 *
cg26 * cg31 * cg5 + cg10 * cg17 * cg2 * cg25 * cg33 - cg10 * cg17 *
cg2 * cg27 * cg31 - cg10 * cg17 * cg25 * cg3 * cg32 + cg10 * cg17 *
cg26 * cg3 * cg31 - cg11 * cg13 * cg2 * cg27 * cg34 + cg11 * cg13 *
cg2 * cg28 * cg33 + cg11 * cg13 * cg26 * cg3 * cg34 - cg11 * cg13 *
cg26 * cg33 * cg4 + cg11 * cg13 * cg27 * cg32 * cg4 - cg11 * cg13 *
cg28 * cg3 * cg32 - cg11 * cg14 * cg25 * cg3 * cg34 + cg11 * cg14 *
cg25 * cg33 * cg4 - cg11 * cg14 * cg27 * cg31 * cg4 + cg11 * cg14 *
cg28 * cg3 * cg31 + cg11 * cg15 * cg2 * cg25 * cg34 - cg11 * cg15 *
cg2 * cg28 * cg31 - cg11 * cg15 * cg25 * cg32 * cg4 + cg11 * cg15 *
cg26 * cg31 * cg4 - cg11 * cg16 * cg2 * cg25 * cg33 + cg11 * cg16 *
cg2 * cg27 * cg31 + cg11 * cg16 * cg25 * cg3 * cg32 - cg11 * cg16 *
cg26 * cg3 * cg31 - cg13 * cg2 * cg28 * cg35 * cg9 + cg13 * cg2 *
cg29 * cg34 * cg9 - cg13 * cg26 * cg34 * cg5 * cg9 + cg13 * cg26 *
cg35 * cg4 * cg9 + cg13 * cg27 * cg34 * cg5 * cg8 - cg13 * cg27 *
cg35 * cg4 * cg8 + cg13 * cg28 * cg3 * cg35 * cg8 + cg13 * cg28 *
cg32 * cg5 * cg9 - cg13 * cg28 * cg33 * cg5 * cg8 - cg13 * cg29 *
cg3 * cg34 * cg8 - cg13 * cg29 * cg32 * cg4 * cg9 + cg13 * cg29 *
cg33 * cg4 * cg8 + cg14 * cg25 * cg34 * cg5 * cg9 - cg14 * cg25 *
cg35 * cg4 * cg9 - cg14 * cg27 * cg34 * cg5 * cg7 + cg14 * cg27 *
cg35 * cg4 * cg7 - cg14 * cg28 * cg3 * cg35 * cg7 - cg14 * cg28 *
cg31 * cg5 * cg9 + cg14 * cg28 * cg33 * cg5 * cg7 + cg14 * cg29 *
cg3 * cg34 * cg7 + cg14 * cg29 * cg31 * cg4 * cg9 - cg14 * cg29 *
cg33 * cg4 * cg7 + cg15 * cg2 * cg28 * cg35 * cg7 - cg15 * cg2 *
cg29 * cg34 * cg7 - cg15 * cg25 * cg34 * cg5 * cg8 + cg15 * cg25 *
cg35 * cg4 * cg8 + cg15 * cg26 * cg34 * cg5 * cg7 - cg15 * cg26 *
cg35 * cg4 * cg7 + cg15 * cg28 * cg31 * cg5 * cg8 - cg15 * cg28 *
cg32 * cg5 * cg7 - cg15 * cg29 * cg31 * cg4 * cg8 + cg15 * cg29 *
cg32 * cg4 * cg7 + cg16 * cg2 * cg25 * cg35 * cg9 - cg16 * cg2 *
cg27 * cg35 * cg7 - cg16 * cg2 * cg29 * cg31 * cg9 + cg16 * cg2 *
cg29 * cg33 * cg7 - cg16 * cg25 * cg3 * cg35 * cg8 - cg16 * cg25 *
cg32 * cg5 * cg9 + cg16 * cg25 * cg33 * cg5 * cg8 + cg16 * cg26 *
cg3 * cg35 * cg7 + cg16 * cg26 * cg31 * cg5 * cg9 - cg16 * cg26 *
cg33 * cg5 * cg7 - cg16 * cg27 * cg31 * cg5 * cg8 + cg16 * cg27 *
cg32 * cg5 * cg7 + cg16 * cg29 * cg3 * cg31 * cg8 - cg16 * cg29 *
cg3 * cg32 * cg7 - cg17 * cg2 * cg25 * cg34 * cg9 + cg17 * cg2 *
cg27 * cg34 * cg7 + cg17 * cg2 * cg28 * cg31 * cg9 - cg17 * cg2 *
cg28 * cg33 * cg7 + cg17 * cg25 * cg3 * cg34 * cg8 + cg17 * cg25 *
cg32 * cg4 * cg9 - cg17 * cg25 * cg33 * cg4 * cg8 - cg17 * cg26 *
cg3 * cg34 * cg7 - cg17 * cg26 * cg31 * cg4 * cg9 + cg17 * cg26 *
cg33 * cg4 * cg7 + cg17 * cg27 * cg31 * cg4 * cg8 - cg17 * cg27 *
cg32 * cg4 * cg7 - cg17 * cg28 * cg3 * cg31 * cg8 + cg17 * cg28 *
cg3 * cg32 * cg7;
J_inverse_array[0][4] = cg1 * cg10 * cg14 * cg21 * cg35 - cg1 * cg10 * cg14 *
cg23 * cg33 - cg1 * cg10 * cg15 * cg20 * cg35 + cg1 * cg10 * cg15 *
cg23 * cg32 + cg1 * cg10 * cg17 * cg20 * cg33 - cg1 * cg10 * cg17 *
cg21 * cg32 - cg1 * cg11 * cg14 * cg21 * cg34 + cg1 * cg11 * cg14 *
cg22 * cg33 + cg1 * cg11 * cg15 * cg20 * cg34 - cg1 * cg11 * cg15 *
cg22 * cg32 - cg1 * cg11 * cg16 * cg20 * cg33 + cg1 * cg11 * cg16 *
cg21 * cg32 - cg1 * cg14 * cg22 * cg35 * cg9 + cg1 * cg14 * cg23 *
cg34 * cg9 + cg1 * cg15 * cg22 * cg35 * cg8 - cg1 * cg15 * cg23 *
cg34 * cg8 + cg1 * cg16 * cg20 * cg35 * cg9 - cg1 * cg16 * cg21 *
cg35 * cg8 - cg1 * cg16 * cg23 * cg32 * cg9 + cg1 * cg16 * cg23 *
cg33 * cg8 - cg1 * cg17 * cg20 * cg34 * cg9 + cg1 * cg17 * cg21 *
cg34 * cg8 + cg1 * cg17 * cg22 * cg32 * cg9 - cg1 * cg17 * cg22 *
cg33 * cg8 - cg10 * cg13 * cg2 * cg21 * cg35 + cg10 * cg13 * cg2 *
cg23 * cg33 + cg10 * cg13 * cg20 * cg3 * cg35 - cg10 * cg13 * cg20
* cg33 * cg5 + cg10 * cg13 * cg21 * cg32 * cg5 - cg10 * cg13 * cg23
* cg3 * cg32 - cg10 * cg14 * cg19 * cg3 * cg35 + cg10 * cg14 * cg19
* cg33 * cg5 - cg10 * cg14 * cg21 * cg31 * cg5 + cg10 * cg14 * cg23
* cg3 * cg31 + cg10 * cg15 * cg19 * cg2 * cg35 - cg10 * cg15 * cg19
* cg32 * cg5 - cg10 * cg15 * cg2 * cg23 * cg31 + cg10 * cg15 * cg20
* cg31 * cg5 - cg10 * cg17 * cg19 * cg2 * cg33 + cg10 * cg17 * cg19
* cg3 * cg32 + cg10 * cg17 * cg2 * cg21 * cg31 - cg10 * cg17 * cg20
* cg3 * cg31 + cg11 * cg13 * cg2 * cg21 * cg34 - cg11 * cg13 * cg2
* cg22 * cg33 - cg11 * cg13 * cg20 * cg3 * cg34 + cg11 * cg13 *
cg20 * cg33 * cg4 - cg11 * cg13 * cg21 * cg32 * cg4 + cg11 * cg13 *
cg22 * cg3 * cg32 + cg11 * cg14 * cg19 * cg3 * cg34 - cg11 * cg14 *
cg19 * cg33 * cg4 + cg11 * cg14 * cg21 * cg31 * cg4 - cg11 * cg14 *
cg22 * cg3 * cg31 - cg11 * cg15 * cg19 * cg2 * cg34 + cg11 * cg15 *
cg19 * cg32 * cg4 + cg11 * cg15 * cg2 * cg22 * cg31 - cg11 * cg15 *
cg20 * cg31 * cg4 + cg11 * cg16 * cg19 * cg2 * cg33 - cg11 * cg16 *
cg19 * cg3 * cg32 - cg11 * cg16 * cg2 * cg21 * cg31 + cg11 * cg16 *
cg20 * cg3 * cg31 + cg13 * cg2 * cg22 * cg35 * cg9 - cg13 * cg2 *
cg23 * cg34 * cg9 + cg13 * cg20 * cg34 * cg5 * cg9 - cg13 * cg20 *
cg35 * cg4 * cg9 - cg13 * cg21 * cg34 * cg5 * cg8 + cg13 * cg21 *
cg35 * cg4 * cg8 - cg13 * cg22 * cg3 * cg35 * cg8 - cg13 * cg22 *
cg32 * cg5 * cg9 + cg13 * cg22 * cg33 * cg5 * cg8 + cg13 * cg23 *
cg3 * cg34 * cg8 + cg13 * cg23 * cg32 * cg4 * cg9 - cg13 * cg23 *
cg33 * cg4 * cg8 - cg14 * cg19 * cg34 * cg5 * cg9 + cg14 * cg19 *
cg35 * cg4 * cg9 + cg14 * cg21 * cg34 * cg5 * cg7 - cg14 * cg21 *
cg35 * cg4 * cg7 + cg14 * cg22 * cg3 * cg35 * cg7 + cg14 * cg22 *
cg31 * cg5 * cg9 - cg14 * cg22 * cg33 * cg5 * cg7 - cg14 * cg23 *
cg3 * cg34 * cg7 - cg14 * cg23 * cg31 * cg4 * cg9 + cg14 * cg23 *
cg33 * cg4 * cg7 + cg15 * cg19 * cg34 * cg5 * cg8 - cg15 * cg19 *
cg35 * cg4 * cg8 - cg15 * cg2 * cg22 * cg35 * cg7 + cg15 * cg2 *
cg23 * cg34 * cg7 - cg15 * cg20 * cg34 * cg5 * cg7 + cg15 * cg20 *
cg35 * cg4 * cg7 - cg15 * cg22 * cg31 * cg5 * cg8 + cg15 * cg22 *
cg32 * cg5 * cg7 + cg15 * cg23 * cg31 * cg4 * cg8 - cg15 * cg23 *
cg32 * cg4 * cg7 - cg16 * cg19 * cg2 * cg35 * cg9 + cg16 * cg19 *
cg3 * cg35 * cg8 + cg16 * cg19 * cg32 * cg5 * cg9 - cg16 * cg19 *
cg33 * cg5 * cg8 + cg16 * cg2 * cg21 * cg35 * cg7 + cg16 * cg2 *
cg23 * cg31 * cg9 - cg16 * cg2 * cg23 * cg33 * cg7 - cg16 * cg20 *
cg3 * cg35 * cg7 - cg16 * cg20 * cg31 * cg5 * cg9 + cg16 * cg20 *
cg33 * cg5 * cg7 + cg16 * cg21 * cg31 * cg5 * cg8 - cg16 * cg21 *
cg32 * cg5 * cg7 - cg16 * cg23 * cg3 * cg31 * cg8 + cg16 * cg23 *
cg3 * cg32 * cg7 + cg17 * cg19 * cg2 * cg34 * cg9 - cg17 * cg19 *
cg3 * cg34 * cg8 - cg17 * cg19 * cg32 * cg4 * cg9 + cg17 * cg19 *
cg33 * cg4 * cg8 - cg17 * cg2 * cg21 * cg34 * cg7 - cg17 * cg2 *
cg22 * cg31 * cg9 + cg17 * cg2 * cg22 * cg33 * cg7 + cg17 * cg20 *
cg3 * cg34 * cg7 + cg17 * cg20 * cg31 * cg4 * cg9 - cg17 * cg20 *
cg33 * cg4 * cg7 - cg17 * cg21 * cg31 * cg4 * cg8 + cg17 * cg21 *
cg32 * cg4 * cg7 + cg17 * cg22 * cg3 * cg31 * cg8 - cg17 * cg22 *
cg3 * cg32 * cg7;
J_inverse_array[0][5] = -cg1 * cg10 * cg14 * cg21 * cg29 + cg1 * cg10 * cg14 *
cg23 * cg27 + cg1 * cg10 * cg15 * cg20 * cg29 - cg1 * cg10 * cg15 *
cg23 * cg26 - cg1 * cg10 * cg17 * cg20 * cg27 + cg1 * cg10 * cg17 *
cg21 * cg26 + cg1 * cg11 * cg14 * cg21 * cg28 - cg1 * cg11 * cg14 *
cg22 * cg27 - cg1 * cg11 * cg15 * cg20 * cg28 + cg1 * cg11 * cg15 *
cg22 * cg26 + cg1 * cg11 * cg16 * cg20 * cg27 - cg1 * cg11 * cg16 *
cg21 * cg26 + cg1 * cg14 * cg22 * cg29 * cg9 - cg1 * cg14 * cg23 *
cg28 * cg9 - cg1 * cg15 * cg22 * cg29 * cg8 + cg1 * cg15 * cg23 *
cg28 * cg8 - cg1 * cg16 * cg20 * cg29 * cg9 + cg1 * cg16 * cg21 *
cg29 * cg8 + cg1 * cg16 * cg23 * cg26 * cg9 - cg1 * cg16 * cg23 *
cg27 * cg8 + cg1 * cg17 * cg20 * cg28 * cg9 - cg1 * cg17 * cg21 *
cg28 * cg8 - cg1 * cg17 * cg22 * cg26 * cg9 + cg1 * cg17 * cg22 *
cg27 * cg8 + cg10 * cg13 * cg2 * cg21 * cg29 - cg10 * cg13 * cg2 *
cg23 * cg27 + cg10 * cg13 * cg20 * cg27 * cg5 - cg10 * cg13 * cg20
* cg29 * cg3 - cg10 * cg13 * cg21 * cg26 * cg5 + cg10 * cg13 * cg23
* cg26 * cg3 - cg10 * cg14 * cg19 * cg27 * cg5 + cg10 * cg14 * cg19
* cg29 * cg3 + cg10 * cg14 * cg21 * cg25 * cg5 - cg10 * cg14 * cg23
* cg25 * cg3 - cg10 * cg15 * cg19 * cg2 * cg29 + cg10 * cg15 * cg19
* cg26 * cg5 + cg10 * cg15 * cg2 * cg23 * cg25 - cg10 * cg15 * cg20
* cg25 * cg5 + cg10 * cg17 * cg19 * cg2 * cg27 - cg10 * cg17 * cg19
* cg26 * cg3 - cg10 * cg17 * cg2 * cg21 * cg25 + cg10 * cg17 * cg20
* cg25 * cg3 - cg11 * cg13 * cg2 * cg21 * cg28 + cg11 * cg13 * cg2
* cg22 * cg27 - cg11 * cg13 * cg20 * cg27 * cg4 + cg11 * cg13 *
cg20 * cg28 * cg3 + cg11 * cg13 * cg21 * cg26 * cg4 - cg11 * cg13 *
cg22 * cg26 * cg3 + cg11 * cg14 * cg19 * cg27 * cg4 - cg11 * cg14 *
cg19 * cg28 * cg3 - cg11 * cg14 * cg21 * cg25 * cg4 + cg11 * cg14 *
cg22 * cg25 * cg3 + cg11 * cg15 * cg19 * cg2 * cg28 - cg11 * cg15 *
cg19 * cg26 * cg4 - cg11 * cg15 * cg2 * cg22 * cg25 + cg11 * cg15 *
cg20 * cg25 * cg4 - cg11 * cg16 * cg19 * cg2 * cg27 + cg11 * cg16 *
cg19 * cg26 * cg3 + cg11 * cg16 * cg2 * cg21 * cg25 - cg11 * cg16 *
cg20 * cg25 * cg3 - cg13 * cg2 * cg22 * cg29 * cg9 + cg13 * cg2 *
cg23 * cg28 * cg9 - cg13 * cg20 * cg28 * cg5 * cg9 + cg13 * cg20 *
cg29 * cg4 * cg9 + cg13 * cg21 * cg28 * cg5 * cg8 - cg13 * cg21 *
cg29 * cg4 * cg8 + cg13 * cg22 * cg26 * cg5 * cg9 - cg13 * cg22 *
cg27 * cg5 * cg8 + cg13 * cg22 * cg29 * cg3 * cg8 - cg13 * cg23 *
cg26 * cg4 * cg9 + cg13 * cg23 * cg27 * cg4 * cg8 - cg13 * cg23 *
cg28 * cg3 * cg8 + cg14 * cg19 * cg28 * cg5 * cg9 - cg14 * cg19 *
cg29 * cg4 * cg9 - cg14 * cg21 * cg28 * cg5 * cg7 + cg14 * cg21 *
cg29 * cg4 * cg7 - cg14 * cg22 * cg25 * cg5 * cg9 + cg14 * cg22 *
cg27 * cg5 * cg7 - cg14 * cg22 * cg29 * cg3 * cg7 + cg14 * cg23 *
cg25 * cg4 * cg9 - cg14 * cg23 * cg27 * cg4 * cg7 + cg14 * cg23 *
cg28 * cg3 * cg7 - cg15 * cg19 * cg28 * cg5 * cg8 + cg15 * cg19 *
cg29 * cg4 * cg8 + cg15 * cg2 * cg22 * cg29 * cg7 - cg15 * cg2 *
cg23 * cg28 * cg7 + cg15 * cg20 * cg28 * cg5 * cg7 - cg15 * cg20 *
cg29 * cg4 * cg7 + cg15 * cg22 * cg25 * cg5 * cg8 - cg15 * cg22 *
cg26 * cg5 * cg7 - cg15 * cg23 * cg25 * cg4 * cg8 + cg15 * cg23 *
cg26 * cg4 * cg7 + cg16 * cg19 * cg2 * cg29 * cg9 - cg16 * cg19 *
cg26 * cg5 * cg9 + cg16 * cg19 * cg27 * cg5 * cg8 - cg16 * cg19 *
cg29 * cg3 * cg8 - cg16 * cg2 * cg21 * cg29 * cg7 - cg16 * cg2 *
cg23 * cg25 * cg9 + cg16 * cg2 * cg23 * cg27 * cg7 + cg16 * cg20 *
cg25 * cg5 * cg9 - cg16 * cg20 * cg27 * cg5 * cg7 + cg16 * cg20 *
cg29 * cg3 * cg7 - cg16 * cg21 * cg25 * cg5 * cg8 + cg16 * cg21 *
cg26 * cg5 * cg7 + cg16 * cg23 * cg25 * cg3 * cg8 - cg16 * cg23 *
cg26 * cg3 * cg7 - cg17 * cg19 * cg2 * cg28 * cg9 + cg17 * cg19 *
cg26 * cg4 * cg9 - cg17 * cg19 * cg27 * cg4 * cg8 + cg17 * cg19 *
cg28 * cg3 * cg8 + cg17 * cg2 * cg21 * cg28 * cg7 + cg17 * cg2 *
cg22 * cg25 * cg9 - cg17 * cg2 * cg22 * cg27 * cg7 - cg17 * cg20 *
cg25 * cg4 * cg9 + cg17 * cg20 * cg27 * cg4 * cg7 - cg17 * cg20 *
cg28 * cg3 * cg7 + cg17 * cg21 * cg25 * cg4 * cg8 - cg17 * cg21 *
cg26 * cg4 * cg7 - cg17 * cg22 * cg25 * cg3 * cg8 + cg17 * cg22 *
cg26 * cg3 * cg7;
J_inverse_array[1][0] = cg10 * cg12 * cg20 * cg27 * cg35 - cg10 * cg12 * cg20 *
cg29 * cg33 - cg10 * cg12 * cg21 * cg26 * cg35 + cg10 * cg12 * cg21
* cg29 * cg32 + cg10 * cg12 * cg23 * cg26 * cg33 - cg10 * cg12 *
cg23 * cg27 * cg32 - cg10 * cg14 * cg18 * cg27 * cg35 + cg10 * cg14
* cg18 * cg29 * cg33 + cg10 * cg14 * cg21 * cg24 * cg35 - cg10 *
cg14 * cg21 * cg29 * cg30 - cg10 * cg14 * cg23 * cg24 * cg33 + cg10
* cg14 * cg23 * cg27 * cg30 + cg10 * cg15 * cg18 * cg26 * cg35 -
cg10 * cg15 * cg18 * cg29 * cg32 - cg10 * cg15 * cg20 * cg24 * cg35
+ cg10 * cg15 * cg20 * cg29 * cg30 + cg10 * cg15 * cg23 * cg24 *
cg32 - cg10 * cg15 * cg23 * cg26 * cg30 - cg10 * cg17 * cg18 * cg26
* cg33 + cg10 * cg17 * cg18 * cg27 * cg32 + cg10 * cg17 * cg20 *
cg24 * cg33 - cg10 * cg17 * cg20 * cg27 * cg30 - cg10 * cg17 * cg21
* cg24 * cg32 + cg10 * cg17 * cg21 * cg26 * cg30 - cg11 * cg12 *
cg20 * cg27 * cg34 + cg11 * cg12 * cg20 * cg28 * cg33 + cg11 * cg12
* cg21 * cg26 * cg34 - cg11 * cg12 * cg21 * cg28 * cg32 - cg11 *
cg12 * cg22 * cg26 * cg33 + cg11 * cg12 * cg22 * cg27 * cg32 + cg11
* cg14 * cg18 * cg27 * cg34 - cg11 * cg14 * cg18 * cg28 * cg33 -
cg11 * cg14 * cg21 * cg24 * cg34 + cg11 * cg14 * cg21 * cg28 * cg30
+ cg11 * cg14 * cg22 * cg24 * cg33 - cg11 * cg14 * cg22 * cg27 *
cg30 - cg11 * cg15 * cg18 * cg26 * cg34 + cg11 * cg15 * cg18 * cg28
* cg32 + cg11 * cg15 * cg20 * cg24 * cg34 - cg11 * cg15 * cg20 *
cg28 * cg30 - cg11 * cg15 * cg22 * cg24 * cg32 + cg11 * cg15 * cg22
* cg26 * cg30 + cg11 * cg16 * cg18 * cg26 * cg33 - cg11 * cg16 *
cg18 * cg27 * cg32 - cg11 * cg16 * cg20 * cg24 * cg33 + cg11 * cg16
* cg20 * cg27 * cg30 + cg11 * cg16 * cg21 * cg24 * cg32 - cg11 *
cg16 * cg21 * cg26 * cg30 - cg12 * cg20 * cg28 * cg35 * cg9 + cg12
* cg20 * cg29 * cg34 * cg9 + cg12 * cg21 * cg28 * cg35 * cg8 - cg12
* cg21 * cg29 * cg34 * cg8 + cg12 * cg22 * cg26 * cg35 * cg9 - cg12
* cg22 * cg27 * cg35 * cg8 - cg12 * cg22 * cg29 * cg32 * cg9 + cg12
* cg22 * cg29 * cg33 * cg8 - cg12 * cg23 * cg26 * cg34 * cg9 + cg12
* cg23 * cg27 * cg34 * cg8 + cg12 * cg23 * cg28 * cg32 * cg9 - cg12
* cg23 * cg28 * cg33 * cg8 + cg14 * cg18 * cg28 * cg35 * cg9 - cg14
* cg18 * cg29 * cg34 * cg9 - cg14 * cg21 * cg28 * cg35 * cg6 + cg14
* cg21 * cg29 * cg34 * cg6 - cg14 * cg22 * cg24 * cg35 * cg9 + cg14
* cg22 * cg27 * cg35 * cg6 + cg14 * cg22 * cg29 * cg30 * cg9 - cg14
* cg22 * cg29 * cg33 * cg6 + cg14 * cg23 * cg24 * cg34 * cg9 - cg14
* cg23 * cg27 * cg34 * cg6 - cg14 * cg23 * cg28 * cg30 * cg9 + cg14
* cg23 * cg28 * cg33 * cg6 - cg15 * cg18 * cg28 * cg35 * cg8 + cg15
* cg18 * cg29 * cg34 * cg8 + cg15 * cg20 * cg28 * cg35 * cg6 - cg15
* cg20 * cg29 * cg34 * cg6 + cg15 * cg22 * cg24 * cg35 * cg8 - cg15
* cg22 * cg26 * cg35 * cg6 - cg15 * cg22 * cg29 * cg30 * cg8 + cg15
* cg22 * cg29 * cg32 * cg6 - cg15 * cg23 * cg24 * cg34 * cg8 + cg15
* cg23 * cg26 * cg34 * cg6 + cg15 * cg23 * cg28 * cg30 * cg8 - cg15
* cg23 * cg28 * cg32 * cg6 - cg16 * cg18 * cg26 * cg35 * cg9 + cg16
* cg18 * cg27 * cg35 * cg8 + cg16 * cg18 * cg29 * cg32 * cg9 - cg16
* cg18 * cg29 * cg33 * cg8 + cg16 * cg20 * cg24 * cg35 * cg9 - cg16
* cg20 * cg27 * cg35 * cg6 - cg16 * cg20 * cg29 * cg30 * cg9 + cg16
* cg20 * cg29 * cg33 * cg6 - cg16 * cg21 * cg24 * cg35 * cg8 + cg16
* cg21 * cg26 * cg35 * cg6 + cg16 * cg21 * cg29 * cg30 * cg8 - cg16
* cg21 * cg29 * cg32 * cg6 - cg16 * cg23 * cg24 * cg32 * cg9 + cg16
* cg23 * cg24 * cg33 * cg8 + cg16 * cg23 * cg26 * cg30 * cg9 - cg16
* cg23 * cg26 * cg33 * cg6 - cg16 * cg23 * cg27 * cg30 * cg8 + cg16
* cg23 * cg27 * cg32 * cg6 + cg17 * cg18 * cg26 * cg34 * cg9 - cg17
* cg18 * cg27 * cg34 * cg8 - cg17 * cg18 * cg28 * cg32 * cg9 + cg17
* cg18 * cg28 * cg33 * cg8 - cg17 * cg20 * cg24 * cg34 * cg9 + cg17
* cg20 * cg27 * cg34 * cg6 + cg17 * cg20 * cg28 * cg30 * cg9 - cg17
* cg20 * cg28 * cg33 * cg6 + cg17 * cg21 * cg24 * cg34 * cg8 - cg17
* cg21 * cg26 * cg34 * cg6 - cg17 * cg21 * cg28 * cg30 * cg8 + cg17
* cg21 * cg28 * cg32 * cg6 + cg17 * cg22 * cg24 * cg32 * cg9 - cg17
* cg22 * cg24 * cg33 * cg8 - cg17 * cg22 * cg26 * cg30 * cg9 + cg17
* cg22 * cg26 * cg33 * cg6 + cg17 * cg22 * cg27 * cg30 * cg8 - cg17
* cg22 * cg27 * cg32 * cg6;
J_inverse_array[1][1] = cg0 * cg14 * cg21 * cg28 * cg35 - cg0 * cg14 * cg21 *
cg29 * cg34 - cg0 * cg14 * cg22 * cg27 * cg35 + cg0 * cg14 * cg22 *
cg29 * cg33 + cg0 * cg14 * cg23 * cg27 * cg34 - cg0 * cg14 * cg23 *
cg28 * cg33 - cg0 * cg15 * cg20 * cg28 * cg35 + cg0 * cg15 * cg20 *
cg29 * cg34 + cg0 * cg15 * cg22 * cg26 * cg35 - cg0 * cg15 * cg22 *
cg29 * cg32 - cg0 * cg15 * cg23 * cg26 * cg34 + cg0 * cg15 * cg23 *
cg28 * cg32 + cg0 * cg16 * cg20 * cg27 * cg35 - cg0 * cg16 * cg20 *
cg29 * cg33 - cg0 * cg16 * cg21 * cg26 * cg35 + cg0 * cg16 * cg21 *
cg29 * cg32 + cg0 * cg16 * cg23 * cg26 * cg33 - cg0 * cg16 * cg23 *
cg27 * cg32 - cg0 * cg17 * cg20 * cg27 * cg34 + cg0 * cg17 * cg20 *
cg28 * cg33 + cg0 * cg17 * cg21 * cg26 * cg34 - cg0 * cg17 * cg21 *
cg28 * cg32 - cg0 * cg17 * cg22 * cg26 * cg33 + cg0 * cg17 * cg22 *
cg27 * cg32 - cg12 * cg2 * cg21 * cg28 * cg35 + cg12 * cg2 * cg21 *
cg29 * cg34 + cg12 * cg2 * cg22 * cg27 * cg35 - cg12 * cg2 * cg22 *
cg29 * cg33 - cg12 * cg2 * cg23 * cg27 * cg34 + cg12 * cg2 * cg23 *
cg28 * cg33 + cg12 * cg20 * cg27 * cg34 * cg5 - cg12 * cg20 * cg27
* cg35 * cg4 + cg12 * cg20 * cg28 * cg3 * cg35 - cg12 * cg20 * cg28
* cg33 * cg5 - cg12 * cg20 * cg29 * cg3 * cg34 + cg12 * cg20 * cg29
* cg33 * cg4 - cg12 * cg21 * cg26 * cg34 * cg5 + cg12 * cg21 * cg26
* cg35 * cg4 + cg12 * cg21 * cg28 * cg32 * cg5 - cg12 * cg21 * cg29
* cg32 * cg4 - cg12 * cg22 * cg26 * cg3 * cg35 + cg12 * cg22 * cg26
* cg33 * cg5 - cg12 * cg22 * cg27 * cg32 * cg5 + cg12 * cg22 * cg29
* cg3 * cg32 + cg12 * cg23 * cg26 * cg3 * cg34 - cg12 * cg23 * cg26
* cg33 * cg4 + cg12 * cg23 * cg27 * cg32 * cg4 - cg12 * cg23 * cg28
* cg3 * cg32 - cg14 * cg18 * cg27 * cg34 * cg5 + cg14 * cg18 * cg27
* cg35 * cg4 - cg14 * cg18 * cg28 * cg3 * cg35 + cg14 * cg18 * cg28
* cg33 * cg5 + cg14 * cg18 * cg29 * cg3 * cg34 - cg14 * cg18 * cg29
* cg33 * cg4 + cg14 * cg21 * cg24 * cg34 * cg5 - cg14 * cg21 * cg24
* cg35 * cg4 - cg14 * cg21 * cg28 * cg30 * cg5 + cg14 * cg21 * cg29
* cg30 * cg4 + cg14 * cg22 * cg24 * cg3 * cg35 - cg14 * cg22 * cg24
* cg33 * cg5 + cg14 * cg22 * cg27 * cg30 * cg5 - cg14 * cg22 * cg29
* cg3 * cg30 - cg14 * cg23 * cg24 * cg3 * cg34 + cg14 * cg23 * cg24
* cg33 * cg4 - cg14 * cg23 * cg27 * cg30 * cg4 + cg14 * cg23 * cg28
* cg3 * cg30 + cg15 * cg18 * cg2 * cg28 * cg35 - cg15 * cg18 * cg2
* cg29 * cg34 + cg15 * cg18 * cg26 * cg34 * cg5 - cg15 * cg18 *
cg26 * cg35 * cg4 - cg15 * cg18 * cg28 * cg32 * cg5 + cg15 * cg18 *
cg29 * cg32 * cg4 - cg15 * cg2 * cg22 * cg24 * cg35 + cg15 * cg2 *
cg22 * cg29 * cg30 + cg15 * cg2 * cg23 * cg24 * cg34 - cg15 * cg2 *
cg23 * cg28 * cg30 - cg15 * cg20 * cg24 * cg34 * cg5 + cg15 * cg20
* cg24 * cg35 * cg4 + cg15 * cg20 * cg28 * cg30 * cg5 - cg15 * cg20
* cg29 * cg30 * cg4 + cg15 * cg22 * cg24 * cg32 * cg5 - cg15 * cg22
* cg26 * cg30 * cg5 - cg15 * cg23 * cg24 * cg32 * cg4 + cg15 * cg23
* cg26 * cg30 * cg4 - cg16 * cg18 * cg2 * cg27 * cg35 + cg16 * cg18
* cg2 * cg29 * cg33 + cg16 * cg18 * cg26 * cg3 * cg35 - cg16 * cg18
* cg26 * cg33 * cg5 + cg16 * cg18 * cg27 * cg32 * cg5 - cg16 * cg18
* cg29 * cg3 * cg32 + cg16 * cg2 * cg21 * cg24 * cg35 - cg16 * cg2
* cg21 * cg29 * cg30 - cg16 * cg2 * cg23 * cg24 * cg33 + cg16 * cg2
* cg23 * cg27 * cg30 - cg16 * cg20 * cg24 * cg3 * cg35 + cg16 *
cg20 * cg24 * cg33 * cg5 - cg16 * cg20 * cg27 * cg30 * cg5 + cg16 *
cg20 * cg29 * cg3 * cg30 - cg16 * cg21 * cg24 * cg32 * cg5 + cg16 *
cg21 * cg26 * cg30 * cg5 + cg16 * cg23 * cg24 * cg3 * cg32 - cg16 *
cg23 * cg26 * cg3 * cg30 + cg17 * cg18 * cg2 * cg27 * cg34 - cg17 *
cg18 * cg2 * cg28 * cg33 - cg17 * cg18 * cg26 * cg3 * cg34 + cg17 *
cg18 * cg26 * cg33 * cg4 - cg17 * cg18 * cg27 * cg32 * cg4 + cg17 *
cg18 * cg28 * cg3 * cg32 - cg17 * cg2 * cg21 * cg24 * cg34 + cg17 *
cg2 * cg21 * cg28 * cg30 + cg17 * cg2 * cg22 * cg24 * cg33 - cg17 *
cg2 * cg22 * cg27 * cg30 + cg17 * cg20 * cg24 * cg3 * cg34 - cg17 *
cg20 * cg24 * cg33 * cg4 + cg17 * cg20 * cg27 * cg30 * cg4 - cg17 *
cg20 * cg28 * cg3 * cg30 + cg17 * cg21 * cg24 * cg32 * cg4 - cg17 *
cg21 * cg26 * cg30 * cg4 - cg17 * cg22 * cg24 * cg3 * cg32 + cg17 *
cg22 * cg26 * cg3 * cg30;
J_inverse_array[1][2] = -cg0 * cg10 * cg20 * cg27 * cg35 + cg0 * cg10 * cg20 *
cg29 * cg33 + cg0 * cg10 * cg21 * cg26 * cg35 - cg0 * cg10 * cg21 *
cg29 * cg32 - cg0 * cg10 * cg23 * cg26 * cg33 + cg0 * cg10 * cg23 *
cg27 * cg32 + cg0 * cg11 * cg20 * cg27 * cg34 - cg0 * cg11 * cg20 *
cg28 * cg33 - cg0 * cg11 * cg21 * cg26 * cg34 + cg0 * cg11 * cg21 *
cg28 * cg32 + cg0 * cg11 * cg22 * cg26 * cg33 - cg0 * cg11 * cg22 *
cg27 * cg32 + cg0 * cg20 * cg28 * cg35 * cg9 - cg0 * cg20 * cg29 *
cg34 * cg9 - cg0 * cg21 * cg28 * cg35 * cg8 + cg0 * cg21 * cg29 *
cg34 * cg8 - cg0 * cg22 * cg26 * cg35 * cg9 + cg0 * cg22 * cg27 *
cg35 * cg8 + cg0 * cg22 * cg29 * cg32 * cg9 - cg0 * cg22 * cg29 *
cg33 * cg8 + cg0 * cg23 * cg26 * cg34 * cg9 - cg0 * cg23 * cg27 *
cg34 * cg8 - cg0 * cg23 * cg28 * cg32 * cg9 + cg0 * cg23 * cg28 *
cg33 * cg8 + cg10 * cg18 * cg2 * cg27 * cg35 - cg10 * cg18 * cg2 *
cg29 * cg33 - cg10 * cg18 * cg26 * cg3 * cg35 + cg10 * cg18 * cg26
* cg33 * cg5 - cg10 * cg18 * cg27 * cg32 * cg5 + cg10 * cg18 * cg29
* cg3 * cg32 - cg10 * cg2 * cg21 * cg24 * cg35 + cg10 * cg2 * cg21
* cg29 * cg30 + cg10 * cg2 * cg23 * cg24 * cg33 - cg10 * cg2 * cg23
* cg27 * cg30 + cg10 * cg20 * cg24 * cg3 * cg35 - cg10 * cg20 *
cg24 * cg33 * cg5 + cg10 * cg20 * cg27 * cg30 * cg5 - cg10 * cg20 *
cg29 * cg3 * cg30 + cg10 * cg21 * cg24 * cg32 * cg5 - cg10 * cg21 *
cg26 * cg30 * cg5 - cg10 * cg23 * cg24 * cg3 * cg32 + cg10 * cg23 *
cg26 * cg3 * cg30 - cg11 * cg18 * cg2 * cg27 * cg34 + cg11 * cg18 *
cg2 * cg28 * cg33 + cg11 * cg18 * cg26 * cg3 * cg34 - cg11 * cg18 *
cg26 * cg33 * cg4 + cg11 * cg18 * cg27 * cg32 * cg4 - cg11 * cg18 *
cg28 * cg3 * cg32 + cg11 * cg2 * cg21 * cg24 * cg34 - cg11 * cg2 *
cg21 * cg28 * cg30 - cg11 * cg2 * cg22 * cg24 * cg33 + cg11 * cg2 *
cg22 * cg27 * cg30 - cg11 * cg20 * cg24 * cg3 * cg34 + cg11 * cg20
* cg24 * cg33 * cg4 - cg11 * cg20 * cg27 * cg30 * cg4 + cg11 * cg20
* cg28 * cg3 * cg30 - cg11 * cg21 * cg24 * cg32 * cg4 + cg11 * cg21
* cg26 * cg30 * cg4 + cg11 * cg22 * cg24 * cg3 * cg32 - cg11 * cg22
* cg26 * cg3 * cg30 - cg18 * cg2 * cg28 * cg35 * cg9 + cg18 * cg2 *
cg29 * cg34 * cg9 - cg18 * cg26 * cg34 * cg5 * cg9 + cg18 * cg26 *
cg35 * cg4 * cg9 + cg18 * cg27 * cg34 * cg5 * cg8 - cg18 * cg27 *
cg35 * cg4 * cg8 + cg18 * cg28 * cg3 * cg35 * cg8 + cg18 * cg28 *
cg32 * cg5 * cg9 - cg18 * cg28 * cg33 * cg5 * cg8 - cg18 * cg29 *
cg3 * cg34 * cg8 - cg18 * cg29 * cg32 * cg4 * cg9 + cg18 * cg29 *
cg33 * cg4 * cg8 + cg2 * cg21 * cg28 * cg35 * cg6 - cg2 * cg21 *
cg29 * cg34 * cg6 + cg2 * cg22 * cg24 * cg35 * cg9 - cg2 * cg22 *
cg27 * cg35 * cg6 - cg2 * cg22 * cg29 * cg30 * cg9 + cg2 * cg22 *
cg29 * cg33 * cg6 - cg2 * cg23 * cg24 * cg34 * cg9 + cg2 * cg23 *
cg27 * cg34 * cg6 + cg2 * cg23 * cg28 * cg30 * cg9 - cg2 * cg23 *
cg28 * cg33 * cg6 + cg20 * cg24 * cg34 * cg5 * cg9 - cg20 * cg24 *
cg35 * cg4 * cg9 - cg20 * cg27 * cg34 * cg5 * cg6 + cg20 * cg27 *
cg35 * cg4 * cg6 - cg20 * cg28 * cg3 * cg35 * cg6 - cg20 * cg28 *
cg30 * cg5 * cg9 + cg20 * cg28 * cg33 * cg5 * cg6 + cg20 * cg29 *
cg3 * cg34 * cg6 + cg20 * cg29 * cg30 * cg4 * cg9 - cg20 * cg29 *
cg33 * cg4 * cg6 - cg21 * cg24 * cg34 * cg5 * cg8 + cg21 * cg24 *
cg35 * cg4 * cg8 + cg21 * cg26 * cg34 * cg5 * cg6 - cg21 * cg26 *
cg35 * cg4 * cg6 + cg21 * cg28 * cg30 * cg5 * cg8 - cg21 * cg28 *
cg32 * cg5 * cg6 - cg21 * cg29 * cg30 * cg4 * cg8 + cg21 * cg29 *
cg32 * cg4 * cg6 - cg22 * cg24 * cg3 * cg35 * cg8 - cg22 * cg24 *
cg32 * cg5 * cg9 + cg22 * cg24 * cg33 * cg5 * cg8 + cg22 * cg26 *
cg3 * cg35 * cg6 + cg22 * cg26 * cg30 * cg5 * cg9 - cg22 * cg26 *
cg33 * cg5 * cg6 - cg22 * cg27 * cg30 * cg5 * cg8 + cg22 * cg27 *
cg32 * cg5 * cg6 + cg22 * cg29 * cg3 * cg30 * cg8 - cg22 * cg29 *
cg3 * cg32 * cg6 + cg23 * cg24 * cg3 * cg34 * cg8 + cg23 * cg24 *
cg32 * cg4 * cg9 - cg23 * cg24 * cg33 * cg4 * cg8 - cg23 * cg26 *
cg3 * cg34 * cg6 - cg23 * cg26 * cg30 * cg4 * cg9 + cg23 * cg26 *
cg33 * cg4 * cg6 + cg23 * cg27 * cg30 * cg4 * cg8 - cg23 * cg27 *
cg32 * cg4 * cg6 - cg23 * cg28 * cg3 * cg30 * cg8 + cg23 * cg28 *
cg3 * cg32 * cg6;
J_inverse_array[1][3] = cg0 * cg10 * cg14 * cg27 * cg35 - cg0 * cg10 * cg14 *
cg29 * cg33 - cg0 * cg10 * cg15 * cg26 * cg35 + cg0 * cg10 * cg15 *
cg29 * cg32 + cg0 * cg10 * cg17 * cg26 * cg33 - cg0 * cg10 * cg17 *
cg27 * cg32 - cg0 * cg11 * cg14 * cg27 * cg34 + cg0 * cg11 * cg14 *
cg28 * cg33 + cg0 * cg11 * cg15 * cg26 * cg34 - cg0 * cg11 * cg15 *
cg28 * cg32 - cg0 * cg11 * cg16 * cg26 * cg33 + cg0 * cg11 * cg16 *
cg27 * cg32 - cg0 * cg14 * cg28 * cg35 * cg9 + cg0 * cg14 * cg29 *
cg34 * cg9 + cg0 * cg15 * cg28 * cg35 * cg8 - cg0 * cg15 * cg29 *
cg34 * cg8 + cg0 * cg16 * cg26 * cg35 * cg9 - cg0 * cg16 * cg27 *
cg35 * cg8 - cg0 * cg16 * cg29 * cg32 * cg9 + cg0 * cg16 * cg29 *
cg33 * cg8 - cg0 * cg17 * cg26 * cg34 * cg9 + cg0 * cg17 * cg27 *
cg34 * cg8 + cg0 * cg17 * cg28 * cg32 * cg9 - cg0 * cg17 * cg28 *
cg33 * cg8 - cg10 * cg12 * cg2 * cg27 * cg35 + cg10 * cg12 * cg2 *
cg29 * cg33 + cg10 * cg12 * cg26 * cg3 * cg35 - cg10 * cg12 * cg26
* cg33 * cg5 + cg10 * cg12 * cg27 * cg32 * cg5 - cg10 * cg12 * cg29
* cg3 * cg32 - cg10 * cg14 * cg24 * cg3 * cg35 + cg10 * cg14 * cg24
* cg33 * cg5 - cg10 * cg14 * cg27 * cg30 * cg5 + cg10 * cg14 * cg29
* cg3 * cg30 + cg10 * cg15 * cg2 * cg24 * cg35 - cg10 * cg15 * cg2
* cg29 * cg30 - cg10 * cg15 * cg24 * cg32 * cg5 + cg10 * cg15 *
cg26 * cg30 * cg5 - cg10 * cg17 * cg2 * cg24 * cg33 + cg10 * cg17 *
cg2 * cg27 * cg30 + cg10 * cg17 * cg24 * cg3 * cg32 - cg10 * cg17 *
cg26 * cg3 * cg30 + cg11 * cg12 * cg2 * cg27 * cg34 - cg11 * cg12 *
cg2 * cg28 * cg33 - cg11 * cg12 * cg26 * cg3 * cg34 + cg11 * cg12 *
cg26 * cg33 * cg4 - cg11 * cg12 * cg27 * cg32 * cg4 + cg11 * cg12 *
cg28 * cg3 * cg32 + cg11 * cg14 * cg24 * cg3 * cg34 - cg11 * cg14 *
cg24 * cg33 * cg4 + cg11 * cg14 * cg27 * cg30 * cg4 - cg11 * cg14 *
cg28 * cg3 * cg30 - cg11 * cg15 * cg2 * cg24 * cg34 + cg11 * cg15 *
cg2 * cg28 * cg30 + cg11 * cg15 * cg24 * cg32 * cg4 - cg11 * cg15 *
cg26 * cg30 * cg4 + cg11 * cg16 * cg2 * cg24 * cg33 - cg11 * cg16 *
cg2 * cg27 * cg30 - cg11 * cg16 * cg24 * cg3 * cg32 + cg11 * cg16 *
cg26 * cg3 * cg30 + cg12 * cg2 * cg28 * cg35 * cg9 - cg12 * cg2 *
cg29 * cg34 * cg9 + cg12 * cg26 * cg34 * cg5 * cg9 - cg12 * cg26 *
cg35 * cg4 * cg9 - cg12 * cg27 * cg34 * cg5 * cg8 + cg12 * cg27 *
cg35 * cg4 * cg8 - cg12 * cg28 * cg3 * cg35 * cg8 - cg12 * cg28 *
cg32 * cg5 * cg9 + cg12 * cg28 * cg33 * cg5 * cg8 + cg12 * cg29 *
cg3 * cg34 * cg8 + cg12 * cg29 * cg32 * cg4 * cg9 - cg12 * cg29 *
cg33 * cg4 * cg8 - cg14 * cg24 * cg34 * cg5 * cg9 + cg14 * cg24 *
cg35 * cg4 * cg9 + cg14 * cg27 * cg34 * cg5 * cg6 - cg14 * cg27 *
cg35 * cg4 * cg6 + cg14 * cg28 * cg3 * cg35 * cg6 + cg14 * cg28 *
cg30 * cg5 * cg9 - cg14 * cg28 * cg33 * cg5 * cg6 - cg14 * cg29 *
cg3 * cg34 * cg6 - cg14 * cg29 * cg30 * cg4 * cg9 + cg14 * cg29 *
cg33 * cg4 * cg6 - cg15 * cg2 * cg28 * cg35 * cg6 + cg15 * cg2 *
cg29 * cg34 * cg6 + cg15 * cg24 * cg34 * cg5 * cg8 - cg15 * cg24 *
cg35 * cg4 * cg8 - cg15 * cg26 * cg34 * cg5 * cg6 + cg15 * cg26 *
cg35 * cg4 * cg6 - cg15 * cg28 * cg30 * cg5 * cg8 + cg15 * cg28 *
cg32 * cg5 * cg6 + cg15 * cg29 * cg30 * cg4 * cg8 - cg15 * cg29 *
cg32 * cg4 * cg6 - cg16 * cg2 * cg24 * cg35 * cg9 + cg16 * cg2 *
cg27 * cg35 * cg6 + cg16 * cg2 * cg29 * cg30 * cg9 - cg16 * cg2 *
cg29 * cg33 * cg6 + cg16 * cg24 * cg3 * cg35 * cg8 + cg16 * cg24 *
cg32 * cg5 * cg9 - cg16 * cg24 * cg33 * cg5 * cg8 - cg16 * cg26 *
cg3 * cg35 * cg6 - cg16 * cg26 * cg30 * cg5 * cg9 + cg16 * cg26 *
cg33 * cg5 * cg6 + cg16 * cg27 * cg30 * cg5 * cg8 - cg16 * cg27 *
cg32 * cg5 * cg6 - cg16 * cg29 * cg3 * cg30 * cg8 + cg16 * cg29 *
cg3 * cg32 * cg6 + cg17 * cg2 * cg24 * cg34 * cg9 - cg17 * cg2 *
cg27 * cg34 * cg6 - cg17 * cg2 * cg28 * cg30 * cg9 + cg17 * cg2 *
cg28 * cg33 * cg6 - cg17 * cg24 * cg3 * cg34 * cg8 - cg17 * cg24 *
cg32 * cg4 * cg9 + cg17 * cg24 * cg33 * cg4 * cg8 + cg17 * cg26 *
cg3 * cg34 * cg6 + cg17 * cg26 * cg30 * cg4 * cg9 - cg17 * cg26 *
cg33 * cg4 * cg6 - cg17 * cg27 * cg30 * cg4 * cg8 + cg17 * cg27 *
cg32 * cg4 * cg6 + cg17 * cg28 * cg3 * cg30 * cg8 - cg17 * cg28 *
cg3 * cg32 * cg6;
J_inverse_array[1][4] = -cg0 * cg10 * cg14 * cg21 * cg35 + cg0 * cg10 * cg14 *
cg23 * cg33 + cg0 * cg10 * cg15 * cg20 * cg35 - cg0 * cg10 * cg15 *
cg23 * cg32 - cg0 * cg10 * cg17 * cg20 * cg33 + cg0 * cg10 * cg17 *
cg21 * cg32 + cg0 * cg11 * cg14 * cg21 * cg34 - cg0 * cg11 * cg14 *
cg22 * cg33 - cg0 * cg11 * cg15 * cg20 * cg34 + cg0 * cg11 * cg15 *
cg22 * cg32 + cg0 * cg11 * cg16 * cg20 * cg33 - cg0 * cg11 * cg16 *
cg21 * cg32 + cg0 * cg14 * cg22 * cg35 * cg9 - cg0 * cg14 * cg23 *
cg34 * cg9 - cg0 * cg15 * cg22 * cg35 * cg8 + cg0 * cg15 * cg23 *
cg34 * cg8 - cg0 * cg16 * cg20 * cg35 * cg9 + cg0 * cg16 * cg21 *
cg35 * cg8 + cg0 * cg16 * cg23 * cg32 * cg9 - cg0 * cg16 * cg23 *
cg33 * cg8 + cg0 * cg17 * cg20 * cg34 * cg9 - cg0 * cg17 * cg21 *
cg34 * cg8 - cg0 * cg17 * cg22 * cg32 * cg9 + cg0 * cg17 * cg22 *
cg33 * cg8 + cg10 * cg12 * cg2 * cg21 * cg35 - cg10 * cg12 * cg2 *
cg23 * cg33 - cg10 * cg12 * cg20 * cg3 * cg35 + cg10 * cg12 * cg20
* cg33 * cg5 - cg10 * cg12 * cg21 * cg32 * cg5 + cg10 * cg12 * cg23
* cg3 * cg32 + cg10 * cg14 * cg18 * cg3 * cg35 - cg10 * cg14 * cg18
* cg33 * cg5 + cg10 * cg14 * cg21 * cg30 * cg5 - cg10 * cg14 * cg23
* cg3 * cg30 - cg10 * cg15 * cg18 * cg2 * cg35 + cg10 * cg15 * cg18
* cg32 * cg5 + cg10 * cg15 * cg2 * cg23 * cg30 - cg10 * cg15 * cg20
* cg30 * cg5 + cg10 * cg17 * cg18 * cg2 * cg33 - cg10 * cg17 * cg18
* cg3 * cg32 - cg10 * cg17 * cg2 * cg21 * cg30 + cg10 * cg17 * cg20
* cg3 * cg30 - cg11 * cg12 * cg2 * cg21 * cg34 + cg11 * cg12 * cg2
* cg22 * cg33 + cg11 * cg12 * cg20 * cg3 * cg34 - cg11 * cg12 *
cg20 * cg33 * cg4 + cg11 * cg12 * cg21 * cg32 * cg4 - cg11 * cg12 *
cg22 * cg3 * cg32 - cg11 * cg14 * cg18 * cg3 * cg34 + cg11 * cg14 *
cg18 * cg33 * cg4 - cg11 * cg14 * cg21 * cg30 * cg4 + cg11 * cg14 *
cg22 * cg3 * cg30 + cg11 * cg15 * cg18 * cg2 * cg34 - cg11 * cg15 *
cg18 * cg32 * cg4 - cg11 * cg15 * cg2 * cg22 * cg30 + cg11 * cg15 *
cg20 * cg30 * cg4 - cg11 * cg16 * cg18 * cg2 * cg33 + cg11 * cg16 *
cg18 * cg3 * cg32 + cg11 * cg16 * cg2 * cg21 * cg30 - cg11 * cg16 *
cg20 * cg3 * cg30 - cg12 * cg2 * cg22 * cg35 * cg9 + cg12 * cg2 *
cg23 * cg34 * cg9 - cg12 * cg20 * cg34 * cg5 * cg9 + cg12 * cg20 *
cg35 * cg4 * cg9 + cg12 * cg21 * cg34 * cg5 * cg8 - cg12 * cg21 *
cg35 * cg4 * cg8 + cg12 * cg22 * cg3 * cg35 * cg8 + cg12 * cg22 *
cg32 * cg5 * cg9 - cg12 * cg22 * cg33 * cg5 * cg8 - cg12 * cg23 *
cg3 * cg34 * cg8 - cg12 * cg23 * cg32 * cg4 * cg9 + cg12 * cg23 *
cg33 * cg4 * cg8 + cg14 * cg18 * cg34 * cg5 * cg9 - cg14 * cg18 *
cg35 * cg4 * cg9 - cg14 * cg21 * cg34 * cg5 * cg6 + cg14 * cg21 *
cg35 * cg4 * cg6 - cg14 * cg22 * cg3 * cg35 * cg6 - cg14 * cg22 *
cg30 * cg5 * cg9 + cg14 * cg22 * cg33 * cg5 * cg6 + cg14 * cg23 *
cg3 * cg34 * cg6 + cg14 * cg23 * cg30 * cg4 * cg9 - cg14 * cg23 *
cg33 * cg4 * cg6 - cg15 * cg18 * cg34 * cg5 * cg8 + cg15 * cg18 *
cg35 * cg4 * cg8 + cg15 * cg2 * cg22 * cg35 * cg6 - cg15 * cg2 *
cg23 * cg34 * cg6 + cg15 * cg20 * cg34 * cg5 * cg6 - cg15 * cg20 *
cg35 * cg4 * cg6 + cg15 * cg22 * cg30 * cg5 * cg8 - cg15 * cg22 *
cg32 * cg5 * cg6 - cg15 * cg23 * cg30 * cg4 * cg8 + cg15 * cg23 *
cg32 * cg4 * cg6 + cg16 * cg18 * cg2 * cg35 * cg9 - cg16 * cg18 *
cg3 * cg35 * cg8 - cg16 * cg18 * cg32 * cg5 * cg9 + cg16 * cg18 *
cg33 * cg5 * cg8 - cg16 * cg2 * cg21 * cg35 * cg6 - cg16 * cg2 *
cg23 * cg30 * cg9 + cg16 * cg2 * cg23 * cg33 * cg6 + cg16 * cg20 *
cg3 * cg35 * cg6 + cg16 * cg20 * cg30 * cg5 * cg9 - cg16 * cg20 *
cg33 * cg5 * cg6 - cg16 * cg21 * cg30 * cg5 * cg8 + cg16 * cg21 *
cg32 * cg5 * cg6 + cg16 * cg23 * cg3 * cg30 * cg8 - cg16 * cg23 *
cg3 * cg32 * cg6 - cg17 * cg18 * cg2 * cg34 * cg9 + cg17 * cg18 *
cg3 * cg34 * cg8 + cg17 * cg18 * cg32 * cg4 * cg9 - cg17 * cg18 *
cg33 * cg4 * cg8 + cg17 * cg2 * cg21 * cg34 * cg6 + cg17 * cg2 *
cg22 * cg30 * cg9 - cg17 * cg2 * cg22 * cg33 * cg6 - cg17 * cg20 *
cg3 * cg34 * cg6 - cg17 * cg20 * cg30 * cg4 * cg9 + cg17 * cg20 *
cg33 * cg4 * cg6 + cg17 * cg21 * cg30 * cg4 * cg8 - cg17 * cg21 *
cg32 * cg4 * cg6 - cg17 * cg22 * cg3 * cg30 * cg8 + cg17 * cg22 *
cg3 * cg32 * cg6;
J_inverse_array[1][5] = cg0 * cg10 * cg14 * cg21 * cg29 - cg0 * cg10 * cg14 *
cg23 * cg27 - cg0 * cg10 * cg15 * cg20 * cg29 + cg0 * cg10 * cg15 *
cg23 * cg26 + cg0 * cg10 * cg17 * cg20 * cg27 - cg0 * cg10 * cg17 *
cg21 * cg26 - cg0 * cg11 * cg14 * cg21 * cg28 + cg0 * cg11 * cg14 *
cg22 * cg27 + cg0 * cg11 * cg15 * cg20 * cg28 - cg0 * cg11 * cg15 *
cg22 * cg26 - cg0 * cg11 * cg16 * cg20 * cg27 + cg0 * cg11 * cg16 *
cg21 * cg26 - cg0 * cg14 * cg22 * cg29 * cg9 + cg0 * cg14 * cg23 *
cg28 * cg9 + cg0 * cg15 * cg22 * cg29 * cg8 - cg0 * cg15 * cg23 *
cg28 * cg8 + cg0 * cg16 * cg20 * cg29 * cg9 - cg0 * cg16 * cg21 *
cg29 * cg8 - cg0 * cg16 * cg23 * cg26 * cg9 + cg0 * cg16 * cg23 *
cg27 * cg8 - cg0 * cg17 * cg20 * cg28 * cg9 + cg0 * cg17 * cg21 *
cg28 * cg8 + cg0 * cg17 * cg22 * cg26 * cg9 - cg0 * cg17 * cg22 *
cg27 * cg8 - cg10 * cg12 * cg2 * cg21 * cg29 + cg10 * cg12 * cg2 *
cg23 * cg27 - cg10 * cg12 * cg20 * cg27 * cg5 + cg10 * cg12 * cg20
* cg29 * cg3 + cg10 * cg12 * cg21 * cg26 * cg5 - cg10 * cg12 * cg23
* cg26 * cg3 + cg10 * cg14 * cg18 * cg27 * cg5 - cg10 * cg14 * cg18
* cg29 * cg3 - cg10 * cg14 * cg21 * cg24 * cg5 + cg10 * cg14 * cg23
* cg24 * cg3 + cg10 * cg15 * cg18 * cg2 * cg29 - cg10 * cg15 * cg18
* cg26 * cg5 - cg10 * cg15 * cg2 * cg23 * cg24 + cg10 * cg15 * cg20
* cg24 * cg5 - cg10 * cg17 * cg18 * cg2 * cg27 + cg10 * cg17 * cg18
* cg26 * cg3 + cg10 * cg17 * cg2 * cg21 * cg24 - cg10 * cg17 * cg20
* cg24 * cg3 + cg11 * cg12 * cg2 * cg21 * cg28 - cg11 * cg12 * cg2
* cg22 * cg27 + cg11 * cg12 * cg20 * cg27 * cg4 - cg11 * cg12 *
cg20 * cg28 * cg3 - cg11 * cg12 * cg21 * cg26 * cg4 + cg11 * cg12 *
cg22 * cg26 * cg3 - cg11 * cg14 * cg18 * cg27 * cg4 + cg11 * cg14 *
cg18 * cg28 * cg3 + cg11 * cg14 * cg21 * cg24 * cg4 - cg11 * cg14 *
cg22 * cg24 * cg3 - cg11 * cg15 * cg18 * cg2 * cg28 + cg11 * cg15 *
cg18 * cg26 * cg4 + cg11 * cg15 * cg2 * cg22 * cg24 - cg11 * cg15 *
cg20 * cg24 * cg4 + cg11 * cg16 * cg18 * cg2 * cg27 - cg11 * cg16 *
cg18 * cg26 * cg3 - cg11 * cg16 * cg2 * cg21 * cg24 + cg11 * cg16 *
cg20 * cg24 * cg3 + cg12 * cg2 * cg22 * cg29 * cg9 - cg12 * cg2 *
cg23 * cg28 * cg9 + cg12 * cg20 * cg28 * cg5 * cg9 - cg12 * cg20 *
cg29 * cg4 * cg9 - cg12 * cg21 * cg28 * cg5 * cg8 + cg12 * cg21 *
cg29 * cg4 * cg8 - cg12 * cg22 * cg26 * cg5 * cg9 + cg12 * cg22 *
cg27 * cg5 * cg8 - cg12 * cg22 * cg29 * cg3 * cg8 + cg12 * cg23 *
cg26 * cg4 * cg9 - cg12 * cg23 * cg27 * cg4 * cg8 + cg12 * cg23 *
cg28 * cg3 * cg8 - cg14 * cg18 * cg28 * cg5 * cg9 + cg14 * cg18 *
cg29 * cg4 * cg9 + cg14 * cg21 * cg28 * cg5 * cg6 - cg14 * cg21 *
cg29 * cg4 * cg6 + cg14 * cg22 * cg24 * cg5 * cg9 - cg14 * cg22 *
cg27 * cg5 * cg6 + cg14 * cg22 * cg29 * cg3 * cg6 - cg14 * cg23 *
cg24 * cg4 * cg9 + cg14 * cg23 * cg27 * cg4 * cg6 - cg14 * cg23 *
cg28 * cg3 * cg6 + cg15 * cg18 * cg28 * cg5 * cg8 - cg15 * cg18 *
cg29 * cg4 * cg8 - cg15 * cg2 * cg22 * cg29 * cg6 + cg15 * cg2 *
cg23 * cg28 * cg6 - cg15 * cg20 * cg28 * cg5 * cg6 + cg15 * cg20 *
cg29 * cg4 * cg6 - cg15 * cg22 * cg24 * cg5 * cg8 + cg15 * cg22 *
cg26 * cg5 * cg6 + cg15 * cg23 * cg24 * cg4 * cg8 - cg15 * cg23 *
cg26 * cg4 * cg6 - cg16 * cg18 * cg2 * cg29 * cg9 + cg16 * cg18 *
cg26 * cg5 * cg9 - cg16 * cg18 * cg27 * cg5 * cg8 + cg16 * cg18 *
cg29 * cg3 * cg8 + cg16 * cg2 * cg21 * cg29 * cg6 + cg16 * cg2 *
cg23 * cg24 * cg9 - cg16 * cg2 * cg23 * cg27 * cg6 - cg16 * cg20 *
cg24 * cg5 * cg9 + cg16 * cg20 * cg27 * cg5 * cg6 - cg16 * cg20 *
cg29 * cg3 * cg6 + cg16 * cg21 * cg24 * cg5 * cg8 - cg16 * cg21 *
cg26 * cg5 * cg6 - cg16 * cg23 * cg24 * cg3 * cg8 + cg16 * cg23 *
cg26 * cg3 * cg6 + cg17 * cg18 * cg2 * cg28 * cg9 - cg17 * cg18 *
cg26 * cg4 * cg9 + cg17 * cg18 * cg27 * cg4 * cg8 - cg17 * cg18 *
cg28 * cg3 * cg8 - cg17 * cg2 * cg21 * cg28 * cg6 - cg17 * cg2 *
cg22 * cg24 * cg9 + cg17 * cg2 * cg22 * cg27 * cg6 + cg17 * cg20 *
cg24 * cg4 * cg9 - cg17 * cg20 * cg27 * cg4 * cg6 + cg17 * cg20 *
cg28 * cg3 * cg6 - cg17 * cg21 * cg24 * cg4 * cg8 + cg17 * cg21 *
cg26 * cg4 * cg6 + cg17 * cg22 * cg24 * cg3 * cg8 - cg17 * cg22 *
cg26 * cg3 * cg6;
J_inverse_array[2][0] = -cg10 * cg12 * cg19 * cg27 * cg35 + cg10 * cg12 * cg19
* cg29 * cg33 + cg10 * cg12 * cg21 * cg25 * cg35 - cg10 * cg12 *
cg21 * cg29 * cg31 - cg10 * cg12 * cg23 * cg25 * cg33 + cg10 * cg12
* cg23 * cg27 * cg31 + cg10 * cg13 * cg18 * cg27 * cg35 - cg10 *
cg13 * cg18 * cg29 * cg33 - cg10 * cg13 * cg21 * cg24 * cg35 + cg10
* cg13 * cg21 * cg29 * cg30 + cg10 * cg13 * cg23 * cg24 * cg33 -
cg10 * cg13 * cg23 * cg27 * cg30 - cg10 * cg15 * cg18 * cg25 * cg35
+ cg10 * cg15 * cg18 * cg29 * cg31 + cg10 * cg15 * cg19 * cg24 *
cg35 - cg10 * cg15 * cg19 * cg29 * cg30 - cg10 * cg15 * cg23 * cg24
* cg31 + cg10 * cg15 * cg23 * cg25 * cg30 + cg10 * cg17 * cg18 *
cg25 * cg33 - cg10 * cg17 * cg18 * cg27 * cg31 - cg10 * cg17 * cg19
* cg24 * cg33 + cg10 * cg17 * cg19 * cg27 * cg30 + cg10 * cg17 *
cg21 * cg24 * cg31 - cg10 * cg17 * cg21 * cg25 * cg30 + cg11 * cg12
* cg19 * cg27 * cg34 - cg11 * cg12 * cg19 * cg28 * cg33 - cg11 *
cg12 * cg21 * cg25 * cg34 + cg11 * cg12 * cg21 * cg28 * cg31 + cg11
* cg12 * cg22 * cg25 * cg33 - cg11 * cg12 * cg22 * cg27 * cg31 -
cg11 * cg13 * cg18 * cg27 * cg34 + cg11 * cg13 * cg18 * cg28 * cg33
+ cg11 * cg13 * cg21 * cg24 * cg34 - cg11 * cg13 * cg21 * cg28 *
cg30 - cg11 * cg13 * cg22 * cg24 * cg33 + cg11 * cg13 * cg22 * cg27
* cg30 + cg11 * cg15 * cg18 * cg25 * cg34 - cg11 * cg15 * cg18 *
cg28 * cg31 - cg11 * cg15 * cg19 * cg24 * cg34 + cg11 * cg15 * cg19
* cg28 * cg30 + cg11 * cg15 * cg22 * cg24 * cg31 - cg11 * cg15 *
cg22 * cg25 * cg30 - cg11 * cg16 * cg18 * cg25 * cg33 + cg11 * cg16
* cg18 * cg27 * cg31 + cg11 * cg16 * cg19 * cg24 * cg33 - cg11 *
cg16 * cg19 * cg27 * cg30 - cg11 * cg16 * cg21 * cg24 * cg31 + cg11
* cg16 * cg21 * cg25 * cg30 + cg12 * cg19 * cg28 * cg35 * cg9 -
cg12 * cg19 * cg29 * cg34 * cg9 - cg12 * cg21 * cg28 * cg35 * cg7 +
cg12 * cg21 * cg29 * cg34 * cg7 - cg12 * cg22 * cg25 * cg35 * cg9 +
cg12 * cg22 * cg27 * cg35 * cg7 + cg12 * cg22 * cg29 * cg31 * cg9 -
cg12 * cg22 * cg29 * cg33 * cg7 + cg12 * cg23 * cg25 * cg34 * cg9 -
cg12 * cg23 * cg27 * cg34 * cg7 - cg12 * cg23 * cg28 * cg31 * cg9 +
cg12 * cg23 * cg28 * cg33 * cg7 - cg13 * cg18 * cg28 * cg35 * cg9 +
cg13 * cg18 * cg29 * cg34 * cg9 + cg13 * cg21 * cg28 * cg35 * cg6 -
cg13 * cg21 * cg29 * cg34 * cg6 + cg13 * cg22 * cg24 * cg35 * cg9 -
cg13 * cg22 * cg27 * cg35 * cg6 - cg13 * cg22 * cg29 * cg30 * cg9 +
cg13 * cg22 * cg29 * cg33 * cg6 - cg13 * cg23 * cg24 * cg34 * cg9 +
cg13 * cg23 * cg27 * cg34 * cg6 + cg13 * cg23 * cg28 * cg30 * cg9 -
cg13 * cg23 * cg28 * cg33 * cg6 + cg15 * cg18 * cg28 * cg35 * cg7 -
cg15 * cg18 * cg29 * cg34 * cg7 - cg15 * cg19 * cg28 * cg35 * cg6 +
cg15 * cg19 * cg29 * cg34 * cg6 - cg15 * cg22 * cg24 * cg35 * cg7 +
cg15 * cg22 * cg25 * cg35 * cg6 + cg15 * cg22 * cg29 * cg30 * cg7 -
cg15 * cg22 * cg29 * cg31 * cg6 + cg15 * cg23 * cg24 * cg34 * cg7 -
cg15 * cg23 * cg25 * cg34 * cg6 - cg15 * cg23 * cg28 * cg30 * cg7 +
cg15 * cg23 * cg28 * cg31 * cg6 + cg16 * cg18 * cg25 * cg35 * cg9 -
cg16 * cg18 * cg27 * cg35 * cg7 - cg16 * cg18 * cg29 * cg31 * cg9 +
cg16 * cg18 * cg29 * cg33 * cg7 - cg16 * cg19 * cg24 * cg35 * cg9 +
cg16 * cg19 * cg27 * cg35 * cg6 + cg16 * cg19 * cg29 * cg30 * cg9 -
cg16 * cg19 * cg29 * cg33 * cg6 + cg16 * cg21 * cg24 * cg35 * cg7 -
cg16 * cg21 * cg25 * cg35 * cg6 - cg16 * cg21 * cg29 * cg30 * cg7 +
cg16 * cg21 * cg29 * cg31 * cg6 + cg16 * cg23 * cg24 * cg31 * cg9 -
cg16 * cg23 * cg24 * cg33 * cg7 - cg16 * cg23 * cg25 * cg30 * cg9 +
cg16 * cg23 * cg25 * cg33 * cg6 + cg16 * cg23 * cg27 * cg30 * cg7 -
cg16 * cg23 * cg27 * cg31 * cg6 - cg17 * cg18 * cg25 * cg34 * cg9 +
cg17 * cg18 * cg27 * cg34 * cg7 + cg17 * cg18 * cg28 * cg31 * cg9 -
cg17 * cg18 * cg28 * cg33 * cg7 + cg17 * cg19 * cg24 * cg34 * cg9 -
cg17 * cg19 * cg27 * cg34 * cg6 - cg17 * cg19 * cg28 * cg30 * cg9 +
cg17 * cg19 * cg28 * cg33 * cg6 - cg17 * cg21 * cg24 * cg34 * cg7 +
cg17 * cg21 * cg25 * cg34 * cg6 + cg17 * cg21 * cg28 * cg30 * cg7 -
cg17 * cg21 * cg28 * cg31 * cg6 - cg17 * cg22 * cg24 * cg31 * cg9 +
cg17 * cg22 * cg24 * cg33 * cg7 + cg17 * cg22 * cg25 * cg30 * cg9 -
cg17 * cg22 * cg25 * cg33 * cg6 - cg17 * cg22 * cg27 * cg30 * cg7 +
cg17 * cg22 * cg27 * cg31 * cg6;
J_inverse_array[2][1] = -cg0 * cg13 * cg21 * cg28 * cg35 + cg0 * cg13 * cg21 *
cg29 * cg34 + cg0 * cg13 * cg22 * cg27 * cg35 - cg0 * cg13 * cg22 *
cg29 * cg33 - cg0 * cg13 * cg23 * cg27 * cg34 + cg0 * cg13 * cg23 *
cg28 * cg33 + cg0 * cg15 * cg19 * cg28 * cg35 - cg0 * cg15 * cg19 *
cg29 * cg34 - cg0 * cg15 * cg22 * cg25 * cg35 + cg0 * cg15 * cg22 *
cg29 * cg31 + cg0 * cg15 * cg23 * cg25 * cg34 - cg0 * cg15 * cg23 *
cg28 * cg31 - cg0 * cg16 * cg19 * cg27 * cg35 + cg0 * cg16 * cg19 *
cg29 * cg33 + cg0 * cg16 * cg21 * cg25 * cg35 - cg0 * cg16 * cg21 *
cg29 * cg31 - cg0 * cg16 * cg23 * cg25 * cg33 + cg0 * cg16 * cg23 *
cg27 * cg31 + cg0 * cg17 * cg19 * cg27 * cg34 - cg0 * cg17 * cg19 *
cg28 * cg33 - cg0 * cg17 * cg21 * cg25 * cg34 + cg0 * cg17 * cg21 *
cg28 * cg31 + cg0 * cg17 * cg22 * cg25 * cg33 - cg0 * cg17 * cg22 *
cg27 * cg31 + cg1 * cg12 * cg21 * cg28 * cg35 - cg1 * cg12 * cg21 *
cg29 * cg34 - cg1 * cg12 * cg22 * cg27 * cg35 + cg1 * cg12 * cg22 *
cg29 * cg33 + cg1 * cg12 * cg23 * cg27 * cg34 - cg1 * cg12 * cg23 *
cg28 * cg33 - cg1 * cg15 * cg18 * cg28 * cg35 + cg1 * cg15 * cg18 *
cg29 * cg34 + cg1 * cg15 * cg22 * cg24 * cg35 - cg1 * cg15 * cg22 *
cg29 * cg30 - cg1 * cg15 * cg23 * cg24 * cg34 + cg1 * cg15 * cg23 *
cg28 * cg30 + cg1 * cg16 * cg18 * cg27 * cg35 - cg1 * cg16 * cg18 *
cg29 * cg33 - cg1 * cg16 * cg21 * cg24 * cg35 + cg1 * cg16 * cg21 *
cg29 * cg30 + cg1 * cg16 * cg23 * cg24 * cg33 - cg1 * cg16 * cg23 *
cg27 * cg30 - cg1 * cg17 * cg18 * cg27 * cg34 + cg1 * cg17 * cg18 *
cg28 * cg33 + cg1 * cg17 * cg21 * cg24 * cg34 - cg1 * cg17 * cg21 *
cg28 * cg30 - cg1 * cg17 * cg22 * cg24 * cg33 + cg1 * cg17 * cg22 *
cg27 * cg30 - cg12 * cg19 * cg27 * cg34 * cg5 + cg12 * cg19 * cg27
* cg35 * cg4 - cg12 * cg19 * cg28 * cg3 * cg35 + cg12 * cg19 * cg28
* cg33 * cg5 + cg12 * cg19 * cg29 * cg3 * cg34 - cg12 * cg19 * cg29
* cg33 * cg4 + cg12 * cg21 * cg25 * cg34 * cg5 - cg12 * cg21 * cg25
* cg35 * cg4 - cg12 * cg21 * cg28 * cg31 * cg5 + cg12 * cg21 * cg29
* cg31 * cg4 + cg12 * cg22 * cg25 * cg3 * cg35 - cg12 * cg22 * cg25
* cg33 * cg5 + cg12 * cg22 * cg27 * cg31 * cg5 - cg12 * cg22 * cg29
* cg3 * cg31 - cg12 * cg23 * cg25 * cg3 * cg34 + cg12 * cg23 * cg25
* cg33 * cg4 - cg12 * cg23 * cg27 * cg31 * cg4 + cg12 * cg23 * cg28
* cg3 * cg31 + cg13 * cg18 * cg27 * cg34 * cg5 - cg13 * cg18 * cg27
* cg35 * cg4 + cg13 * cg18 * cg28 * cg3 * cg35 - cg13 * cg18 * cg28
* cg33 * cg5 - cg13 * cg18 * cg29 * cg3 * cg34 + cg13 * cg18 * cg29
* cg33 * cg4 - cg13 * cg21 * cg24 * cg34 * cg5 + cg13 * cg21 * cg24
* cg35 * cg4 + cg13 * cg21 * cg28 * cg30 * cg5 - cg13 * cg21 * cg29
* cg30 * cg4 - cg13 * cg22 * cg24 * cg3 * cg35 + cg13 * cg22 * cg24
* cg33 * cg5 - cg13 * cg22 * cg27 * cg30 * cg5 + cg13 * cg22 * cg29
* cg3 * cg30 + cg13 * cg23 * cg24 * cg3 * cg34 - cg13 * cg23 * cg24
* cg33 * cg4 + cg13 * cg23 * cg27 * cg30 * cg4 - cg13 * cg23 * cg28
* cg3 * cg30 - cg15 * cg18 * cg25 * cg34 * cg5 + cg15 * cg18 * cg25
* cg35 * cg4 + cg15 * cg18 * cg28 * cg31 * cg5 - cg15 * cg18 * cg29
* cg31 * cg4 + cg15 * cg19 * cg24 * cg34 * cg5 - cg15 * cg19 * cg24
* cg35 * cg4 - cg15 * cg19 * cg28 * cg30 * cg5 + cg15 * cg19 * cg29
* cg30 * cg4 - cg15 * cg22 * cg24 * cg31 * cg5 + cg15 * cg22 * cg25
* cg30 * cg5 + cg15 * cg23 * cg24 * cg31 * cg4 - cg15 * cg23 * cg25
* cg30 * cg4 - cg16 * cg18 * cg25 * cg3 * cg35 + cg16 * cg18 * cg25
* cg33 * cg5 - cg16 * cg18 * cg27 * cg31 * cg5 + cg16 * cg18 * cg29
* cg3 * cg31 + cg16 * cg19 * cg24 * cg3 * cg35 - cg16 * cg19 * cg24
* cg33 * cg5 + cg16 * cg19 * cg27 * cg30 * cg5 - cg16 * cg19 * cg29
* cg3 * cg30 + cg16 * cg21 * cg24 * cg31 * cg5 - cg16 * cg21 * cg25
* cg30 * cg5 - cg16 * cg23 * cg24 * cg3 * cg31 + cg16 * cg23 * cg25
* cg3 * cg30 + cg17 * cg18 * cg25 * cg3 * cg34 - cg17 * cg18 * cg25
* cg33 * cg4 + cg17 * cg18 * cg27 * cg31 * cg4 - cg17 * cg18 * cg28
* cg3 * cg31 - cg17 * cg19 * cg24 * cg3 * cg34 + cg17 * cg19 * cg24
* cg33 * cg4 - cg17 * cg19 * cg27 * cg30 * cg4 + cg17 * cg19 * cg28
* cg3 * cg30 - cg17 * cg21 * cg24 * cg31 * cg4 + cg17 * cg21 * cg25
* cg30 * cg4 + cg17 * cg22 * cg24 * cg3 * cg31 - cg17 * cg22 * cg25
* cg3 * cg30;
J_inverse_array[2][2] = cg0 * cg10 * cg19 * cg27 * cg35 - cg0 * cg10 * cg19 *
cg29 * cg33 - cg0 * cg10 * cg21 * cg25 * cg35 + cg0 * cg10 * cg21 *
cg29 * cg31 + cg0 * cg10 * cg23 * cg25 * cg33 - cg0 * cg10 * cg23 *
cg27 * cg31 - cg0 * cg11 * cg19 * cg27 * cg34 + cg0 * cg11 * cg19 *
cg28 * cg33 + cg0 * cg11 * cg21 * cg25 * cg34 - cg0 * cg11 * cg21 *
cg28 * cg31 - cg0 * cg11 * cg22 * cg25 * cg33 + cg0 * cg11 * cg22 *
cg27 * cg31 - cg0 * cg19 * cg28 * cg35 * cg9 + cg0 * cg19 * cg29 *
cg34 * cg9 + cg0 * cg21 * cg28 * cg35 * cg7 - cg0 * cg21 * cg29 *
cg34 * cg7 + cg0 * cg22 * cg25 * cg35 * cg9 - cg0 * cg22 * cg27 *
cg35 * cg7 - cg0 * cg22 * cg29 * cg31 * cg9 + cg0 * cg22 * cg29 *
cg33 * cg7 - cg0 * cg23 * cg25 * cg34 * cg9 + cg0 * cg23 * cg27 *
cg34 * cg7 + cg0 * cg23 * cg28 * cg31 * cg9 - cg0 * cg23 * cg28 *
cg33 * cg7 - cg1 * cg10 * cg18 * cg27 * cg35 + cg1 * cg10 * cg18 *
cg29 * cg33 + cg1 * cg10 * cg21 * cg24 * cg35 - cg1 * cg10 * cg21 *
cg29 * cg30 - cg1 * cg10 * cg23 * cg24 * cg33 + cg1 * cg10 * cg23 *
cg27 * cg30 + cg1 * cg11 * cg18 * cg27 * cg34 - cg1 * cg11 * cg18 *
cg28 * cg33 - cg1 * cg11 * cg21 * cg24 * cg34 + cg1 * cg11 * cg21 *
cg28 * cg30 + cg1 * cg11 * cg22 * cg24 * cg33 - cg1 * cg11 * cg22 *
cg27 * cg30 + cg1 * cg18 * cg28 * cg35 * cg9 - cg1 * cg18 * cg29 *
cg34 * cg9 - cg1 * cg21 * cg28 * cg35 * cg6 + cg1 * cg21 * cg29 *
cg34 * cg6 - cg1 * cg22 * cg24 * cg35 * cg9 + cg1 * cg22 * cg27 *
cg35 * cg6 + cg1 * cg22 * cg29 * cg30 * cg9 - cg1 * cg22 * cg29 *
cg33 * cg6 + cg1 * cg23 * cg24 * cg34 * cg9 - cg1 * cg23 * cg27 *
cg34 * cg6 - cg1 * cg23 * cg28 * cg30 * cg9 + cg1 * cg23 * cg28 *
cg33 * cg6 + cg10 * cg18 * cg25 * cg3 * cg35 - cg10 * cg18 * cg25 *
cg33 * cg5 + cg10 * cg18 * cg27 * cg31 * cg5 - cg10 * cg18 * cg29 *
cg3 * cg31 - cg10 * cg19 * cg24 * cg3 * cg35 + cg10 * cg19 * cg24 *
cg33 * cg5 - cg10 * cg19 * cg27 * cg30 * cg5 + cg10 * cg19 * cg29 *
cg3 * cg30 - cg10 * cg21 * cg24 * cg31 * cg5 + cg10 * cg21 * cg25 *
cg30 * cg5 + cg10 * cg23 * cg24 * cg3 * cg31 - cg10 * cg23 * cg25 *
cg3 * cg30 - cg11 * cg18 * cg25 * cg3 * cg34 + cg11 * cg18 * cg25 *
cg33 * cg4 - cg11 * cg18 * cg27 * cg31 * cg4 + cg11 * cg18 * cg28 *
cg3 * cg31 + cg11 * cg19 * cg24 * cg3 * cg34 - cg11 * cg19 * cg24 *
cg33 * cg4 + cg11 * cg19 * cg27 * cg30 * cg4 - cg11 * cg19 * cg28 *
cg3 * cg30 + cg11 * cg21 * cg24 * cg31 * cg4 - cg11 * cg21 * cg25 *
cg30 * cg4 - cg11 * cg22 * cg24 * cg3 * cg31 + cg11 * cg22 * cg25 *
cg3 * cg30 + cg18 * cg25 * cg34 * cg5 * cg9 - cg18 * cg25 * cg35 *
cg4 * cg9 - cg18 * cg27 * cg34 * cg5 * cg7 + cg18 * cg27 * cg35 *
cg4 * cg7 - cg18 * cg28 * cg3 * cg35 * cg7 - cg18 * cg28 * cg31 *
cg5 * cg9 + cg18 * cg28 * cg33 * cg5 * cg7 + cg18 * cg29 * cg3 *
cg34 * cg7 + cg18 * cg29 * cg31 * cg4 * cg9 - cg18 * cg29 * cg33 *
cg4 * cg7 - cg19 * cg24 * cg34 * cg5 * cg9 + cg19 * cg24 * cg35 *
cg4 * cg9 + cg19 * cg27 * cg34 * cg5 * cg6 - cg19 * cg27 * cg35 *
cg4 * cg6 + cg19 * cg28 * cg3 * cg35 * cg6 + cg19 * cg28 * cg30 *
cg5 * cg9 - cg19 * cg28 * cg33 * cg5 * cg6 - cg19 * cg29 * cg3 *
cg34 * cg6 - cg19 * cg29 * cg30 * cg4 * cg9 + cg19 * cg29 * cg33 *
cg4 * cg6 + cg21 * cg24 * cg34 * cg5 * cg7 - cg21 * cg24 * cg35 *
cg4 * cg7 - cg21 * cg25 * cg34 * cg5 * cg6 + cg21 * cg25 * cg35 *
cg4 * cg6 - cg21 * cg28 * cg30 * cg5 * cg7 + cg21 * cg28 * cg31 *
cg5 * cg6 + cg21 * cg29 * cg30 * cg4 * cg7 - cg21 * cg29 * cg31 *
cg4 * cg6 + cg22 * cg24 * cg3 * cg35 * cg7 + cg22 * cg24 * cg31 *
cg5 * cg9 - cg22 * cg24 * cg33 * cg5 * cg7 - cg22 * cg25 * cg3 *
cg35 * cg6 - cg22 * cg25 * cg30 * cg5 * cg9 + cg22 * cg25 * cg33 *
cg5 * cg6 + cg22 * cg27 * cg30 * cg5 * cg7 - cg22 * cg27 * cg31 *
cg5 * cg6 - cg22 * cg29 * cg3 * cg30 * cg7 + cg22 * cg29 * cg3 *
cg31 * cg6 - cg23 * cg24 * cg3 * cg34 * cg7 - cg23 * cg24 * cg31 *
cg4 * cg9 + cg23 * cg24 * cg33 * cg4 * cg7 + cg23 * cg25 * cg3 *
cg34 * cg6 + cg23 * cg25 * cg30 * cg4 * cg9 - cg23 * cg25 * cg33 *
cg4 * cg6 - cg23 * cg27 * cg30 * cg4 * cg7 + cg23 * cg27 * cg31 *
cg4 * cg6 + cg23 * cg28 * cg3 * cg30 * cg7 - cg23 * cg28 * cg3 *
cg31 * cg6;
J_inverse_array[2][3] = -cg0 * cg10 * cg13 * cg27 * cg35 + cg0 * cg10 * cg13 *
cg29 * cg33 + cg0 * cg10 * cg15 * cg25 * cg35 - cg0 * cg10 * cg15 *
cg29 * cg31 - cg0 * cg10 * cg17 * cg25 * cg33 + cg0 * cg10 * cg17 *
cg27 * cg31 + cg0 * cg11 * cg13 * cg27 * cg34 - cg0 * cg11 * cg13 *
cg28 * cg33 - cg0 * cg11 * cg15 * cg25 * cg34 + cg0 * cg11 * cg15 *
cg28 * cg31 + cg0 * cg11 * cg16 * cg25 * cg33 - cg0 * cg11 * cg16 *
cg27 * cg31 + cg0 * cg13 * cg28 * cg35 * cg9 - cg0 * cg13 * cg29 *
cg34 * cg9 - cg0 * cg15 * cg28 * cg35 * cg7 + cg0 * cg15 * cg29 *
cg34 * cg7 - cg0 * cg16 * cg25 * cg35 * cg9 + cg0 * cg16 * cg27 *
cg35 * cg7 + cg0 * cg16 * cg29 * cg31 * cg9 - cg0 * cg16 * cg29 *
cg33 * cg7 + cg0 * cg17 * cg25 * cg34 * cg9 - cg0 * cg17 * cg27 *
cg34 * cg7 - cg0 * cg17 * cg28 * cg31 * cg9 + cg0 * cg17 * cg28 *
cg33 * cg7 + cg1 * cg10 * cg12 * cg27 * cg35 - cg1 * cg10 * cg12 *
cg29 * cg33 - cg1 * cg10 * cg15 * cg24 * cg35 + cg1 * cg10 * cg15 *
cg29 * cg30 + cg1 * cg10 * cg17 * cg24 * cg33 - cg1 * cg10 * cg17 *
cg27 * cg30 - cg1 * cg11 * cg12 * cg27 * cg34 + cg1 * cg11 * cg12 *
cg28 * cg33 + cg1 * cg11 * cg15 * cg24 * cg34 - cg1 * cg11 * cg15 *
cg28 * cg30 - cg1 * cg11 * cg16 * cg24 * cg33 + cg1 * cg11 * cg16 *
cg27 * cg30 - cg1 * cg12 * cg28 * cg35 * cg9 + cg1 * cg12 * cg29 *
cg34 * cg9 + cg1 * cg15 * cg28 * cg35 * cg6 - cg1 * cg15 * cg29 *
cg34 * cg6 + cg1 * cg16 * cg24 * cg35 * cg9 - cg1 * cg16 * cg27 *
cg35 * cg6 - cg1 * cg16 * cg29 * cg30 * cg9 + cg1 * cg16 * cg29 *
cg33 * cg6 - cg1 * cg17 * cg24 * cg34 * cg9 + cg1 * cg17 * cg27 *
cg34 * cg6 + cg1 * cg17 * cg28 * cg30 * cg9 - cg1 * cg17 * cg28 *
cg33 * cg6 - cg10 * cg12 * cg25 * cg3 * cg35 + cg10 * cg12 * cg25 *
cg33 * cg5 - cg10 * cg12 * cg27 * cg31 * cg5 + cg10 * cg12 * cg29 *
cg3 * cg31 + cg10 * cg13 * cg24 * cg3 * cg35 - cg10 * cg13 * cg24 *
cg33 * cg5 + cg10 * cg13 * cg27 * cg30 * cg5 - cg10 * cg13 * cg29 *
cg3 * cg30 + cg10 * cg15 * cg24 * cg31 * cg5 - cg10 * cg15 * cg25 *
cg30 * cg5 - cg10 * cg17 * cg24 * cg3 * cg31 + cg10 * cg17 * cg25 *
cg3 * cg30 + cg11 * cg12 * cg25 * cg3 * cg34 - cg11 * cg12 * cg25 *
cg33 * cg4 + cg11 * cg12 * cg27 * cg31 * cg4 - cg11 * cg12 * cg28 *
cg3 * cg31 - cg11 * cg13 * cg24 * cg3 * cg34 + cg11 * cg13 * cg24 *
cg33 * cg4 - cg11 * cg13 * cg27 * cg30 * cg4 + cg11 * cg13 * cg28 *
cg3 * cg30 - cg11 * cg15 * cg24 * cg31 * cg4 + cg11 * cg15 * cg25 *
cg30 * cg4 + cg11 * cg16 * cg24 * cg3 * cg31 - cg11 * cg16 * cg25 *
cg3 * cg30 - cg12 * cg25 * cg34 * cg5 * cg9 + cg12 * cg25 * cg35 *
cg4 * cg9 + cg12 * cg27 * cg34 * cg5 * cg7 - cg12 * cg27 * cg35 *
cg4 * cg7 + cg12 * cg28 * cg3 * cg35 * cg7 + cg12 * cg28 * cg31 *
cg5 * cg9 - cg12 * cg28 * cg33 * cg5 * cg7 - cg12 * cg29 * cg3 *
cg34 * cg7 - cg12 * cg29 * cg31 * cg4 * cg9 + cg12 * cg29 * cg33 *
cg4 * cg7 + cg13 * cg24 * cg34 * cg5 * cg9 - cg13 * cg24 * cg35 *
cg4 * cg9 - cg13 * cg27 * cg34 * cg5 * cg6 + cg13 * cg27 * cg35 *
cg4 * cg6 - cg13 * cg28 * cg3 * cg35 * cg6 - cg13 * cg28 * cg30 *
cg5 * cg9 + cg13 * cg28 * cg33 * cg5 * cg6 + cg13 * cg29 * cg3 *
cg34 * cg6 + cg13 * cg29 * cg30 * cg4 * cg9 - cg13 * cg29 * cg33 *
cg4 * cg6 - cg15 * cg24 * cg34 * cg5 * cg7 + cg15 * cg24 * cg35 *
cg4 * cg7 + cg15 * cg25 * cg34 * cg5 * cg6 - cg15 * cg25 * cg35 *
cg4 * cg6 + cg15 * cg28 * cg30 * cg5 * cg7 - cg15 * cg28 * cg31 *
cg5 * cg6 - cg15 * cg29 * cg30 * cg4 * cg7 + cg15 * cg29 * cg31 *
cg4 * cg6 - cg16 * cg24 * cg3 * cg35 * cg7 - cg16 * cg24 * cg31 *
cg5 * cg9 + cg16 * cg24 * cg33 * cg5 * cg7 + cg16 * cg25 * cg3 *
cg35 * cg6 + cg16 * cg25 * cg30 * cg5 * cg9 - cg16 * cg25 * cg33 *
cg5 * cg6 - cg16 * cg27 * cg30 * cg5 * cg7 + cg16 * cg27 * cg31 *
cg5 * cg6 + cg16 * cg29 * cg3 * cg30 * cg7 - cg16 * cg29 * cg3 *
cg31 * cg6 + cg17 * cg24 * cg3 * cg34 * cg7 + cg17 * cg24 * cg31 *
cg4 * cg9 - cg17 * cg24 * cg33 * cg4 * cg7 - cg17 * cg25 * cg3 *
cg34 * cg6 - cg17 * cg25 * cg30 * cg4 * cg9 + cg17 * cg25 * cg33 *
cg4 * cg6 + cg17 * cg27 * cg30 * cg4 * cg7 - cg17 * cg27 * cg31 *
cg4 * cg6 - cg17 * cg28 * cg3 * cg30 * cg7 + cg17 * cg28 * cg3 *
cg31 * cg6;
J_inverse_array[2][4] = cg0 * cg10 * cg13 * cg21 * cg35 - cg0 * cg10 * cg13 *
cg23 * cg33 - cg0 * cg10 * cg15 * cg19 * cg35 + cg0 * cg10 * cg15 *
cg23 * cg31 + cg0 * cg10 * cg17 * cg19 * cg33 - cg0 * cg10 * cg17 *
cg21 * cg31 - cg0 * cg11 * cg13 * cg21 * cg34 + cg0 * cg11 * cg13 *
cg22 * cg33 + cg0 * cg11 * cg15 * cg19 * cg34 - cg0 * cg11 * cg15 *
cg22 * cg31 - cg0 * cg11 * cg16 * cg19 * cg33 + cg0 * cg11 * cg16 *
cg21 * cg31 - cg0 * cg13 * cg22 * cg35 * cg9 + cg0 * cg13 * cg23 *
cg34 * cg9 + cg0 * cg15 * cg22 * cg35 * cg7 - cg0 * cg15 * cg23 *
cg34 * cg7 + cg0 * cg16 * cg19 * cg35 * cg9 - cg0 * cg16 * cg21 *
cg35 * cg7 - cg0 * cg16 * cg23 * cg31 * cg9 + cg0 * cg16 * cg23 *
cg33 * cg7 - cg0 * cg17 * cg19 * cg34 * cg9 + cg0 * cg17 * cg21 *
cg34 * cg7 + cg0 * cg17 * cg22 * cg31 * cg9 - cg0 * cg17 * cg22 *
cg33 * cg7 - cg1 * cg10 * cg12 * cg21 * cg35 + cg1 * cg10 * cg12 *
cg23 * cg33 + cg1 * cg10 * cg15 * cg18 * cg35 - cg1 * cg10 * cg15 *
cg23 * cg30 - cg1 * cg10 * cg17 * cg18 * cg33 + cg1 * cg10 * cg17 *
cg21 * cg30 + cg1 * cg11 * cg12 * cg21 * cg34 - cg1 * cg11 * cg12 *
cg22 * cg33 - cg1 * cg11 * cg15 * cg18 * cg34 + cg1 * cg11 * cg15 *
cg22 * cg30 + cg1 * cg11 * cg16 * cg18 * cg33 - cg1 * cg11 * cg16 *
cg21 * cg30 + cg1 * cg12 * cg22 * cg35 * cg9 - cg1 * cg12 * cg23 *
cg34 * cg9 - cg1 * cg15 * cg22 * cg35 * cg6 + cg1 * cg15 * cg23 *
cg34 * cg6 - cg1 * cg16 * cg18 * cg35 * cg9 + cg1 * cg16 * cg21 *
cg35 * cg6 + cg1 * cg16 * cg23 * cg30 * cg9 - cg1 * cg16 * cg23 *
cg33 * cg6 + cg1 * cg17 * cg18 * cg34 * cg9 - cg1 * cg17 * cg21 *
cg34 * cg6 - cg1 * cg17 * cg22 * cg30 * cg9 + cg1 * cg17 * cg22 *
cg33 * cg6 + cg10 * cg12 * cg19 * cg3 * cg35 - cg10 * cg12 * cg19 *
cg33 * cg5 + cg10 * cg12 * cg21 * cg31 * cg5 - cg10 * cg12 * cg23 *
cg3 * cg31 - cg10 * cg13 * cg18 * cg3 * cg35 + cg10 * cg13 * cg18 *
cg33 * cg5 - cg10 * cg13 * cg21 * cg30 * cg5 + cg10 * cg13 * cg23 *
cg3 * cg30 - cg10 * cg15 * cg18 * cg31 * cg5 + cg10 * cg15 * cg19 *
cg30 * cg5 + cg10 * cg17 * cg18 * cg3 * cg31 - cg10 * cg17 * cg19 *
cg3 * cg30 - cg11 * cg12 * cg19 * cg3 * cg34 + cg11 * cg12 * cg19 *
cg33 * cg4 - cg11 * cg12 * cg21 * cg31 * cg4 + cg11 * cg12 * cg22 *
cg3 * cg31 + cg11 * cg13 * cg18 * cg3 * cg34 - cg11 * cg13 * cg18 *
cg33 * cg4 + cg11 * cg13 * cg21 * cg30 * cg4 - cg11 * cg13 * cg22 *
cg3 * cg30 + cg11 * cg15 * cg18 * cg31 * cg4 - cg11 * cg15 * cg19 *
cg30 * cg4 - cg11 * cg16 * cg18 * cg3 * cg31 + cg11 * cg16 * cg19 *
cg3 * cg30 + cg12 * cg19 * cg34 * cg5 * cg9 - cg12 * cg19 * cg35 *
cg4 * cg9 - cg12 * cg21 * cg34 * cg5 * cg7 + cg12 * cg21 * cg35 *
cg4 * cg7 - cg12 * cg22 * cg3 * cg35 * cg7 - cg12 * cg22 * cg31 *
cg5 * cg9 + cg12 * cg22 * cg33 * cg5 * cg7 + cg12 * cg23 * cg3 *
cg34 * cg7 + cg12 * cg23 * cg31 * cg4 * cg9 - cg12 * cg23 * cg33 *
cg4 * cg7 - cg13 * cg18 * cg34 * cg5 * cg9 + cg13 * cg18 * cg35 *
cg4 * cg9 + cg13 * cg21 * cg34 * cg5 * cg6 - cg13 * cg21 * cg35 *
cg4 * cg6 + cg13 * cg22 * cg3 * cg35 * cg6 + cg13 * cg22 * cg30 *
cg5 * cg9 - cg13 * cg22 * cg33 * cg5 * cg6 - cg13 * cg23 * cg3 *
cg34 * cg6 - cg13 * cg23 * cg30 * cg4 * cg9 + cg13 * cg23 * cg33 *
cg4 * cg6 + cg15 * cg18 * cg34 * cg5 * cg7 - cg15 * cg18 * cg35 *
cg4 * cg7 - cg15 * cg19 * cg34 * cg5 * cg6 + cg15 * cg19 * cg35 *
cg4 * cg6 - cg15 * cg22 * cg30 * cg5 * cg7 + cg15 * cg22 * cg31 *
cg5 * cg6 + cg15 * cg23 * cg30 * cg4 * cg7 - cg15 * cg23 * cg31 *
cg4 * cg6 + cg16 * cg18 * cg3 * cg35 * cg7 + cg16 * cg18 * cg31 *
cg5 * cg9 - cg16 * cg18 * cg33 * cg5 * cg7 - cg16 * cg19 * cg3 *
cg35 * cg6 - cg16 * cg19 * cg30 * cg5 * cg9 + cg16 * cg19 * cg33 *
cg5 * cg6 + cg16 * cg21 * cg30 * cg5 * cg7 - cg16 * cg21 * cg31 *
cg5 * cg6 - cg16 * cg23 * cg3 * cg30 * cg7 + cg16 * cg23 * cg3 *
cg31 * cg6 - cg17 * cg18 * cg3 * cg34 * cg7 - cg17 * cg18 * cg31 *
cg4 * cg9 + cg17 * cg18 * cg33 * cg4 * cg7 + cg17 * cg19 * cg3 *
cg34 * cg6 + cg17 * cg19 * cg30 * cg4 * cg9 - cg17 * cg19 * cg33 *
cg4 * cg6 - cg17 * cg21 * cg30 * cg4 * cg7 + cg17 * cg21 * cg31 *
cg4 * cg6 + cg17 * cg22 * cg3 * cg30 * cg7 - cg17 * cg22 * cg3 *
cg31 * cg6;
J_inverse_array[2][5] = -cg0 * cg10 * cg13 * cg21 * cg29 + cg0 * cg10 * cg13 *
cg23 * cg27 + cg0 * cg10 * cg15 * cg19 * cg29 - cg0 * cg10 * cg15 *
cg23 * cg25 - cg0 * cg10 * cg17 * cg19 * cg27 + cg0 * cg10 * cg17 *
cg21 * cg25 + cg0 * cg11 * cg13 * cg21 * cg28 - cg0 * cg11 * cg13 *
cg22 * cg27 - cg0 * cg11 * cg15 * cg19 * cg28 + cg0 * cg11 * cg15 *
cg22 * cg25 + cg0 * cg11 * cg16 * cg19 * cg27 - cg0 * cg11 * cg16 *
cg21 * cg25 + cg0 * cg13 * cg22 * cg29 * cg9 - cg0 * cg13 * cg23 *
cg28 * cg9 - cg0 * cg15 * cg22 * cg29 * cg7 + cg0 * cg15 * cg23 *
cg28 * cg7 - cg0 * cg16 * cg19 * cg29 * cg9 + cg0 * cg16 * cg21 *
cg29 * cg7 + cg0 * cg16 * cg23 * cg25 * cg9 - cg0 * cg16 * cg23 *
cg27 * cg7 + cg0 * cg17 * cg19 * cg28 * cg9 - cg0 * cg17 * cg21 *
cg28 * cg7 - cg0 * cg17 * cg22 * cg25 * cg9 + cg0 * cg17 * cg22 *
cg27 * cg7 + cg1 * cg10 * cg12 * cg21 * cg29 - cg1 * cg10 * cg12 *
cg23 * cg27 - cg1 * cg10 * cg15 * cg18 * cg29 + cg1 * cg10 * cg15 *
cg23 * cg24 + cg1 * cg10 * cg17 * cg18 * cg27 - cg1 * cg10 * cg17 *
cg21 * cg24 - cg1 * cg11 * cg12 * cg21 * cg28 + cg1 * cg11 * cg12 *
cg22 * cg27 + cg1 * cg11 * cg15 * cg18 * cg28 - cg1 * cg11 * cg15 *
cg22 * cg24 - cg1 * cg11 * cg16 * cg18 * cg27 + cg1 * cg11 * cg16 *
cg21 * cg24 - cg1 * cg12 * cg22 * cg29 * cg9 + cg1 * cg12 * cg23 *
cg28 * cg9 + cg1 * cg15 * cg22 * cg29 * cg6 - cg1 * cg15 * cg23 *
cg28 * cg6 + cg1 * cg16 * cg18 * cg29 * cg9 - cg1 * cg16 * cg21 *
cg29 * cg6 - cg1 * cg16 * cg23 * cg24 * cg9 + cg1 * cg16 * cg23 *
cg27 * cg6 - cg1 * cg17 * cg18 * cg28 * cg9 + cg1 * cg17 * cg21 *
cg28 * cg6 + cg1 * cg17 * cg22 * cg24 * cg9 - cg1 * cg17 * cg22 *
cg27 * cg6 + cg10 * cg12 * cg19 * cg27 * cg5 - cg10 * cg12 * cg19 *
cg29 * cg3 - cg10 * cg12 * cg21 * cg25 * cg5 + cg10 * cg12 * cg23 *
cg25 * cg3 - cg10 * cg13 * cg18 * cg27 * cg5 + cg10 * cg13 * cg18 *
cg29 * cg3 + cg10 * cg13 * cg21 * cg24 * cg5 - cg10 * cg13 * cg23 *
cg24 * cg3 + cg10 * cg15 * cg18 * cg25 * cg5 - cg10 * cg15 * cg19 *
cg24 * cg5 - cg10 * cg17 * cg18 * cg25 * cg3 + cg10 * cg17 * cg19 *
cg24 * cg3 - cg11 * cg12 * cg19 * cg27 * cg4 + cg11 * cg12 * cg19 *
cg28 * cg3 + cg11 * cg12 * cg21 * cg25 * cg4 - cg11 * cg12 * cg22 *
cg25 * cg3 + cg11 * cg13 * cg18 * cg27 * cg4 - cg11 * cg13 * cg18 *
cg28 * cg3 - cg11 * cg13 * cg21 * cg24 * cg4 + cg11 * cg13 * cg22 *
cg24 * cg3 - cg11 * cg15 * cg18 * cg25 * cg4 + cg11 * cg15 * cg19 *
cg24 * cg4 + cg11 * cg16 * cg18 * cg25 * cg3 - cg11 * cg16 * cg19 *
cg24 * cg3 - cg12 * cg19 * cg28 * cg5 * cg9 + cg12 * cg19 * cg29 *
cg4 * cg9 + cg12 * cg21 * cg28 * cg5 * cg7 - cg12 * cg21 * cg29 *
cg4 * cg7 + cg12 * cg22 * cg25 * cg5 * cg9 - cg12 * cg22 * cg27 *
cg5 * cg7 + cg12 * cg22 * cg29 * cg3 * cg7 - cg12 * cg23 * cg25 *
cg4 * cg9 + cg12 * cg23 * cg27 * cg4 * cg7 - cg12 * cg23 * cg28 *
cg3 * cg7 + cg13 * cg18 * cg28 * cg5 * cg9 - cg13 * cg18 * cg29 *
cg4 * cg9 - cg13 * cg21 * cg28 * cg5 * cg6 + cg13 * cg21 * cg29 *
cg4 * cg6 - cg13 * cg22 * cg24 * cg5 * cg9 + cg13 * cg22 * cg27 *
cg5 * cg6 - cg13 * cg22 * cg29 * cg3 * cg6 + cg13 * cg23 * cg24 *
cg4 * cg9 - cg13 * cg23 * cg27 * cg4 * cg6 + cg13 * cg23 * cg28 *
cg3 * cg6 - cg15 * cg18 * cg28 * cg5 * cg7 + cg15 * cg18 * cg29 *
cg4 * cg7 + cg15 * cg19 * cg28 * cg5 * cg6 - cg15 * cg19 * cg29 *
cg4 * cg6 + cg15 * cg22 * cg24 * cg5 * cg7 - cg15 * cg22 * cg25 *
cg5 * cg6 - cg15 * cg23 * cg24 * cg4 * cg7 + cg15 * cg23 * cg25 *
cg4 * cg6 - cg16 * cg18 * cg25 * cg5 * cg9 + cg16 * cg18 * cg27 *
cg5 * cg7 - cg16 * cg18 * cg29 * cg3 * cg7 + cg16 * cg19 * cg24 *
cg5 * cg9 - cg16 * cg19 * cg27 * cg5 * cg6 + cg16 * cg19 * cg29 *
cg3 * cg6 - cg16 * cg21 * cg24 * cg5 * cg7 + cg16 * cg21 * cg25 *
cg5 * cg6 + cg16 * cg23 * cg24 * cg3 * cg7 - cg16 * cg23 * cg25 *
cg3 * cg6 + cg17 * cg18 * cg25 * cg4 * cg9 - cg17 * cg18 * cg27 *
cg4 * cg7 + cg17 * cg18 * cg28 * cg3 * cg7 - cg17 * cg19 * cg24 *
cg4 * cg9 + cg17 * cg19 * cg27 * cg4 * cg6 - cg17 * cg19 * cg28 *
cg3 * cg6 + cg17 * cg21 * cg24 * cg4 * cg7 - cg17 * cg21 * cg25 *
cg4 * cg6 - cg17 * cg22 * cg24 * cg3 * cg7 + cg17 * cg22 * cg25 *
cg3 * cg6;
J_inverse_array[3][0] = cg10 * cg12 * cg19 * cg26 * cg35 - cg10 * cg12 * cg19 *
cg29 * cg32 - cg10 * cg12 * cg20 * cg25 * cg35 + cg10 * cg12 * cg20
* cg29 * cg31 + cg10 * cg12 * cg23 * cg25 * cg32 - cg10 * cg12 *
cg23 * cg26 * cg31 - cg10 * cg13 * cg18 * cg26 * cg35 + cg10 * cg13
* cg18 * cg29 * cg32 + cg10 * cg13 * cg20 * cg24 * cg35 - cg10 *
cg13 * cg20 * cg29 * cg30 - cg10 * cg13 * cg23 * cg24 * cg32 + cg10
* cg13 * cg23 * cg26 * cg30 + cg10 * cg14 * cg18 * cg25 * cg35 -
cg10 * cg14 * cg18 * cg29 * cg31 - cg10 * cg14 * cg19 * cg24 * cg35
+ cg10 * cg14 * cg19 * cg29 * cg30 + cg10 * cg14 * cg23 * cg24 *
cg31 - cg10 * cg14 * cg23 * cg25 * cg30 - cg10 * cg17 * cg18 * cg25
* cg32 + cg10 * cg17 * cg18 * cg26 * cg31 + cg10 * cg17 * cg19 *
cg24 * cg32 - cg10 * cg17 * cg19 * cg26 * cg30 - cg10 * cg17 * cg20
* cg24 * cg31 + cg10 * cg17 * cg20 * cg25 * cg30 - cg11 * cg12 *
cg19 * cg26 * cg34 + cg11 * cg12 * cg19 * cg28 * cg32 + cg11 * cg12
* cg20 * cg25 * cg34 - cg11 * cg12 * cg20 * cg28 * cg31 - cg11 *
cg12 * cg22 * cg25 * cg32 + cg11 * cg12 * cg22 * cg26 * cg31 + cg11
* cg13 * cg18 * cg26 * cg34 - cg11 * cg13 * cg18 * cg28 * cg32 -
cg11 * cg13 * cg20 * cg24 * cg34 + cg11 * cg13 * cg20 * cg28 * cg30
+ cg11 * cg13 * cg22 * cg24 * cg32 - cg11 * cg13 * cg22 * cg26 *
cg30 - cg11 * cg14 * cg18 * cg25 * cg34 + cg11 * cg14 * cg18 * cg28
* cg31 + cg11 * cg14 * cg19 * cg24 * cg34 - cg11 * cg14 * cg19 *
cg28 * cg30 - cg11 * cg14 * cg22 * cg24 * cg31 + cg11 * cg14 * cg22
* cg25 * cg30 + cg11 * cg16 * cg18 * cg25 * cg32 - cg11 * cg16 *
cg18 * cg26 * cg31 - cg11 * cg16 * cg19 * cg24 * cg32 + cg11 * cg16
* cg19 * cg26 * cg30 + cg11 * cg16 * cg20 * cg24 * cg31 - cg11 *
cg16 * cg20 * cg25 * cg30 - cg12 * cg19 * cg28 * cg35 * cg8 + cg12
* cg19 * cg29 * cg34 * cg8 + cg12 * cg20 * cg28 * cg35 * cg7 - cg12
* cg20 * cg29 * cg34 * cg7 + cg12 * cg22 * cg25 * cg35 * cg8 - cg12
* cg22 * cg26 * cg35 * cg7 - cg12 * cg22 * cg29 * cg31 * cg8 + cg12
* cg22 * cg29 * cg32 * cg7 - cg12 * cg23 * cg25 * cg34 * cg8 + cg12
* cg23 * cg26 * cg34 * cg7 + cg12 * cg23 * cg28 * cg31 * cg8 - cg12
* cg23 * cg28 * cg32 * cg7 + cg13 * cg18 * cg28 * cg35 * cg8 - cg13
* cg18 * cg29 * cg34 * cg8 - cg13 * cg20 * cg28 * cg35 * cg6 + cg13
* cg20 * cg29 * cg34 * cg6 - cg13 * cg22 * cg24 * cg35 * cg8 + cg13
* cg22 * cg26 * cg35 * cg6 + cg13 * cg22 * cg29 * cg30 * cg8 - cg13
* cg22 * cg29 * cg32 * cg6 + cg13 * cg23 * cg24 * cg34 * cg8 - cg13
* cg23 * cg26 * cg34 * cg6 - cg13 * cg23 * cg28 * cg30 * cg8 + cg13
* cg23 * cg28 * cg32 * cg6 - cg14 * cg18 * cg28 * cg35 * cg7 + cg14
* cg18 * cg29 * cg34 * cg7 + cg14 * cg19 * cg28 * cg35 * cg6 - cg14
* cg19 * cg29 * cg34 * cg6 + cg14 * cg22 * cg24 * cg35 * cg7 - cg14
* cg22 * cg25 * cg35 * cg6 - cg14 * cg22 * cg29 * cg30 * cg7 + cg14
* cg22 * cg29 * cg31 * cg6 - cg14 * cg23 * cg24 * cg34 * cg7 + cg14
* cg23 * cg25 * cg34 * cg6 + cg14 * cg23 * cg28 * cg30 * cg7 - cg14
* cg23 * cg28 * cg31 * cg6 - cg16 * cg18 * cg25 * cg35 * cg8 + cg16
* cg18 * cg26 * cg35 * cg7 + cg16 * cg18 * cg29 * cg31 * cg8 - cg16
* cg18 * cg29 * cg32 * cg7 + cg16 * cg19 * cg24 * cg35 * cg8 - cg16
* cg19 * cg26 * cg35 * cg6 - cg16 * cg19 * cg29 * cg30 * cg8 + cg16
* cg19 * cg29 * cg32 * cg6 - cg16 * cg20 * cg24 * cg35 * cg7 + cg16
* cg20 * cg25 * cg35 * cg6 + cg16 * cg20 * cg29 * cg30 * cg7 - cg16
* cg20 * cg29 * cg31 * cg6 - cg16 * cg23 * cg24 * cg31 * cg8 + cg16
* cg23 * cg24 * cg32 * cg7 + cg16 * cg23 * cg25 * cg30 * cg8 - cg16
* cg23 * cg25 * cg32 * cg6 - cg16 * cg23 * cg26 * cg30 * cg7 + cg16
* cg23 * cg26 * cg31 * cg6 + cg17 * cg18 * cg25 * cg34 * cg8 - cg17
* cg18 * cg26 * cg34 * cg7 - cg17 * cg18 * cg28 * cg31 * cg8 + cg17
* cg18 * cg28 * cg32 * cg7 - cg17 * cg19 * cg24 * cg34 * cg8 + cg17
* cg19 * cg26 * cg34 * cg6 + cg17 * cg19 * cg28 * cg30 * cg8 - cg17
* cg19 * cg28 * cg32 * cg6 + cg17 * cg20 * cg24 * cg34 * cg7 - cg17
* cg20 * cg25 * cg34 * cg6 - cg17 * cg20 * cg28 * cg30 * cg7 + cg17
* cg20 * cg28 * cg31 * cg6 + cg17 * cg22 * cg24 * cg31 * cg8 - cg17
* cg22 * cg24 * cg32 * cg7 - cg17 * cg22 * cg25 * cg30 * cg8 + cg17
* cg22 * cg25 * cg32 * cg6 + cg17 * cg22 * cg26 * cg30 * cg7 - cg17
* cg22 * cg26 * cg31 * cg6;
J_inverse_array[3][1] = cg0 * cg13 * cg20 * cg28 * cg35 - cg0 * cg13 * cg20 *
cg29 * cg34 - cg0 * cg13 * cg22 * cg26 * cg35 + cg0 * cg13 * cg22 *
cg29 * cg32 + cg0 * cg13 * cg23 * cg26 * cg34 - cg0 * cg13 * cg23 *
cg28 * cg32 - cg0 * cg14 * cg19 * cg28 * cg35 + cg0 * cg14 * cg19 *
cg29 * cg34 + cg0 * cg14 * cg22 * cg25 * cg35 - cg0 * cg14 * cg22 *
cg29 * cg31 - cg0 * cg14 * cg23 * cg25 * cg34 + cg0 * cg14 * cg23 *
cg28 * cg31 + cg0 * cg16 * cg19 * cg26 * cg35 - cg0 * cg16 * cg19 *
cg29 * cg32 - cg0 * cg16 * cg20 * cg25 * cg35 + cg0 * cg16 * cg20 *
cg29 * cg31 + cg0 * cg16 * cg23 * cg25 * cg32 - cg0 * cg16 * cg23 *
cg26 * cg31 - cg0 * cg17 * cg19 * cg26 * cg34 + cg0 * cg17 * cg19 *
cg28 * cg32 + cg0 * cg17 * cg20 * cg25 * cg34 - cg0 * cg17 * cg20 *
cg28 * cg31 - cg0 * cg17 * cg22 * cg25 * cg32 + cg0 * cg17 * cg22 *
cg26 * cg31 - cg1 * cg12 * cg20 * cg28 * cg35 + cg1 * cg12 * cg20 *
cg29 * cg34 + cg1 * cg12 * cg22 * cg26 * cg35 - cg1 * cg12 * cg22 *
cg29 * cg32 - cg1 * cg12 * cg23 * cg26 * cg34 + cg1 * cg12 * cg23 *
cg28 * cg32 + cg1 * cg14 * cg18 * cg28 * cg35 - cg1 * cg14 * cg18 *
cg29 * cg34 - cg1 * cg14 * cg22 * cg24 * cg35 + cg1 * cg14 * cg22 *
cg29 * cg30 + cg1 * cg14 * cg23 * cg24 * cg34 - cg1 * cg14 * cg23 *
cg28 * cg30 - cg1 * cg16 * cg18 * cg26 * cg35 + cg1 * cg16 * cg18 *
cg29 * cg32 + cg1 * cg16 * cg20 * cg24 * cg35 - cg1 * cg16 * cg20 *
cg29 * cg30 - cg1 * cg16 * cg23 * cg24 * cg32 + cg1 * cg16 * cg23 *
cg26 * cg30 + cg1 * cg17 * cg18 * cg26 * cg34 - cg1 * cg17 * cg18 *
cg28 * cg32 - cg1 * cg17 * cg20 * cg24 * cg34 + cg1 * cg17 * cg20 *
cg28 * cg30 + cg1 * cg17 * cg22 * cg24 * cg32 - cg1 * cg17 * cg22 *
cg26 * cg30 + cg12 * cg19 * cg2 * cg28 * cg35 - cg12 * cg19 * cg2 *
cg29 * cg34 + cg12 * cg19 * cg26 * cg34 * cg5 - cg12 * cg19 * cg26
* cg35 * cg4 - cg12 * cg19 * cg28 * cg32 * cg5 + cg12 * cg19 * cg29
* cg32 * cg4 - cg12 * cg2 * cg22 * cg25 * cg35 + cg12 * cg2 * cg22
* cg29 * cg31 + cg12 * cg2 * cg23 * cg25 * cg34 - cg12 * cg2 * cg23
* cg28 * cg31 - cg12 * cg20 * cg25 * cg34 * cg5 + cg12 * cg20 *
cg25 * cg35 * cg4 + cg12 * cg20 * cg28 * cg31 * cg5 - cg12 * cg20 *
cg29 * cg31 * cg4 + cg12 * cg22 * cg25 * cg32 * cg5 - cg12 * cg22 *
cg26 * cg31 * cg5 - cg12 * cg23 * cg25 * cg32 * cg4 + cg12 * cg23 *
cg26 * cg31 * cg4 - cg13 * cg18 * cg2 * cg28 * cg35 + cg13 * cg18 *
cg2 * cg29 * cg34 - cg13 * cg18 * cg26 * cg34 * cg5 + cg13 * cg18 *
cg26 * cg35 * cg4 + cg13 * cg18 * cg28 * cg32 * cg5 - cg13 * cg18 *
cg29 * cg32 * cg4 + cg13 * cg2 * cg22 * cg24 * cg35 - cg13 * cg2 *
cg22 * cg29 * cg30 - cg13 * cg2 * cg23 * cg24 * cg34 + cg13 * cg2 *
cg23 * cg28 * cg30 + cg13 * cg20 * cg24 * cg34 * cg5 - cg13 * cg20
* cg24 * cg35 * cg4 - cg13 * cg20 * cg28 * cg30 * cg5 + cg13 * cg20
* cg29 * cg30 * cg4 - cg13 * cg22 * cg24 * cg32 * cg5 + cg13 * cg22
* cg26 * cg30 * cg5 + cg13 * cg23 * cg24 * cg32 * cg4 - cg13 * cg23
* cg26 * cg30 * cg4 + cg14 * cg18 * cg25 * cg34 * cg5 - cg14 * cg18
* cg25 * cg35 * cg4 - cg14 * cg18 * cg28 * cg31 * cg5 + cg14 * cg18
* cg29 * cg31 * cg4 - cg14 * cg19 * cg24 * cg34 * cg5 + cg14 * cg19
* cg24 * cg35 * cg4 + cg14 * cg19 * cg28 * cg30 * cg5 - cg14 * cg19
* cg29 * cg30 * cg4 + cg14 * cg22 * cg24 * cg31 * cg5 - cg14 * cg22
* cg25 * cg30 * cg5 - cg14 * cg23 * cg24 * cg31 * cg4 + cg14 * cg23
* cg25 * cg30 * cg4 + cg16 * cg18 * cg2 * cg25 * cg35 - cg16 * cg18
* cg2 * cg29 * cg31 - cg16 * cg18 * cg25 * cg32 * cg5 + cg16 * cg18
* cg26 * cg31 * cg5 - cg16 * cg19 * cg2 * cg24 * cg35 + cg16 * cg19
* cg2 * cg29 * cg30 + cg16 * cg19 * cg24 * cg32 * cg5 - cg16 * cg19
* cg26 * cg30 * cg5 + cg16 * cg2 * cg23 * cg24 * cg31 - cg16 * cg2
* cg23 * cg25 * cg30 - cg16 * cg20 * cg24 * cg31 * cg5 + cg16 *
cg20 * cg25 * cg30 * cg5 - cg17 * cg18 * cg2 * cg25 * cg34 + cg17 *
cg18 * cg2 * cg28 * cg31 + cg17 * cg18 * cg25 * cg32 * cg4 - cg17 *
cg18 * cg26 * cg31 * cg4 + cg17 * cg19 * cg2 * cg24 * cg34 - cg17 *
cg19 * cg2 * cg28 * cg30 - cg17 * cg19 * cg24 * cg32 * cg4 + cg17 *
cg19 * cg26 * cg30 * cg4 - cg17 * cg2 * cg22 * cg24 * cg31 + cg17 *
cg2 * cg22 * cg25 * cg30 + cg17 * cg20 * cg24 * cg31 * cg4 - cg17 *
cg20 * cg25 * cg30 * cg4;
J_inverse_array[3][2] = -cg0 * cg10 * cg19 * cg26 * cg35 + cg0 * cg10 * cg19 *
cg29 * cg32 + cg0 * cg10 * cg20 * cg25 * cg35 - cg0 * cg10 * cg20 *
cg29 * cg31 - cg0 * cg10 * cg23 * cg25 * cg32 + cg0 * cg10 * cg23 *
cg26 * cg31 + cg0 * cg11 * cg19 * cg26 * cg34 - cg0 * cg11 * cg19 *
cg28 * cg32 - cg0 * cg11 * cg20 * cg25 * cg34 + cg0 * cg11 * cg20 *
cg28 * cg31 + cg0 * cg11 * cg22 * cg25 * cg32 - cg0 * cg11 * cg22 *
cg26 * cg31 + cg0 * cg19 * cg28 * cg35 * cg8 - cg0 * cg19 * cg29 *
cg34 * cg8 - cg0 * cg20 * cg28 * cg35 * cg7 + cg0 * cg20 * cg29 *
cg34 * cg7 - cg0 * cg22 * cg25 * cg35 * cg8 + cg0 * cg22 * cg26 *
cg35 * cg7 + cg0 * cg22 * cg29 * cg31 * cg8 - cg0 * cg22 * cg29 *
cg32 * cg7 + cg0 * cg23 * cg25 * cg34 * cg8 - cg0 * cg23 * cg26 *
cg34 * cg7 - cg0 * cg23 * cg28 * cg31 * cg8 + cg0 * cg23 * cg28 *
cg32 * cg7 + cg1 * cg10 * cg18 * cg26 * cg35 - cg1 * cg10 * cg18 *
cg29 * cg32 - cg1 * cg10 * cg20 * cg24 * cg35 + cg1 * cg10 * cg20 *
cg29 * cg30 + cg1 * cg10 * cg23 * cg24 * cg32 - cg1 * cg10 * cg23 *
cg26 * cg30 - cg1 * cg11 * cg18 * cg26 * cg34 + cg1 * cg11 * cg18 *
cg28 * cg32 + cg1 * cg11 * cg20 * cg24 * cg34 - cg1 * cg11 * cg20 *
cg28 * cg30 - cg1 * cg11 * cg22 * cg24 * cg32 + cg1 * cg11 * cg22 *
cg26 * cg30 - cg1 * cg18 * cg28 * cg35 * cg8 + cg1 * cg18 * cg29 *
cg34 * cg8 + cg1 * cg20 * cg28 * cg35 * cg6 - cg1 * cg20 * cg29 *
cg34 * cg6 + cg1 * cg22 * cg24 * cg35 * cg8 - cg1 * cg22 * cg26 *
cg35 * cg6 - cg1 * cg22 * cg29 * cg30 * cg8 + cg1 * cg22 * cg29 *
cg32 * cg6 - cg1 * cg23 * cg24 * cg34 * cg8 + cg1 * cg23 * cg26 *
cg34 * cg6 + cg1 * cg23 * cg28 * cg30 * cg8 - cg1 * cg23 * cg28 *
cg32 * cg6 - cg10 * cg18 * cg2 * cg25 * cg35 + cg10 * cg18 * cg2 *
cg29 * cg31 + cg10 * cg18 * cg25 * cg32 * cg5 - cg10 * cg18 * cg26
* cg31 * cg5 + cg10 * cg19 * cg2 * cg24 * cg35 - cg10 * cg19 * cg2
* cg29 * cg30 - cg10 * cg19 * cg24 * cg32 * cg5 + cg10 * cg19 *
cg26 * cg30 * cg5 - cg10 * cg2 * cg23 * cg24 * cg31 + cg10 * cg2 *
cg23 * cg25 * cg30 + cg10 * cg20 * cg24 * cg31 * cg5 - cg10 * cg20
* cg25 * cg30 * cg5 + cg11 * cg18 * cg2 * cg25 * cg34 - cg11 * cg18
* cg2 * cg28 * cg31 - cg11 * cg18 * cg25 * cg32 * cg4 + cg11 * cg18
* cg26 * cg31 * cg4 - cg11 * cg19 * cg2 * cg24 * cg34 + cg11 * cg19
* cg2 * cg28 * cg30 + cg11 * cg19 * cg24 * cg32 * cg4 - cg11 * cg19
* cg26 * cg30 * cg4 + cg11 * cg2 * cg22 * cg24 * cg31 - cg11 * cg2
* cg22 * cg25 * cg30 - cg11 * cg20 * cg24 * cg31 * cg4 + cg11 *
cg20 * cg25 * cg30 * cg4 + cg18 * cg2 * cg28 * cg35 * cg7 - cg18 *
cg2 * cg29 * cg34 * cg7 - cg18 * cg25 * cg34 * cg5 * cg8 + cg18 *
cg25 * cg35 * cg4 * cg8 + cg18 * cg26 * cg34 * cg5 * cg7 - cg18 *
cg26 * cg35 * cg4 * cg7 + cg18 * cg28 * cg31 * cg5 * cg8 - cg18 *
cg28 * cg32 * cg5 * cg7 - cg18 * cg29 * cg31 * cg4 * cg8 + cg18 *
cg29 * cg32 * cg4 * cg7 - cg19 * cg2 * cg28 * cg35 * cg6 + cg19 *
cg2 * cg29 * cg34 * cg6 + cg19 * cg24 * cg34 * cg5 * cg8 - cg19 *
cg24 * cg35 * cg4 * cg8 - cg19 * cg26 * cg34 * cg5 * cg6 + cg19 *
cg26 * cg35 * cg4 * cg6 - cg19 * cg28 * cg30 * cg5 * cg8 + cg19 *
cg28 * cg32 * cg5 * cg6 + cg19 * cg29 * cg30 * cg4 * cg8 - cg19 *
cg29 * cg32 * cg4 * cg6 - cg2 * cg22 * cg24 * cg35 * cg7 + cg2 *
cg22 * cg25 * cg35 * cg6 + cg2 * cg22 * cg29 * cg30 * cg7 - cg2 *
cg22 * cg29 * cg31 * cg6 + cg2 * cg23 * cg24 * cg34 * cg7 - cg2 *
cg23 * cg25 * cg34 * cg6 - cg2 * cg23 * cg28 * cg30 * cg7 + cg2 *
cg23 * cg28 * cg31 * cg6 - cg20 * cg24 * cg34 * cg5 * cg7 + cg20 *
cg24 * cg35 * cg4 * cg7 + cg20 * cg25 * cg34 * cg5 * cg6 - cg20 *
cg25 * cg35 * cg4 * cg6 + cg20 * cg28 * cg30 * cg5 * cg7 - cg20 *
cg28 * cg31 * cg5 * cg6 - cg20 * cg29 * cg30 * cg4 * cg7 + cg20 *
cg29 * cg31 * cg4 * cg6 - cg22 * cg24 * cg31 * cg5 * cg8 + cg22 *
cg24 * cg32 * cg5 * cg7 + cg22 * cg25 * cg30 * cg5 * cg8 - cg22 *
cg25 * cg32 * cg5 * cg6 - cg22 * cg26 * cg30 * cg5 * cg7 + cg22 *
cg26 * cg31 * cg5 * cg6 + cg23 * cg24 * cg31 * cg4 * cg8 - cg23 *
cg24 * cg32 * cg4 * cg7 - cg23 * cg25 * cg30 * cg4 * cg8 + cg23 *
cg25 * cg32 * cg4 * cg6 + cg23 * cg26 * cg30 * cg4 * cg7 - cg23 *
cg26 * cg31 * cg4 * cg6;
J_inverse_array[3][3] = cg0 * cg10 * cg13 * cg26 * cg35 - cg0 * cg10 * cg13 *
cg29 * cg32 - cg0 * cg10 * cg14 * cg25 * cg35 + cg0 * cg10 * cg14 *
cg29 * cg31 + cg0 * cg10 * cg17 * cg25 * cg32 - cg0 * cg10 * cg17 *
cg26 * cg31 - cg0 * cg11 * cg13 * cg26 * cg34 + cg0 * cg11 * cg13 *
cg28 * cg32 + cg0 * cg11 * cg14 * cg25 * cg34 - cg0 * cg11 * cg14 *
cg28 * cg31 - cg0 * cg11 * cg16 * cg25 * cg32 + cg0 * cg11 * cg16 *
cg26 * cg31 - cg0 * cg13 * cg28 * cg35 * cg8 + cg0 * cg13 * cg29 *
cg34 * cg8 + cg0 * cg14 * cg28 * cg35 * cg7 - cg0 * cg14 * cg29 *
cg34 * cg7 + cg0 * cg16 * cg25 * cg35 * cg8 - cg0 * cg16 * cg26 *
cg35 * cg7 - cg0 * cg16 * cg29 * cg31 * cg8 + cg0 * cg16 * cg29 *
cg32 * cg7 - cg0 * cg17 * cg25 * cg34 * cg8 + cg0 * cg17 * cg26 *
cg34 * cg7 + cg0 * cg17 * cg28 * cg31 * cg8 - cg0 * cg17 * cg28 *
cg32 * cg7 - cg1 * cg10 * cg12 * cg26 * cg35 + cg1 * cg10 * cg12 *
cg29 * cg32 + cg1 * cg10 * cg14 * cg24 * cg35 - cg1 * cg10 * cg14 *
cg29 * cg30 - cg1 * cg10 * cg17 * cg24 * cg32 + cg1 * cg10 * cg17 *
cg26 * cg30 + cg1 * cg11 * cg12 * cg26 * cg34 - cg1 * cg11 * cg12 *
cg28 * cg32 - cg1 * cg11 * cg14 * cg24 * cg34 + cg1 * cg11 * cg14 *
cg28 * cg30 + cg1 * cg11 * cg16 * cg24 * cg32 - cg1 * cg11 * cg16 *
cg26 * cg30 + cg1 * cg12 * cg28 * cg35 * cg8 - cg1 * cg12 * cg29 *
cg34 * cg8 - cg1 * cg14 * cg28 * cg35 * cg6 + cg1 * cg14 * cg29 *
cg34 * cg6 - cg1 * cg16 * cg24 * cg35 * cg8 + cg1 * cg16 * cg26 *
cg35 * cg6 + cg1 * cg16 * cg29 * cg30 * cg8 - cg1 * cg16 * cg29 *
cg32 * cg6 + cg1 * cg17 * cg24 * cg34 * cg8 - cg1 * cg17 * cg26 *
cg34 * cg6 - cg1 * cg17 * cg28 * cg30 * cg8 + cg1 * cg17 * cg28 *
cg32 * cg6 + cg10 * cg12 * cg2 * cg25 * cg35 - cg10 * cg12 * cg2 *
cg29 * cg31 - cg10 * cg12 * cg25 * cg32 * cg5 + cg10 * cg12 * cg26
* cg31 * cg5 - cg10 * cg13 * cg2 * cg24 * cg35 + cg10 * cg13 * cg2
* cg29 * cg30 + cg10 * cg13 * cg24 * cg32 * cg5 - cg10 * cg13 *
cg26 * cg30 * cg5 - cg10 * cg14 * cg24 * cg31 * cg5 + cg10 * cg14 *
cg25 * cg30 * cg5 + cg10 * cg17 * cg2 * cg24 * cg31 - cg10 * cg17 *
cg2 * cg25 * cg30 - cg11 * cg12 * cg2 * cg25 * cg34 + cg11 * cg12 *
cg2 * cg28 * cg31 + cg11 * cg12 * cg25 * cg32 * cg4 - cg11 * cg12 *
cg26 * cg31 * cg4 + cg11 * cg13 * cg2 * cg24 * cg34 - cg11 * cg13 *
cg2 * cg28 * cg30 - cg11 * cg13 * cg24 * cg32 * cg4 + cg11 * cg13 *
cg26 * cg30 * cg4 + cg11 * cg14 * cg24 * cg31 * cg4 - cg11 * cg14 *
cg25 * cg30 * cg4 - cg11 * cg16 * cg2 * cg24 * cg31 + cg11 * cg16 *
cg2 * cg25 * cg30 - cg12 * cg2 * cg28 * cg35 * cg7 + cg12 * cg2 *
cg29 * cg34 * cg7 + cg12 * cg25 * cg34 * cg5 * cg8 - cg12 * cg25 *
cg35 * cg4 * cg8 - cg12 * cg26 * cg34 * cg5 * cg7 + cg12 * cg26 *
cg35 * cg4 * cg7 - cg12 * cg28 * cg31 * cg5 * cg8 + cg12 * cg28 *
cg32 * cg5 * cg7 + cg12 * cg29 * cg31 * cg4 * cg8 - cg12 * cg29 *
cg32 * cg4 * cg7 + cg13 * cg2 * cg28 * cg35 * cg6 - cg13 * cg2 *
cg29 * cg34 * cg6 - cg13 * cg24 * cg34 * cg5 * cg8 + cg13 * cg24 *
cg35 * cg4 * cg8 + cg13 * cg26 * cg34 * cg5 * cg6 - cg13 * cg26 *
cg35 * cg4 * cg6 + cg13 * cg28 * cg30 * cg5 * cg8 - cg13 * cg28 *
cg32 * cg5 * cg6 - cg13 * cg29 * cg30 * cg4 * cg8 + cg13 * cg29 *
cg32 * cg4 * cg6 + cg14 * cg24 * cg34 * cg5 * cg7 - cg14 * cg24 *
cg35 * cg4 * cg7 - cg14 * cg25 * cg34 * cg5 * cg6 + cg14 * cg25 *
cg35 * cg4 * cg6 - cg14 * cg28 * cg30 * cg5 * cg7 + cg14 * cg28 *
cg31 * cg5 * cg6 + cg14 * cg29 * cg30 * cg4 * cg7 - cg14 * cg29 *
cg31 * cg4 * cg6 + cg16 * cg2 * cg24 * cg35 * cg7 - cg16 * cg2 *
cg25 * cg35 * cg6 - cg16 * cg2 * cg29 * cg30 * cg7 + cg16 * cg2 *
cg29 * cg31 * cg6 + cg16 * cg24 * cg31 * cg5 * cg8 - cg16 * cg24 *
cg32 * cg5 * cg7 - cg16 * cg25 * cg30 * cg5 * cg8 + cg16 * cg25 *
cg32 * cg5 * cg6 + cg16 * cg26 * cg30 * cg5 * cg7 - cg16 * cg26 *
cg31 * cg5 * cg6 - cg17 * cg2 * cg24 * cg34 * cg7 + cg17 * cg2 *
cg25 * cg34 * cg6 + cg17 * cg2 * cg28 * cg30 * cg7 - cg17 * cg2 *
cg28 * cg31 * cg6 - cg17 * cg24 * cg31 * cg4 * cg8 + cg17 * cg24 *
cg32 * cg4 * cg7 + cg17 * cg25 * cg30 * cg4 * cg8 - cg17 * cg25 *
cg32 * cg4 * cg6 - cg17 * cg26 * cg30 * cg4 * cg7 + cg17 * cg26 *
cg31 * cg4 * cg6;
J_inverse_array[3][4] = -cg0 * cg10 * cg13 * cg20 * cg35 + cg0 * cg10 * cg13 *
cg23 * cg32 + cg0 * cg10 * cg14 * cg19 * cg35 - cg0 * cg10 * cg14 *
cg23 * cg31 - cg0 * cg10 * cg17 * cg19 * cg32 + cg0 * cg10 * cg17 *
cg20 * cg31 + cg0 * cg11 * cg13 * cg20 * cg34 - cg0 * cg11 * cg13 *
cg22 * cg32 - cg0 * cg11 * cg14 * cg19 * cg34 + cg0 * cg11 * cg14 *
cg22 * cg31 + cg0 * cg11 * cg16 * cg19 * cg32 - cg0 * cg11 * cg16 *
cg20 * cg31 + cg0 * cg13 * cg22 * cg35 * cg8 - cg0 * cg13 * cg23 *
cg34 * cg8 - cg0 * cg14 * cg22 * cg35 * cg7 + cg0 * cg14 * cg23 *
cg34 * cg7 - cg0 * cg16 * cg19 * cg35 * cg8 + cg0 * cg16 * cg20 *
cg35 * cg7 + cg0 * cg16 * cg23 * cg31 * cg8 - cg0 * cg16 * cg23 *
cg32 * cg7 + cg0 * cg17 * cg19 * cg34 * cg8 - cg0 * cg17 * cg20 *
cg34 * cg7 - cg0 * cg17 * cg22 * cg31 * cg8 + cg0 * cg17 * cg22 *
cg32 * cg7 + cg1 * cg10 * cg12 * cg20 * cg35 - cg1 * cg10 * cg12 *
cg23 * cg32 - cg1 * cg10 * cg14 * cg18 * cg35 + cg1 * cg10 * cg14 *
cg23 * cg30 + cg1 * cg10 * cg17 * cg18 * cg32 - cg1 * cg10 * cg17 *
cg20 * cg30 - cg1 * cg11 * cg12 * cg20 * cg34 + cg1 * cg11 * cg12 *
cg22 * cg32 + cg1 * cg11 * cg14 * cg18 * cg34 - cg1 * cg11 * cg14 *
cg22 * cg30 - cg1 * cg11 * cg16 * cg18 * cg32 + cg1 * cg11 * cg16 *
cg20 * cg30 - cg1 * cg12 * cg22 * cg35 * cg8 + cg1 * cg12 * cg23 *
cg34 * cg8 + cg1 * cg14 * cg22 * cg35 * cg6 - cg1 * cg14 * cg23 *
cg34 * cg6 + cg1 * cg16 * cg18 * cg35 * cg8 - cg1 * cg16 * cg20 *
cg35 * cg6 - cg1 * cg16 * cg23 * cg30 * cg8 + cg1 * cg16 * cg23 *
cg32 * cg6 - cg1 * cg17 * cg18 * cg34 * cg8 + cg1 * cg17 * cg20 *
cg34 * cg6 + cg1 * cg17 * cg22 * cg30 * cg8 - cg1 * cg17 * cg22 *
cg32 * cg6 - cg10 * cg12 * cg19 * cg2 * cg35 + cg10 * cg12 * cg19 *
cg32 * cg5 + cg10 * cg12 * cg2 * cg23 * cg31 - cg10 * cg12 * cg20 *
cg31 * cg5 + cg10 * cg13 * cg18 * cg2 * cg35 - cg10 * cg13 * cg18 *
cg32 * cg5 - cg10 * cg13 * cg2 * cg23 * cg30 + cg10 * cg13 * cg20 *
cg30 * cg5 + cg10 * cg14 * cg18 * cg31 * cg5 - cg10 * cg14 * cg19 *
cg30 * cg5 - cg10 * cg17 * cg18 * cg2 * cg31 + cg10 * cg17 * cg19 *
cg2 * cg30 + cg11 * cg12 * cg19 * cg2 * cg34 - cg11 * cg12 * cg19 *
cg32 * cg4 - cg11 * cg12 * cg2 * cg22 * cg31 + cg11 * cg12 * cg20 *
cg31 * cg4 - cg11 * cg13 * cg18 * cg2 * cg34 + cg11 * cg13 * cg18 *
cg32 * cg4 + cg11 * cg13 * cg2 * cg22 * cg30 - cg11 * cg13 * cg20 *
cg30 * cg4 - cg11 * cg14 * cg18 * cg31 * cg4 + cg11 * cg14 * cg19 *
cg30 * cg4 + cg11 * cg16 * cg18 * cg2 * cg31 - cg11 * cg16 * cg19 *
cg2 * cg30 - cg12 * cg19 * cg34 * cg5 * cg8 + cg12 * cg19 * cg35 *
cg4 * cg8 + cg12 * cg2 * cg22 * cg35 * cg7 - cg12 * cg2 * cg23 *
cg34 * cg7 + cg12 * cg20 * cg34 * cg5 * cg7 - cg12 * cg20 * cg35 *
cg4 * cg7 + cg12 * cg22 * cg31 * cg5 * cg8 - cg12 * cg22 * cg32 *
cg5 * cg7 - cg12 * cg23 * cg31 * cg4 * cg8 + cg12 * cg23 * cg32 *
cg4 * cg7 + cg13 * cg18 * cg34 * cg5 * cg8 - cg13 * cg18 * cg35 *
cg4 * cg8 - cg13 * cg2 * cg22 * cg35 * cg6 + cg13 * cg2 * cg23 *
cg34 * cg6 - cg13 * cg20 * cg34 * cg5 * cg6 + cg13 * cg20 * cg35 *
cg4 * cg6 - cg13 * cg22 * cg30 * cg5 * cg8 + cg13 * cg22 * cg32 *
cg5 * cg6 + cg13 * cg23 * cg30 * cg4 * cg8 - cg13 * cg23 * cg32 *
cg4 * cg6 - cg14 * cg18 * cg34 * cg5 * cg7 + cg14 * cg18 * cg35 *
cg4 * cg7 + cg14 * cg19 * cg34 * cg5 * cg6 - cg14 * cg19 * cg35 *
cg4 * cg6 + cg14 * cg22 * cg30 * cg5 * cg7 - cg14 * cg22 * cg31 *
cg5 * cg6 - cg14 * cg23 * cg30 * cg4 * cg7 + cg14 * cg23 * cg31 *
cg4 * cg6 - cg16 * cg18 * cg2 * cg35 * cg7 - cg16 * cg18 * cg31 *
cg5 * cg8 + cg16 * cg18 * cg32 * cg5 * cg7 + cg16 * cg19 * cg2 *
cg35 * cg6 + cg16 * cg19 * cg30 * cg5 * cg8 - cg16 * cg19 * cg32 *
cg5 * cg6 + cg16 * cg2 * cg23 * cg30 * cg7 - cg16 * cg2 * cg23 *
cg31 * cg6 - cg16 * cg20 * cg30 * cg5 * cg7 + cg16 * cg20 * cg31 *
cg5 * cg6 + cg17 * cg18 * cg2 * cg34 * cg7 + cg17 * cg18 * cg31 *
cg4 * cg8 - cg17 * cg18 * cg32 * cg4 * cg7 - cg17 * cg19 * cg2 *
cg34 * cg6 - cg17 * cg19 * cg30 * cg4 * cg8 + cg17 * cg19 * cg32 *
cg4 * cg6 - cg17 * cg2 * cg22 * cg30 * cg7 + cg17 * cg2 * cg22 *
cg31 * cg6 + cg17 * cg20 * cg30 * cg4 * cg7 - cg17 * cg20 * cg31 *
cg4 * cg6;
J_inverse_array[3][5] = cg0 * cg10 * cg13 * cg20 * cg29 - cg0 * cg10 * cg13 *
cg23 * cg26 - cg0 * cg10 * cg14 * cg19 * cg29 + cg0 * cg10 * cg14 *
cg23 * cg25 + cg0 * cg10 * cg17 * cg19 * cg26 - cg0 * cg10 * cg17 *
cg20 * cg25 - cg0 * cg11 * cg13 * cg20 * cg28 + cg0 * cg11 * cg13 *
cg22 * cg26 + cg0 * cg11 * cg14 * cg19 * cg28 - cg0 * cg11 * cg14 *
cg22 * cg25 - cg0 * cg11 * cg16 * cg19 * cg26 + cg0 * cg11 * cg16 *
cg20 * cg25 - cg0 * cg13 * cg22 * cg29 * cg8 + cg0 * cg13 * cg23 *
cg28 * cg8 + cg0 * cg14 * cg22 * cg29 * cg7 - cg0 * cg14 * cg23 *
cg28 * cg7 + cg0 * cg16 * cg19 * cg29 * cg8 - cg0 * cg16 * cg20 *
cg29 * cg7 - cg0 * cg16 * cg23 * cg25 * cg8 + cg0 * cg16 * cg23 *
cg26 * cg7 - cg0 * cg17 * cg19 * cg28 * cg8 + cg0 * cg17 * cg20 *
cg28 * cg7 + cg0 * cg17 * cg22 * cg25 * cg8 - cg0 * cg17 * cg22 *
cg26 * cg7 - cg1 * cg10 * cg12 * cg20 * cg29 + cg1 * cg10 * cg12 *
cg23 * cg26 + cg1 * cg10 * cg14 * cg18 * cg29 - cg1 * cg10 * cg14 *
cg23 * cg24 - cg1 * cg10 * cg17 * cg18 * cg26 + cg1 * cg10 * cg17 *
cg20 * cg24 + cg1 * cg11 * cg12 * cg20 * cg28 - cg1 * cg11 * cg12 *
cg22 * cg26 - cg1 * cg11 * cg14 * cg18 * cg28 + cg1 * cg11 * cg14 *
cg22 * cg24 + cg1 * cg11 * cg16 * cg18 * cg26 - cg1 * cg11 * cg16 *
cg20 * cg24 + cg1 * cg12 * cg22 * cg29 * cg8 - cg1 * cg12 * cg23 *
cg28 * cg8 - cg1 * cg14 * cg22 * cg29 * cg6 + cg1 * cg14 * cg23 *
cg28 * cg6 - cg1 * cg16 * cg18 * cg29 * cg8 + cg1 * cg16 * cg20 *
cg29 * cg6 + cg1 * cg16 * cg23 * cg24 * cg8 - cg1 * cg16 * cg23 *
cg26 * cg6 + cg1 * cg17 * cg18 * cg28 * cg8 - cg1 * cg17 * cg20 *
cg28 * cg6 - cg1 * cg17 * cg22 * cg24 * cg8 + cg1 * cg17 * cg22 *
cg26 * cg6 + cg10 * cg12 * cg19 * cg2 * cg29 - cg10 * cg12 * cg19 *
cg26 * cg5 - cg10 * cg12 * cg2 * cg23 * cg25 + cg10 * cg12 * cg20 *
cg25 * cg5 - cg10 * cg13 * cg18 * cg2 * cg29 + cg10 * cg13 * cg18 *
cg26 * cg5 + cg10 * cg13 * cg2 * cg23 * cg24 - cg10 * cg13 * cg20 *
cg24 * cg5 - cg10 * cg14 * cg18 * cg25 * cg5 + cg10 * cg14 * cg19 *
cg24 * cg5 + cg10 * cg17 * cg18 * cg2 * cg25 - cg10 * cg17 * cg19 *
cg2 * cg24 - cg11 * cg12 * cg19 * cg2 * cg28 + cg11 * cg12 * cg19 *
cg26 * cg4 + cg11 * cg12 * cg2 * cg22 * cg25 - cg11 * cg12 * cg20 *
cg25 * cg4 + cg11 * cg13 * cg18 * cg2 * cg28 - cg11 * cg13 * cg18 *
cg26 * cg4 - cg11 * cg13 * cg2 * cg22 * cg24 + cg11 * cg13 * cg20 *
cg24 * cg4 + cg11 * cg14 * cg18 * cg25 * cg4 - cg11 * cg14 * cg19 *
cg24 * cg4 - cg11 * cg16 * cg18 * cg2 * cg25 + cg11 * cg16 * cg19 *
cg2 * cg24 + cg12 * cg19 * cg28 * cg5 * cg8 - cg12 * cg19 * cg29 *
cg4 * cg8 - cg12 * cg2 * cg22 * cg29 * cg7 + cg12 * cg2 * cg23 *
cg28 * cg7 - cg12 * cg20 * cg28 * cg5 * cg7 + cg12 * cg20 * cg29 *
cg4 * cg7 - cg12 * cg22 * cg25 * cg5 * cg8 + cg12 * cg22 * cg26 *
cg5 * cg7 + cg12 * cg23 * cg25 * cg4 * cg8 - cg12 * cg23 * cg26 *
cg4 * cg7 - cg13 * cg18 * cg28 * cg5 * cg8 + cg13 * cg18 * cg29 *
cg4 * cg8 + cg13 * cg2 * cg22 * cg29 * cg6 - cg13 * cg2 * cg23 *
cg28 * cg6 + cg13 * cg20 * cg28 * cg5 * cg6 - cg13 * cg20 * cg29 *
cg4 * cg6 + cg13 * cg22 * cg24 * cg5 * cg8 - cg13 * cg22 * cg26 *
cg5 * cg6 - cg13 * cg23 * cg24 * cg4 * cg8 + cg13 * cg23 * cg26 *
cg4 * cg6 + cg14 * cg18 * cg28 * cg5 * cg7 - cg14 * cg18 * cg29 *
cg4 * cg7 - cg14 * cg19 * cg28 * cg5 * cg6 + cg14 * cg19 * cg29 *
cg4 * cg6 - cg14 * cg22 * cg24 * cg5 * cg7 + cg14 * cg22 * cg25 *
cg5 * cg6 + cg14 * cg23 * cg24 * cg4 * cg7 - cg14 * cg23 * cg25 *
cg4 * cg6 + cg16 * cg18 * cg2 * cg29 * cg7 + cg16 * cg18 * cg25 *
cg5 * cg8 - cg16 * cg18 * cg26 * cg5 * cg7 - cg16 * cg19 * cg2 *
cg29 * cg6 - cg16 * cg19 * cg24 * cg5 * cg8 + cg16 * cg19 * cg26 *
cg5 * cg6 - cg16 * cg2 * cg23 * cg24 * cg7 + cg16 * cg2 * cg23 *
cg25 * cg6 + cg16 * cg20 * cg24 * cg5 * cg7 - cg16 * cg20 * cg25 *
cg5 * cg6 - cg17 * cg18 * cg2 * cg28 * cg7 - cg17 * cg18 * cg25 *
cg4 * cg8 + cg17 * cg18 * cg26 * cg4 * cg7 + cg17 * cg19 * cg2 *
cg28 * cg6 + cg17 * cg19 * cg24 * cg4 * cg8 - cg17 * cg19 * cg26 *
cg4 * cg6 + cg17 * cg2 * cg22 * cg24 * cg7 - cg17 * cg2 * cg22 *
cg25 * cg6 - cg17 * cg20 * cg24 * cg4 * cg7 + cg17 * cg20 * cg25 *
cg4 * cg6;
J_inverse_array[4][0] = cg11 * cg12 * cg19 * cg26 * cg33 - cg11 * cg12 * cg19 *
cg27 * cg32 - cg11 * cg12 * cg20 * cg25 * cg33 + cg11 * cg12 * cg20
* cg27 * cg31 + cg11 * cg12 * cg21 * cg25 * cg32 - cg11 * cg12 *
cg21 * cg26 * cg31 - cg11 * cg13 * cg18 * cg26 * cg33 + cg11 * cg13
* cg18 * cg27 * cg32 + cg11 * cg13 * cg20 * cg24 * cg33 - cg11 *
cg13 * cg20 * cg27 * cg30 - cg11 * cg13 * cg21 * cg24 * cg32 + cg11
* cg13 * cg21 * cg26 * cg30 + cg11 * cg14 * cg18 * cg25 * cg33 -
cg11 * cg14 * cg18 * cg27 * cg31 - cg11 * cg14 * cg19 * cg24 * cg33
+ cg11 * cg14 * cg19 * cg27 * cg30 + cg11 * cg14 * cg21 * cg24 *
cg31 - cg11 * cg14 * cg21 * cg25 * cg30 - cg11 * cg15 * cg18 * cg25
* cg32 + cg11 * cg15 * cg18 * cg26 * cg31 + cg11 * cg15 * cg19 *
cg24 * cg32 - cg11 * cg15 * cg19 * cg26 * cg30 - cg11 * cg15 * cg20
* cg24 * cg31 + cg11 * cg15 * cg20 * cg25 * cg30 - cg12 * cg19 *
cg26 * cg35 * cg9 + cg12 * cg19 * cg27 * cg35 * cg8 + cg12 * cg19 *
cg29 * cg32 * cg9 - cg12 * cg19 * cg29 * cg33 * cg8 + cg12 * cg20 *
cg25 * cg35 * cg9 - cg12 * cg20 * cg27 * cg35 * cg7 - cg12 * cg20 *
cg29 * cg31 * cg9 + cg12 * cg20 * cg29 * cg33 * cg7 - cg12 * cg21 *
cg25 * cg35 * cg8 + cg12 * cg21 * cg26 * cg35 * cg7 + cg12 * cg21 *
cg29 * cg31 * cg8 - cg12 * cg21 * cg29 * cg32 * cg7 - cg12 * cg23 *
cg25 * cg32 * cg9 + cg12 * cg23 * cg25 * cg33 * cg8 + cg12 * cg23 *
cg26 * cg31 * cg9 - cg12 * cg23 * cg26 * cg33 * cg7 - cg12 * cg23 *
cg27 * cg31 * cg8 + cg12 * cg23 * cg27 * cg32 * cg7 + cg13 * cg18 *
cg26 * cg35 * cg9 - cg13 * cg18 * cg27 * cg35 * cg8 - cg13 * cg18 *
cg29 * cg32 * cg9 + cg13 * cg18 * cg29 * cg33 * cg8 - cg13 * cg20 *
cg24 * cg35 * cg9 + cg13 * cg20 * cg27 * cg35 * cg6 + cg13 * cg20 *
cg29 * cg30 * cg9 - cg13 * cg20 * cg29 * cg33 * cg6 + cg13 * cg21 *
cg24 * cg35 * cg8 - cg13 * cg21 * cg26 * cg35 * cg6 - cg13 * cg21 *
cg29 * cg30 * cg8 + cg13 * cg21 * cg29 * cg32 * cg6 + cg13 * cg23 *
cg24 * cg32 * cg9 - cg13 * cg23 * cg24 * cg33 * cg8 - cg13 * cg23 *
cg26 * cg30 * cg9 + cg13 * cg23 * cg26 * cg33 * cg6 + cg13 * cg23 *
cg27 * cg30 * cg8 - cg13 * cg23 * cg27 * cg32 * cg6 - cg14 * cg18 *
cg25 * cg35 * cg9 + cg14 * cg18 * cg27 * cg35 * cg7 + cg14 * cg18 *
cg29 * cg31 * cg9 - cg14 * cg18 * cg29 * cg33 * cg7 + cg14 * cg19 *
cg24 * cg35 * cg9 - cg14 * cg19 * cg27 * cg35 * cg6 - cg14 * cg19 *
cg29 * cg30 * cg9 + cg14 * cg19 * cg29 * cg33 * cg6 - cg14 * cg21 *
cg24 * cg35 * cg7 + cg14 * cg21 * cg25 * cg35 * cg6 + cg14 * cg21 *
cg29 * cg30 * cg7 - cg14 * cg21 * cg29 * cg31 * cg6 - cg14 * cg23 *
cg24 * cg31 * cg9 + cg14 * cg23 * cg24 * cg33 * cg7 + cg14 * cg23 *
cg25 * cg30 * cg9 - cg14 * cg23 * cg25 * cg33 * cg6 - cg14 * cg23 *
cg27 * cg30 * cg7 + cg14 * cg23 * cg27 * cg31 * cg6 + cg15 * cg18 *
cg25 * cg35 * cg8 - cg15 * cg18 * cg26 * cg35 * cg7 - cg15 * cg18 *
cg29 * cg31 * cg8 + cg15 * cg18 * cg29 * cg32 * cg7 - cg15 * cg19 *
cg24 * cg35 * cg8 + cg15 * cg19 * cg26 * cg35 * cg6 + cg15 * cg19 *
cg29 * cg30 * cg8 - cg15 * cg19 * cg29 * cg32 * cg6 + cg15 * cg20 *
cg24 * cg35 * cg7 - cg15 * cg20 * cg25 * cg35 * cg6 - cg15 * cg20 *
cg29 * cg30 * cg7 + cg15 * cg20 * cg29 * cg31 * cg6 + cg15 * cg23 *
cg24 * cg31 * cg8 - cg15 * cg23 * cg24 * cg32 * cg7 - cg15 * cg23 *
cg25 * cg30 * cg8 + cg15 * cg23 * cg25 * cg32 * cg6 + cg15 * cg23 *
cg26 * cg30 * cg7 - cg15 * cg23 * cg26 * cg31 * cg6 + cg17 * cg18 *
cg25 * cg32 * cg9 - cg17 * cg18 * cg25 * cg33 * cg8 - cg17 * cg18 *
cg26 * cg31 * cg9 + cg17 * cg18 * cg26 * cg33 * cg7 + cg17 * cg18 *
cg27 * cg31 * cg8 - cg17 * cg18 * cg27 * cg32 * cg7 - cg17 * cg19 *
cg24 * cg32 * cg9 + cg17 * cg19 * cg24 * cg33 * cg8 + cg17 * cg19 *
cg26 * cg30 * cg9 - cg17 * cg19 * cg26 * cg33 * cg6 - cg17 * cg19 *
cg27 * cg30 * cg8 + cg17 * cg19 * cg27 * cg32 * cg6 + cg17 * cg20 *
cg24 * cg31 * cg9 - cg17 * cg20 * cg24 * cg33 * cg7 - cg17 * cg20 *
cg25 * cg30 * cg9 + cg17 * cg20 * cg25 * cg33 * cg6 + cg17 * cg20 *
cg27 * cg30 * cg7 - cg17 * cg20 * cg27 * cg31 * cg6 - cg17 * cg21 *
cg24 * cg31 * cg8 + cg17 * cg21 * cg24 * cg32 * cg7 + cg17 * cg21 *
cg25 * cg30 * cg8 - cg17 * cg21 * cg25 * cg32 * cg6 - cg17 * cg21 *
cg26 * cg30 * cg7 + cg17 * cg21 * cg26 * cg31 * cg6;
J_inverse_array[4][1] = -cg0 * cg13 * cg20 * cg27 * cg35 + cg0 * cg13 * cg20 *
cg29 * cg33 + cg0 * cg13 * cg21 * cg26 * cg35 - cg0 * cg13 * cg21 *
cg29 * cg32 - cg0 * cg13 * cg23 * cg26 * cg33 + cg0 * cg13 * cg23 *
cg27 * cg32 + cg0 * cg14 * cg19 * cg27 * cg35 - cg0 * cg14 * cg19 *
cg29 * cg33 - cg0 * cg14 * cg21 * cg25 * cg35 + cg0 * cg14 * cg21 *
cg29 * cg31 + cg0 * cg14 * cg23 * cg25 * cg33 - cg0 * cg14 * cg23 *
cg27 * cg31 - cg0 * cg15 * cg19 * cg26 * cg35 + cg0 * cg15 * cg19 *
cg29 * cg32 + cg0 * cg15 * cg20 * cg25 * cg35 - cg0 * cg15 * cg20 *
cg29 * cg31 - cg0 * cg15 * cg23 * cg25 * cg32 + cg0 * cg15 * cg23 *
cg26 * cg31 + cg0 * cg17 * cg19 * cg26 * cg33 - cg0 * cg17 * cg19 *
cg27 * cg32 - cg0 * cg17 * cg20 * cg25 * cg33 + cg0 * cg17 * cg20 *
cg27 * cg31 + cg0 * cg17 * cg21 * cg25 * cg32 - cg0 * cg17 * cg21 *
cg26 * cg31 + cg1 * cg12 * cg20 * cg27 * cg35 - cg1 * cg12 * cg20 *
cg29 * cg33 - cg1 * cg12 * cg21 * cg26 * cg35 + cg1 * cg12 * cg21 *
cg29 * cg32 + cg1 * cg12 * cg23 * cg26 * cg33 - cg1 * cg12 * cg23 *
cg27 * cg32 - cg1 * cg14 * cg18 * cg27 * cg35 + cg1 * cg14 * cg18 *
cg29 * cg33 + cg1 * cg14 * cg21 * cg24 * cg35 - cg1 * cg14 * cg21 *
cg29 * cg30 - cg1 * cg14 * cg23 * cg24 * cg33 + cg1 * cg14 * cg23 *
cg27 * cg30 + cg1 * cg15 * cg18 * cg26 * cg35 - cg1 * cg15 * cg18 *
cg29 * cg32 - cg1 * cg15 * cg20 * cg24 * cg35 + cg1 * cg15 * cg20 *
cg29 * cg30 + cg1 * cg15 * cg23 * cg24 * cg32 - cg1 * cg15 * cg23 *
cg26 * cg30 - cg1 * cg17 * cg18 * cg26 * cg33 + cg1 * cg17 * cg18 *
cg27 * cg32 + cg1 * cg17 * cg20 * cg24 * cg33 - cg1 * cg17 * cg20 *
cg27 * cg30 - cg1 * cg17 * cg21 * cg24 * cg32 + cg1 * cg17 * cg21 *
cg26 * cg30 - cg12 * cg19 * cg2 * cg27 * cg35 + cg12 * cg19 * cg2 *
cg29 * cg33 + cg12 * cg19 * cg26 * cg3 * cg35 - cg12 * cg19 * cg26
* cg33 * cg5 + cg12 * cg19 * cg27 * cg32 * cg5 - cg12 * cg19 * cg29
* cg3 * cg32 + cg12 * cg2 * cg21 * cg25 * cg35 - cg12 * cg2 * cg21
* cg29 * cg31 - cg12 * cg2 * cg23 * cg25 * cg33 + cg12 * cg2 * cg23
* cg27 * cg31 - cg12 * cg20 * cg25 * cg3 * cg35 + cg12 * cg20 *
cg25 * cg33 * cg5 - cg12 * cg20 * cg27 * cg31 * cg5 + cg12 * cg20 *
cg29 * cg3 * cg31 - cg12 * cg21 * cg25 * cg32 * cg5 + cg12 * cg21 *
cg26 * cg31 * cg5 + cg12 * cg23 * cg25 * cg3 * cg32 - cg12 * cg23 *
cg26 * cg3 * cg31 + cg13 * cg18 * cg2 * cg27 * cg35 - cg13 * cg18
* cg2 * cg29 * cg33 - cg13 * cg18 * cg26 * cg3 * cg35 + cg13 * cg18
* cg26 * cg33 * cg5 - cg13 * cg18 * cg27 * cg32 * cg5 + cg13 * cg18
* cg29 * cg3 * cg32 - cg13 * cg2 * cg21 * cg24 * cg35 + cg13 * cg2
* cg21 * cg29 * cg30 + cg13 * cg2 * cg23 * cg24 * cg33 - cg13 * cg2
* cg23 * cg27 * cg30 + cg13 * cg20 * cg24 * cg3 * cg35 - cg13 *
cg20 * cg24 * cg33 * cg5 + cg13 * cg20 * cg27 * cg30 * cg5 - cg13 *
cg20 * cg29 * cg3 * cg30 + cg13 * cg21 * cg24 * cg32 * cg5 - cg13 *
cg21 * cg26 * cg30 * cg5 - cg13 * cg23 * cg24 * cg3 * cg32 + cg13 *
cg23 * cg26 * cg3 * cg30 + cg14 * cg18 * cg25 * cg3 * cg35 - cg14 *
cg18 * cg25 * cg33 * cg5 + cg14 * cg18 * cg27 * cg31 * cg5 - cg14 *
cg18 * cg29 * cg3 * cg31 - cg14 * cg19 * cg24 * cg3 * cg35 + cg14 *
cg19 * cg24 * cg33 * cg5 - cg14 * cg19 * cg27 * cg30 * cg5 + cg14 *
cg19 * cg29 * cg3 * cg30 - cg14 * cg21 * cg24 * cg31 * cg5 + cg14 *
cg21 * cg25 * cg30 * cg5 + cg14 * cg23 * cg24 * cg3 * cg31 - cg14 *
cg23 * cg25 * cg3 * cg30 - cg15 * cg18 * cg2 * cg25 * cg35 + cg15 *
cg18 * cg2 * cg29 * cg31 + cg15 * cg18 * cg25 * cg32 * cg5 - cg15 *
cg18 * cg26 * cg31 * cg5 + cg15 * cg19 * cg2 * cg24 * cg35 - cg15 *
cg19 * cg2 * cg29 * cg30 - cg15 * cg19 * cg24 * cg32 * cg5 + cg15 *
cg19 * cg26 * cg30 * cg5 - cg15 * cg2 * cg23 * cg24 * cg31 + cg15 *
cg2 * cg23 * cg25 * cg30 + cg15 * cg20 * cg24 * cg31 * cg5 - cg15 *
cg20 * cg25 * cg30 * cg5 + cg17 * cg18 * cg2 * cg25 * cg33 - cg17 *
cg18 * cg2 * cg27 * cg31 - cg17 * cg18 * cg25 * cg3 * cg32 + cg17 *
cg18 * cg26 * cg3 * cg31 - cg17 * cg19 * cg2 * cg24 * cg33 + cg17 *
cg19 * cg2 * cg27 * cg30 + cg17 * cg19 * cg24 * cg3 * cg32 - cg17 *
cg19 * cg26 * cg3 * cg30 + cg17 * cg2 * cg21 * cg24 * cg31 - cg17 *
cg2 * cg21 * cg25 * cg30 - cg17 * cg20 * cg24 * cg3 * cg31 + cg17 *
cg20 * cg25 * cg3 * cg30;
J_inverse_array[4][2] = -cg0 * cg11 * cg19 * cg26 * cg33 + cg0 * cg11 * cg19 *
cg27 * cg32 + cg0 * cg11 * cg20 * cg25 * cg33 - cg0 * cg11 * cg20 *
cg27 * cg31 - cg0 * cg11 * cg21 * cg25 * cg32 + cg0 * cg11 * cg21 *
cg26 * cg31 + cg0 * cg19 * cg26 * cg35 * cg9 - cg0 * cg19 * cg27 *
cg35 * cg8 - cg0 * cg19 * cg29 * cg32 * cg9 + cg0 * cg19 * cg29 *
cg33 * cg8 - cg0 * cg20 * cg25 * cg35 * cg9 + cg0 * cg20 * cg27 *
cg35 * cg7 + cg0 * cg20 * cg29 * cg31 * cg9 - cg0 * cg20 * cg29 *
cg33 * cg7 + cg0 * cg21 * cg25 * cg35 * cg8 - cg0 * cg21 * cg26 *
cg35 * cg7 - cg0 * cg21 * cg29 * cg31 * cg8 + cg0 * cg21 * cg29 *
cg32 * cg7 + cg0 * cg23 * cg25 * cg32 * cg9 - cg0 * cg23 * cg25 *
cg33 * cg8 - cg0 * cg23 * cg26 * cg31 * cg9 + cg0 * cg23 * cg26 *
cg33 * cg7 + cg0 * cg23 * cg27 * cg31 * cg8 - cg0 * cg23 * cg27 *
cg32 * cg7 + cg1 * cg11 * cg18 * cg26 * cg33 - cg1 * cg11 * cg18 *
cg27 * cg32 - cg1 * cg11 * cg20 * cg24 * cg33 + cg1 * cg11 * cg20 *
cg27 * cg30 + cg1 * cg11 * cg21 * cg24 * cg32 - cg1 * cg11 * cg21 *
cg26 * cg30 - cg1 * cg18 * cg26 * cg35 * cg9 + cg1 * cg18 * cg27 *
cg35 * cg8 + cg1 * cg18 * cg29 * cg32 * cg9 - cg1 * cg18 * cg29 *
cg33 * cg8 + cg1 * cg20 * cg24 * cg35 * cg9 - cg1 * cg20 * cg27 *
cg35 * cg6 - cg1 * cg20 * cg29 * cg30 * cg9 + cg1 * cg20 * cg29 *
cg33 * cg6 - cg1 * cg21 * cg24 * cg35 * cg8 + cg1 * cg21 * cg26 *
cg35 * cg6 + cg1 * cg21 * cg29 * cg30 * cg8 - cg1 * cg21 * cg29 *
cg32 * cg6 - cg1 * cg23 * cg24 * cg32 * cg9 + cg1 * cg23 * cg24 *
cg33 * cg8 + cg1 * cg23 * cg26 * cg30 * cg9 - cg1 * cg23 * cg26 *
cg33 * cg6 - cg1 * cg23 * cg27 * cg30 * cg8 + cg1 * cg23 * cg27 *
cg32 * cg6 - cg11 * cg18 * cg2 * cg25 * cg33 + cg11 * cg18 * cg2 *
cg27 * cg31 + cg11 * cg18 * cg25 * cg3 * cg32 - cg11 * cg18 * cg26
* cg3 * cg31 + cg11 * cg19 * cg2 * cg24 * cg33 - cg11 * cg19 * cg2
* cg27 * cg30 - cg11 * cg19 * cg24 * cg3 * cg32 + cg11 * cg19 *
cg26 * cg3 * cg30 - cg11 * cg2 * cg21 * cg24 * cg31 + cg11 * cg2 *
cg21 * cg25 * cg30 + cg11 * cg20 * cg24 * cg3 * cg31 - cg11 * cg20
* cg25 * cg3 * cg30 + cg18 * cg2 * cg25 * cg35 * cg9 - cg18 * cg2 *
cg27 * cg35 * cg7 - cg18 * cg2 * cg29 * cg31 * cg9 + cg18 * cg2 *
cg29 * cg33 * cg7 - cg18 * cg25 * cg3 * cg35 * cg8 - cg18 * cg25 *
cg32 * cg5 * cg9 + cg18 * cg25 * cg33 * cg5 * cg8 + cg18 * cg26 *
cg3 * cg35 * cg7 + cg18 * cg26 * cg31 * cg5 * cg9 - cg18 * cg26 *
cg33 * cg5 * cg7 - cg18 * cg27 * cg31 * cg5 * cg8 + cg18 * cg27 *
cg32 * cg5 * cg7 + cg18 * cg29 * cg3 * cg31 * cg8 - cg18 * cg29 *
cg3 * cg32 * cg7 - cg19 * cg2 * cg24 * cg35 * cg9 + cg19 * cg2 *
cg27 * cg35 * cg6 + cg19 * cg2 * cg29 * cg30 * cg9 - cg19 * cg2 *
cg29 * cg33 * cg6 + cg19 * cg24 * cg3 * cg35 * cg8 + cg19 * cg24 *
cg32 * cg5 * cg9 - cg19 * cg24 * cg33 * cg5 * cg8 - cg19 * cg26 *
cg3 * cg35 * cg6 - cg19 * cg26 * cg30 * cg5 * cg9 + cg19 * cg26 *
cg33 * cg5 * cg6 + cg19 * cg27 * cg30 * cg5 * cg8 - cg19 * cg27 *
cg32 * cg5 * cg6 - cg19 * cg29 * cg3 * cg30 * cg8 + cg19 * cg29 *
cg3 * cg32 * cg6 + cg2 * cg21 * cg24 * cg35 * cg7 - cg2 * cg21 *
cg25 * cg35 * cg6 - cg2 * cg21 * cg29 * cg30 * cg7 + cg2 * cg21 *
cg29 * cg31 * cg6 + cg2 * cg23 * cg24 * cg31 * cg9 - cg2 * cg23 *
cg24 * cg33 * cg7 - cg2 * cg23 * cg25 * cg30 * cg9 + cg2 * cg23 *
cg25 * cg33 * cg6 + cg2 * cg23 * cg27 * cg30 * cg7 - cg2 * cg23 *
cg27 * cg31 * cg6 - cg20 * cg24 * cg3 * cg35 * cg7 - cg20 * cg24 *
cg31 * cg5 * cg9 + cg20 * cg24 * cg33 * cg5 * cg7 + cg20 * cg25 *
cg3 * cg35 * cg6 + cg20 * cg25 * cg30 * cg5 * cg9 - cg20 * cg25 *
cg33 * cg5 * cg6 - cg20 * cg27 * cg30 * cg5 * cg7 + cg20 * cg27 *
cg31 * cg5 * cg6 + cg20 * cg29 * cg3 * cg30 * cg7 - cg20 * cg29 *
cg3 * cg31 * cg6 + cg21 * cg24 * cg31 * cg5 * cg8 - cg21 * cg24 *
cg32 * cg5 * cg7 - cg21 * cg25 * cg30 * cg5 * cg8 + cg21 * cg25 *
cg32 * cg5 * cg6 + cg21 * cg26 * cg30 * cg5 * cg7 - cg21 * cg26 *
cg31 * cg5 * cg6 - cg23 * cg24 * cg3 * cg31 * cg8 + cg23 * cg24 *
cg3 * cg32 * cg7 + cg23 * cg25 * cg3 * cg30 * cg8 - cg23 * cg25 *
cg3 * cg32 * cg6 - cg23 * cg26 * cg3 * cg30 * cg7 + cg23 * cg26 *
cg3 * cg31 * cg6;
J_inverse_array[4][3] = cg0 * cg11 * cg13 * cg26 * cg33 - cg0 * cg11 * cg13 *
cg27 * cg32 - cg0 * cg11 * cg14 * cg25 * cg33 + cg0 * cg11 * cg14 *
cg27 * cg31 + cg0 * cg11 * cg15 * cg25 * cg32 - cg0 * cg11 * cg15 *
cg26 * cg31 - cg0 * cg13 * cg26 * cg35 * cg9 + cg0 * cg13 * cg27 *
cg35 * cg8 + cg0 * cg13 * cg29 * cg32 * cg9 - cg0 * cg13 * cg29 *
cg33 * cg8 + cg0 * cg14 * cg25 * cg35 * cg9 - cg0 * cg14 * cg27 *
cg35 * cg7 - cg0 * cg14 * cg29 * cg31 * cg9 + cg0 * cg14 * cg29 *
cg33 * cg7 - cg0 * cg15 * cg25 * cg35 * cg8 + cg0 * cg15 * cg26 *
cg35 * cg7 + cg0 * cg15 * cg29 * cg31 * cg8 - cg0 * cg15 * cg29 *
cg32 * cg7 - cg0 * cg17 * cg25 * cg32 * cg9 + cg0 * cg17 * cg25 *
cg33 * cg8 + cg0 * cg17 * cg26 * cg31 * cg9 - cg0 * cg17 * cg26 *
cg33 * cg7 - cg0 * cg17 * cg27 * cg31 * cg8 + cg0 * cg17 * cg27 *
cg32 * cg7 - cg1 * cg11 * cg12 * cg26 * cg33 + cg1 * cg11 * cg12 *
cg27 * cg32 + cg1 * cg11 * cg14 * cg24 * cg33 - cg1 * cg11 * cg14 *
cg27 * cg30 - cg1 * cg11 * cg15 * cg24 * cg32 + cg1 * cg11 * cg15 *
cg26 * cg30 + cg1 * cg12 * cg26 * cg35 * cg9 - cg1 * cg12 * cg27 *
cg35 * cg8 - cg1 * cg12 * cg29 * cg32 * cg9 + cg1 * cg12 * cg29 *
cg33 * cg8 - cg1 * cg14 * cg24 * cg35 * cg9 + cg1 * cg14 * cg27 *
cg35 * cg6 + cg1 * cg14 * cg29 * cg30 * cg9 - cg1 * cg14 * cg29 *
cg33 * cg6 + cg1 * cg15 * cg24 * cg35 * cg8 - cg1 * cg15 * cg26 *
cg35 * cg6 - cg1 * cg15 * cg29 * cg30 * cg8 + cg1 * cg15 * cg29 *
cg32 * cg6 + cg1 * cg17 * cg24 * cg32 * cg9 - cg1 * cg17 * cg24 *
cg33 * cg8 - cg1 * cg17 * cg26 * cg30 * cg9 + cg1 * cg17 * cg26 *
cg33 * cg6 + cg1 * cg17 * cg27 * cg30 * cg8 - cg1 * cg17 * cg27 *
cg32 * cg6 + cg11 * cg12 * cg2 * cg25 * cg33 - cg11 * cg12 * cg2 *
cg27 * cg31 - cg11 * cg12 * cg25 * cg3 * cg32 + cg11 * cg12 * cg26
* cg3 * cg31 - cg11 * cg13 * cg2 * cg24 * cg33 + cg11 * cg13 * cg2
* cg27 * cg30 + cg11 * cg13 * cg24 * cg3 * cg32 - cg11 * cg13 *
cg26 * cg3 * cg30 - cg11 * cg14 * cg24 * cg3 * cg31 + cg11 * cg14 *
cg25 * cg3 * cg30 + cg11 * cg15 * cg2 * cg24 * cg31 - cg11 * cg15 *
cg2 * cg25 * cg30 - cg12 * cg2 * cg25 * cg35 * cg9 + cg12 * cg2 *
cg27 * cg35 * cg7 + cg12 * cg2 * cg29 * cg31 * cg9 - cg12 * cg2 *
cg29 * cg33 * cg7 + cg12 * cg25 * cg3 * cg35 * cg8 + cg12 * cg25 *
cg32 * cg5 * cg9 - cg12 * cg25 * cg33 * cg5 * cg8 - cg12 * cg26 *
cg3 * cg35 * cg7 - cg12 * cg26 * cg31 * cg5 * cg9 + cg12 * cg26 *
cg33 * cg5 * cg7 + cg12 * cg27 * cg31 * cg5 * cg8 - cg12 * cg27 *
cg32 * cg5 * cg7 - cg12 * cg29 * cg3 * cg31 * cg8 + cg12 * cg29 *
cg3 * cg32 * cg7 + cg13 * cg2 * cg24 * cg35 * cg9 - cg13 * cg2 *
cg27 * cg35 * cg6 - cg13 * cg2 * cg29 * cg30 * cg9 + cg13 * cg2 *
cg29 * cg33 * cg6 - cg13 * cg24 * cg3 * cg35 * cg8 - cg13 * cg24 *
cg32 * cg5 * cg9 + cg13 * cg24 * cg33 * cg5 * cg8 + cg13 * cg26 *
cg3 * cg35 * cg6 + cg13 * cg26 * cg30 * cg5 * cg9 - cg13 * cg26 *
cg33 * cg5 * cg6 - cg13 * cg27 * cg30 * cg5 * cg8 + cg13 * cg27 *
cg32 * cg5 * cg6 + cg13 * cg29 * cg3 * cg30 * cg8 - cg13 * cg29 *
cg3 * cg32 * cg6 + cg14 * cg24 * cg3 * cg35 * cg7 + cg14 * cg24 *
cg31 * cg5 * cg9 - cg14 * cg24 * cg33 * cg5 * cg7 - cg14 * cg25 *
cg3 * cg35 * cg6 - cg14 * cg25 * cg30 * cg5 * cg9 + cg14 * cg25 *
cg33 * cg5 * cg6 + cg14 * cg27 * cg30 * cg5 * cg7 - cg14 * cg27 *
cg31 * cg5 * cg6 - cg14 * cg29 * cg3 * cg30 * cg7 + cg14 * cg29 *
cg3 * cg31 * cg6 - cg15 * cg2 * cg24 * cg35 * cg7 + cg15 * cg2 *
cg25 * cg35 * cg6 + cg15 * cg2 * cg29 * cg30 * cg7 - cg15 * cg2 *
cg29 * cg31 * cg6 - cg15 * cg24 * cg31 * cg5 * cg8 + cg15 * cg24 *
cg32 * cg5 * cg7 + cg15 * cg25 * cg30 * cg5 * cg8 - cg15 * cg25 *
cg32 * cg5 * cg6 - cg15 * cg26 * cg30 * cg5 * cg7 + cg15 * cg26 *
cg31 * cg5 * cg6 - cg17 * cg2 * cg24 * cg31 * cg9 + cg17 * cg2 *
cg24 * cg33 * cg7 + cg17 * cg2 * cg25 * cg30 * cg9 - cg17 * cg2 *
cg25 * cg33 * cg6 - cg17 * cg2 * cg27 * cg30 * cg7 + cg17 * cg2 *
cg27 * cg31 * cg6 + cg17 * cg24 * cg3 * cg31 * cg8 - cg17 * cg24 *
cg3 * cg32 * cg7 - cg17 * cg25 * cg3 * cg30 * cg8 + cg17 * cg25 *
cg3 * cg32 * cg6 + cg17 * cg26 * cg3 * cg30 * cg7 - cg17 * cg26 *
cg3 * cg31 * cg6;
J_inverse_array[4][4] = -cg0 * cg11 * cg13 * cg20 * cg33 + cg0 * cg11 * cg13 *
cg21 * cg32 + cg0 * cg11 * cg14 * cg19 * cg33 - cg0 * cg11 * cg14 *
cg21 * cg31 - cg0 * cg11 * cg15 * cg19 * cg32 + cg0 * cg11 * cg15 *
cg20 * cg31 + cg0 * cg13 * cg20 * cg35 * cg9 - cg0 * cg13 * cg21 *
cg35 * cg8 - cg0 * cg13 * cg23 * cg32 * cg9 + cg0 * cg13 * cg23 *
cg33 * cg8 - cg0 * cg14 * cg19 * cg35 * cg9 + cg0 * cg14 * cg21 *
cg35 * cg7 + cg0 * cg14 * cg23 * cg31 * cg9 - cg0 * cg14 * cg23 *
cg33 * cg7 + cg0 * cg15 * cg19 * cg35 * cg8 - cg0 * cg15 * cg20 *
cg35 * cg7 - cg0 * cg15 * cg23 * cg31 * cg8 + cg0 * cg15 * cg23 *
cg32 * cg7 + cg0 * cg17 * cg19 * cg32 * cg9 - cg0 * cg17 * cg19 *
cg33 * cg8 - cg0 * cg17 * cg20 * cg31 * cg9 + cg0 * cg17 * cg20 *
cg33 * cg7 + cg0 * cg17 * cg21 * cg31 * cg8 - cg0 * cg17 * cg21 *
cg32 * cg7 + cg1 * cg11 * cg12 * cg20 * cg33 - cg1 * cg11 * cg12 *
cg21 * cg32 - cg1 * cg11 * cg14 * cg18 * cg33 + cg1 * cg11 * cg14 *
cg21 * cg30 + cg1 * cg11 * cg15 * cg18 * cg32 - cg1 * cg11 * cg15 *
cg20 * cg30 - cg1 * cg12 * cg20 * cg35 * cg9 + cg1 * cg12 * cg21 *
cg35 * cg8 + cg1 * cg12 * cg23 * cg32 * cg9 - cg1 * cg12 * cg23 *
cg33 * cg8 + cg1 * cg14 * cg18 * cg35 * cg9 - cg1 * cg14 * cg21 *
cg35 * cg6 - cg1 * cg14 * cg23 * cg30 * cg9 + cg1 * cg14 * cg23 *
cg33 * cg6 - cg1 * cg15 * cg18 * cg35 * cg8 + cg1 * cg15 * cg20 *
cg35 * cg6 + cg1 * cg15 * cg23 * cg30 * cg8 - cg1 * cg15 * cg23 *
cg32 * cg6 - cg1 * cg17 * cg18 * cg32 * cg9 + cg1 * cg17 * cg18 *
cg33 * cg8 + cg1 * cg17 * cg20 * cg30 * cg9 - cg1 * cg17 * cg20 *
cg33 * cg6 - cg1 * cg17 * cg21 * cg30 * cg8 + cg1 * cg17 * cg21 *
cg32 * cg6 - cg11 * cg12 * cg19 * cg2 * cg33 + cg11 * cg12 * cg19 *
cg3 * cg32 + cg11 * cg12 * cg2 * cg21 * cg31 - cg11 * cg12 * cg20 *
cg3 * cg31 + cg11 * cg13 * cg18 * cg2 * cg33 - cg11 * cg13 * cg18 *
cg3 * cg32 - cg11 * cg13 * cg2 * cg21 * cg30 + cg11 * cg13 * cg20 *
cg3 * cg30 + cg11 * cg14 * cg18 * cg3 * cg31 - cg11 * cg14 * cg19 *
cg3 * cg30 - cg11 * cg15 * cg18 * cg2 * cg31 + cg11 * cg15 * cg19 *
cg2 * cg30 + cg12 * cg19 * cg2 * cg35 * cg9 - cg12 * cg19 * cg3 *
cg35 * cg8 - cg12 * cg19 * cg32 * cg5 * cg9 + cg12 * cg19 * cg33 *
cg5 * cg8 - cg12 * cg2 * cg21 * cg35 * cg7 - cg12 * cg2 * cg23 *
cg31 * cg9 + cg12 * cg2 * cg23 * cg33 * cg7 + cg12 * cg20 * cg3 *
cg35 * cg7 + cg12 * cg20 * cg31 * cg5 * cg9 - cg12 * cg20 * cg33 *
cg5 * cg7 - cg12 * cg21 * cg31 * cg5 * cg8 + cg12 * cg21 * cg32 *
cg5 * cg7 + cg12 * cg23 * cg3 * cg31 * cg8 - cg12 * cg23 * cg3 *
cg32 * cg7 - cg13 * cg18 * cg2 * cg35 * cg9 + cg13 * cg18 * cg3 *
cg35 * cg8 + cg13 * cg18 * cg32 * cg5 * cg9 - cg13 * cg18 * cg33 *
cg5 * cg8 + cg13 * cg2 * cg21 * cg35 * cg6 + cg13 * cg2 * cg23 *
cg30 * cg9 - cg13 * cg2 * cg23 * cg33 * cg6 - cg13 * cg20 * cg3 *
cg35 * cg6 - cg13 * cg20 * cg30 * cg5 * cg9 + cg13 * cg20 * cg33 *
cg5 * cg6 + cg13 * cg21 * cg30 * cg5 * cg8 - cg13 * cg21 * cg32 *
cg5 * cg6 - cg13 * cg23 * cg3 * cg30 * cg8 + cg13 * cg23 * cg3 *
cg32 * cg6 - cg14 * cg18 * cg3 * cg35 * cg7 - cg14 * cg18 * cg31 *
cg5 * cg9 + cg14 * cg18 * cg33 * cg5 * cg7 + cg14 * cg19 * cg3 *
cg35 * cg6 + cg14 * cg19 * cg30 * cg5 * cg9 - cg14 * cg19 * cg33 *
cg5 * cg6 - cg14 * cg21 * cg30 * cg5 * cg7 + cg14 * cg21 * cg31 *
cg5 * cg6 + cg14 * cg23 * cg3 * cg30 * cg7 - cg14 * cg23 * cg3 *
cg31 * cg6 + cg15 * cg18 * cg2 * cg35 * cg7 + cg15 * cg18 * cg31 *
cg5 * cg8 - cg15 * cg18 * cg32 * cg5 * cg7 - cg15 * cg19 * cg2 *
cg35 * cg6 - cg15 * cg19 * cg30 * cg5 * cg8 + cg15 * cg19 * cg32 *
cg5 * cg6 - cg15 * cg2 * cg23 * cg30 * cg7 + cg15 * cg2 * cg23 *
cg31 * cg6 + cg15 * cg20 * cg30 * cg5 * cg7 - cg15 * cg20 * cg31 *
cg5 * cg6 + cg17 * cg18 * cg2 * cg31 * cg9 - cg17 * cg18 * cg2 *
cg33 * cg7 - cg17 * cg18 * cg3 * cg31 * cg8 + cg17 * cg18 * cg3 *
cg32 * cg7 - cg17 * cg19 * cg2 * cg30 * cg9 + cg17 * cg19 * cg2 *
cg33 * cg6 + cg17 * cg19 * cg3 * cg30 * cg8 - cg17 * cg19 * cg3 *
cg32 * cg6 + cg17 * cg2 * cg21 * cg30 * cg7 - cg17 * cg2 * cg21 *
cg31 * cg6 - cg17 * cg20 * cg3 * cg30 * cg7 + cg17 * cg20 * cg3 *
cg31 * cg6;
J_inverse_array[4][5] = cg0 * cg11 * cg13 * cg20 * cg27 - cg0 * cg11 * cg13 *
cg21 * cg26 - cg0 * cg11 * cg14 * cg19 * cg27 + cg0 * cg11 * cg14 *
cg21 * cg25 + cg0 * cg11 * cg15 * cg19 * cg26 - cg0 * cg11 * cg15 *
cg20 * cg25 - cg0 * cg13 * cg20 * cg29 * cg9 + cg0 * cg13 * cg21 *
cg29 * cg8 + cg0 * cg13 * cg23 * cg26 * cg9 - cg0 * cg13 * cg23 *
cg27 * cg8 + cg0 * cg14 * cg19 * cg29 * cg9 - cg0 * cg14 * cg21 *
cg29 * cg7 - cg0 * cg14 * cg23 * cg25 * cg9 + cg0 * cg14 * cg23 *
cg27 * cg7 - cg0 * cg15 * cg19 * cg29 * cg8 + cg0 * cg15 * cg20 *
cg29 * cg7 + cg0 * cg15 * cg23 * cg25 * cg8 - cg0 * cg15 * cg23 *
cg26 * cg7 - cg0 * cg17 * cg19 * cg26 * cg9 + cg0 * cg17 * cg19 *
cg27 * cg8 + cg0 * cg17 * cg20 * cg25 * cg9 - cg0 * cg17 * cg20 *
cg27 * cg7 - cg0 * cg17 * cg21 * cg25 * cg8 + cg0 * cg17 * cg21 *
cg26 * cg7 - cg1 * cg11 * cg12 * cg20 * cg27 + cg1 * cg11 * cg12 *
cg21 * cg26 + cg1 * cg11 * cg14 * cg18 * cg27 - cg1 * cg11 * cg14 *
cg21 * cg24 - cg1 * cg11 * cg15 * cg18 * cg26 + cg1 * cg11 * cg15 *
cg20 * cg24 + cg1 * cg12 * cg20 * cg29 * cg9 - cg1 * cg12 * cg21 *
cg29 * cg8 - cg1 * cg12 * cg23 * cg26 * cg9 + cg1 * cg12 * cg23 *
cg27 * cg8 - cg1 * cg14 * cg18 * cg29 * cg9 + cg1 * cg14 * cg21 *
cg29 * cg6 + cg1 * cg14 * cg23 * cg24 * cg9 - cg1 * cg14 * cg23 *
cg27 * cg6 + cg1 * cg15 * cg18 * cg29 * cg8 - cg1 * cg15 * cg20 *
cg29 * cg6 - cg1 * cg15 * cg23 * cg24 * cg8 + cg1 * cg15 * cg23 *
cg26 * cg6 + cg1 * cg17 * cg18 * cg26 * cg9 - cg1 * cg17 * cg18 *
cg27 * cg8 - cg1 * cg17 * cg20 * cg24 * cg9 + cg1 * cg17 * cg20 *
cg27 * cg6 + cg1 * cg17 * cg21 * cg24 * cg8 - cg1 * cg17 * cg21 *
cg26 * cg6 + cg11 * cg12 * cg19 * cg2 * cg27 - cg11 * cg12 * cg19 *
cg26 * cg3 - cg11 * cg12 * cg2 * cg21 * cg25 + cg11 * cg12 * cg20 *
cg25 * cg3 - cg11 * cg13 * cg18 * cg2 * cg27 + cg11 * cg13 * cg18 *
cg26 * cg3 + cg11 * cg13 * cg2 * cg21 * cg24 - cg11 * cg13 * cg20 *
cg24 * cg3 - cg11 * cg14 * cg18 * cg25 * cg3 + cg11 * cg14 * cg19 *
cg24 * cg3 + cg11 * cg15 * cg18 * cg2 * cg25 - cg11 * cg15 * cg19 *
cg2 * cg24 - cg12 * cg19 * cg2 * cg29 * cg9 + cg12 * cg19 * cg26 *
cg5 * cg9 - cg12 * cg19 * cg27 * cg5 * cg8 + cg12 * cg19 * cg29 *
cg3 * cg8 + cg12 * cg2 * cg21 * cg29 * cg7 + cg12 * cg2 * cg23 *
cg25 * cg9 - cg12 * cg2 * cg23 * cg27 * cg7 - cg12 * cg20 * cg25 *
cg5 * cg9 + cg12 * cg20 * cg27 * cg5 * cg7 - cg12 * cg20 * cg29 *
cg3 * cg7 + cg12 * cg21 * cg25 * cg5 * cg8 - cg12 * cg21 * cg26 *
cg5 * cg7 - cg12 * cg23 * cg25 * cg3 * cg8 + cg12 * cg23 * cg26 *
cg3 * cg7 + cg13 * cg18 * cg2 * cg29 * cg9 - cg13 * cg18 * cg26 *
cg5 * cg9 + cg13 * cg18 * cg27 * cg5 * cg8 - cg13 * cg18 * cg29 *
cg3 * cg8 - cg13 * cg2 * cg21 * cg29 * cg6 - cg13 * cg2 * cg23 *
cg24 * cg9 + cg13 * cg2 * cg23 * cg27 * cg6 + cg13 * cg20 * cg24 *
cg5 * cg9 - cg13 * cg20 * cg27 * cg5 * cg6 + cg13 * cg20 * cg29 *
cg3 * cg6 - cg13 * cg21 * cg24 * cg5 * cg8 + cg13 * cg21 * cg26 *
cg5 * cg6 + cg13 * cg23 * cg24 * cg3 * cg8 - cg13 * cg23 * cg26 *
cg3 * cg6 + cg14 * cg18 * cg25 * cg5 * cg9 - cg14 * cg18 * cg27 *
cg5 * cg7 + cg14 * cg18 * cg29 * cg3 * cg7 - cg14 * cg19 * cg24 *
cg5 * cg9 + cg14 * cg19 * cg27 * cg5 * cg6 - cg14 * cg19 * cg29 *
cg3 * cg6 + cg14 * cg21 * cg24 * cg5 * cg7 - cg14 * cg21 * cg25 *
cg5 * cg6 - cg14 * cg23 * cg24 * cg3 * cg7 + cg14 * cg23 * cg25 *
cg3 * cg6 - cg15 * cg18 * cg2 * cg29 * cg7 - cg15 * cg18 * cg25 *
cg5 * cg8 + cg15 * cg18 * cg26 * cg5 * cg7 + cg15 * cg19 * cg2 *
cg29 * cg6 + cg15 * cg19 * cg24 * cg5 * cg8 - cg15 * cg19 * cg26 *
cg5 * cg6 + cg15 * cg2 * cg23 * cg24 * cg7 - cg15 * cg2 * cg23 *
cg25 * cg6 - cg15 * cg20 * cg24 * cg5 * cg7 + cg15 * cg20 * cg25 *
cg5 * cg6 - cg17 * cg18 * cg2 * cg25 * cg9 + cg17 * cg18 * cg2 *
cg27 * cg7 + cg17 * cg18 * cg25 * cg3 * cg8 - cg17 * cg18 * cg26 *
cg3 * cg7 + cg17 * cg19 * cg2 * cg24 * cg9 - cg17 * cg19 * cg2 *
cg27 * cg6 - cg17 * cg19 * cg24 * cg3 * cg8 + cg17 * cg19 * cg26 *
cg3 * cg6 - cg17 * cg2 * cg21 * cg24 * cg7 + cg17 * cg2 * cg21 *
cg25 * cg6 + cg17 * cg20 * cg24 * cg3 * cg7 - cg17 * cg20 * cg25 *
cg3 * cg6;
J_inverse_array[5][0] = -cg10 * cg12 * cg19 * cg26 * cg33 + cg10 * cg12 * cg19
* cg27 * cg32 + cg10 * cg12 * cg20 * cg25 * cg33 - cg10 * cg12 *
cg20 * cg27 * cg31 - cg10 * cg12 * cg21 * cg25 * cg32 + cg10 * cg12
* cg21 * cg26 * cg31 + cg10 * cg13 * cg18 * cg26 * cg33 - cg10 *
cg13 * cg18 * cg27 * cg32 - cg10 * cg13 * cg20 * cg24 * cg33 + cg10
* cg13 * cg20 * cg27 * cg30 + cg10 * cg13 * cg21 * cg24 * cg32 -
cg10 * cg13 * cg21 * cg26 * cg30 - cg10 * cg14 * cg18 * cg25 * cg33
+ cg10 * cg14 * cg18 * cg27 * cg31 + cg10 * cg14 * cg19 * cg24 *
cg33 - cg10 * cg14 * cg19 * cg27 * cg30 - cg10 * cg14 * cg21 * cg24
* cg31 + cg10 * cg14 * cg21 * cg25 * cg30 + cg10 * cg15 * cg18 *
cg25 * cg32 - cg10 * cg15 * cg18 * cg26 * cg31 - cg10 * cg15 * cg19
* cg24 * cg32 + cg10 * cg15 * cg19 * cg26 * cg30 + cg10 * cg15 *
cg20 * cg24 * cg31 - cg10 * cg15 * cg20 * cg25 * cg30 + cg12 * cg19
* cg26 * cg34 * cg9 - cg12 * cg19 * cg27 * cg34 * cg8 - cg12 * cg19
* cg28 * cg32 * cg9 + cg12 * cg19 * cg28 * cg33 * cg8 - cg12 * cg20
* cg25 * cg34 * cg9 + cg12 * cg20 * cg27 * cg34 * cg7 + cg12 * cg20
* cg28 * cg31 * cg9 - cg12 * cg20 * cg28 * cg33 * cg7 + cg12 * cg21
* cg25 * cg34 * cg8 - cg12 * cg21 * cg26 * cg34 * cg7 - cg12 * cg21
* cg28 * cg31 * cg8 + cg12 * cg21 * cg28 * cg32 * cg7 + cg12 * cg22
* cg25 * cg32 * cg9 - cg12 * cg22 * cg25 * cg33 * cg8 - cg12 * cg22
* cg26 * cg31 * cg9 + cg12 * cg22 * cg26 * cg33 * cg7 + cg12 * cg22
* cg27 * cg31 * cg8 - cg12 * cg22 * cg27 * cg32 * cg7 - cg13 * cg18
* cg26 * cg34 * cg9 + cg13 * cg18 * cg27 * cg34 * cg8 + cg13 * cg18
* cg28 * cg32 * cg9 - cg13 * cg18 * cg28 * cg33 * cg8 + cg13 * cg20
* cg24 * cg34 * cg9 - cg13 * cg20 * cg27 * cg34 * cg6 - cg13 * cg20
* cg28 * cg30 * cg9 + cg13 * cg20 * cg28 * cg33 * cg6 - cg13 * cg21
* cg24 * cg34 * cg8 + cg13 * cg21 * cg26 * cg34 * cg6 + cg13 * cg21
* cg28 * cg30 * cg8 - cg13 * cg21 * cg28 * cg32 * cg6 - cg13 * cg22
* cg24 * cg32 * cg9 + cg13 * cg22 * cg24 * cg33 * cg8 + cg13 * cg22
* cg26 * cg30 * cg9 - cg13 * cg22 * cg26 * cg33 * cg6 - cg13 * cg22
* cg27 * cg30 * cg8 + cg13 * cg22 * cg27 * cg32 * cg6 + cg14 * cg18
* cg25 * cg34 * cg9 - cg14 * cg18 * cg27 * cg34 * cg7 - cg14 * cg18
* cg28 * cg31 * cg9 + cg14 * cg18 * cg28 * cg33 * cg7 - cg14 * cg19
* cg24 * cg34 * cg9 + cg14 * cg19 * cg27 * cg34 * cg6 + cg14 * cg19
* cg28 * cg30 * cg9 - cg14 * cg19 * cg28 * cg33 * cg6 + cg14 * cg21
* cg24 * cg34 * cg7 - cg14 * cg21 * cg25 * cg34 * cg6 - cg14 * cg21
* cg28 * cg30 * cg7 + cg14 * cg21 * cg28 * cg31 * cg6 + cg14 * cg22
* cg24 * cg31 * cg9 - cg14 * cg22 * cg24 * cg33 * cg7 - cg14 * cg22
* cg25 * cg30 * cg9 + cg14 * cg22 * cg25 * cg33 * cg6 + cg14 * cg22
* cg27 * cg30 * cg7 - cg14 * cg22 * cg27 * cg31 * cg6 - cg15 * cg18
* cg25 * cg34 * cg8 + cg15 * cg18 * cg26 * cg34 * cg7 + cg15 * cg18
* cg28 * cg31 * cg8 - cg15 * cg18 * cg28 * cg32 * cg7 + cg15 * cg19
* cg24 * cg34 * cg8 - cg15 * cg19 * cg26 * cg34 * cg6 - cg15 * cg19
* cg28 * cg30 * cg8 + cg15 * cg19 * cg28 * cg32 * cg6 - cg15 * cg20
* cg24 * cg34 * cg7 + cg15 * cg20 * cg25 * cg34 * cg6 + cg15 * cg20
* cg28 * cg30 * cg7 - cg15 * cg20 * cg28 * cg31 * cg6 - cg15 * cg22
* cg24 * cg31 * cg8 + cg15 * cg22 * cg24 * cg32 * cg7 + cg15 * cg22
* cg25 * cg30 * cg8 - cg15 * cg22 * cg25 * cg32 * cg6 - cg15 * cg22
* cg26 * cg30 * cg7 + cg15 * cg22 * cg26 * cg31 * cg6 - cg16 * cg18
* cg25 * cg32 * cg9 + cg16 * cg18 * cg25 * cg33 * cg8 + cg16 * cg18
* cg26 * cg31 * cg9 - cg16 * cg18 * cg26 * cg33 * cg7 - cg16 * cg18
* cg27 * cg31 * cg8 + cg16 * cg18 * cg27 * cg32 * cg7 + cg16 * cg19
* cg24 * cg32 * cg9 - cg16 * cg19 * cg24 * cg33 * cg8 - cg16 *
cg19 * cg26 * cg30 * cg9 + cg16 * cg19 * cg26 * cg33 * cg6 + cg16 *
cg19 * cg27 * cg30 * cg8 - cg16 * cg19 * cg27 * cg32 * cg6 - cg16 *
cg20 * cg24 * cg31 * cg9 + cg16 * cg20 * cg24 * cg33 * cg7 + cg16 *
cg20 * cg25 * cg30 * cg9 - cg16 * cg20 * cg25 * cg33 * cg6 - cg16 *
cg20 * cg27 * cg30 * cg7 + cg16 * cg20 * cg27 * cg31 * cg6 + cg16 *
cg21 * cg24 * cg31 * cg8 - cg16 * cg21 * cg24 * cg32 * cg7 - cg16 *
cg21 * cg25 * cg30 * cg8 + cg16 * cg21 * cg25 * cg32 * cg6 + cg16 *
cg21 * cg26 * cg30 * cg7 - cg16 * cg21 * cg26 * cg31 * cg6;
J_inverse_array[5][1] = cg0 * cg13 * cg20 * cg27 * cg34 - cg0 * cg13 * cg20 *
cg28 * cg33 - cg0 * cg13 * cg21 * cg26 * cg34 + cg0 * cg13 * cg21 *
cg28 * cg32 + cg0 * cg13 * cg22 * cg26 * cg33 - cg0 * cg13 * cg22 *
cg27 * cg32 - cg0 * cg14 * cg19 * cg27 * cg34 + cg0 * cg14 * cg19 *
cg28 * cg33 + cg0 * cg14 * cg21 * cg25 * cg34 - cg0 * cg14 * cg21 *
cg28 * cg31 - cg0 * cg14 * cg22 * cg25 * cg33 + cg0 * cg14 * cg22 *
cg27 * cg31 + cg0 * cg15 * cg19 * cg26 * cg34 - cg0 * cg15 * cg19 *
cg28 * cg32 - cg0 * cg15 * cg20 * cg25 * cg34 + cg0 * cg15 * cg20 *
cg28 * cg31 + cg0 * cg15 * cg22 * cg25 * cg32 - cg0 * cg15 * cg22 *
cg26 * cg31 - cg0 * cg16 * cg19 * cg26 * cg33 + cg0 * cg16 * cg19 *
cg27 * cg32 + cg0 * cg16 * cg20 * cg25 * cg33 - cg0 * cg16 * cg20 *
cg27 * cg31 - cg0 * cg16 * cg21 * cg25 * cg32 + cg0 * cg16 * cg21 *
cg26 * cg31 - cg1 * cg12 * cg20 * cg27 * cg34 + cg1 * cg12 * cg20 *
cg28 * cg33 + cg1 * cg12 * cg21 * cg26 * cg34 - cg1 * cg12 * cg21 *
cg28 * cg32 - cg1 * cg12 * cg22 * cg26 * cg33 + cg1 * cg12 * cg22 *
cg27 * cg32 + cg1 * cg14 * cg18 * cg27 * cg34 - cg1 * cg14 * cg18 *
cg28 * cg33 - cg1 * cg14 * cg21 * cg24 * cg34 + cg1 * cg14 * cg21 *
cg28 * cg30 + cg1 * cg14 * cg22 * cg24 * cg33 - cg1 * cg14 * cg22 *
cg27 * cg30 - cg1 * cg15 * cg18 * cg26 * cg34 + cg1 * cg15 * cg18 *
cg28 * cg32 + cg1 * cg15 * cg20 * cg24 * cg34 - cg1 * cg15 * cg20 *
cg28 * cg30 - cg1 * cg15 * cg22 * cg24 * cg32 + cg1 * cg15 * cg22 *
cg26 * cg30 + cg1 * cg16 * cg18 * cg26 * cg33 - cg1 * cg16 * cg18 *
cg27 * cg32 - cg1 * cg16 * cg20 * cg24 * cg33 + cg1 * cg16 * cg20 *
cg27 * cg30 + cg1 * cg16 * cg21 * cg24 * cg32 - cg1 * cg16 * cg21 *
cg26 * cg30 + cg12 * cg19 * cg2 * cg27 * cg34 - cg12 * cg19 * cg2 *
cg28 * cg33 - cg12 * cg19 * cg26 * cg3 * cg34 + cg12 * cg19 * cg26
* cg33 * cg4 - cg12 * cg19 * cg27 * cg32 * cg4 + cg12 * cg19 * cg28
* cg3 * cg32 - cg12 * cg2 * cg21 * cg25 * cg34 + cg12 * cg2 * cg21
* cg28 * cg31 + cg12 * cg2 * cg22 * cg25 * cg33 - cg12 * cg2 * cg22
* cg27 * cg31 + cg12 * cg20 * cg25 * cg3 * cg34 - cg12 * cg20 *
cg25 * cg33 * cg4 + cg12 * cg20 * cg27 * cg31 * cg4 - cg12 * cg20 *
cg28 * cg3 * cg31 + cg12 * cg21 * cg25 * cg32 * cg4 - cg12 * cg21 *
cg26 * cg31 * cg4 - cg12 * cg22 * cg25 * cg3 * cg32 + cg12 * cg22 *
cg26 * cg3 * cg31 - cg13 * cg18 * cg2 * cg27 * cg34 + cg13 * cg18 *
cg2 * cg28 * cg33 + cg13 * cg18 * cg26 * cg3 * cg34 - cg13 * cg18 *
cg26 * cg33 * cg4 + cg13 * cg18 * cg27 * cg32 * cg4 - cg13 * cg18 *
cg28 * cg3 * cg32 + cg13 * cg2 * cg21 * cg24 * cg34 - cg13 * cg2 *
cg21 * cg28 * cg30 - cg13 * cg2 * cg22 * cg24 * cg33 + cg13 * cg2 *
cg22 * cg27 * cg30 - cg13 * cg20 * cg24 * cg3 * cg34 + cg13 * cg20
* cg24 * cg33 * cg4 - cg13 * cg20 * cg27 * cg30 * cg4 + cg13 * cg20
* cg28 * cg3 * cg30 - cg13 * cg21 * cg24 * cg32 * cg4 + cg13 * cg21
* cg26 * cg30 * cg4 + cg13 * cg22 * cg24 * cg3 * cg32 - cg13 * cg22
* cg26 * cg3 * cg30 - cg14 * cg18 * cg25 * cg3 * cg34 + cg14 * cg18
* cg25 * cg33 * cg4 - cg14 * cg18 * cg27 * cg31 * cg4 + cg14 * cg18
* cg28 * cg3 * cg31 + cg14 * cg19 * cg24 * cg3 * cg34 - cg14 * cg19
* cg24 * cg33 * cg4 + cg14 * cg19 * cg27 * cg30 * cg4 - cg14 * cg19
* cg28 * cg3 * cg30 + cg14 * cg21 * cg24 * cg31 * cg4 - cg14 * cg21
* cg25 * cg30 * cg4 - cg14 * cg22 * cg24 * cg3 * cg31 + cg14 * cg22
* cg25 * cg3 * cg30 + cg15 * cg18 * cg2 * cg25 * cg34 - cg15 * cg18
* cg2 * cg28 * cg31 - cg15 * cg18 * cg25 * cg32 * cg4 + cg15 * cg18
* cg26 * cg31 * cg4 - cg15 * cg19 * cg2 * cg24 * cg34 + cg15 * cg19
* cg2 * cg28 * cg30 + cg15 * cg19 * cg24 * cg32 * cg4 - cg15 * cg19
* cg26 * cg30 * cg4 + cg15 * cg2 * cg22 * cg24 * cg31 - cg15 * cg2
* cg22 * cg25 * cg30 - cg15 * cg20 * cg24 * cg31 * cg4 + cg15 *
cg20 * cg25 * cg30 * cg4 - cg16 * cg18 * cg2 * cg25 * cg33 + cg16 *
cg18 * cg2 * cg27 * cg31 + cg16 * cg18 * cg25 * cg3 * cg32 - cg16 *
cg18 * cg26 * cg3 * cg31 + cg16 * cg19 * cg2 * cg24 * cg33 - cg16 *
cg19 * cg2 * cg27 * cg30 - cg16 * cg19 * cg24 * cg3 * cg32 + cg16 *
cg19 * cg26 * cg3 * cg30 - cg16 * cg2 * cg21 * cg24 * cg31 + cg16 *
cg2 * cg21 * cg25 * cg30 + cg16 * cg20 * cg24 * cg3 * cg31 - cg16 *
cg20 * cg25 * cg3 * cg30;
J_inverse_array[5][2] = cg0 * cg10 * cg19 * cg26 * cg33 - cg0 * cg10 * cg19 *
cg27 * cg32 - cg0 * cg10 * cg20 * cg25 * cg33 + cg0 * cg10 * cg20 *
cg27 * cg31 + cg0 * cg10 * cg21 * cg25 * cg32 - cg0 * cg10 * cg21 *
cg26 * cg31 - cg0 * cg19 * cg26 * cg34 * cg9 + cg0 * cg19 * cg27 *
cg34 * cg8 + cg0 * cg19 * cg28 * cg32 * cg9 - cg0 * cg19 * cg28 *
cg33 * cg8 + cg0 * cg20 * cg25 * cg34 * cg9 - cg0 * cg20 * cg27 *
cg34 * cg7 - cg0 * cg20 * cg28 * cg31 * cg9 + cg0 * cg20 * cg28 *
cg33 * cg7 - cg0 * cg21 * cg25 * cg34 * cg8 + cg0 * cg21 * cg26 *
cg34 * cg7 + cg0 * cg21 * cg28 * cg31 * cg8 - cg0 * cg21 * cg28 *
cg32 * cg7 - cg0 * cg22 * cg25 * cg32 * cg9 + cg0 * cg22 * cg25 *
cg33 * cg8 + cg0 * cg22 * cg26 * cg31 * cg9 - cg0 * cg22 * cg26 *
cg33 * cg7 - cg0 * cg22 * cg27 * cg31 * cg8 + cg0 * cg22 * cg27 *
cg32 * cg7 - cg1 * cg10 * cg18 * cg26 * cg33 + cg1 * cg10 * cg18 *
cg27 * cg32 + cg1 * cg10 * cg20 * cg24 * cg33 - cg1 * cg10 * cg20 *
cg27 * cg30 - cg1 * cg10 * cg21 * cg24 * cg32 + cg1 * cg10 * cg21 *
cg26 * cg30 + cg1 * cg18 * cg26 * cg34 * cg9 - cg1 * cg18 * cg27 *
cg34 * cg8 - cg1 * cg18 * cg28 * cg32 * cg9 + cg1 * cg18 * cg28 *
cg33 * cg8 - cg1 * cg20 * cg24 * cg34 * cg9 + cg1 * cg20 * cg27 *
cg34 * cg6 + cg1 * cg20 * cg28 * cg30 * cg9 - cg1 * cg20 * cg28 *
cg33 * cg6 + cg1 * cg21 * cg24 * cg34 * cg8 - cg1 * cg21 * cg26 *
cg34 * cg6 - cg1 * cg21 * cg28 * cg30 * cg8 + cg1 * cg21 * cg28 *
cg32 * cg6 + cg1 * cg22 * cg24 * cg32 * cg9 - cg1 * cg22 * cg24 *
cg33 * cg8 - cg1 * cg22 * cg26 * cg30 * cg9 + cg1 * cg22 * cg26 *
cg33 * cg6 + cg1 * cg22 * cg27 * cg30 * cg8 - cg1 * cg22 * cg27 *
cg32 * cg6 + cg10 * cg18 * cg2 * cg25 * cg33 - cg10 * cg18 * cg2 *
cg27 * cg31 - cg10 * cg18 * cg25 * cg3 * cg32 + cg10 * cg18 * cg26
* cg3 * cg31 - cg10 * cg19 * cg2 * cg24 * cg33 + cg10 * cg19 * cg2
* cg27 * cg30 + cg10 * cg19 * cg24 * cg3 * cg32 - cg10 * cg19 *
cg26 * cg3 * cg30 + cg10 * cg2 * cg21 * cg24 * cg31 - cg10 * cg2 *
cg21 * cg25 * cg30 - cg10 * cg20 * cg24 * cg3 * cg31 + cg10 * cg20
* cg25 * cg3 * cg30 - cg18 * cg2 * cg25 * cg34 * cg9 + cg18 * cg2 *
cg27 * cg34 * cg7 + cg18 * cg2 * cg28 * cg31 * cg9 - cg18 * cg2 *
cg28 * cg33 * cg7 + cg18 * cg25 * cg3 * cg34 * cg8 + cg18 * cg25 *
cg32 * cg4 * cg9 - cg18 * cg25 * cg33 * cg4 * cg8 - cg18 * cg26 *
cg3 * cg34 * cg7 - cg18 * cg26 * cg31 * cg4 * cg9 + cg18 * cg26 *
cg33 * cg4 * cg7 + cg18 * cg27 * cg31 * cg4 * cg8 - cg18 * cg27 *
cg32 * cg4 * cg7 - cg18 * cg28 * cg3 * cg31 * cg8 + cg18 * cg28 *
cg3 * cg32 * cg7 + cg19 * cg2 * cg24 * cg34 * cg9 - cg19 * cg2 *
cg27 * cg34 * cg6 - cg19 * cg2 * cg28 * cg30 * cg9 + cg19 * cg2 *
cg28 * cg33 * cg6 - cg19 * cg24 * cg3 * cg34 * cg8 - cg19 * cg24 *
cg32 * cg4 * cg9 + cg19 * cg24 * cg33 * cg4 * cg8 + cg19 * cg26 *
cg3 * cg34 * cg6 + cg19 * cg26 * cg30 * cg4 * cg9 - cg19 * cg26 *
cg33 * cg4 * cg6 - cg19 * cg27 * cg30 * cg4 * cg8 + cg19 * cg27 *
cg32 * cg4 * cg6 + cg19 * cg28 * cg3 * cg30 * cg8 - cg19 * cg28 *
cg3 * cg32 * cg6 - cg2 * cg21 * cg24 * cg34 * cg7 + cg2 * cg21 *
cg25 * cg34 * cg6 + cg2 * cg21 * cg28 * cg30 * cg7 - cg2 * cg21 *
cg28 * cg31 * cg6 - cg2 * cg22 * cg24 * cg31 * cg9 + cg2 * cg22 *
cg24 * cg33 * cg7 + cg2 * cg22 * cg25 * cg30 * cg9 - cg2 * cg22 *
cg25 * cg33 * cg6 - cg2 * cg22 * cg27 * cg30 * cg7 + cg2 * cg22 *
cg27 * cg31 * cg6 + cg20 * cg24 * cg3 * cg34 * cg7 + cg20 * cg24 *
cg31 * cg4 * cg9 - cg20 * cg24 * cg33 * cg4 * cg7 - cg20 * cg25 *
cg3 * cg34 * cg6 - cg20 * cg25 * cg30 * cg4 * cg9 + cg20 * cg25 *
cg33 * cg4 * cg6 + cg20 * cg27 * cg30 * cg4 * cg7 - cg20 * cg27 *
cg31 * cg4 * cg6 - cg20 * cg28 * cg3 * cg30 * cg7 + cg20 * cg28 *
cg3 * cg31 * cg6 - cg21 * cg24 * cg31 * cg4 * cg8 + cg21 * cg24 *
cg32 * cg4 * cg7 + cg21 * cg25 * cg30 * cg4 * cg8 - cg21 * cg25 *
cg32 * cg4 * cg6 - cg21 * cg26 * cg30 * cg4 * cg7 + cg21 * cg26 *
cg31 * cg4 * cg6 + cg22 * cg24 * cg3 * cg31 * cg8 - cg22 * cg24 *
cg3 * cg32 * cg7 - cg22 * cg25 * cg3 * cg30 * cg8 + cg22 * cg25 *
cg3 * cg32 * cg6 + cg22 * cg26 * cg3 * cg30 * cg7 - cg22 * cg26 *
cg3 * cg31 * cg6;
J_inverse_array[5][3] = -cg0 * cg10 * cg13 * cg26 * cg33 + cg0 * cg10 * cg13 *
cg27 * cg32 + cg0 * cg10 * cg14 * cg25 * cg33 - cg0 * cg10 * cg14 *
cg27 * cg31 - cg0 * cg10 * cg15 * cg25 * cg32 + cg0 * cg10 * cg15 *
cg26 * cg31 + cg0 * cg13 * cg26 * cg34 * cg9 - cg0 * cg13 * cg27 *
cg34 * cg8 - cg0 * cg13 * cg28 * cg32 * cg9 + cg0 * cg13 * cg28 *
cg33 * cg8 - cg0 * cg14 * cg25 * cg34 * cg9 + cg0 * cg14 * cg27 *
cg34 * cg7 + cg0 * cg14 * cg28 * cg31 * cg9 - cg0 * cg14 * cg28 *
cg33 * cg7 + cg0 * cg15 * cg25 * cg34 * cg8 - cg0 * cg15 * cg26 *
cg34 * cg7 - cg0 * cg15 * cg28 * cg31 * cg8 + cg0 * cg15 * cg28 *
cg32 * cg7 + cg0 * cg16 * cg25 * cg32 * cg9 - cg0 * cg16 * cg25 *
cg33 * cg8 - cg0 * cg16 * cg26 * cg31 * cg9 + cg0 * cg16 * cg26 *
cg33 * cg7 + cg0 * cg16 * cg27 * cg31 * cg8 - cg0 * cg16 * cg27 *
cg32 * cg7 + cg1 * cg10 * cg12 * cg26 * cg33 - cg1 * cg10 * cg12 *
cg27 * cg32 - cg1 * cg10 * cg14 * cg24 * cg33 + cg1 * cg10 * cg14 *
cg27 * cg30 + cg1 * cg10 * cg15 * cg24 * cg32 - cg1 * cg10 * cg15 *
cg26 * cg30 - cg1 * cg12 * cg26 * cg34 * cg9 + cg1 * cg12 * cg27 *
cg34 * cg8 + cg1 * cg12 * cg28 * cg32 * cg9 - cg1 * cg12 * cg28 *
cg33 * cg8 + cg1 * cg14 * cg24 * cg34 * cg9 - cg1 * cg14 * cg27 *
cg34 * cg6 - cg1 * cg14 * cg28 * cg30 * cg9 + cg1 * cg14 * cg28 *
cg33 * cg6 - cg1 * cg15 * cg24 * cg34 * cg8 + cg1 * cg15 * cg26 *
cg34 * cg6 + cg1 * cg15 * cg28 * cg30 * cg8 - cg1 * cg15 * cg28 *
cg32 * cg6 - cg1 * cg16 * cg24 * cg32 * cg9 + cg1 * cg16 * cg24 *
cg33 * cg8 + cg1 * cg16 * cg26 * cg30 * cg9 - cg1 * cg16 * cg26 *
cg33 * cg6 - cg1 * cg16 * cg27 * cg30 * cg8 + cg1 * cg16 * cg27 *
cg32 * cg6 - cg10 * cg12 * cg2 * cg25 * cg33 + cg10 * cg12 * cg2 *
cg27 * cg31 + cg10 * cg12 * cg25 * cg3 * cg32 - cg10 * cg12 * cg26
* cg3 * cg31 + cg10 * cg13 * cg2 * cg24 * cg33 - cg10 * cg13 * cg2
* cg27 * cg30 - cg10 * cg13 * cg24 * cg3 * cg32 + cg10 * cg13 *
cg26 * cg3 * cg30 + cg10 * cg14 * cg24 * cg3 * cg31 - cg10 * cg14 *
cg25 * cg3 * cg30 - cg10 * cg15 * cg2 * cg24 * cg31 + cg10 * cg15 *
cg2 * cg25 * cg30 + cg12 * cg2 * cg25 * cg34 * cg9 - cg12 * cg2 *
cg27 * cg34 * cg7 - cg12 * cg2 * cg28 * cg31 * cg9 + cg12 * cg2 *
cg28 * cg33 * cg7 - cg12 * cg25 * cg3 * cg34 * cg8 - cg12 * cg25 *
cg32 * cg4 * cg9 + cg12 * cg25 * cg33 * cg4 * cg8 + cg12 * cg26 *
cg3 * cg34 * cg7 + cg12 * cg26 * cg31 * cg4 * cg9 - cg12 * cg26 *
cg33 * cg4 * cg7 - cg12 * cg27 * cg31 * cg4 * cg8 + cg12 * cg27 *
cg32 * cg4 * cg7 + cg12 * cg28 * cg3 * cg31 * cg8 - cg12 * cg28 *
cg3 * cg32 * cg7 - cg13 * cg2 * cg24 * cg34 * cg9 + cg13 * cg2 *
cg27 * cg34 * cg6 + cg13 * cg2 * cg28 * cg30 * cg9 - cg13 * cg2 *
cg28 * cg33 * cg6 + cg13 * cg24 * cg3 * cg34 * cg8 + cg13 * cg24 *
cg32 * cg4 * cg9 - cg13 * cg24 * cg33 * cg4 * cg8 - cg13 * cg26 *
cg3 * cg34 * cg6 - cg13 * cg26 * cg30 * cg4 * cg9 + cg13 * cg26 *
cg33 * cg4 * cg6 + cg13 * cg27 * cg30 * cg4 * cg8 - cg13 * cg27 *
cg32 * cg4 * cg6 - cg13 * cg28 * cg3 * cg30 * cg8 + cg13 * cg28 *
cg3 * cg32 * cg6 - cg14 * cg24 * cg3 * cg34 * cg7 - cg14 * cg24 *
cg31 * cg4 * cg9 + cg14 * cg24 * cg33 * cg4 * cg7 + cg14 * cg25 *
cg3 * cg34 * cg6 + cg14 * cg25 * cg30 * cg4 * cg9 - cg14 * cg25 *
cg33 * cg4 * cg6 - cg14 * cg27 * cg30 * cg4 * cg7 + cg14 * cg27 *
cg31 * cg4 * cg6 + cg14 * cg28 * cg3 * cg30 * cg7 - cg14 * cg28 *
cg3 * cg31 * cg6 + cg15 * cg2 * cg24 * cg34 * cg7 - cg15 * cg2 *
cg25 * cg34 * cg6 - cg15 * cg2 * cg28 * cg30 * cg7 + cg15 * cg2 *
cg28 * cg31 * cg6 + cg15 * cg24 * cg31 * cg4 * cg8 - cg15 * cg24 *
cg32 * cg4 * cg7 - cg15 * cg25 * cg30 * cg4 * cg8 + cg15 * cg25 *
cg32 * cg4 * cg6 + cg15 * cg26 * cg30 * cg4 * cg7 - cg15 * cg26 *
cg31 * cg4 * cg6 + cg16 * cg2 * cg24 * cg31 * cg9 - cg16 * cg2 *
cg24 * cg33 * cg7 - cg16 * cg2 * cg25 * cg30 * cg9 + cg16 * cg2 *
cg25 * cg33 * cg6 + cg16 * cg2 * cg27 * cg30 * cg7 - cg16 * cg2 *
cg27 * cg31 * cg6 - cg16 * cg24 * cg3 * cg31 * cg8 + cg16 * cg24 *
cg3 * cg32 * cg7 + cg16 * cg25 * cg3 * cg30 * cg8 - cg16 * cg25 *
cg3 * cg32 * cg6 - cg16 * cg26 * cg3 * cg30 * cg7 + cg16 * cg26 *
cg3 * cg31 * cg6;
J_inverse_array[5][4] = cg0 * cg10 * cg13 * cg20 * cg33 - cg0 * cg10 * cg13 *
cg21 * cg32 - cg0 * cg10 * cg14 * cg19 * cg33 + cg0 * cg10 * cg14 *
cg21 * cg31 + cg0 * cg10 * cg15 * cg19 * cg32 - cg0 * cg10 * cg15 *
cg20 * cg31 - cg0 * cg13 * cg20 * cg34 * cg9 + cg0 * cg13 * cg21 *
cg34 * cg8 + cg0 * cg13 * cg22 * cg32 * cg9 - cg0 * cg13 * cg22 *
cg33 * cg8 + cg0 * cg14 * cg19 * cg34 * cg9 - cg0 * cg14 * cg21 *
cg34 * cg7 - cg0 * cg14 * cg22 * cg31 * cg9 + cg0 * cg14 * cg22 *
cg33 * cg7 - cg0 * cg15 * cg19 * cg34 * cg8 + cg0 * cg15 * cg20 *
cg34 * cg7 + cg0 * cg15 * cg22 * cg31 * cg8 - cg0 * cg15 * cg22 *
cg32 * cg7 - cg0 * cg16 * cg19 * cg32 * cg9 + cg0 * cg16 * cg19 *
cg33 * cg8 + cg0 * cg16 * cg20 * cg31 * cg9 - cg0 * cg16 * cg20 *
cg33 * cg7 - cg0 * cg16 * cg21 * cg31 * cg8 + cg0 * cg16 * cg21 *
cg32 * cg7 - cg1 * cg10 * cg12 * cg20 * cg33 + cg1 * cg10 * cg12 *
cg21 * cg32 + cg1 * cg10 * cg14 * cg18 * cg33 - cg1 * cg10 * cg14 *
cg21 * cg30 - cg1 * cg10 * cg15 * cg18 * cg32 + cg1 * cg10 * cg15 *
cg20 * cg30 + cg1 * cg12 * cg20 * cg34 * cg9 - cg1 * cg12 * cg21 *
cg34 * cg8 - cg1 * cg12 * cg22 * cg32 * cg9 + cg1 * cg12 * cg22 *
cg33 * cg8 - cg1 * cg14 * cg18 * cg34 * cg9 + cg1 * cg14 * cg21 *
cg34 * cg6 + cg1 * cg14 * cg22 * cg30 * cg9 - cg1 * cg14 * cg22 *
cg33 * cg6 + cg1 * cg15 * cg18 * cg34 * cg8 - cg1 * cg15 * cg20 *
cg34 * cg6 - cg1 * cg15 * cg22 * cg30 * cg8 + cg1 * cg15 * cg22 *
cg32 * cg6 + cg1 * cg16 * cg18 * cg32 * cg9 - cg1 * cg16 * cg18 *
cg33 * cg8 - cg1 * cg16 * cg20 * cg30 * cg9 + cg1 * cg16 * cg20 *
cg33 * cg6 + cg1 * cg16 * cg21 * cg30 * cg8 - cg1 * cg16 * cg21 *
cg32 * cg6 + cg10 * cg12 * cg19 * cg2 * cg33 - cg10 * cg12 * cg19 *
cg3 * cg32 - cg10 * cg12 * cg2 * cg21 * cg31 + cg10 * cg12 * cg20 *
cg3 * cg31 - cg10 * cg13 * cg18 * cg2 * cg33 + cg10 * cg13 * cg18 *
cg3 * cg32 + cg10 * cg13 * cg2 * cg21 * cg30 - cg10 * cg13 * cg20 *
cg3 * cg30 - cg10 * cg14 * cg18 * cg3 * cg31 + cg10 * cg14 * cg19 *
cg3 * cg30 + cg10 * cg15 * cg18 * cg2 * cg31 - cg10 * cg15 * cg19 *
cg2 * cg30 - cg12 * cg19 * cg2 * cg34 * cg9 + cg12 * cg19 * cg3 *
cg34 * cg8 + cg12 * cg19 * cg32 * cg4 * cg9 - cg12 * cg19 * cg33 *
cg4 * cg8 + cg12 * cg2 * cg21 * cg34 * cg7 + cg12 * cg2 * cg22 *
cg31 * cg9 - cg12 * cg2 * cg22 * cg33 * cg7 - cg12 * cg20 * cg3 *
cg34 * cg7 - cg12 * cg20 * cg31 * cg4 * cg9 + cg12 * cg20 * cg33 *
cg4 * cg7 + cg12 * cg21 * cg31 * cg4 * cg8 - cg12 * cg21 * cg32 *
cg4 * cg7 - cg12 * cg22 * cg3 * cg31 * cg8 + cg12 * cg22 * cg3 *
cg32 * cg7 + cg13 * cg18 * cg2 * cg34 * cg9 - cg13 * cg18 * cg3 *
cg34 * cg8 - cg13 * cg18 * cg32 * cg4 * cg9 + cg13 * cg18 * cg33 *
cg4 * cg8 - cg13 * cg2 * cg21 * cg34 * cg6 - cg13 * cg2 * cg22 *
cg30 * cg9 + cg13 * cg2 * cg22 * cg33 * cg6 + cg13 * cg20 * cg3 *
cg34 * cg6 + cg13 * cg20 * cg30 * cg4 * cg9 - cg13 * cg20 * cg33 *
cg4 * cg6 - cg13 * cg21 * cg30 * cg4 * cg8 + cg13 * cg21 * cg32 *
cg4 * cg6 + cg13 * cg22 * cg3 * cg30 * cg8 - cg13 * cg22 * cg3 *
cg32 * cg6 + cg14 * cg18 * cg3 * cg34 * cg7 + cg14 * cg18 * cg31 *
cg4 * cg9 - cg14 * cg18 * cg33 * cg4 * cg7 - cg14 * cg19 * cg3 *
cg34 * cg6 - cg14 * cg19 * cg30 * cg4 * cg9 + cg14 * cg19 * cg33 *
cg4 * cg6 + cg14 * cg21 * cg30 * cg4 * cg7 - cg14 * cg21 * cg31 *
cg4 * cg6 - cg14 * cg22 * cg3 * cg30 * cg7 + cg14 * cg22 * cg3 *
cg31 * cg6 - cg15 * cg18 * cg2 * cg34 * cg7 - cg15 * cg18 * cg31 *
cg4 * cg8 + cg15 * cg18 * cg32 * cg4 * cg7 + cg15 * cg19 * cg2 *
cg34 * cg6 + cg15 * cg19 * cg30 * cg4 * cg8 - cg15 * cg19 * cg32 *
cg4 * cg6 + cg15 * cg2 * cg22 * cg30 * cg7 - cg15 * cg2 * cg22 *
cg31 * cg6 - cg15 * cg20 * cg30 * cg4 * cg7 + cg15 * cg20 * cg31 *
cg4 * cg6 - cg16 * cg18 * cg2 * cg31 * cg9 + cg16 * cg18 * cg2 *
cg33 * cg7 + cg16 * cg18 * cg3 * cg31 * cg8 - cg16 * cg18 * cg3 *
cg32 * cg7 + cg16 * cg19 * cg2 * cg30 * cg9 - cg16 * cg19 * cg2 *
cg33 * cg6 - cg16 * cg19 * cg3 * cg30 * cg8 + cg16 * cg19 * cg3 *
cg32 * cg6 - cg16 * cg2 * cg21 * cg30 * cg7 + cg16 * cg2 * cg21 *
cg31 * cg6 + cg16 * cg20 * cg3 * cg30 * cg7 - cg16 * cg20 * cg3 *
cg31 * cg6;
J_inverse_array[5][5] = -cg0 * cg10 * cg13 * cg20 * cg27 + cg0 * cg10 * cg13 *
cg21 * cg26 + cg0 * cg10 * cg14 * cg19 * cg27 - cg0 * cg10 * cg14 *
cg21 * cg25 - cg0 * cg10 * cg15 * cg19 * cg26 + cg0 * cg10 * cg15 *
cg20 * cg25 + cg0 * cg13 * cg20 * cg28 * cg9 - cg0 * cg13 * cg21 *
cg28 * cg8 - cg0 * cg13 * cg22 * cg26 * cg9 + cg0 * cg13 * cg22 *
cg27 * cg8 - cg0 * cg14 * cg19 * cg28 * cg9 + cg0 * cg14 * cg21 *
cg28 * cg7 + cg0 * cg14 * cg22 * cg25 * cg9 - cg0 * cg14 * cg22 *
cg27 * cg7 + cg0 * cg15 * cg19 * cg28 * cg8 - cg0 * cg15 * cg20 *
cg28 * cg7 - cg0 * cg15 * cg22 * cg25 * cg8 + cg0 * cg15 * cg22 *
cg26 * cg7 + cg0 * cg16 * cg19 * cg26 * cg9 - cg0 * cg16 * cg19 *
cg27 * cg8 - cg0 * cg16 * cg20 * cg25 * cg9 + cg0 * cg16 * cg20 *
cg27 * cg7 + cg0 * cg16 * cg21 * cg25 * cg8 - cg0 * cg16 * cg21 *
cg26 * cg7 + cg1 * cg10 * cg12 * cg20 * cg27 - cg1 * cg10 * cg12 *
cg21 * cg26 - cg1 * cg10 * cg14 * cg18 * cg27 + cg1 * cg10 * cg14 *
cg21 * cg24 + cg1 * cg10 * cg15 * cg18 * cg26 - cg1 * cg10 * cg15 *
cg20 * cg24 - cg1 * cg12 * cg20 * cg28 * cg9 + cg1 * cg12 * cg21 *
cg28 * cg8 + cg1 * cg12 * cg22 * cg26 * cg9 - cg1 * cg12 * cg22 *
cg27 * cg8 + cg1 * cg14 * cg18 * cg28 * cg9 - cg1 * cg14 * cg21 *
cg28 * cg6 - cg1 * cg14 * cg22 * cg24 * cg9 + cg1 * cg14 * cg22 *
cg27 * cg6 - cg1 * cg15 * cg18 * cg28 * cg8 + cg1 * cg15 * cg20 *
cg28 * cg6 + cg1 * cg15 * cg22 * cg24 * cg8 - cg1 * cg15 * cg22 *
cg26 * cg6 - cg1 * cg16 * cg18 * cg26 * cg9 + cg1 * cg16 * cg18 *
cg27 * cg8 + cg1 * cg16 * cg20 * cg24 * cg9 - cg1 * cg16 * cg20 *
cg27 * cg6 - cg1 * cg16 * cg21 * cg24 * cg8 + cg1 * cg16 * cg21 *
cg26 * cg6 - cg10 * cg12 * cg19 * cg2 * cg27 + cg10 * cg12 * cg19 *
cg26 * cg3 + cg10 * cg12 * cg2 * cg21 * cg25 - cg10 * cg12 * cg20 *
cg25 * cg3 + cg10 * cg13 * cg18 * cg2 * cg27 - cg10 * cg13 * cg18 *
cg26 * cg3 - cg10 * cg13 * cg2 * cg21 * cg24 + cg10 * cg13 * cg20 *
cg24 * cg3 + cg10 * cg14 * cg18 * cg25 * cg3 - cg10 * cg14 * cg19 *
cg24 * cg3 - cg10 * cg15 * cg18 * cg2 * cg25 + cg10 * cg15 * cg19 *
cg2 * cg24 + cg12 * cg19 * cg2 * cg28 * cg9 - cg12 * cg19 * cg26 *
cg4 * cg9 + cg12 * cg19 * cg27 * cg4 * cg8 - cg12 * cg19 * cg28 *
cg3 * cg8 - cg12 * cg2 * cg21 * cg28 * cg7 - cg12 * cg2 * cg22 *
cg25 * cg9 + cg12 * cg2 * cg22 * cg27 * cg7 + cg12 * cg20 * cg25 *
cg4 * cg9 - cg12 * cg20 * cg27 * cg4 * cg7 + cg12 * cg20 * cg28 *
cg3 * cg7 - cg12 * cg21 * cg25 * cg4 * cg8 + cg12 * cg21 * cg26 *
cg4 * cg7 + cg12 * cg22 * cg25 * cg3 * cg8 - cg12 * cg22 * cg26 *
cg3 * cg7 - cg13 * cg18 * cg2 * cg28 * cg9 + cg13 * cg18 * cg26 *
cg4 * cg9 - cg13 * cg18 * cg27 * cg4 * cg8 + cg13 * cg18 * cg28 *
cg3 * cg8 + cg13 * cg2 * cg21 * cg28 * cg6 + cg13 * cg2 * cg22 *
cg24 * cg9 - cg13 * cg2 * cg22 * cg27 * cg6 - cg13 * cg20 * cg24 *
cg4 * cg9 + cg13 * cg20 * cg27 * cg4 * cg6 - cg13 * cg20 * cg28 *
cg3 * cg6 + cg13 * cg21 * cg24 * cg4 * cg8 - cg13 * cg21 * cg26 *
cg4 * cg6 - cg13 * cg22 * cg24 * cg3 * cg8 + cg13 * cg22 * cg26 *
cg3 * cg6 - cg14 * cg18 * cg25 * cg4 * cg9 + cg14 * cg18 * cg27 *
cg4 * cg7 - cg14 * cg18 * cg28 * cg3 * cg7 + cg14 * cg19 * cg24 *
cg4 * cg9 - cg14 * cg19 * cg27 * cg4 * cg6 + cg14 * cg19 * cg28 *
cg3 * cg6 - cg14 * cg21 * cg24 * cg4 * cg7 + cg14 * cg21 * cg25 *
cg4 * cg6 + cg14 * cg22 * cg24 * cg3 * cg7 - cg14 * cg22 * cg25 *
cg3 * cg6 + cg15 * cg18 * cg2 * cg28 * cg7 + cg15 * cg18 * cg25 *
cg4 * cg8 - cg15 * cg18 * cg26 * cg4 * cg7 - cg15 * cg19 * cg2 *
cg28 * cg6 - cg15 * cg19 * cg24 * cg4 * cg8 + cg15 * cg19 * cg26 *
cg4 * cg6 - cg15 * cg2 * cg22 * cg24 * cg7 + cg15 * cg2 * cg22 *
cg25 * cg6 + cg15 * cg20 * cg24 * cg4 * cg7 - cg15 * cg20 * cg25 *
cg4 * cg6 + cg16 * cg18 * cg2 * cg25 * cg9 - cg16 * cg18 * cg2 *
cg27 * cg7 - cg16 * cg18 * cg25 * cg3 * cg8 + cg16 * cg18 * cg26 *
cg3 * cg7 - cg16 * cg19 * cg2 * cg24 * cg9 + cg16 * cg19 * cg2 *
cg27 * cg6 + cg16 * cg19 * cg24 * cg3 * cg8 - cg16 * cg19 * cg26 *
cg3 * cg6 + cg16 * cg2 * cg21 * cg24 * cg7 - cg16 * cg2 * cg21 *
cg25 * cg6 - cg16 * cg20 * cg24 * cg3 * cg7 + cg16 * cg20 * cg25 *
cg3 * cg6;


J_inverse << J_inverse_array[0][0]/J_det,   J_inverse_array[0][1]/J_det,  J_inverse_array[0][2]/J_det,  J_inverse_array[0][3]/J_det,  J_inverse_array[0][4]/J_det,  J_inverse_array[0][5]/J_det,
             J_inverse_array[1][0]/J_det,   J_inverse_array[1][1]/J_det,  J_inverse_array[1][2]/J_det,  J_inverse_array[1][3]/J_det,  J_inverse_array[1][4]/J_det,  J_inverse_array[1][5]/J_det,
             J_inverse_array[2][0]/J_det,   J_inverse_array[2][1]/J_det,  J_inverse_array[2][2]/J_det,  J_inverse_array[2][3]/J_det,  J_inverse_array[2][4]/J_det,  J_inverse_array[2][5]/J_det,
             J_inverse_array[3][0]/J_det,   J_inverse_array[3][1]/J_det,  J_inverse_array[3][2]/J_det,  J_inverse_array[3][3]/J_det,  J_inverse_array[3][4]/J_det,  J_inverse_array[3][5]/J_det,
             J_inverse_array[4][0]/J_det,   J_inverse_array[4][1]/J_det,  J_inverse_array[4][2]/J_det,  J_inverse_array[4][3]/J_det,  J_inverse_array[4][4]/J_det,  J_inverse_array[4][5]/J_det,
             J_inverse_array[5][0]/J_det,   J_inverse_array[5][1]/J_det,  J_inverse_array[5][2]/J_det,  J_inverse_array[5][3]/J_det,  J_inverse_array[5][4]/J_det,  J_inverse_array[5][5]/J_det;
/*
        std::cout << "jacobi_Matrix_inverse:" << "\n";
        std::cout << J_inverse << "\n";*/
        return J_inverse;
    }
     Eigen::Matrix<double, 6, 1> Iteraltion(Eigen::Matrix<double, 6, 1> l_given)
    {
      Eigen::Matrix<double, 6, 1> q_i;
      Eigen::Matrix<double, 6, 1> q_i_1;

        int iteraltime = 0;
        double loss = 10;
        double loss_last = 1e10;
        q_i = q_init;
        q_i_1 = q_i;
      
      while ((iteraltime < 10) && (loss > 1e-1))
      {
        Eigen::Matrix<double, 6, 1> delta_l = caculatelength(q_i) - l_given;


        float minWert  = delta_l.minCoeff();
        minWert = fabs(minWert);
        float maxWert = delta_l.maxCoeff();
        maxWert = fabs(maxWert);
        loss = maxWert;
        if (minWert > maxWert)
        {
          loss = minWert;
        }

        q_i = q_i - jacobi_Matrix(q_i)*delta_l;

       
        if (loss_last < loss){
          q_i = q_i_1;
          iteraltime = 10;
        }
        loss_last = loss;
        q_i_1 = q_i;



        iteraltime ++;
      }
      return q_i;
    }

    float height;
    float radius_p;  
    float radius_b; 
    float wb;  //  Grad
    float wp;

    Eigen::Matrix<double, 6, 4> b, p;
    Eigen::Matrix<double, 6, 1> q_init;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_forInit;
  
  };

 int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematicsStewart2>());
    rclcpp::shutdown();
    return 0;
  }
