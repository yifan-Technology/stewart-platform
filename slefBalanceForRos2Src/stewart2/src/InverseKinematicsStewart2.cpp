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
#define PI 3.14159265358979323846


class InverseKinematicsStewart2 : public rclcpp::Node  // 2 means ros2
{
  public:
    InverseKinematicsStewart2()
    : Node("InverseKinematicsStewart2")
    {
        radius_p = 84;  
        radius_b = 150; 
        // wb = 30;  //  Grad
        // wp =30;
        height = 218.0;
        b <<  radius_b*0.70710678,    radius_b*0.70710678,   0, 1,  //45
              radius_b*0.96592582,    radius_b*0.25881904,   0, 1,  //15
              radius_b*0.25881904,    radius_b*-0.96592582,  0, 1,  //285
              radius_b*-0.25881904,   radius_b*-0.96592582,  0, 1,  //255
              radius_b*-0.96592582,   radius_b*0.25881904,   0, 1,  //165
              radius_b*-0.70710678,   radius_b*0.70710678,   0, 1;  //135

        p <<  radius_p*0.25881904,    radius_p*0.96592582,   0, 1,  //75
              radius_p*0.96592582,    radius_p*-0.25881904,  0, 1,  //345
              radius_p*0.70710678,    radius_p*-0.70710678,  0, 1,  //315
              radius_p*-0.70710678,   radius_p*-0.70710678,  0, 1,  //225
              radius_p*-0.96592582,   radius_p*-0.25881904,  0, 1,  //195
              radius_p*-0.25881904,   radius_p*0.96592582,   0, 1;  //105


      for (int i = 0; i < 6; i++)
        {
            f32ma_msg.data.push_back(0);
        }
         //this publisher_ for test,future can delete
     //publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/stewart/test_length", 1);
     publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/Stewart_norm_JointState", 1);

     subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/stewart/norm_platform_twist", 1, std::bind(&InverseKinematicsStewart2::callback, this, _1));

}
  private:
//this callback for test,future can delete
    
  // void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  //   {
  //       float x = msg->linear.x;
  //       float y = msg->linear.y;
  //       float z = msg->linear.z;
  //       float roll = msg->angular.x;
  //       float pitch = msg->angular.y;
  //       float yaw = msg->angular.z;
  //       this->caculatelength(x,y,z,roll,pitch,yaw);
  //       publisher_->publish(f32ma_msg);
  //   }
    
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float x = msg->linear.x;
        float y = msg->linear.y;
        float z = msg->linear.z;
        float roll = msg->angular.x*PI/180.0;
        float pitch = msg->angular.y*PI/180.0;
        float yaw = msg->angular.z*PI/180.0;
        this->caculatelength(x,y,z,roll,pitch,yaw);

        sensor_msgs::msg::JointState pubMsg;
        for (size_t i = 0; i < 6; i++)
        {
        pubMsg.position.push_back(f32ma_msg.data[i]);
        }
        publisher_->publish(pubMsg);
    }

    double Deg2Rad(double angular)
    {
          double rad = angular * 3.1415926 / 180;
          return rad;
    }
   
    Eigen::Matrix<float, 4, 4> transformation_matrix(float x, float y, float z, float r, float p, float yaw)
    {
        Eigen::Matrix<float, 4, 4> T;
        T << cos(yaw)*cos(p), -sin(yaw)*cos(r) + cos(yaw)*sin(p)*sin(r),  sin(yaw)*sin(r)+cos(yaw)*sin(p)*cos(r), x,
             sin(yaw)*cos(p),  cos(yaw)*cos(r) + sin(yaw)*sin(p)*sin(r), -cos(yaw)*sin(r)+sin(yaw)*sin(p)*cos(r), y,
                     -sin(p),                             cos(p)*sin(r),                         cos(p)*cos(yaw), z,
                           0,                                         0,                                       0, 1;
        return T;
    }

    void caculatelength(float x, float y, float z, float roll, float pitch, float yaw)
    {
      double data[6];
      bool protect = false;
        Eigen::Matrix<float, 4, 4> T = transformation_matrix(x, y, z + height, roll, pitch, yaw);
        for (size_t i = 0; i < 6; i++)
        {
            Eigen::Matrix<float, 4, 1> length = T*p.row(i).transpose() - b.row(i).transpose();
            data[i] = sqrt(pow(length(0), 2) + pow(length(1), 2) + pow(length(2), 2))-186.0;
        }
        for(size_t i = 0; i < 6; i++)
        {
          if((data[i] > 99.8)||(data[i] < 0.2)){
            protect = true;
          } 
        }
        if(protect==false){
          for(size_t i = 0; i < 6; i++){
          f32ma_msg.data[i]= data[i];
          }
        }
        

    }

    float height;
    float radius_p;  
    float radius_b; 
    float wb;  //  Grad
    float wp;

    Eigen::Matrix<float, 6, 4> b, p;

   //this publisher_ for test,future can delete
    //rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std_msgs::msg::Float32MultiArray f32ma_msg;
  
  };

 int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsStewart2>());
    rclcpp::shutdown();
    return 0;
  }