#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;
float konst = 100; 
float offset = 10; 
class PID
{
public:
    float kp;
    float ki;
    float kd;
    float e=0;
    float e_pre=0;
    float integral=0;
    float sollwert;
    float istwert;
    float PID_run(PID*);
    PID(float p, float i, float d, float soll, float ist):kp(p),ki(i),kd(d),sollwert(soll),istwert(ist){}
};

float PID::PID_run(PID*)
{
    this->e = this->sollwert - this->istwert;
    this->integral = this->integral + this->e;
    float output_controller = this->kp*this->e + this->ki*this->integral + this->kd*(this->e - this->e_pre);
    this->e_pre = this->e;
    return output_controller;
}

class Controller : public rclcpp::Node
{
public:
  Controller()
  : Node("PID_Controll")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "stewart2/ballPositon", 10, std::bind(&Controller::topic_callback, this, _1));
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/stewart/norm_platform_twist", 10);
      //timer_ = this->create_wall_timer(100ms, std::bind(&Controller::topic_callback, this)); //10hz
  }

private:
  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) const
  {
    //show recieved msg from camera in the console
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->x).c_str());
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->y).c_str());
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", std::to_string(msg->z).c_str());
    float x = msg->x;
    float y = msg->y;
    float z = 0;
    float ReglerOutput_x = 0;
    float ReglerOutput_y = 0;

    //PID Regler nach Fallunterscheidung veraendbar
    if(x<-offset||x>offset){
      PID *pid1 = new PID(60,10,0.28,0,x);//kp,ki,kd,soll,ist 
      ReglerOutput_x = (pid1->PID_run(pid1))/konst;
    }
    
    if(y<-offset||y>offset){
      PID *pid2 = new PID(60,10,0.28,0,y);//kp,ki,kd,soll,ist 
      ReglerOutput_y = (pid2->PID_run(pid2))/konst;

    }

    if(ReglerOutput_x<-20)ReglerOutput_x=-20;
    if(ReglerOutput_x>20)ReglerOutput_x=20;
    if(ReglerOutput_y<-20)ReglerOutput_y=-20;
    if(ReglerOutput_y>20)ReglerOutput_y=20;

    geometry_msgs::msg::Twist pubmsg;

    pubmsg.linear.x = 0;
    pubmsg.linear.y = 0;
    pubmsg.linear.z = 0;
    pubmsg.angular.x = ReglerOutput_y; //Vorzeichen!!!!!
    pubmsg.angular.y = -ReglerOutput_x;
    pubmsg.angular.z = 0;
    
    publisher_->publish(pubmsg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  //rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}