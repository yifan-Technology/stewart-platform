#include <ros2arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define XRCEDDS_PORT  Serial
#define PUBLISH_FREQUENCY 10 //hz

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;      //sunmingjun
double x = -1000000, y = -1000000 , z = -1000000; //sunmingjun

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);      //sunmingjun

/*gloabl variale for i2c Measurements. if necessary, covariance ist seted*/
double actual_angular_velocity_x = 1;    //sunmingjun
double actual_angular_velocity_y = 1;   //sunmingjun
double actual_angular_velocity_z = 1;   //sunmingjun
 
double actual_linear_acceleration_x = 2;   //sunmingjun
double actual_linear_acceleration_y = 2;  //sunmingjun
double actual_linear_acceleration_z = 2;   //sunmingjun

double actual_orientation_x = 0;    //sunmingjun
double actual_orientation_y = 0;    //sunmingjun
double actual_orientation_z = 0;   //sunmingjun
double actual_orientation_w = 0;    //sunmingjun

char imu_frame_id[] = "map";// kann change another frame_id, but tf is needed.

void publishImu(sensor_msgs::Imu* msg, void* arg)
{
  (void)(arg);
  
  static unsigned long startTime = 0;
  if (startTime == 0){

  msg->header.stamp.sec = 0;
  msg->header.stamp.nanosec = 0;
    startTime = millis();
  }
  else{
    
    unsigned int time_diff = millis() - startTime;//ms
    msg->header.stamp.sec = time_diff/1000.0;
    msg->header.stamp.nanosec = time_diff*1000000;
    
    }
for (int i = 0; i < sizeof(imu_frame_id);i++){
  msg->header.frame_id[i] = imu_frame_id[i];
}
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);//sun
  actual_angular_velocity_x = acc.x();//sun
  actual_angular_velocity_y = acc.y();//sun
  actual_angular_velocity_z = acc.z();//sun
  
  msg->angular_velocity.x = actual_angular_velocity_x;     //sunmingjun
  msg->angular_velocity.y = actual_angular_velocity_y;     //sunmingjun
  msg->angular_velocity.z = actual_angular_velocity_z;     //sunmingjun
  
  imu::Vector<3> lic = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //sun
  actual_linear_acceleration_x = lic.x();//sun
  actual_linear_acceleration_y = lic.y();//sun
  actual_linear_acceleration_z = lic.z();//sun
  msg->linear_acceleration.x = actual_linear_acceleration_x;   //sunmingjun
  msg->linear_acceleration.y = actual_linear_acceleration_y;   //sunmingjun
  msg->linear_acceleration.z = actual_linear_acceleration_z;   //sunmingjun

 
  //sensors_event_t orientationData , angVelocityData , linearAccelData;  //gong
  imu::Quaternion quat = bno.getQuat();   //gong
  actual_orientation_x = -quat.x();   //gong
  actual_orientation_y = -quat.y();   //gongsun
  actual_orientation_z = quat.z();   //gongsun
  actual_orientation_w = quat.w();   //gong

  
  msg->orientation.x = actual_orientation_x;
  msg->orientation.y = actual_orientation_y;
  msg->orientation.z = actual_orientation_z;
  msg->orientation.w = actual_orientation_w;

}



class arduinoPubImu : public ros2::Node
{
  public:
    arduinoPubImu()
      : Node("StewartImu_pub_node")
    {
      ros2::Publisher<sensor_msgs::Imu>* publisher_Imu = this->createPublisher<sensor_msgs::Imu>("Stewart_actual_Imu");
      this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishImu, nullptr, publisher_Imu);
    }
};



void setup()
{
  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT);
  
  ros2::init(&XRCEDDS_PORT);
  while (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
     ;
  }//gong
  delay(1000);

}

void loop()
{

  static arduinoPubImu arduinoPubImuNode;
  ros2::spin(&arduinoPubImuNode);

}
