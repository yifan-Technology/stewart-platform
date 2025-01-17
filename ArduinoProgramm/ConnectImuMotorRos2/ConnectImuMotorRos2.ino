
#include "AutoPID.h"
#include <ros2arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#define ANALOG_BIT 12
#define PWM_MIN -4095 //mega:8bit max 255, due:12bit 4095 
#define PWM_MAX 4095

#define XRCEDDS_PORT  Serial
#define PUBLISH_FREQUENCY 10 //hz

//----------------------------BNO055 setup-----------------------------------------//
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;     
double x = -1000000, y = -1000000 , z = -1000000; 
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);   

/*gloabl variale for i2c Measurements. if necessary, covariance ist seted*/
double actual_angular_velocity_x = 0; 
double actual_angular_velocity_y = 0;   
double actual_angular_velocity_z = 0;  
 
double actual_linear_acceleration_x = 0;   
double actual_linear_acceleration_y = 0;  
double actual_linear_acceleration_z = 0;   

double actual_orientation_x = 0;   
double actual_orientation_y = 0;  
double actual_orientation_z = 0;   
double actual_orientation_w = 0;    

char imu_frame_id[] = "map";// kann change another frame_id, but tf is needed.
//----------------------------Declaration Pin Nummer------------------------------//
byte motor1PinNum[6] = {30, 32, 34, 36, 38, 40};
byte motor2PinNum[6] = {31, 33, 35, 37, 39, 41};
byte motorPwmPinNum[6] = { 2, 3, 4, 5, 6, 7};
byte motorPosPinNum[6] = {A1, A2, A3, A4, A5, A6};
//----------------------------Declaration controller-------------------------------//
double normPos[6] = {50, 50, 50, 50, 50, 50};
double actualPos[6] = {0, 0, 0, 0, 0, 0};
double pwm[6] = {0, 0, 0, 0, 0, 0};
double Kp[6] = {100, 100, 100, 100, 100, 100};
double Ki[6] = {0, 0, 0, 0, 0, 0};
double Kd[6] = {10, 10, 10, 10, 10, 10};

AutoPID motorPID0(&actualPos[0], &normPos[0], &pwm[0], PWM_MIN, PWM_MAX, Kp[0], Ki[0], Kd[0]);
AutoPID motorPID1(&actualPos[1], &normPos[1], &pwm[1], PWM_MIN, PWM_MAX, Kp[1], Ki[1], Kd[1]);
AutoPID motorPID2(&actualPos[2], &normPos[2], &pwm[2], PWM_MIN, PWM_MAX, Kp[2], Ki[2], Kd[2]);
AutoPID motorPID3(&actualPos[3], &normPos[3], &pwm[3], PWM_MIN, PWM_MAX, Kp[3], Ki[3], Kd[3]);
AutoPID motorPID4(&actualPos[4], &normPos[4], &pwm[4], PWM_MIN, PWM_MAX, Kp[4], Ki[4], Kd[4]);
AutoPID motorPID5(&actualPos[5], &normPos[5], &pwm[5], PWM_MIN, PWM_MAX, Kp[5], Ki[5], Kd[5]);

AutoPID motorPID[6] = {motorPID0, motorPID1, motorPID2, motorPID3, motorPID4, motorPID5};

//---------------------------fuction motor controller------------------------------------------
void motorController(byte motorNum) {
  analogReadResolution(ANALOG_BIT);
  analogWriteResolution(ANALOG_BIT);
  actualPos[motorNum] = 100.0 * analogRead(motorPosPinNum[motorNum]) / PWM_MAX;
  double diffposition = actualPos[motorNum] - normPos[motorNum];
  if ((actualPos[motorNum] > 99.8) || (actualPos[motorNum] < 0.2)) {
    digitalWrite(motor1PinNum[motorNum], LOW);          //当杆长100mm或小于0.2mm时停止运作
    digitalWrite(motor2PinNum[motorNum], LOW);
    analogWrite(motorPwmPinNum[motorNum], 0);
  }
  else {

    if (abs(diffposition) > 1.5) {
      if (diffposition < 0)  {
        digitalWrite(motor1PinNum[motorNum], HIGH);         //前进
        digitalWrite(motor2PinNum[motorNum], LOW);
        analogWrite(motorPwmPinNum[motorNum], PWM_MAX);    //全速

      }
      else if (diffposition > 0) {
        digitalWrite(motor1PinNum[motorNum], LOW);          //后退
        digitalWrite(motor2PinNum[motorNum], HIGH);
        analogWrite(motorPwmPinNum[motorNum], PWM_MAX);    //全速

      }
    }

    else if (abs(diffposition) < 1.5)
    { //pid介入控制
      motorPID[motorNum].run();
      if (pwm[motorNum] > 0) {
        digitalWrite(motor1PinNum[motorNum], HIGH);       //前进
        digitalWrite(motor2PinNum[motorNum], LOW);
        analogWrite(motorPwmPinNum[motorNum], abs(pwm[motorNum]));

      }
      else if (pwm[motorNum] < 0) {                       //后退
        digitalWrite(motor1PinNum[motorNum], LOW);
        digitalWrite(motor2PinNum[motorNum], HIGH);
        analogWrite(motorPwmPinNum[motorNum], abs(pwm[motorNum]));

      }
      else {
        digitalWrite(motor1PinNum[motorNum], LOW);        //停止
        digitalWrite(motor2PinNum[motorNum], LOW);
        analogWrite(motorPwmPinNum[motorNum], 0);
      }
    }
  }

  //1->2 is positive; 2->1 is negative;
}
//---------------------------ros2-------------------------------------------------------------
//-----------------------------------Imu-------------------------------------//
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
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  actual_angular_velocity_x = acc.x();
  actual_angular_velocity_y = acc.y();
  actual_angular_velocity_z = acc.z();
  
  msg->angular_velocity.x = actual_angular_velocity_x;     
  msg->angular_velocity.y = actual_angular_velocity_y;    
  msg->angular_velocity.z = actual_angular_velocity_z;    
  
  imu::Vector<3> lic = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //sun
  actual_linear_acceleration_x = lic.x();
  actual_linear_acceleration_y = lic.y();
  actual_linear_acceleration_z = lic.z();
  msg->linear_acceleration.x = actual_linear_acceleration_x;   
  msg->linear_acceleration.y = actual_linear_acceleration_y;   
  msg->linear_acceleration.z = actual_linear_acceleration_z;  

 
  //sensors_event_t orientationData , angVelocityData , linearAccelData;  //gong
  imu::Quaternion quat = bno.getQuat();   
  actual_orientation_x = -quat.x();   
  actual_orientation_y = -quat.y();   
  actual_orientation_z = quat.z();   
  actual_orientation_w = quat.w();   

  
  msg->orientation.x = actual_orientation_x;
  msg->orientation.y = actual_orientation_y;
  msg->orientation.z = actual_orientation_z;
  msg->orientation.w = actual_orientation_w;

}




//------------------------------------jointState-------------------------------//



void publishJointState(sensor_msgs::JointState* msg, void* arg)
{
  (void)(arg);

  static unsigned long startTime = 0;
  if (startTime == 0) {

    msg->header.stamp.sec = 0;
    msg->header.stamp.nanosec = 0;
    startTime = millis();
  }
  else {

    unsigned int time_diff = millis() - startTime;//ms
    msg->header.stamp.sec = time_diff / 1000.0;
    msg->header.stamp.nanosec = time_diff * 1000000;

  }
  msg->position_size = 6;
  msg->velocity_size = 6;

  for (int i = 0; i < 6; i++) {
    (msg->position)[i] = actualPos[i];
  }

}
void subscribeJointState(sensor_msgs::JointState* msg, void* arg)
{
  (void)(arg);

  for (int i = 0; i < 6; i++) {
    normPos[i] = (msg->position)[i];
  }

}

//class arduinoPubImu : public ros2::Node
//{
//  public:
//    arduinoPubImu()
//      : Node("StewartImu_pub_node")
//    {
//      ros2::Publisher<sensor_msgs::Imu>* publisher_Imu = this->createPublisher<sensor_msgs::Imu>("Stewart_actual_Imu");
//      this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishImu, nullptr, publisher_Imu);
//    }
//};



class JointStatePubAndSub : public ros2::Node
{
  public:
    JointStatePubAndSub()
      : Node("ros2arduino_pub_sub_node")
    {
      /*Imu*/
      ros2::Publisher<sensor_msgs::Imu>* publisher_Imu = this->createPublisher<sensor_msgs::Imu>("Stewart_actual_Imu");         
      this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishImu, nullptr, publisher_Imu);
      /*Joint*/
      ros2::Publisher<sensor_msgs::JointState>* publisher_ = this->createPublisher<sensor_msgs::JointState>("Stewart_actual_JointState");
      this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishJointState, nullptr, publisher_);
      this->createSubscriber<sensor_msgs::JointState>("Stewart_norm_JointState", (ros2::CallbackFunc)subscribeJointState, nullptr);
    }
};


// --------------------------setup-------------------------------------------------------------
void setup() {
  
  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT);
  ros2::init(&XRCEDDS_PORT);
  while (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
     ;
  }//gong
  delay(1000);
  for (byte i = 0; i < 6; i++) {
    pinMode(motor1PinNum[i], OUTPUT);
    pinMode(motor2PinNum[i], OUTPUT);
    pinMode(motorPwmPinNum[i], OUTPUT);
    digitalWrite(motor1PinNum[i], LOW);
    digitalWrite(motor2PinNum[i], LOW);
    
  }


 

  
}

//------------------------------------ loop -----------------------------------------------------

void loop(){
  static JointStatePubAndSub JointStateNode;
  //static arduinoPubImu arduinoPubImuNode;
  //ros2::spin(&arduinoPubImuNode);
  ros2::spin(&JointStateNode);
  for (byte motorNum = 0; motorNum < 6; motorNum++)
    
    motorController(motorNum);

  }
  
/* 
void loop() {
  motorController(0);
  Serial.print(pwm[0]);
  Serial.print(',');
  Serial.print(normPos[0]);
  Serial.print(',');
  Serial.println(actualPos[0]);
}
*/
/* 
void loop() {
  digitalWrite(motor1PinNum[0], HIGH);
  digitalWrite(motor2PinNum[0], LOW);
  analogWriteResolution(ANALOG_BIT);
  analogWrite(motorPwmPinNum[0], 500 );
}
*/