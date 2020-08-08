
#include "AutoPID.h"
#include <ros2arduino.h>
#define ANALOG_BIT 12
#define PWM_MIN -4095 //mega:8bit max 255, due:12bit 4095 
#define PWM_MAX 4095

#define XRCEDDS_PORT  Serial
#define PUBLISH_FREQUENCY 2 //hz

//----------------------------Declaration Pin Nummer------------------------------//
byte motorAPinNum[6] = {30, 32, 34, 36, 38, 40};
byte motorBPinNum[6] = {31, 33, 35, 37, 39, 41};
byte motorPwmPinNum[6] = { 2, 3, 4, 5, 6, 7};
byte motorPosPinNum[6] = {A1, A2, A3, A4, A5, A6};
//----------------------------Declaration controller-------------------------------//
double normPos[6] = {100, 0, 0, 0, 0, 0};
double actualPos[6] = {0, 0, 0, 0, 0, 0};
double pwm[6] = {0, 0, 0, 0, 0, 0};
double Kp[6] = {100, 100, 100, 100, 100, 90};
double Ki[6] = {0, 0, 10, 10, 10, 0};
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
    digitalWrite(motorAPinNum[motorNum], LOW);          //当杆长100mm或小于0.2mm时停止运作
    digitalWrite(motorBPinNum[motorNum], LOW);
    analogWrite(motorPwmPinNum[motorNum], 0);
  }
  else {

    if (abs(diffposition) > 1.3) {
      if (diffposition < 0)  {
        digitalWrite(motorAPinNum[motorNum], HIGH);         //前进
        digitalWrite(motorBPinNum[motorNum], LOW);
        analogWrite(motorPwmPinNum[motorNum], PWM_MAX);    //全速

      }
      else if (diffposition > 0) {
        digitalWrite(motorAPinNum[motorNum], LOW);          //后退
        digitalWrite(motorBPinNum[motorNum], HIGH);
        analogWrite(motorPwmPinNum[motorNum], PWM_MAX);    //全速

      }
    }

    else if (abs(diffposition) < 1.3)
    { //pid介入控制
      motorPID[motorNum].run();
      if (pwm[motorNum] > 0) {
        digitalWrite(motorAPinNum[motorNum], HIGH);       //前进
        digitalWrite(motorBPinNum[motorNum], LOW);
        analogWrite(motorPwmPinNum[motorNum], abs(pwm[motorNum]));

      }
      else if (pwm[motorNum] < 0) {                       //后退
        digitalWrite(motorAPinNum[motorNum], LOW);
        digitalWrite(motorBPinNum[motorNum], HIGH);
        analogWrite(motorPwmPinNum[motorNum], abs(pwm[motorNum]));

      }
      else {
        digitalWrite(motorAPinNum[motorNum], LOW);        //停止
        digitalWrite(motorBPinNum[motorNum], LOW);
        analogWrite(motorPwmPinNum[motorNum], 0);
      }
    }
  }

  //A->B is positive; B->A is negative;
}
//---------------------------ros2-------------------------------------------------------------
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

class JointStatePubAndSub : public ros2::Node
{
  public:
    JointStatePubAndSub()
      : Node("ros2arduino_pub_sub_node")
    {
      ros2::Publisher<sensor_msgs::JointState>* publisher_ = this->createPublisher<sensor_msgs::JointState>("Stewart_actual_JointState");
      this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishJointState, nullptr, publisher_);
      this->createSubscriber<sensor_msgs::JointState>("Stewart_norm_JointState", (ros2::CallbackFunc)subscribeJointState, nullptr);
    }
};


// --------------------------setup-------------------------------------------------------------
void setup() {

  for (byte i = 0; i < 6; i++) {
    pinMode(motorAPinNum[i], OUTPUT);
    pinMode(motorBPinNum[i], OUTPUT);
    pinMode(motorPwmPinNum[i], OUTPUT);
    digitalWrite(motorAPinNum[i], LOW);
    digitalWrite(motorBPinNum[i], LOW);
  }

  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT);
  ros2::init(&XRCEDDS_PORT);
  
}

//------------------------------------ loop -----------------------------------------------------

void loop(){
  static JointStatePubAndSub JointStateNode;
  ros2::spin(&JointStateNode);
  for (byte i = 0; i < 6; i++)
    motorController(i);
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
  digitalWrite(motorAPinNum[0], HIGH);
  digitalWrite(motorBPinNum[0], LOW);
  analogWriteResolution(ANALOG_BIT);
  analogWrite(motorPwmPinNum[0], 500 );
}
*/
