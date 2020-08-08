
#include "AutoPID.h"
#define ANALOG_BIT 12
#define PWM_MIN -4095 //mega:8bit max 255, due:12bit 4095 
#define PWM_MAX 4095


//----------------------------Declaration Pin Nummer------------------------------/
byte motorAPinNum[6] = {30, 32, 34, 36, 38, 40};
byte motorBPinNum[6] = {31, 33, 35, 37, 39, 41};
byte motorPwmPinNum[6] = { 2, 3, 4, 5, 6, 7};
byte motorPosPinNum[6] = {A1, A2, A3, A4, A5, A6};
//----------------------------Declaration controller-------------------------------
double normPos[6] = {80, 0, 0, 0, 0, 0};
double actualPos[6] = {0, 0, 0, 0, 0, 0};
double pwm[6] = {0, 0, 0, 0, 0, 0};
double Kp[6] = {100, 0, 0, 0, 0, 0};
double Ki[6] = {0, 0, 0, 0, 0, 0};
double Kd[6] = {10, 0, 0, 0, 0, 0};

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
  actualPos[motorNum] = 100.0 *analogRead(motorPosPinNum[motorNum]) / PWM_MAX;
 // motorPID[motorNum].run();
 double difposition = abs(actualPos[motorNum]-normPos[motorNum]);
 
 if (difposition > 1.5){                           //全速
    digitalWrite(motorAPinNum[motorNum], HIGH);
    digitalWrite(motorBPinNum[motorNum], LOW);
    analogWrite(motorPwmPinNum[motorNum], PWM_MAX);
 }
 else if(difposition < 1.5)
 {                                            //pid介入控制
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
 
  //A->B is positive; B->A is negative;  
}

// setup
void setup() {
  Serial.begin(9600);
  for (byte i = 0; i < 6; i++) {
    pinMode(motorAPinNum[i], OUTPUT);
    pinMode(motorBPinNum[i], OUTPUT);
    pinMode(motorPwmPinNum[i], OUTPUT);
    digitalWrite(motorAPinNum[i], LOW);
    digitalWrite(motorBPinNum[i], LOW);
  }

}

// loop
/*
  void loop() {
  for (byte i = 0; i < 6; i++)
    motorController(i);
  }*/

void loop() {
  motorController(0);
  Serial.print(pwm[0]);
  Serial.print(',');
  Serial.print(normPos[0]);
  Serial.print(',');
  Serial.println(actualPos[0]);
}

/* 
void loop() {
  digitalWrite(motorAPinNum[0], HIGH);
  digitalWrite(motorBPinNum[0], LOW);
  analogWriteResolution(ANALOG_BIT);
  analogWrite(motorPwmPinNum[0], 500 );
}
*/
