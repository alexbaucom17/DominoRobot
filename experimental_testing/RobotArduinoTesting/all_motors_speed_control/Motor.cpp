#include "Motor.h"
#include <math.h>

Motor::Motor(int pwmPin, int dirPin, int encPinA, int encPinB, double Kp, double Ki, double Kd, float velFilterFreq)
: pwmPin_(pwmPin),
  dirPin_(dirPin),
  inputVel_(0.0),
  currentVelRaw_(0.0),
  currentVelFiltered_(0.0),
  outputCmd_(0.0),
  prevCount_(0),
  prevMillis_(millis()),
  enc_(encPinA, encPinB),
  controller_(&currentVelFiltered_, &outputCmd_, &inputVel_, Kp, Ki, Kd, DIRECT),
  velFilter_(LOWPASS, velFilterFreq)
{
  pinMode(pwmPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  controller_.SetMode(AUTOMATIC);
  controller_.SetOutputLimits(-255, 255);
}

void Motor::setCommand(double vel)
{
  inputVel_ = vel;  
}

void Motor::runLoop()
{

  // Read current values
  long curCount = enc_.read();
  unsigned long curMillis = millis();

  // Compute delta
  long deltaCount = curCount - prevCount_;
  double deltaRevs = static_cast<double>(deltaCount) * COUNTS_PER_REV;
  unsigned long deltaMillis = curMillis - prevMillis_;

  // Compute current velocity in revs/second
  currentVelRaw_ = deltaRevs / (static_cast<double>(deltaMillis) * 1000.0);
  currentVelFiltered_ = velFilter_.input(currentVelRaw_);

  // Run PID controller
  controller_.Compute();

  // Update output values
  if(outputCmd_ < 0)
  {
    digitalWrite(dirPin_,0);
  }
  else
  {
    digitalWrite(dirPin_,1);
  }
  analogWrite(pwmPin_, abs(outputCmd_));

  // Copy current values into previous
  prevCount_ = curCount;
  prevMillis_ = curMillis;
//
//  Serial.print(curCount);
//  Serial.print(" ");
//  Serial.print(currentVelRaw_);
//  Serial.print(" ");
//  Serial.print(currentVelFiltered_);
//  Serial.print(" ");
//  Serial.print(inputVel_);
//  Serial.print(" ");
//  Serial.print(outputCmd_);
//  Serial.println("");
  
}
