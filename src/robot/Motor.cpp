#include "Motor.h"

Motor::Motor(int pwmPin, int dirPin, int encPinA, int encPinB, double Kp, double Ki, double Kd)
: pwmPin_(pwmPin),
  dirPin_(dirPin),
  inputVel_(0.0),
  currentVelRaw_(0.0),
  currentVelFiltered_(0.0),
  pidOut_(0.0),
  outputCmd_(0.0),
  prevCount_(0),
  prevMillis_(millis()),
  enc_(encPinA, encPinB),
  controller_(&currentVelFiltered_, &pidOut_, &inputVel_, Kp, Ki, Kd, DIRECT),
  velFilter_(LOWPASS, VEL_FILTER_FREQ)
{
  pinMode(pwmPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  controller_.SetMode(AUTOMATIC);
  controller_.SetOutputLimits(-255, 255);
  controller_.SetSampleTime(5);
}

void Motor::setCommand(double vel)
{
  inputVel_ = vel;  
}

float Motor::getCurrentVelocity()
{
  return static_cast<float>(currentVelFiltered_);
}

void Motor::runLoop()
{

  // Read current values
  long curCount = enc_.read();
  unsigned long curMillis = millis();

  // Compute delta
  long deltaCount = curCount - prevCount_;
  double deltaRevs = static_cast<double>(deltaCount) / COUNTS_PER_SHAFT_REV;
  unsigned long deltaMillis = curMillis - prevMillis_;

  // Copy current values into previous
  prevCount_ = curCount;
  prevMillis_ = curMillis;

  // Do a check for large time deltas. This likely means the controller hasn't run for a while and we should ignore this value
  if(deltaMillis > 100)
  {
    return;
  }

  // Compute current velocity in revs/second
  currentVelRaw_ = 1000.0 * deltaRevs / static_cast<double>(deltaMillis);
  currentVelFiltered_ = velFilter_.input(currentVelRaw_);
  
  // Run PID controller
  controller_.Compute();

  /* Use output from PID to update our current command. Since this is a velocity controller, when the error is 0
  *  and the PID controller drives the output to 0, we actually want to maintain a certian PWM value. Hence, the 
  *  PID output is used to modify our control singal and not drive it directly
  */
  outputCmd_ += int(pidOut_);

  // Set a deadband region based on input vel to avoid integral windup
  if(fabs(inputVel_) < 0.005)
  {
    outputCmd_ = 0;
  }

  // Make sure we don't exceed max/min power
  if(outputCmd_ > 255)
  {
    outputCmd_ = 255;
  }
  else if(outputCmd_ < -255)
  {
    outputCmd_ = -255;
  }

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
/*
//  Serial.print(curCount);
//  Serial.print(" ");
   Serial.print(currentVelRaw_);
   Serial.print(" ");
   Serial.print(currentVelFiltered_);
   Serial.print(" ");
   Serial.print(inputVel_);
   Serial.print(" ");
   Serial.print(outputCmd_/127.0);
//   Serial.print(" ");
//   Serial.print(deltaMillis);
   Serial.println("");*/
  
}
