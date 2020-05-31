#include "Motor.h"
#include "constants.h"

constexpr double COUNTS_TO_RADS = 2.0 * PI / static_cast<double>(COUNTS_PER_OUTPUT_SHAFT_REV);

Motor::Motor(int pwmPin, int dirPin, int encPinA, int encPinB, double Kp, double Ki, double Kd)
: pwmPin_(pwmPin),
  dirPin_(dirPin),
  inputVel_(0.0),
  currentVelRaw_(0.0),
  currentVelFiltered_(0.0),
  pidOut_(0.0),
  outputCmd_(0.0),
  prevCount_(0),
  prevMicros_(micros()),
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

long Motor::getCounts()
{
  // Sign change is needed for directions to work out with kinematics
  return -1*enc_.read();
}

void Motor::runLoop()
{

  // Read current values
  long curCount = getCounts();
  unsigned long curMicros = micros();

  // Compute delta
  long deltaCount = curCount - prevCount_;
  double deltaRads = static_cast<double>(deltaCount) * COUNTS_TO_RADS;
  unsigned long deltaMicros = curMicros - prevMicros_;

  if(deltaMicros == 0)
  {
    // This will break things due to div by 0, so just skip
    return;
  }

  // Copy current values into previous
  prevCount_ = curCount;
  prevMicros_ = curMicros;

  // Do a check for large time deltas. This likely means the controller hasn't run for a while and we should ignore this value
  if(deltaMicros > 100000)
  {
    return;
  }

  // Compute current velocity in rads/second
  currentVelRaw_ = 1000000.0 * deltaRads / static_cast<double>(deltaMicros);
  currentVelFiltered_ = currentVelRaw_; // velFilter_.input(currentVelRaw_);

  // Run PID controller
  controller_.Compute();
  
  /* Use output from PID to update our current command. Since this is a velocity controller, when the error is 0
  *  and the PID controller drives the output to 0, we actually want to maintain a certian PWM value. Hence, the 
  *  PID output is used to modify our control singal and not drive it directly
  */
  outputCmd_ += int(pidOut_);

  // Set a deadband region based on input vel to avoid integral windup
  if(fabs(inputVel_) < 0.001)
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

  // Update output direction, note that to flip these you also have 
  // to change the sign for reading the encoder
  if(outputCmd_ < 0)
  {
    digitalWrite(dirPin_,1);
  }
  else
  {
    digitalWrite(dirPin_,0);
  }
  
  // Actually write out the motor power
  analogWrite(pwmPin_, abs(outputCmd_));

  // Debugging prints
  //Serial.print(deltaMicros);
  //Serial.print(" ");
  //Serial.print(curCount);
  //Serial.print(" ");
  /*Serial.print("currentVelRaw_:");
  Serial.print(currentVelRaw_, 5);
  Serial.print(" ");
  Serial.print("currentVelFiltered_:");
  Serial.print(currentVelFiltered_, 5);
  Serial.print(" ");
  Serial.print("inputVel_:");
  Serial.print(inputVel_, 5);
  Serial.print(" ");
  //Serial.print(pidOut_, 5);
  //Serial.print(" ");
  Serial.print("outputCmd_:");
  Serial.print(outputCmd_/127.0);
  Serial.println(""); */

  
}
