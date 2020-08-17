/* Requires libraries:
 *  Filters: https://github.com/JonHub/Filters
 *  PID: https://playground.arduino.cc/Code/PIDLibrary/
 *  Encoder: https://www.pjrc.com/teensy/td_libs_Encoder.html
 */

#include "Motor.h"

// Pinouts
#define PIN_ENABLE 52

#define PIN_BL_PWM 6
#define PIN_BL_DIR 48


#define PIN_BL_ENC_A 19
#define PIN_BL_ENC_B 29

#define VEL_FILTER_FREQ 10 // HZ

const double Kp = 200;
const double Ki = 2000;
const double Kd = 0;

Motor myMotor = Motor(PIN_BL_PWM, PIN_BL_DIR, PIN_BL_ENC_A, PIN_BL_ENC_B, Kp, Ki, Kd, VEL_FILTER_FREQ);


void setup() 
{
  // Setup pin modes
  pinMode(PIN_ENABLE,OUTPUT);
  
  Serial.begin(250000);
}

void setMotorCommands(float rps)
{
  int curDirection = 1;
  float curSpeed = curDirection * rps;
//  Serial.print("Direction ");
//  Serial.print(curDirection);
//  Serial.print(", Speed ");
//  Serial.println(curSpeed);
  
  myMotor.setCommand(curSpeed);
  digitalWrite(PIN_ENABLE, 1);
}

void runMotors()
{
  unsigned long prevTime = millis();
  unsigned long startTime = prevTime;
  int targetDelta = 15;
  int targetTotalTime = 3000;

  while (millis() - startTime < targetTotalTime)
  {
    if(millis() - prevTime > targetDelta)
    {
      prevTime = millis();
      myMotor.runLoop();
    }
  }
}

void loop() 
{

//  // Get revolutions per second
//  Serial.println("Please enter target revolutions per second");
//  bool gotInput = false;
//  float rps = 0.0;
//  while(!gotInput)
//  {
//    if(Serial.available() > 0)
//    {
//      rps = Serial.parseFloat();
//      if(rps == 0)
//      {
//        continue;
//      }
//      else
//      {
//        break;
//      }
//    }
//    delay(10);
//  }

  delay(1000);
  float rps = 0.5;

  setMotorCommands(rps);

  runMotors();

  // Stop motors
  digitalWrite(PIN_ENABLE, 0);

  

  
}
