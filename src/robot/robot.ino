/* Requires libraries:
 *  Filters: https://github.com/JonHub/Filters
 *  PID: https://playground.arduino.cc/Code/PIDLibrary/
 *  Encoder: https://www.pjrc.com/teensy/td_libs_Encoder.html
 *  ArduinoJson: https://arduinojson.org
 */

#include "globals.h"
#include "RobotServer.h"
#include "Motor.h"

const double Kp = 200;
const double Ki = 2000;
const double Kd = 0;

Motor myMotor = Motor(PIN_BL_PWM, PIN_BL_DIR, PIN_BL_ENC_A, PIN_BL_ENC_B, Kp, Ki, Kd, VEL_FILTER_FREQ);

RobotServer server = RobotServer(Serial3, Serial);

void setup() {

  // Communication with the host computer
  Serial.begin(9600); 
  Serial.println("Wifi client starting");

    // Setup pin modes
  pinMode(PIN_ENABLE,OUTPUT);
  
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

void loop() {

  int cmd = server.oneLoop();
  float rps = 0.5;

  setMotorCommands(rps);

  runMotors();

  // Stop motors
  digitalWrite(PIN_ENABLE, 0);

  delay(1000);

}
