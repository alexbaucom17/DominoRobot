/* Requires libraries:
*  Filters: https://github.com/JonHub/Filters
*  PID: https://playground.arduino.cc/Code/PIDLibrary/
*  Encoder: https://www.pjrc.com/teensy/td_libs_Encoder.html
*  ArduinoJson: https://arduinojson.org
*  LinAlgebra: https://github.com/dams666/LinAlgebra
*/

#include "RobotServer.h"
#include "RobotController.h"
#include "StatusUpdater.h"


StatusUpdater statusUpdater;
RobotServer server = RobotServer(Serial3, Serial, statusUpdater);
RobotController controller = RobotController(Serial, statusUpdater);

void setup()
{
    // Communication with the host computer
    Serial.begin(115200); 
    Serial.println("Robot starting");
}


void loop() 
{
    // Check for new command
    RobotServer::COMMAND newCmd = server.oneLoop();

    // Handle new command
    if(newCmd == RobotServer::COMMAND::MOVE)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPosition(data.x, data.y, data.a);
    }
    else if(newCmd == RobotServer::COMMAND::MOVE_REL)
    {
        RobotServer::PositionData data = server.getMoveData();
        controller.moveToPositionRelative(data.x, data.y, data.a);
    }
    else if (newCmd == RobotServer::COMMAND::POSITION)
    {
        RobotServer::PositionData data = server.getPositionData();
        controller.inputPosition(data.x, data.y, data.a);
    }

    // Service controller
    controller.update();
    
}


/*
// Motor tuning

#include "Motor.h"
#include "globals.h"

const double MotorKp = 70;
const double MotorKi = 1;
const double MotorKd = 0;

Motor myMotor = Motor(PIN_PWM_3, PIN_DIR_3, PIN_ENCA_3, PIN_ENCB_3, MotorKp, MotorKi, MotorKd);


void setup() 
{
  // Setup pin modes
  pinMode(PIN_ENABLE,OUTPUT);
  Serial.begin(115200);
}


int targetDelta = 15;
int targetTotalTime = 1000;
int pauseTime = 1000;
float rps = 1;  // This is about the max the current motors can handle under load
int curDirection = 1;
float curSpeed = curDirection * rps;
int i = 0;
unsigned long motorLoopTime = millis();
unsigned long stepTime = millis();

void loop() 
{

  // Pause step
  if(i == 0)
  {
    if (millis() - stepTime > pauseTime)
    {
      // Done with pause
      stepTime = millis();
      i = 1;
      // Start motor
      digitalWrite(PIN_ENABLE, 1);
    }
  }

  // Motor spin up step
  if(i == 1)
  {
    unsigned long dt = millis() - stepTime;
    float frac = float(dt)/float(targetTotalTime);
    myMotor.setCommand(curSpeed * frac);
    if(dt > targetTotalTime)
    {
      // Done with spin up
      stepTime = millis();
      i = 2;
      //Stop motor
      myMotor.setCommand(curSpeed);
    }
  }

  // Motor const vel step
  if(i == 2)
  {
    if(millis() - stepTime > targetTotalTime)
    {
      // Done with const vel step
      stepTime = millis();
      i = 3;
    }
  }

  // Motor spin down step
  if(i == 3)
  {
    unsigned long dt = millis() - stepTime;
    float frac = float(dt)/float(targetTotalTime);
    myMotor.setCommand(curSpeed * (1-frac));
    if(millis() - stepTime > targetTotalTime)
    {
      // Done with spin up
      stepTime = millis();
      i = 0;
      //Stop motor
      myMotor.setCommand(0);
      digitalWrite(PIN_ENABLE, 0);
    }
  }
   

  // Run this all the time to keep motor updated
  if(millis() - motorLoopTime > targetDelta)
  {
    motorLoopTime = millis();
    myMotor.runLoop();
  }


}*/
