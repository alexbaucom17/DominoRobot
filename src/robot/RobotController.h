#ifndef RobotController_h
#define RobotController_h

#include "Motor.h"
#include <HardwareSerial.h>

class RobotController
{
  public:

    RobotController(HardwareSerial& debug);

    void update();

    void setCommand(float x, float y, float a);

    void enableAll();

    void disableAll();

    void inputPosition();

  private:
    Motor motors[4];
    unsigned long prevTime_;
    unsigned long moveStartTime_;
    HardwareSerial& debug_;
    bool enabled_;


};

#endif