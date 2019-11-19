#ifndef RobotController_h
#define RobotController_h

#include "Motor.h"
#include "TrajectoryGenerator.h"
#include <HardwareSerial.h>

class RobotController
{
  public:

    RobotController(HardwareSerial& debug);

    void moveToPosition(float x, float y, float a);

    void moveToPositionFine(float x, float y, float a);

    void update();

    void enableAllMotors();

    void disableAllMotors();

    void inputPosition();

  private:

    //Internal methods
    void setCartVelCommand(float x, float y, float a);
    void updateMotors();
    void computeControl(PVTPoint cmd);

    // Member variables
    Motor motors[4];
    unsigned long prevMotorLoopTime_;
    HardwareSerial& debug_;
    bool enabled_;
    TrajectoryGenerator trajGen_;
    Point cartPos_;
    Point cartVel_;
    bool trajRunning_;
    unsigned long trajTime_;
    float errSumX_;
    float errSumY_;
    float errSumA_;
    float prevControlLoopTime_;

};

#endif