#include "RobotController.h"
#include "globals.h"
#include <math.h>

#define targetDelta 15
#define targetMoveTime 3000

const float speed = 0.5; 

const double Kp = 200;
const double Ki = 2000;
const double Kd = 0;

// Utility for getting sign of values
// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


RobotController::RobotController(HardwareSerial& debug)
: motors{
    Motor(PIN_PWM_1, PIN_DIR_1, PIN_ENCA_1, PIN_ENCB_1, Kp, Ki, Kd),
    Motor(PIN_PWM_2, PIN_DIR_2, PIN_ENCA_2, PIN_ENCB_2, Kp, Ki, Kd),
    Motor(PIN_PWM_3, PIN_DIR_3, PIN_ENCA_3, PIN_ENCB_3, Kp, Ki, Kd),
    Motor(PIN_PWM_4, PIN_DIR_4, PIN_ENCA_4, PIN_ENCB_4, Kp, Ki, Kd)},
  prevTime_(millis()),
  moveStartTime_(millis()),
  debug_(debug),
  enabled_(false)
{
    pinMode(PIN_ENABLE,OUTPUT);
}

void RobotController::enableAll()
{
    digitalWrite(PIN_ENABLE, 1);
    enabled_ = true;
    debug_.println("Enabling motors");
}

void RobotController::disableAll()
{
    for(int i = 0; i < 4; i++)
    {
        motors[i].setCommand(0);
    }
    digitalWrite(PIN_ENABLE, 0);
    enabled_ = false;
    debug_.println("Disabling motors");
}

void RobotController::update()
{
    if(enabled_)
    {
        unsigned long curTime = millis();
        if(curTime - prevTime_ > targetDelta)
        {
            for(int i = 0; i < 4; i++)
            {
                motors[i].runLoop();
            }
            prevTime_ = millis();
        }

        if(curTime - moveStartTime_ > targetMoveTime)
        {
            disableAll();
        }
    }
}

void RobotController::setCommand(float x, float y, float a)
{
    // For now this is setting the velocity command. Eventually this will be setting the position command


    // TODO - handle total transtlational vel magnitude correctly
    // Clamp input velocities
    if(abs(x) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping x velocity");
        x = sgn(x) * MAX_TRANS_SPEED;
    }
    if(abs(y) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping y velocity");
        y = sgn(y) * MAX_TRANS_SPEED;
    }
    if(abs(a) > MAX_ROT_SPEED)
    {
        debug_.println("Capping angular velocity");
        a = sgn(a) * MAX_ROT_SPEED;
    }

    // Convert input velocities to wheel speeds
    float motorSpeed[4];
    double s0 = sin(PI/4);
    double c0 = cos(PI/4);

    motorSpeed[0] = 1/WHEEL_DIAMETER * (-c0*x + s0*y + WHEEL_DIST_FROM_CENTER*a);
    motorSpeed[1] = 1/WHEEL_DIAMETER * ( s0*x + c0*y + WHEEL_DIST_FROM_CENTER*a);
    motorSpeed[2] = 1/WHEEL_DIAMETER * ( c0*x - s0*y + WHEEL_DIST_FROM_CENTER*a);
    motorSpeed[3] = 1/WHEEL_DIAMETER * (-s0*x - c0*y + WHEEL_DIST_FROM_CENTER*a);

    // Set the commanded values for each motor
    for (int i = 0; i < 4; i++)
    {
        motors[i].setCommand(motorSpeed[i]);
    }

    moveStartTime_ = millis();
    enableAll();
}

void RobotController::inputPosition()
{
  //TODO   
}