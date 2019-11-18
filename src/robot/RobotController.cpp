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
  enabled_(false),
  trajGen_(),
  cartPos_(0),
  trajRunning_(false),
  trajTime_(0)
{
    pinMode(PIN_ENABLE,OUTPUT);
}

void RobotController::moveToPosition(float x, float y, float a)
{
    trajGen_.generate(cartPos_, Point(x,y,a))
    trajRunning_ = true;
    trajTime_ = millis();
}

void RobotController::moveToPositionFine(float x, float y, float a)
{
    // TODO
}

void RobotController::update()
{
    if(trajRunning_)
    {
        float dt = static_cast<float>((millis() - trajTime_) / 1000.0); // Convert to seconds
        PVTPoint cmd = trajGen_.lookup(dt);
        computeControl(cmd);

        // TODO: stop traj
    }
}

void RobotController::computeControl(PVTPoint cmd)
{
    // TODO
}

void RobotController::enableAllMotors()
{
    digitalWrite(PIN_ENABLE, 1);
    enabled_ = true;
    debug_.println("Enabling motors");
}

void RobotController::disableAllMotors()
{
    for(int i = 0; i < 4; i++)
    {
        motors[i].setCommand(0);
    }
    digitalWrite(PIN_ENABLE, 0);
    enabled_ = false;
    debug_.println("Disabling motors");
}

void RobotController::inputPosition()
{
  //TODO   
}

void RobotController::updateMotors()
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
            disableAllMotors();
        }
    }
}

void RobotController::setCartVelCommand(float vx, float vy, float va)
{
    // TODO - handle total transtlational vel magnitude correctly
    // Clamp input velocities
    if(abs(vx) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping vx velocity");
        vx = sgn(vx) * MAX_TRANS_SPEED;
    }
    if(abs(vy) > MAX_TRANS_SPEED)
    {
        debug_.println("Capping vy velocity");
        vy = sgn(vy) * MAX_TRANS_SPEED;
    }
    if(abs(va) > MAX_ROT_SPEED)
    {
        debug_.println("Capping angular velocity");
        va = sgn(va) * MAX_ROT_SPEED;
    }

    // Convert input velocities to wheel speeds
    float motorSpeed[4];
    double s0 = sin(PI/4);
    double c0 = cos(PI/4);

    motorSpeed[0] = 1/WHEEL_DIAMETER * (-c0*vx + s0*vy + WHEEL_DIST_FROM_CENTER*va);
    motorSpeed[1] = 1/WHEEL_DIAMETER * ( s0*vx + c0*vy + WHEEL_DIST_FROM_CENTER*va);
    motorSpeed[2] = 1/WHEEL_DIAMETER * ( c0*vx - s0*vy + WHEEL_DIST_FROM_CENTER*va);
    motorSpeed[3] = 1/WHEEL_DIAMETER * (-s0*vx - c0*vy + WHEEL_DIST_FROM_CENTER*va);

    // Set the commanded values for each motor
    for (int i = 0; i < 4; i++)
    {
        motors[i].setCommand(motorSpeed[i]);
    }

    moveStartTime_ = millis();
    enableAllMotors();
}
