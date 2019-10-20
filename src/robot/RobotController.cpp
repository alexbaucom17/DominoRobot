#include "RobotController.h"
#include "globals.h"

#define targetDelta 15
#define targetMoveTime 3000

const float speed = 0.5; 

const double Kp = 200;
const double Ki = 2000;
const double Kd = 0;

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
    // TODO: Make this use an actual controller method :)

    int motorDirs[4];

    // Forward
    if(x == 1)
    {
        motorDirs[0] = -1;
        motorDirs[1] = 1;
        motorDirs[2] = -1;
        motorDirs[3] = 1;
        debug_.println("Setting motion to forward");
    }
    // Backwards
    else if(x == -1)
    {
        motorDirs[0] = 1;
        motorDirs[1] = -1;
        motorDirs[2] = 1;
        motorDirs[3] = -1;
        debug_.println("Setting motion to backward");
    }
    // Left
    else if(y == 1)
    {
        motorDirs[0] = 1;
        motorDirs[1] = 1;
        motorDirs[2] = -1;
        motorDirs[3] = -1;
        debug_.println("Setting motion to left");
    }
    // Right
    else if(y == -1)
    {
        motorDirs[0] = -1;
        motorDirs[1] = -1;
        motorDirs[2] = 1;
        motorDirs[3] = 1;
        debug_.println("Setting motion to right");
    }
    // Clockwise
    else if(a == -1)
    {
        motorDirs[0] = -1;
        motorDirs[1] = -1;
        motorDirs[2] = -1;
        motorDirs[3] = -1;
        debug_.println("Setting motion to cw");
    }
    // Counter Clockwise
    else if(a == 1)
    {
        motorDirs[0] = 1;
        motorDirs[1] = 1;
        motorDirs[2] = 1;
        motorDirs[3] = 1;
        debug_.println("Setting motion to ccw");
    }
    // Invalid
    else
    {
        debug_.println("Invalid motion");
        return;
    }

    // Set the commanded values for each motor
    for (int i = 0; i < 4; i++)
    {
        motors[i].setCommand(speed * motorDirs[i]);
    }

    moveStartTime_ = millis();
    enableAll();
}

void RobotController::inputPosition()
{
  //TODO   
}